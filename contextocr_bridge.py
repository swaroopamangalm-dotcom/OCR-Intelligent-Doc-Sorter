"""
contextocr_bridge.py - connects the ESP32 sorter to the ContextOCR-GraphRAG backend.

  ESP32  --"SCAN,DOC1"-->  this script
                           1. grabs a frame from the USB camera
                           2. POST /api/documents/upload
                           3. POST /api/runs  (confidence_threshold, domain)
                           4. polls GET /api/runs/{id} until COMPLETED / FAILED
                              (sends "WAIT,DOC1" heartbeats meanwhile)
                           5. maps the run result to ACCEPT / REVIEW
  ESP32  <--"DECISION,DOC1,ACCEPT,<reason>"--

Install:  pip install pyserial requests opencv-python
Run    :  start the backend first (uvicorn app.main:app --port 8000), then
          python contextocr_bridge.py --port COM5          (Windows)
          python contextocr_bridge.py --port /dev/ttyUSB0  (Linux)
Close the Arduino IDE Serial Monitor first - only one program can own the port.
"""
import argparse
import time
from pathlib import Path

import cv2
import requests
import serial

# ---- decision rules (mirror slide 37: high confidence -> ACCEPT, else REVIEW) ----
MIN_OCR_TOKENS = 5          # fewer tokens = blank / unreadable page -> REVIEW


def decide(run: dict, accept_min_conf: float) -> tuple[str, str]:
    """Return (ACCEPT|REVIEW, reason) from a ContextOCR run record."""
    if run.get("status") != "COMPLETED":
        return "REVIEW", f"run_{str(run.get('status', 'unknown')).lower()}"
    result = run.get("result") or {}
    stats = result.get("stats") or {}
    conf = (result.get("confidence") or {}).get("average")

    if stats.get("ocr_tokens", 0) < MIN_OCR_TOKENS:
        return "REVIEW", "too_little_text"
    if conf is None:
        return "REVIEW", "no_confidence"
    if stats.get("corrections_rejected", 0) > 0:
        return "REVIEW", "rejected_corrections"
    if any(v.get("status") != "ACCEPTED" for v in result.get("validations", [])):
        return "REVIEW", "validation_not_accepted"
    if any(d.get("status") in ("ERROR", "UNAVAILABLE") for d in result.get("slm_decisions", [])):
        return "REVIEW", "slm_unavailable"
    if conf < accept_min_conf:
        return "REVIEW", f"low_conf_{conf:.3f}"
    return "ACCEPT", f"conf_{conf:.3f}"


def capture(cam: cv2.VideoCapture, out_dir: Path, doc_id: str) -> Path:
    for _ in range(5):          # flush stale frames buffered while the belt moved
        cam.grab()
    ok, frame = cam.read()
    if not ok:
        raise RuntimeError("camera read failed")
    path = out_dir / f"{doc_id}_{int(time.time())}.png"
    cv2.imwrite(str(path), frame)
    return path


def process(doc_id: str, image: Path, args, ser: serial.Serial) -> tuple[str, str]:
    api = args.api.rstrip("/")
    with image.open("rb") as fh:
        r = requests.post(f"{api}/api/documents/upload",
                          files={"file": (image.name, fh, "image/png")}, timeout=60)
    r.raise_for_status()
    document_id = r.json()["id"]

    r = requests.post(f"{api}/api/runs", json={
        "document_id": document_id,
        "confidence_threshold": args.threshold,
        "domain": args.domain,
    }, timeout=30)
    r.raise_for_status()
    run_id = r.json()["id"]
    print(f"[{doc_id}] run {run_id} started")

    deadline = time.time() + args.max_wait
    while time.time() < deadline:
        run = requests.get(f"{api}/api/runs/{run_id}", timeout=30).json()
        if run.get("status") in ("COMPLETED", "FAILED"):
            return decide(run, args.accept_conf)
        ser.write(f"WAIT,{doc_id}\n".encode())      # keep ESP32 from timing out
        time.sleep(2)
    return "REVIEW", "bridge_timeout"


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="ESP32 serial port, e.g. COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--api", default="http://localhost:8000")
    p.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    p.add_argument("--domain", default="general")
    p.add_argument("--threshold", type=float, default=0.90, help="pipeline low-confidence gate")
    p.add_argument("--accept-conf", type=float, default=0.90, help="min avg confidence to ACCEPT")
    p.add_argument("--max-wait", type=float, default=280, help="seconds per document (< ESP32 hard cap)")
    p.add_argument("--captures", default="captures")
    args = p.parse_args()

    out_dir = Path(args.captures); out_dir.mkdir(exist_ok=True)
    cam = cv2.VideoCapture(args.camera)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    if not cam.isOpened():
        raise SystemExit(f"Camera {args.camera} not found")

    requests.get(f"{args.api.rstrip('/')}/api/system/status", timeout=10).raise_for_status()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)   # opening resets the ESP32
    print("Waiting for ESP32 READY ...")
    print("Bridge running. Place a document on the input tray.  Ctrl+C to quit.")
    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if not line.startswith("HB,"):
                print("ESP32>", line)
            if line.startswith("SCAN,"):
                doc_id = line.split(",", 1)[1]
                try:
                    img = capture(cam, out_dir, doc_id)
                    decision, reason = process(doc_id, img, args, ser)
                except Exception as exc:                   # any failure -> safe tray
                    decision, reason = "REVIEW", f"bridge_error_{type(exc).__name__}"
                print(f"[{doc_id}] -> {decision} ({reason})")
                ser.write(f"DECISION,{doc_id},{decision},{reason}\n".encode())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close(); cam.release()


if __name__ == "__main__":
    main()
