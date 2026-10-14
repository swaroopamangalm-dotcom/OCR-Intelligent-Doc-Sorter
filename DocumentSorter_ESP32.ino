/*
 * ============================================================================
 *  AI-Driven Intelligent Document Analysis and Robotic Sorting System
 *  ESP32 controller firmware  (Arduino IDE)
 *
 *  Software side : ContextOCR-GraphRAG  (github.com/Aravind-rr/ContextOCR-GraphRAG)
 *                  runs on the laptop; contextocr_bridge.py turns each run's
 *                  result into ACCEPT / REVIEW and sends it here over USB serial.
 *  Hardware side : ESP32 Dev Board, 2x IR sensors, SG90 servo gate,
 *                  L298N + DC geared conveyor motor, 5 V adapter, common GND.
 *
 *  Pin map (from the team's Tinkercad circuit):
 *      IR Sensor 1 OUT (document entry) -> GPIO 32
 *      IR Sensor 2 OUT (sorting gate)   -> GPIO 33
 *      Servo SG90 SIG                   -> GPIO 25
 *      L298N IN1                        -> GPIO 26
 *      L298N IN2                        -> GPIO 27
 *      L298N ENA (PWM, remove jumper)   -> GPIO 14
 *
 *  Working flow (slides 36/37 + circuit "Working Flow"):
 *      1 Document on input tray      2 IR1 detects entry
 *      3 Conveyor moves it to the scanning zone and stops
 *      4 ESP32 -> "SCAN,<id>"  ; laptop captures + runs OCR/GraphRAG/Qwen
 *      5 Laptop -> "DECISION,<id>,ACCEPT|REVIEW"
 *      6 Servo turns the gate      7 Conveyor runs until IR2 confirms exit
 *  Safe routing: no reply / timeout / error / sensor mismatch -> REVIEW tray.
 *
 *  Library: "ESP32Servo" by Kevin Harrington (Library Manager).
 *  Board  : "ESP32 Dev Module" (esp32 by Espressif, core 2.x or 3.x).
 * ============================================================================
 */

#include <ESP32Servo.h>

// ----------------------------- PIN MAP --------------------------------------
const int PIN_IR_ENTRY = 32;   // IR Sensor 1
const int PIN_IR_GATE  = 33;   // IR Sensor 2
const int PIN_SERVO    = 25;   // SG90 signal
const int PIN_IN1      = 26;   // L298N IN1
const int PIN_IN2      = 27;   // L298N IN2
const int PIN_ENA      = 14;   // L298N ENA (PWM)

// ----------------------------- TUNING (calibrate on the real rig) -----------
// Most FC-51 / TCRT IR modules pull OUT LOW when an object is detected.
const bool IR_ACTIVE_LOW = true;

const int  SERVO_ACCEPT_DEG  = 30;    // gate angle that guides paper to ACCEPT tray
const int  SERVO_REVIEW_DEG  = 150;   // gate angle that guides paper to REVIEW tray
const int  SERVO_HOME_DEG    = SERVO_REVIEW_DEG;  // fail-safe resting position
const unsigned long SERVO_SETTLE_MS = 450;

const uint8_t MOTOR_SPEED      = 170;  // 0-255 PWM duty for the belt
const bool    MOTOR_REVERSE    = false; // flip if belt runs backwards

const unsigned long FEED_TO_SCAN_MS     = 1200;  // run time from IR1 to camera zone
const unsigned long SCAN_SETTLE_MS      = 300;   // paper stops wobbling before capture
const unsigned long DECISION_TIMEOUT_MS = 30000; // reset by every "WAIT" heartbeat
const unsigned long DECISION_HARD_MAX_MS= 300000;// absolute cap (Qwen can be slow)
const unsigned long GATE_REACH_TIMEOUT_MS = 6000;// belt run allowed until IR2 sees paper
const unsigned long GATE_CLEAR_TIMEOUT_MS = 6000;// paper must leave IR2 within this
const unsigned long EXIT_EXTRA_RUN_MS   = 600;   // push paper fully into tray
const unsigned long IR_DEBOUNCE_MS      = 40;
const unsigned long HEARTBEAT_MS        = 5000;  // "HB" status line to laptop

// ----------------------------- STATE MACHINE --------------------------------
enum State {
  ST_IDLE,            // waiting for a document at IR1
  ST_FEEDING,         // belt moving paper into the scanning zone
  ST_SETTLING,        // belt stopped, letting the sheet settle
  ST_WAIT_DECISION,   // "SCAN" sent, waiting for laptop decision
  ST_GATE_MOVE,       // servo turning to the chosen tray
  ST_TO_GATE,         // belt running until IR2 sees the paper
  ST_CLEAR_GATE,      // belt running until IR2 clears
  ST_EXIT_RUN,        // short extra run so the sheet drops into the tray
  ST_FAULT            // jam / sensor fault: motor off, needs "RESET"
};

enum Route { ROUTE_NONE, ROUTE_ACCEPT, ROUTE_REVIEW };

Servo gateServo;

State  state = ST_IDLE;
Route  route = ROUTE_NONE;
unsigned long stateStart   = 0;
unsigned long decisionDeadline = 0;
unsigned long decisionStart    = 0;
unsigned long lastHeartbeat    = 0;
uint32_t docCounter = 0;
String   currentDocId = "";
String   routeReason  = "";
String   faultReason  = "";
bool     autoMode = true;          // "MANUAL" disables IR1 auto-start
String   rxLine;

// debounced IR readings
struct IrInput {
  int pin; bool stable; bool lastRaw; unsigned long changedAt;
};
IrInput irEntry = {PIN_IR_ENTRY, false, false, 0};
IrInput irGate  = {PIN_IR_GATE,  false, false, 0};

// ----------------------------- HELPERS --------------------------------------
bool readIrRaw(int pin) {
  int v = digitalRead(pin);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void updateIr(IrInput &ir) {
  bool raw = readIrRaw(ir.pin);
  if (raw != ir.lastRaw) { ir.lastRaw = raw; ir.changedAt = millis(); }
  if (raw != ir.stable && millis() - ir.changedAt >= IR_DEBOUNCE_MS) ir.stable = raw;
}

// ENA PWM via LEDC (works on esp32 core 2.x and 3.x, no clash with ESP32Servo)
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif
const int MOTOR_PWM_FREQ = 5000;   // L298N is happiest at a few kHz
const int MOTOR_PWM_BITS = 8;
const int MOTOR_LEDC_CH  = 10;     // core 2.x only; kept off the servo's timer

void motorPwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
  ledcSetup(MOTOR_LEDC_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(PIN_ENA, MOTOR_LEDC_CH);
#endif
}

void motorDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_ENA, duty);
#else
  ledcWrite(MOTOR_LEDC_CH, duty);
#endif
}

void motorRun(uint8_t speed) {
  digitalWrite(PIN_IN1, MOTOR_REVERSE ? LOW : HIGH);
  digitalWrite(PIN_IN2, MOTOR_REVERSE ? HIGH : LOW);
  motorDuty(speed);
}

void motorStop() {
  motorDuty(0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

void setGate(Route r) {
  gateServo.write(r == ROUTE_ACCEPT ? SERVO_ACCEPT_DEG : SERVO_REVIEW_DEG);
}

const char* routeName(Route r) {
  return r == ROUTE_ACCEPT ? "ACCEPT" : (r == ROUTE_REVIEW ? "REVIEW" : "NONE");
}

const char* stateName(State s) {
  switch (s) {
    case ST_IDLE: return "IDLE";            case ST_FEEDING: return "FEEDING";
    case ST_SETTLING: return "SETTLING";    case ST_WAIT_DECISION: return "WAIT_DECISION";
    case ST_GATE_MOVE: return "GATE_MOVE";  case ST_TO_GATE: return "TO_GATE";
    case ST_CLEAR_GATE: return "CLEAR_GATE";case ST_EXIT_RUN: return "EXIT_RUN";
    default: return "FAULT";
  }
}

void enterState(State s) {
  state = s;
  stateStart = millis();
  Serial.print("STATE,"); Serial.print(currentDocId.length() ? currentDocId : "-");
  Serial.print(","); Serial.println(stateName(s));
}

void fault(const String &why) {
  motorStop();
  faultReason = why;
  Serial.print("FAULT,"); Serial.print(currentDocId); Serial.print(","); Serial.println(why);
  enterState(ST_FAULT);
}

void startDocument() {
  docCounter++;
  currentDocId = "DOC" + String(docCounter);
  route = ROUTE_NONE;
  routeReason = "";
  Serial.print("DETECTED,"); Serial.println(currentDocId);
  setGate(ROUTE_REVIEW);          // safe default while unknown
  motorRun(MOTOR_SPEED);
  enterState(ST_FEEDING);
}

void applyRoute(Route r, const String &reason) {
  route = r;
  routeReason = reason;
  Serial.print("ROUTE,"); Serial.print(currentDocId); Serial.print(",");
  Serial.print(routeName(r)); Serial.print(","); Serial.println(reason);
  setGate(r);
  enterState(ST_GATE_MOVE);
}

// ----------------------------- SERIAL PROTOCOL ------------------------------
/*  Laptop -> ESP32 (one command per line, 115200 baud):
 *    DECISION,<docId>,ACCEPT|REVIEW[,reason]   result for the pending document
 *    WAIT,<docId>                              heartbeat: OCR still running
 *    PING                                      -> PONG
 *    STATUS                                    -> STATUS,<state>,<doc>,<ir1>,<ir2>
 *    RESET                                     clear FAULT, return to IDLE
 *    START                                     start a cycle without IR1 (testing)
 *    MANUAL / AUTO                             disable / enable IR1 auto-start
 *    SERVO,<deg>   MOTOR,<0-255>   STOP        hardware tests (IDLE/FAULT only)
 *  ESP32 -> Laptop:
 *    READY | DETECTED,<id> | SCAN,<id> | ROUTE,<id>,<tray>,<reason>
 *    DONE,<id>,<tray> | FAULT,<id>,<reason> | STATE,<id>,<state> | HB,... | ERR,...
 */
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // split on commas (max 4 fields)
  String f[4]; int n = 0, start = 0;
  for (int i = 0; i <= (int)line.length() && n < 4; i++) {
    if (i == (int)line.length() || line[i] == ',') {
      f[n++] = line.substring(start, i); f[n - 1].trim(); start = i + 1;
    }
  }
  String cmd = f[0]; cmd.toUpperCase();

  if (cmd == "PING") { Serial.println("PONG"); return; }

  if (cmd == "STATUS") {
    Serial.print("STATUS,"); Serial.print(stateName(state)); Serial.print(",");
    Serial.print(currentDocId.length() ? currentDocId : "-"); Serial.print(",");
    Serial.print(irEntry.stable ? 1 : 0); Serial.print(","); Serial.println(irGate.stable ? 1 : 0);
    return;
  }

  if (cmd == "DECISION") {
    if (state != ST_WAIT_DECISION) { Serial.println("ERR,NOT_WAITING_FOR_DECISION"); return; }
    if (f[1] != currentDocId)      { Serial.println("ERR,DOC_ID_MISMATCH," + f[1]); return; }
    String d = f[2]; d.toUpperCase();
    String reason = n >= 4 ? f[3] : "ai";
    if (d == "ACCEPT")      applyRoute(ROUTE_ACCEPT, reason);
    else                    applyRoute(ROUTE_REVIEW, d == "REVIEW" ? reason : "unknown_decision");
    return;
  }

  if (cmd == "WAIT") {
    if (state == ST_WAIT_DECISION && f[1] == currentDocId) {
      unsigned long next = millis() + DECISION_TIMEOUT_MS;
      unsigned long cap  = decisionStart + DECISION_HARD_MAX_MS;
      decisionDeadline = (long)(next - cap) > 0 ? cap : next;
    }
    return;
  }

  if (cmd == "RESET") {
    motorStop(); setGate(ROUTE_REVIEW);
    currentDocId = ""; faultReason = "";
    enterState(ST_IDLE);
    return;
  }

  if (cmd == "MANUAL") { autoMode = false; Serial.println("OK,MANUAL"); return; }
  if (cmd == "AUTO")   { autoMode = true;  Serial.println("OK,AUTO");   return; }

  if (cmd == "START") {
    if (state == ST_IDLE) startDocument(); else Serial.println("ERR,BUSY");
    return;
  }

  // ---- hardware test commands (only when nothing is moving) ----
  bool testable = (state == ST_IDLE || state == ST_FAULT);
  if (cmd == "STOP")  { motorStop(); Serial.println("OK,STOP"); return; }
  if (cmd == "SERVO") {
    if (!testable) { Serial.println("ERR,BUSY"); return; }
    int deg = constrain(f[1].toInt(), 0, 180); gateServo.write(deg);
    Serial.println("OK,SERVO," + String(deg)); return;
  }
  if (cmd == "MOTOR") {
    if (!testable) { Serial.println("ERR,BUSY"); return; }
    int sp = constrain(f[1].toInt(), 0, 255);
    if (sp == 0) motorStop(); else motorRun(sp);
    Serial.println("OK,MOTOR," + String(sp)); return;
  }

  Serial.println("ERR,UNKNOWN_COMMAND," + cmd);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleCommand(rxLine); rxLine = ""; }
    else if (rxLine.length() < 120) rxLine += c;
  }
}

// ----------------------------- SETUP / LOOP ---------------------------------
void setup() {
  // Stop the motor first: GPIO14 can glitch during boot.
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_ENA, LOW);

  pinMode(PIN_IR_ENTRY, INPUT);   // modules have their own pull-up
  pinMode(PIN_IR_GATE,  INPUT);

  Serial.begin(115200);
  delay(200);

  ESP32PWM::allocateTimer(0);     // keep servo and motor PWM on separate timers
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);   // SG90 pulse range
  gateServo.write(SERVO_HOME_DEG);

  motorPwmSetup();                // after the servo so it keeps its own channel
  motorStop();

  irEntry.stable = irEntry.lastRaw = readIrRaw(PIN_IR_ENTRY);
  irGate.stable  = irGate.lastRaw  = readIrRaw(PIN_IR_GATE);

  Serial.println("READY");
  enterState(ST_IDLE);
}

void loop() {
  readSerial();
  updateIr(irEntry);
  updateIr(irGate);
  unsigned long now = millis();
  unsigned long inState = now - stateStart;

  switch (state) {

    case ST_IDLE:
      if (autoMode && irEntry.stable) {
        if (irGate.stable) fault("IR2_BLOCKED_AT_START");   // paper stuck at gate
        else startDocument();
      }
      break;

    case ST_FEEDING:
      if (irGate.stable) {                       // overshot the camera zone
        motorStop();
        applyRoute(ROUTE_REVIEW, "overshoot_before_scan");
      } else if (inState >= FEED_TO_SCAN_MS) {
        motorStop();
        enterState(ST_SETTLING);
      }
      break;

    case ST_SETTLING:
      if (inState >= SCAN_SETTLE_MS) {
        Serial.print("SCAN,"); Serial.println(currentDocId);
        decisionStart = now;
        decisionDeadline = now + DECISION_TIMEOUT_MS;
        enterState(ST_WAIT_DECISION);
      }
      break;

    case ST_WAIT_DECISION:
      if (irGate.stable) {                       // paper slid into gate while waiting
        applyRoute(ROUTE_REVIEW, "sensor_mismatch");
      } else if ((long)(now - decisionDeadline) >= 0) {
        applyRoute(ROUTE_REVIEW, "decision_timeout");
      }
      break;

    case ST_GATE_MOVE:
      if (inState >= SERVO_SETTLE_MS) {
        motorRun(MOTOR_SPEED);
        enterState(irGate.stable ? ST_CLEAR_GATE : ST_TO_GATE);
      }
      break;

    case ST_TO_GATE:
      if (irGate.stable)                          enterState(ST_CLEAR_GATE);
      else if (inState >= GATE_REACH_TIMEOUT_MS)  fault("JAM_BEFORE_GATE");
      break;

    case ST_CLEAR_GATE:
      if (!irGate.stable)                         enterState(ST_EXIT_RUN);
      else if (inState >= GATE_CLEAR_TIMEOUT_MS)  fault("JAM_AT_GATE");
      break;

    case ST_EXIT_RUN:
      if (inState >= EXIT_EXTRA_RUN_MS) {
        motorStop();
        Serial.print("DONE,"); Serial.print(currentDocId); Serial.print(",");
        Serial.print(routeName(route)); Serial.print(","); Serial.println(routeReason);
        setGate(ROUTE_REVIEW);                    // back to safe position
        currentDocId = "";
        enterState(ST_IDLE);
      }
      break;

    case ST_FAULT:
      motorStop();                                // stay stopped until RESET
      break;
  }

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    Serial.print("HB,"); Serial.print(stateName(state)); Serial.print(",");
    Serial.print(irEntry.stable ? 1 : 0); Serial.print(","); Serial.println(irGate.stable ? 1 : 0);
  }
}
