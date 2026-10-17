/*
 * Multi-Node CAN Stepper Example
 * THINGS BY JOSH 2026
 * 
 * Each Node can receive commands over serial (USB) and CANBUS.
 * Messages received over USB serial will be forwarded onto the CAN bus (USB to CAN bridge).
 * See the CAN protocol document for CAN and serial message formats
 * 
 *
 * Architecture:
 * 1. Motion Task (1000Hz): Calculates ramping and sets target delay. For speed reasons this uses Q-fixed integer math (no floats)
 * 2. Step ISR (One-Shot): Generates pulses based on target delay. Direct register writes for speed.
 *    Both of the above run as genuine hardware timer interrupts (timerBegin()/timerAttachInterrupt(),
 *    NOT esp_timer), explicitly tied to Core 1 -- see "TIMERS SETUP" in setup() for why this
 *    matters: a real interrupt preempts task-level code (including WiFi's own high-priority
 *    internal task) almost unconditionally, unlike esp_timer's background dispatch task, whose
 *    CPU core isn't fixed and could get delayed by WiFi activity regardless of core pinning
 *    anywhere else in the sketch.
 * 3. WiFi / WebSocket (optional, port 81): once provisioned over USB serial (WIFI_SET command), the node
 *    joins the given network and exposes the exact same command/telemetry line protocol over a WebSocket,
 *    so the Web GUI can control this one node wirelessly instead of over USB. Credentials are only ever
 *    sent over the USB link (never the CAN bus, which is shared by every node) and persist in flash, so
 *    the node reconnects automatically on every boot. Hostname is canbus-stepper-<NODE_ID>.local (mDNS).
 *    Requires the "WebSockets" library by Markus Sattler (Links2004) - Arduino Library Manager -
 *    https://github.com/Links2004/arduinoWebSockets
 *    All WiFi/WebSocket/mDNS calls run on their own FreeRTOS task (wifiTaskFn(), see
 *    "WiFi / WebSocket (optional, serial-provisioned)" below), pinned to Core 0 as general good
 *    practice (keeps it off Core 1, where loop() and the hardware step/motion interrupts above
 *    live) -- though the step/motion timers' own hardware-interrupt affinity (point 1/2 above) is
 *    what actually makes them immune to WiFi activity, not this pinning by itself. It talks to the
 *    main loop() (Core 1) only via two small queues (wsInQueue/wsOutQueue) and a small mutex
 *    (wifiConfigMux) guarding the SSID/password/hostname buffers -- loop()/executeCommand() never
 *    touch WiFi/wsServer/MDNS directly. WiFi station power-save ("modem sleep") is also disabled
 *    (WiFi.setSleep(false), see startWifiConnect()) to avoid its periodic RF wake/resync stalls.
 *
 *
 * TODO:
 * - max out I2C speed?
 * - Give error for invalid commands
 * - Encoder calibration option for offset magnet?
 * - Add header file with CAN ID's instead of magic numbers!
 * - Better handle change of microsteps / accel / dir etc. better (update all values which rely on these when changed)
 * - Verify accel/decel/speed timing
 * - Add better input sanitation for other commands (range checks, e.g. as done for Set Node ID)
 * - Fault if another node with same ID exists
 * - Decide if EEPROM settings should be overwritten on flash?
 * - Error if too many steps per sec output
 * - [DONE v0.15] WiFi / WebSocket control -- see WIFI_SET / WIFI_STATUS?
 * - [DONE] WiFi/WebSocket/mDNS moved to their own task pinned to Core 0 (wifiTaskFn) so
 *   WiFi traffic can't make motion choppy -- see wsInQueue/wsOutQueue/wifiConfigMux
 * - [DONE] Motion/step timers switched from esp_timer to true hardware timer interrupts
 *   (timerBegin/timerAttachInterrupt) -- root-caused the remaining WiFi choppiness to
 *   esp_timer's background dispatch task floating onto whatever core WiFi's own
 *   high-priority internal task needed; a real hardware ISR isn't subject to that
 * - [DONE] Fixed boot-loop (Coprocessor exception / Core 1 panic) introduced by the above:
 *   onMotionTimer() did one bit of float math per tick (posSpeed -> Q-fixed max speed).
 *   That's fine inside a FreeRTOS task but crashes inside a real hardware ISR, which
 *   doesn't save/restore FPU context. Moved to recalcMotionParams() (task context) and
 *   cached in posSpeedStepsQ -- onMotionTimer()/onStepTimer() are now float-free
 */


#include <ESP32-TWAI-CAN.hpp> //https://github.com/handmade0octopus/ESP32-TWAI-CAN
#include <TMC2209.h> //https://github.com/janelia-arduino/TMC2209
#include <Wire.h>
#include <Preferences.h>
#include "esp_rom_sys.h" // esp_rom_delay_us() -- tight busy-wait used in onStepTimer()
#include <math.h> // used only in readNTCTemperature for log()
#include <cstdint> // INT32_MAX, used by the MAX_STEP_RATE_HZ static_assert below

#include <WiFi.h>              // ESP32 core
#include <ESPmDNS.h>           // ESP32 core -- advertises canbus-stepper-<NODE_ID>.local
#include <WebSocketsServer.h>  // "WebSockets" by Markus Sattler (Links2004) - Arduino Library Manager - https://github.com/Links2004/arduinoWebSockets

// Explicit FreeRTOS includes for the dedicated WiFi task below (xTaskCreatePinnedToCore,
// the wsIn/wsOutQueue queues). portMUX_TYPE/portENTER_CRITICAL already work without these
// (pulled in transitively via Arduino.h), but task/queue APIs are made explicit here.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

Preferences preferences;

// -------------------- Pin Defines --------------------
// CAN
#define CAN_TX   48
#define CAN_RX   47
#define STB      37

//AUX
#define AUX1    14
#define AUX2    13   

// Stepper Driver (TMC2209)
#define TMC_EN  21
#define STEP    5
#define DIR     6
#define MS1     1
#define MS2     2
#define TMC_TX  17
#define TMC_RX  18
#define DIAG    16
#define INDEX   11

// Precomputed bit masks (for faster step outputs)
static const uint32_t STEP_MASK = (1UL << STEP);
static const uint32_t DIR_MASK  = (1UL << DIR);

// I2C Encoder
#define SCL 9
#define SDA 8
#define MT6701_ADDRESS  0x06

// Other
#define LED1 10
#define LED2 12
#define SW1 35
#define SW2 36
#define VBUS 4
#define NTC 7
#define DISABLE_3V3 38

const int MAX_CAN_ID = 31;

// TMC2209 has no power when running from USB alone; only powered via
// external VBUS. Above this threshold the driver is considered powered.
const float DRIVER_VBUS_THRESHOLD = 6.0; // Volts

// How long to hold the motor at standstill (motionHold) after the TMC2209
// is (re-)enabled, letting current regulation settle before ramping.
const unsigned long DRIVER_SETTLE_MS = 100;

// ---------------- Fault Thresholds (see CAN Protocol.md "Fault Codes") ---------------
const float PCB_TEMP_WARNING_C    = 100.0f; // PCB temperature (NTC, MsgType 38) warning threshold
const float PCB_TEMP_SHUTDOWN_C   = 125.0f; // PCB temperature (NTC, MsgType 38) shutdown threshold
const float PCB_TEMP_HYSTERESIS_C = 5.0f;   // fault clears once temp drops this far back below its threshold

// ---------------- Fault Code (MsgType 39) bit definitions ---------------
// Bitfield -- multiple faults can be active at once. See CAN Protocol.md.
const uint16_t FAULT_PCB_OVERTEMP_WARNING  = (1 << 0); // PCB temp >= PCB_TEMP_WARNING_C
const uint16_t FAULT_PCB_OVERTEMP_SHUTDOWN = (1 << 1); // PCB temp >= PCB_TEMP_SHUTDOWN_C

// TMC2209-reported faults, read over UART via getStatus()/getGlobalStatus().
// Report-only, mirrors the driver's live status each check.
const uint16_t FAULT_DRIVER_OVERTEMP_WARNING          = (1 << 2); // driver die temp warning (independent of PCB NTC)
const uint16_t FAULT_DRIVER_OVERTEMP_SHUTDOWN         = (1 << 3); // driver die temp shutdown (independent of PCB NTC)
const uint16_t FAULT_DRIVER_SHORT_TO_GROUND_A         = (1 << 4); // short to ground, motor phase A
const uint16_t FAULT_DRIVER_SHORT_TO_GROUND_B         = (1 << 5); // short to ground, motor phase B
const uint16_t FAULT_DRIVER_LOW_SIDE_SHORT_A          = (1 << 6); // low-side short, motor phase A
const uint16_t FAULT_DRIVER_LOW_SIDE_SHORT_B          = (1 << 7); // low-side short, motor phase B
const uint16_t FAULT_DRIVER_CHARGE_PUMP_UNDERVOLTAGE  = (1 << 8); // charge pump undervoltage (uv_cp)
// Bits 9-15 reserved for future fault types.

CanFrame rxFrame;
CanFrame txFrame;

// ---------------- FIRMWARE VERSION ----------------------------------------------------------
float firmwareVersion = 0.15; // 0.15 adds WiFi / WebSocket control -- the Web GUI prompts to update any node reporting 0.14 or lower
// --------------------------------------------------------------------------------------------

// ---------------- Stepper Driver ---------------
TMC2209 stepper_driver;
HardwareSerial & serial_stream = Serial2;
const long SERIAL_BAUD_RATE = 115200;

// ---------------- Encoder ---------------
int64_t total_encoder_counts = 0;
double angle = 0.0; // degrees for telemetry (kept as float for telemetry payload)
int encoder_offset = 0; //initial offset (if enables)

// Closed-loop position sync deadband (see loop()), in microsteps. Only
// resync isr_currentPos to the encoder once the discrepancy exceeds this.
const int64_t CLOSED_LOOP_SYNC_DEADBAND_STEPS = 2;

// ---------------- Scheduling ---------------
unsigned long lastFreq1 = 0;
unsigned long lastFreq2 = 0;

// -------------------- Fixed-point (Q-scale) Motion Control Structures --------------------
// Fixed point helpers. Ramp state (speed/accel) is stored as (value * ONE_Q)
// in int32_t, so max representable speed = INT32_MAX / ONE_Q steps/sec.
static const int32_t Q = 12;
static const int32_t ONE_Q = 1 << Q;            // 4096
static const int64_t ONE_Q64 = (int64_t)ONE_Q;

// Maximum step rate (microsteps/sec) the ramp will ever target, in Position
// or Velocity mode -- bench-measured ceiling of the step-generation path
// (onStepTimer()) at 200 steps/rev / 256 and 128 microsteps.
const int32_t MAX_STEP_RATE_HZ = 25000;
const int32_t MAX_STEP_RATE_Q  = MAX_STEP_RATE_HZ * ONE_Q;
static_assert((int64_t)MAX_STEP_RATE_HZ * (int64_t)ONE_Q <= INT32_MAX,
              "MAX_STEP_RATE_HZ * ONE_Q overflows int32_t -- see comment above");

// Speed the ramp snaps to immediately when starting from a stop (see
// onMotionTimer()), instead of ramping up from 0. Tune to taste.
const int32_t MIN_START_SPEED_HZ = 30;
const int32_t MIN_START_SPEED_Q  = MIN_START_SPEED_HZ * ONE_Q;

struct MotionState {
    // velocities/accels stored in Q-fixed point (steps/sec * ONE_Q)
    volatile int32_t currentSpeed_q;     // Q-fixed steps/sec
    volatile int32_t targetSpeed_q;      // Q-fixed steps/sec
    volatile int32_t accelSteps_q;       // Q-fixed steps/sec^2
    volatile int32_t decelSteps_q;       // Q-fixed steps/sec^2

    volatile int64_t targetPos;          // Target Position in steps (for Position Mode)
    volatile bool isRunning;             // Is the step timer active?
};

volatile MotionState motion = {0,0,0,0,0,false};

// Position mode's max speed (posSpeed, deg/sec), pre-converted to Q-fixed steps/sec and
// clamped to MAX_STEP_RATE_Q. Recomputed by recalcMotionParams() (plain task context, float
// math is fine there) whenever posSpeed/stepsPerRev/microsteps change -- NOT recomputed
// every tick inside onMotionTimer() anymore. onMotionTimer() now runs as a true hardware
// timer ISR (see "TIMERS SETUP" in setup()), and floating-point instructions inside a real
// ISR crash with a Coprocessor Exception on Xtensa/ESP32 -- the FPU context-save that makes
// float math safe is only set up for task-level code, not interrupt-level code. This mirrors
// motion.accelSteps_q/decelSteps_q, which were already handled the same way.
volatile int32_t posSpeedStepsQ = 0;

// Position control (main-thread copies, used for UI/telemetry)
bool posControl = 1; // 0 = Velocity Mode, 1 = Position Mode

// ISR / Timer state (shared with ISR)
volatile int64_t isr_currentPos = 0;   // current logical step count (in microsteps)
volatile uint32_t isr_stepDelay_us = 0; // period between steps (0 = stopped)
volatile bool isr_dirState = false;    // The specific direction pin state to write

// STEP-high pulse width for onStepTimer(), in microseconds (TMC2209 needs ~100ns).
const uint32_t STEP_PULSE_HIGH_US = 2;

// Adjustable deadband for position mode (avoids oscillation at target)
volatile int32_t isr_deadband = 2;

// When true, onMotionTimer() forces the ramp target to zero regardless of
// mode -- used to hold a genuine standstill (see configureDriver()).
volatile bool motionHold = false;

// ---------------- Default Values (per CAN Protocol doc) ---------------
// Single source of truth for startup defaults and Reset to Default (MsgType 23).
const unsigned int DEFAULT_MICROSTEPS      = 16;
const unsigned int DEFAULT_CURRENT         = 30;      // %
const unsigned int DEFAULT_HOLD_CURRENT    = 30;      // % (see MsgType 27)
const unsigned int DEFAULT_STALL_THRESH    = 10;
const unsigned int DEFAULT_STEPS_PER_REV   = 200;
const unsigned int DEFAULT_CONTROL_TYPE    = 0;       // 0=Open Loop, 1=Closed Loop
const unsigned int DEFAULT_STANDSTILL_MODE = 0;       // 0=Normal
const bool         DEFAULT_MAP_DIRECTION   = 0;       // 0=Normal
const float        DEFAULT_POS_SPEED       = 2000.0f; // deg/sec
const float        DEFAULT_ACCEL           = 720.0f;  // deg/sec^2
const float        DEFAULT_DECEL           = 720.0f;  // deg/sec^2
const unsigned int DEFAULT_REPORT_FREQ1    = 10;      // Hz (angle)
const unsigned int DEFAULT_REPORT_FREQ2    = 1;       // Hz (other)
const bool         DEFAULT_ENABLE_ON_BOOT  = 1;
const bool         DEFAULT_LED3V3_DISABLE  = 0;
const bool         DEFAULT_ZERO_ENC_BOOT   = 0;
const bool         DEFAULT_LED1_STATE      = 0;
const bool         DEFAULT_LED2_STATE      = 0;
const uint8_t      DEFAULT_AUX_PAYLOAD[8]  = {0, 0, 0, 0, 0, 0, 0, 0}; // both pins disabled

// ---------------- Settings from EEPROM ---------------
unsigned int NODE_ID = 1;  // Change per node (0-31). NOT reset by MsgType 23 -- see resetToDefaults()

unsigned int microsteps = DEFAULT_MICROSTEPS;
unsigned int current = DEFAULT_CURRENT;
unsigned int holdCurrent = DEFAULT_HOLD_CURRENT;
unsigned int stallThresh = DEFAULT_STALL_THRESH;
unsigned int stepsPerRev = DEFAULT_STEPS_PER_REV;
volatile unsigned int controlType = DEFAULT_CONTROL_TYPE;
unsigned int standstillMode = DEFAULT_STANDSTILL_MODE;
bool mapDirection = DEFAULT_MAP_DIRECTION;
float posSpeed = DEFAULT_POS_SPEED; // Max Speed (deg/sec) - human-friendly
float accel = DEFAULT_ACCEL;    // Acceleration (deg/sec^2) - human-friendly
float decel = DEFAULT_DECEL;    // Deceleration (deg/sec^2) - human-friendly
unsigned int reportFreq1 = DEFAULT_REPORT_FREQ1;
unsigned int reportFreq2 = DEFAULT_REPORT_FREQ2;
bool enableOnBoot = DEFAULT_ENABLE_ON_BOOT;
bool LED3V3Disable = DEFAULT_LED3V3_DISABLE;
bool zeroEncAtBoot = DEFAULT_ZERO_ENC_BOOT;
uint8_t storedAUXPayload[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //AUX config (full CAN payload)

// ---------------- WiFi / WebSocket (optional, provisioned over USB serial only -- see WIFI_SET) ---------------
// All WiFi.*/wsServer.*/MDNS.* calls happen exclusively on the dedicated wifiTaskFn() task
// (pinned to Core 0, created in setup()) -- never from loop()/executeCommand() (Core 1) --
// so this I/O can't contend with step generation. The two tasks only talk via wsInQueue /
// wsOutQueue (line-oriented, see WsLine) and wifiConfigMux (guards the SSID/password/
// hostname buffers below, which are written from loop() but read from the WiFi task).
#define WS_PORT 81
Preferences wifiPreferences;              // separate namespace ("wifinet") from the main settings preferences
WebSocketsServer wsServer(WS_PORT);       // owned exclusively by wifiTaskFn() -- see above
volatile bool wifiEnabled = false;        // "enable WiFi / WebSocket control" -- off by default, opt-in per node. Simple flag, single-writer (loop()).
portMUX_TYPE wifiConfigMux = portMUX_INITIALIZER_UNLOCKED; // guards wifiSsid/wifiPassword/wifiHostname (multi-byte, touched by both tasks)
char wifiSsid[33] = "";
char wifiPassword[64] = "";
char wifiHostname[24] = "";               // "canbus-stepper-<NODE_ID>" (mDNS adds the .local)
volatile bool wifiHostnameDirty = false;  // set by executeCommand() case 25 (Set Node ID); wifiTaskFn() notices and restarts mDNS under the new name
volatile bool wifiConnectRequested = false;    // set by handleWifiSetCommand() (loop()); wifiTaskFn() notices and calls startWifiConnect()
volatile bool wifiDisconnectRequested = false; // set by handleWifiSetCommand() (loop()); wifiTaskFn() notices and tears WiFi down
bool wifiConnecting = false;               // WiFi-task-exclusive from here on (only ever touched inside wifiTaskFn())
bool wifiWasConnected = false;             // WiFi-task-exclusive
unsigned long wifiConnectStartMs = 0;      // WiFi-task-exclusive
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000; // retry if a connect attempt doesn't resolve in this long
uint8_t wsClientCount = 0;                 // WiFi-task-exclusive (only onWsEvent()/wifiTaskFn() touch it)
bool wsServerStarted = false;              // WiFi-task-exclusive -- true once wsServer.begin() has actually been called (see startWifiConnect())
TaskHandle_t wifiTaskHandle = nullptr;

// Line-oriented messages passed between loop() and wifiTaskFn(). 48 bytes comfortably
// covers both a "<CAN_ID_HEX> <RTR> <PAYLOAD_HEX>" command line and the short telemetry
// lines built by forwardTelemetryToSerial() (char buf[32]).
struct WsLine {
    char text[48];
};
QueueHandle_t wsInQueue = nullptr;   // wifiTaskFn() -> loop(): command lines received over the WebSocket
QueueHandle_t wsOutQueue = nullptr;  // loop() -> wifiTaskFn(): outgoing telemetry lines to broadcast over the WebSocket

// ---------------- Variables for storing current values for RTR reporting & others ---------------
int64_t posSetpoint = 0;
float velocitySetpoint = 0;
bool driverEnabled = 0;
bool ledState = DEFAULT_LED1_STATE;
bool ledState2 = DEFAULT_LED2_STATE;

bool VbusState = 0;
uint16_t AUX1Function = 0;
uint16_t AUX2Function = 0;

// ---------------- Button state tracking (for change-on-event telemetry) ---------------
bool lastSW1State = HIGH; // HIGH = not pressed (buttons read LOW when pressed)
bool lastSW2State = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// ---------------- StallGuard / Sensorless Homing ---------------
// Flag set by DIAG pin interrupt, cleared and sent in main loop
volatile bool stallguardTriggered = false;

// StallGuard action mode (MsgType 7):
//   0 = Do nothing (stall only sends MsgType 37 telemetry)
//   1 = Instant stop
//   2 = Instant stop and set current position as zero
//   3 = Instant stop ONCE (auto-resets to mode 0 after triggered)
//   4 = Instant stop and set as zero ONCE (auto-resets to mode 0 after triggered)
uint8_t sensorlessHomeMode = 0;

// ---------------- Fault Code (MsgType 39) ---------------
// Current fault bitfield -- see FAULT_* bit definitions and checkFaults().
uint16_t faultCode = 0;

// True while a fault-triggered shutdown holds the driver disabled regardless
// of driverEnabled; cleared automatically once checkFaults() clears the fault.
bool faultShutdownActive = false;

// ---------------- Timers and critical section ---------------
// Hardware timers (NOT esp_timer -- see the long comment in setup()'s "TIMERS SETUP"
// section for why: esp_timer's background dispatch task could land on either core and
// get delayed by WiFi's own high-priority internal task, causing the WiFi-related motion
// choppiness. A hardware timer fires a genuine CPU interrupt tied to whichever core calls
// timerAttachInterrupt() -- Core 1 here -- which preempts task-level code (including
// WiFi's) almost unconditionally, regardless of FreeRTOS scheduling.
static hw_timer_t *motionHwTimer = nullptr;
static hw_timer_t *stepHwTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Forward declarations ---------------
void IRAM_ATTR onStepTimer();   // One-Shot pulse generator (hardware timer ISR, no arg)
void IRAM_ATTR onMotionTimer(); // 1000Hz Ramp Calculator (hardware timer ISR, no arg)
void IRAM_ATTR armStepTimerOnce(uint32_t delay_us); // arms stepHwTimer for one fire, delay_us from now
void IRAM_ATTR onDiagInterrupt();        // DIAG pin ISR for stall detection
void readEncoder();
int64_t encoderCountsToSteps(int64_t counts);
void canSendTelemetry(uint8_t msgType, const void *data, uint8_t size);
void forwardTelemetryToSerial(const CanFrame &frame);
void executeCommand(uint8_t targetNode, uint8_t msgType, const uint8_t *payload, uint8_t len, uint8_t rtr);
void handleCANFrame(const CanFrame &frame);
void handleSerialInput();
void handleAuxSerialInput();
void sendCANFrame(unsigned long canId, uint8_t *payload, uint8_t len, bool rtr);
void readSettings();
void writeSettings();
void resetToDefaults();
void haltMotionKeepTarget();
void configureDriver();
void powerUpDriver();
void powerDownDriver();
void recalcMotionParams();
void rescaleStepCounts(unsigned int oldMicrosteps, unsigned int oldStepsPerRev);
float readInputVoltage();
float readNTCTemperature();
void checkButtons();
void checkFaults();
void handleSerialInputFrom(Stream &port);
void handleCommandLine(const String &line);
void loadWifiCredentials();
void saveWifiCredentials();
void updateWifiHostname();
void startWifiConnect();
void handleWiFiStateMachine();
void handleWifiSetCommand(const String &line);
void handleWifiStatusQuery(Print &out);
String buildWifiStatusLine();
void broadcastLineToWS(const char *buf);
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void wifiTaskFn(void *pvParameters);

// -------------------- Setup --------------------
void setup() {
    delay(400);
    Serial.begin(115200);
    delay(200);
    Serial.println("Node starting...");

    // Pins
    pinMode(STB, OUTPUT);
    digitalWrite(STB, LOW); // Enable CAN normal mode
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(SW1, INPUT);
    pinMode(SW2, INPUT);
    pinMode(VBUS, INPUT);
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    digitalWrite(STEP, LOW);
    digitalWrite(DIR, LOW);
    digitalWrite(DISABLE_3V3, LOW); //SHOULD NEVER BE HIGH

    // Setup serial comms with TMC2209
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(TMC_EN, OUTPUT);
    pinMode(DIAG, INPUT);
    digitalWrite(TMC_EN, HIGH); // Hardware disabled to start with

    digitalWrite(MS1, LOW); // used to set serial address in UART mode
    digitalWrite(MS2, LOW);

    // Attach DIAG interrupt for StallGuard stall detection
    // DIAG pin goes HIGH when a stall is detected (requires SpreadCycle / TCOOLTHRS to be set)
    attachInterrupt(digitalPinToInterrupt(DIAG), onDiagInterrupt, RISING);

    // start I2C
    Wire.begin(SDA, SCL);
    // I2C clock for the MT6701 encoder (datasheet supports up to ~1MHz).
    Wire.setClock(400000);

    // read from EEPROM
    readSettings();

    // Restore LED output states from EEPROM
    digitalWrite(LED1, ledState);
    digitalWrite(LED2, ledState2);

    recalcMotionParams(); // Calculate initial acceleration in steps/s^2 (Q-fixed)

    //apply AUX conenctor config based on EEPROM values:
    executeCommand(NODE_ID, 26, storedAUXPayload, 8, 0); //this is done by emualting a received command based on the CAN payload saved in EEPROM

    //apply encoder offset if enabled
    if (zeroEncAtBoot){
      readEncoder();
      encoder_offset = total_encoder_counts;
      
    }

    // if button held down at boot set new NODE_ID
    unsigned long lastBlink = 0;
    int count = 0;
    if (!digitalRead(SW1)){
      while(!digitalRead(SW1)) {
        // Increase count every 700 ms while held
        if(millis() - lastBlink >= 700) {
            lastBlink = millis();
            count++;
            if(count > MAX_CAN_ID) count = 1;

            // Blink LED
            digitalWrite(LED1, HIGH);
            delay(120);
            digitalWrite(LED1, LOW);

            Serial.printf("ID candidate: %d\n", count);
        }
      }
      if(count > 0) {
        NODE_ID = count;
        Serial.printf("CAN_ID set to %d\n", NODE_ID);
        writeSettings(); //save to EEPROM
      }
      delay(2000);
    }

    // CAN setup
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if(ESP32Can.begin(ESP32Can.convertSpeed(1000))) {  //1Mbps
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }

    //ADC setup (must happen before the boot-time voltage check just below,
    // so readInputVoltage() is reading with the correct calibrated attenuation)
    analogSetPinAttenuation(VBUS, ADC_11db);
    analogSetPinAttenuation(NTC, ADC_11db);

    // Stepper setup (using values from EEPROM). driverEnabled reflects the
    // desired state; actual UART config happens once VBUS is present.
    driverEnabled = enableOnBoot;

    if (readInputVoltage() >= DRIVER_VBUS_THRESHOLD) {
        powerUpDriver(); // driver is already powered at boot - configure from scratch, then hardware-enable
    } else {
        digitalWrite(TMC_EN, HIGH); // stay hardware-disabled until VBUS is confirmed present (already HIGH, kept explicit here)
        Serial.println("TMC2209 not powered at boot (VBUS below threshold) - will configure once external supply is detected");
    }

    // --- TIMERS SETUP ---

    // Hardware timers, not esp_timer. esp_timer's periodic/one-shot callbacks run on a
    // shared background FreeRTOS task (ESP_TIMER_TASK) whose CPU core isn't fixed -- it
    // can float onto either core, including whichever one ESP-IDF's own internal WiFi
    // driver task needs at that moment (that task runs at very high priority so it never
    // misses a radio event, e.g. an AP beacon roughly 10x/second -- independent of how
    // much WiFi traffic *we* generate). When the floating esp_timer task lands on the
    // same core as that WiFi activity, it gets delayed -- that's what was causing the
    // step pulses to stutter whenever WiFi was enabled, no matter which core our own
    // WiFi task was pinned to.
    //
    // A genuine hardware timer interrupt doesn't have this problem: timerAttachInterrupt()
    // ties the interrupt to whichever core calls it (Core 1 here, since setup() runs on
    // Core 1), and a real interrupt preempts task-level code on that core almost
    // unconditionally -- including WiFi's own task -- rather than waiting its turn in a
    // FreeRTOS ready queue. Both timers run at 1MHz (1 tick = 1us), so every existing
    // microsecond-based delay value below (isr_stepDelay_us, STEP_PULSE_HIGH_US, etc.)
    // is used completely unchanged -- only how onMotionTimer()/onStepTimer() get
    // scheduled has changed, not the ramp/step math inside them.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // arduino-esp32 3.x (IDF 5.x) timer API: timerBegin(frequencyHz) picks a free timer.
    // 1. Motion Profile Timer (Periodic 1000Hz) -- calculates the S-curve/Trapezoid ramp
    motionHwTimer = timerBegin(1000000); // 1MHz
    timerAttachInterrupt(motionHwTimer, &onMotionTimer);
    timerAlarm(motionHwTimer, 1000, true, 0); // periodic, 1000us = 1kHz

    // 2. Step Pulse Timer (One-Shot) -- fires once per step, toggles pin, re-arms itself
    stepHwTimer = timerBegin(1000000); // 1MHz
    timerAttachInterrupt(stepHwTimer, &onStepTimer);
    // Left un-armed here -- armed on demand by armStepTimerOnce(), first called from
    // onMotionTimer()'s kickstart logic below, exactly like esp_timer_start_once() was.
#else
    // arduino-esp32 2.x (IDF 4.x) timer API: timerBegin(timerNum, divider, countUp).
    // Divider 80 against the 80MHz APB clock also gives 1MHz (1 tick = 1us).
    // 1. Motion Profile Timer (Periodic 1000Hz) -- calculates the S-curve/Trapezoid ramp
    motionHwTimer = timerBegin(0, 80, true); // hardware timer group/number 0
    timerAttachInterrupt(motionHwTimer, &onMotionTimer, true);
    timerAlarmWrite(motionHwTimer, 1000, true); // periodic, 1000us = 1kHz
    timerAlarmEnable(motionHwTimer);

    // 2. Step Pulse Timer (One-Shot) -- fires once per step, toggles pin, re-arms itself
    stepHwTimer = timerBegin(1, 80, true); // hardware timer group/number 1
    timerAttachInterrupt(stepHwTimer, &onStepTimer, true);
    // Left un-armed here -- armed on demand by armStepTimerOnce(), first called from
    // onMotionTimer()'s kickstart logic below, exactly like esp_timer_start_once() was.
#endif

    // Show current NODE_ID with fast blink
    for(int i = 0; i < NODE_ID; i++) {
        digitalWrite(LED1, HIGH);
        delay(150);
        digitalWrite(LED1, LOW);
        delay(150);
    }

    //disable 3V3 LED if eeprom value says so
    if (LED3V3Disable) {
      digitalWrite(DISABLE_3V3, LOW);
      pinMode(DISABLE_3V3, OUTPUT); //low signal to disable LED
    } else {
      pinMode(DISABLE_3V3, INPUT); //high impedance to enable LED
    }

    // --- WiFi / WebSocket (optional) ---
    // NOTE: wsServer.begin() is deliberately NOT called here -- it opens a TCP listen
    // socket, and doing that before WiFi.mode() has ever run (i.e. before the netif/lwIP
    // stack exists) crashes with "assert failed: xQueueSemaphoreTake" on an uninitialized
    // queue. It's started lazily, once, inside startWifiConnect() instead -- right after
    // the first WiFi.mode(WIFI_STA) call -- so it never runs on a node that has the
    // feature disabled.
    WiFi.persistent(false); // credentials are kept in our own Preferences namespace instead
    loadWifiCredentials();  // safe to call directly here -- wifiTaskFn() doesn't exist yet
    updateWifiHostname();   // ditto
    wsServer.onEvent(onWsEvent); // just registers a callback -- safe to call anytime

    // All WiFi.*/wsServer.*/MDNS.* calls move onto this dedicated task from here on (see
    // the comment above the WiFi globals), pinned to Core 0 so WiFi I/O can't contend with
    // the Motion Task / Step ISR (Core 1) and cause the choppiness WiFi used to introduce.
    // wifiTaskFn() itself checks wifiEnabled/wifiSsid (already loaded above) on entry and
    // kicks off the first connect if the node is provisioned -- nothing further needed here.
    wsInQueue = xQueueCreate(8, sizeof(WsLine));
    wsOutQueue = xQueueCreate(16, sizeof(WsLine));
    xTaskCreatePinnedToCore(wifiTaskFn, "wifi_service", 8192, NULL, 1, &wifiTaskHandle, 0);
}

// -------------------- Main Loop --------------------
void loop() {

    // Check StallGuard DIAG interrupt flag (set by onDiagInterrupt ISR)
    if (stallguardTriggered) {
        stallguardTriggered = false;
        Serial.println("StallGuard triggered! (DIAG pin)");

        // --- Sensorless homing action ---
        if (sensorlessHomeMode >= 1) {
            // Instant stop — no deceleration ramp
            velocitySetpoint = 0;  // clear stored setpoint so re-sending same velocity works
            portENTER_CRITICAL(&timerMux);
            motion.targetSpeed_q  = 0;
            motion.currentSpeed_q = 0;
            isr_stepDelay_us      = 0;
            motion.isRunning      = false;
            // Lock target to current position so position mode doesn't restart motion
            motion.targetPos      = isr_currentPos;
            portEXIT_CRITICAL(&timerMux);
            posSetpoint = isr_currentPos;
            Serial.println("Sensorless home: motor stopped instantly");

            if (sensorlessHomeMode == 2 || sensorlessHomeMode == 4) {
                // Zero both the step counter and encoder offset so this becomes position 0
                portENTER_CRITICAL(&timerMux);
                isr_currentPos   = 0;
                motion.targetPos = 0;
                portEXIT_CRITICAL(&timerMux);
                posSetpoint = 0;

                // Zero encoder by reading its current raw value and storing it as offset
                readEncoder();
                // total_encoder_counts is relative to previous offset; add it back in
                encoder_offset += (int)total_encoder_counts;
                total_encoder_counts = 0;
                angle = 0.0;
                Serial.println("Sensorless home: position zeroed (step + encoder)");
            }

            if (sensorlessHomeMode == 3 || sensorlessHomeMode == 4) {
                // One-shot mode — disarm after triggering
                sensorlessHomeMode = 0;
                Serial.println("Sensorless home: mode reset to 0 (one-shot complete)");
            }
        }

        // Always send MsgType 37 stall telemetry
        uint8_t stallPayload = 1;
        canSendTelemetry(37, &stallPayload, sizeof(stallPayload));  // msgType 37
    }

    // -------------------- Closed-loop sync --------------------
    if (posControl && controlType == 1) { // closed-loop position mode
        // Read encoder
        readEncoder();
        // Convert encoder counts to microsteps (rounded, not truncated -- see
        // encoderCountsToSteps())
        int64_t encoderSteps = encoderCountsToSteps(total_encoder_counts);

        // Update ISR current position from encoder
        // CRITICAL: We only sync if there is a discrepancy to avoid jittering the ramp math
        portENTER_CRITICAL(&timerMux);
        int64_t diff = isr_currentPos - encoderSteps;
        if (llabs(diff) > CLOSED_LOOP_SYNC_DEADBAND_STEPS) {
             isr_currentPos = encoderSteps;
        }
        portEXIT_CRITICAL(&timerMux);
    }

    // Closed-loop mode already read the encoder above; open-loop mode reads it
    // here instead, independent of reportFreq1.
    if (!(posControl && controlType == 1)) {
        readEncoder();
    }
    angle = ((double)total_encoder_counts) * 360.0 / 16384.0; // get degrees from encoder count

    // Send angle at chosen frequency
    if ((reportFreq1 != 0)){
        if ((millis() - lastFreq1) > 1000/reportFreq1){
        canSendTelemetry(33, &angle, sizeof(angle));  // msgType 33
        lastFreq1 = millis();

        //send Current Velocity:
        float currentVelocity = ((float)motion.currentSpeed_q / (float)ONE_Q) * (360.0f / (stepsPerRev * microsteps)); //convert from Q-fixed int to deg/s float
        canSendTelemetry(42, &currentVelocity, sizeof(currentVelocity));  // msgType 42

        }
    }

    // Secondary periodic CAN outputs
    if ((reportFreq2 != 0)){
        if ((millis() - lastFreq2) > 1000/reportFreq2){
        //Send voltage:
        float voltage = readInputVoltage();
        canSendTelemetry(34, &voltage, sizeof(voltage));  // msgType 34

        //send NTC temp:
        float NTCtemp = readNTCTemperature();
        canSendTelemetry(38, &NTCtemp, sizeof(NTCtemp));  // msgType 38

        //send Fault Code (checkFaults() also sends this immediately on change; this is just the periodic re-send):
        canSendTelemetry(39, &faultCode, sizeof(faultCode));  // msgType 39

        //send ESP temp:
        float ESPtemp = temperatureRead();
        canSendTelemetry(41, &ESPtemp, sizeof(ESPtemp));  // msgType 41

        //send Firmware version:
        canSendTelemetry(40, &firmwareVersion, sizeof(firmwareVersion));  // msgType 40

        // Send StallGuard value (SG_RESULT from TMC2209 — 10-bit, 0 = stall):
        uint32_t sgValue = stepper_driver.getStallGuardResult();
        canSendTelemetry(36, &sgValue, sizeof(sgValue));  // msgType 36

        // //send Current Velocity:
        // float currentVelocity = ((float)motion.currentSpeed_q / (float)ONE_Q) * (360.0f / (stepsPerRev * microsteps)); //convert from Q-fixed int to deg/s float
        // canSendTelemetry(42, &currentVelocity, sizeof(currentVelocity));  // msgType 42
        
        lastFreq2 = millis();
        }
    }

    // Handle CAN input
    if (ESP32Can.readFrame(rxFrame, 0)) { // timeout set to 0 so does not block
        handleCANFrame(rxFrame);
    }

    // Handle Serial input (USB)
    if (Serial.available()) {
        handleSerialInput();
    }

    // Handle AUX serial input if that mode is active
    if (AUX1Function == 1 && Serial1.available()) {
        handleAuxSerialInput();
    }

    // WiFi / WebSocket: wsServer.loop()/handleWiFiStateMachine()/MDNS now run exclusively
    // on the dedicated wifiTaskFn() task (Core 0, see setup()). Here on Core 1 we only
    // drain command lines the WiFi task received over the WebSocket and run them through
    // the same handleCommandLine() path USB serial uses -- see wsInQueue in the WiFi globals.
    if (wsInQueue != nullptr) {
        WsLine incomingWsLine;
        while (xQueueReceive(wsInQueue, &incomingWsLine, 0) == pdTRUE) {
            handleCommandLine(String(incomingWsLine.text));
        }
    }

    // Check for button state changes and send telemetry on change
    checkButtons();

    // Checks fault conditions and sends MsgType 39 on any change, independent
    // of reportFreq2.
    checkFaults();

    // Monitors VBUS and brings the TMC2209 up/down with it -- see
    // configureDriver()/powerUpDriver()/powerDownDriver().
    float inputVoltage = readInputVoltage();
    bool driverPowered = (inputVoltage >= DRIVER_VBUS_THRESHOLD);

    if (!driverPowered) {
      // Enforced every iteration: TMC_EN must stay high whenever VBUS is below threshold.
      digitalWrite(TMC_EN, HIGH);
      if (VbusState == 1) {
        powerDownDriver(); // VBUS has gone below threshold - handle the transition (halt motion, etc.)
      }
    } else if (VbusState == 0) {
      powerUpDriver(); // VBUS has come back - reconfigure from scratch, then hardware-enable
    }

    // Whenever the driver isn't powered AND enabled, pins reported/internal
    // velocity to zero every iteration -- covers every disable path uniformly.
    if (!(driverPowered && driverEnabled) || faultShutdownActive) {
      motionHold = true;
      haltMotionKeepTarget();
    }

    // Small yield to keep system responsive
    delay(1);
}

// -------------------- DIAG Pin ISR (StallGuard) --------------------
// Fires on rising edge of DIAG pin — indicates a stall has been detected.
// Only sets a flag; actual CAN transmission happens in the main loop.
void IRAM_ATTR onDiagInterrupt() {
    stallguardTriggered = true;
}

// Arms stepHwTimer for a single one-shot fire, delay_us ticks (= microseconds, timer runs
// at 1MHz) from now. Replaces esp_timer_start_once(stepTimer, delay_us) -- called from
// onMotionTimer()'s kickstart and from onStepTimer()'s own self-rearm, both already
// running in hardware ISR context, so this must stay IRAM-resident and avoid anything
// blocking. timerWrite() resets the counter so the alarm is always delay_us away from
// *now*, matching esp_timer_start_once()'s semantics exactly.
void IRAM_ATTR armStepTimerOnce(uint32_t delay_us) {
    timerWrite(stepHwTimer, 0);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    timerAlarm(stepHwTimer, delay_us, false, 0); // one-shot (no auto-reload)
#else
    timerAlarmWrite(stepHwTimer, delay_us, false); // one-shot (no auto-reload)
    timerAlarmEnable(stepHwTimer); // idempotent -- safe to call on every rearm
#endif
}

// -------------------- Motion Control Loop (1000Hz) --------------------
// Integer (Q-fixed) implementation
void IRAM_ATTR onMotionTimer() {
    // Use local copies outside critical section where possible
    int32_t v_q;          // Q-fixed current speed
    int32_t accel_q;
    int32_t decel_q;
    int32_t targetSpeed_q;
    int32_t local_posSpeed_steps_q; // Q-fixed max steps/sec
    int64_t targetPos_local;
    bool local_posControl;
    bool local_mapDirection;
    int32_t local_deadband;
    bool local_motionHold;

    // Copy minimal shared state
    portENTER_CRITICAL_ISR(&timerMux);
    v_q = motion.currentSpeed_q;
    accel_q = motion.accelSteps_q;
    decel_q = motion.decelSteps_q;
    targetSpeed_q = motion.targetSpeed_q;
    targetPos_local = motion.targetPos;
    local_posControl = posControl;
    local_mapDirection = mapDirection;
    local_deadband = isr_deadband;
    local_motionHold = motionHold;
    // Max speed in Q-fixed steps/sec -- pre-converted from posSpeed (float, deg/sec) by
    // recalcMotionParams() (plain task context) rather than here. This now runs as a true
    // hardware timer ISR (see "TIMERS SETUP" in setup()), and floating-point math inside a
    // real ISR crashes with a Coprocessor Exception on Xtensa/ESP32 -- see posSpeedStepsQ.
    local_posSpeed_steps_q = posSpeedStepsQ;
    portEXIT_CRITICAL_ISR(&timerMux);

    int32_t targetV_q = 0;

    // --- 1. Determine Target Velocity (Q-fixed) ---
    if (local_motionHold) {
        // Forced standstill (see motionHold) -- overrides both modes below.
        targetV_q = 0;
    } else if (local_posControl) {
        // position mode
        int64_t error = targetPos_local - isr_currentPos;
        int64_t absErr64 = llabs(error);

        if (absErr64 <= local_deadband) {
            targetV_q = 0;
        } else {
            // Stopping distance = v^2 / (2*a), or 0 if decel disabled. Uses the max
            // configured speed (not v_q) so accel=0 doesn't cause overshoot.
            int32_t v_for_stopdist = (accel_q == 0) ? local_posSpeed_steps_q : abs(v_q);
            int64_t v2 = (int64_t)v_for_stopdist * (int64_t)v_for_stopdist; // Q32
            int64_t denom_q = ((int64_t)decel_q << 1); // Q-fixed
            int64_t stopDist = 0;
            if (decel_q != 0 && denom_q != 0) {
                stopDist = v2 / (denom_q * ONE_Q);
            } else {
                stopDist = 0;
            }

            if (error > 0) {
                if (absErr64 > stopDist) targetV_q = local_posSpeed_steps_q;
                else targetV_q = 0;
            } else {
                if (absErr64 > stopDist) targetV_q = -local_posSpeed_steps_q;
                else targetV_q = 0;
            }
        }
    } else {
        // Velocity mode target, clamped to MAX_STEP_RATE_Q like position mode.
        targetV_q = targetSpeed_q;
        if (targetV_q > MAX_STEP_RATE_Q) targetV_q = MAX_STEP_RATE_Q;
        else if (targetV_q < -MAX_STEP_RATE_Q) targetV_q = -MAX_STEP_RATE_Q;
    }

    // --- 2. Apply Acceleration/Deceleration (Q-fixed arithmetic) ---
    // If accel or decel is 0, velocity snaps instantly to target (no ramping).
    // dt_q = ONE_Q / 1000 for 1 kHz loop
    const int32_t dt_q = ONE_Q / 1000; // 65
    bool wasStopped = (v_q == 0);
    if (v_q < targetV_q) {
        // speeding up; choose accel based on sign of v (if negative use decel to cross zero)
        int32_t chosen_a_q = (v_q >= 0) ? accel_q : decel_q;
        if (chosen_a_q == 0) {
            v_q = targetV_q; // instant
        } else {
            int32_t dv = (int32_t)(((int64_t)chosen_a_q * (int64_t)dt_q) >> Q);
            v_q += dv;
            if (v_q > targetV_q) v_q = targetV_q;
        }
    } else if (v_q > targetV_q) {
        int32_t chosen_a_q = (v_q > 0) ? decel_q : accel_q;
        if (chosen_a_q == 0) {
            v_q = targetV_q; // instant
        } else {
            int32_t dv = (int32_t)(((int64_t)chosen_a_q * (int64_t)dt_q) >> Q);
            v_q -= dv;
            if (v_q < targetV_q) v_q = targetV_q;
        }
    }

    // Delay-from-speed (below) is 1e6/v -- steep near v=0, so the ramp's
    // first tiny increment out of a stop would land a disproportionately
    // long first step. Snap straight to MIN_START_SPEED_Q instead, capped
    // at the actual target so a deliberately slower target isn't overshot.
    if (wasStopped && v_q != 0) {
        int32_t targetAbs_q = (targetV_q >= 0) ? targetV_q : -targetV_q;
        int32_t floor_q = (MIN_START_SPEED_Q < targetAbs_q) ? MIN_START_SPEED_Q : targetAbs_q;
        if (v_q > 0 && v_q < floor_q) v_q = floor_q;
        else if (v_q < 0 && v_q > -floor_q) v_q = -floor_q;
    }

    // Write back current speed
    portENTER_CRITICAL_ISR(&timerMux);
    motion.currentSpeed_q = v_q;
    portEXIT_CRITICAL_ISR(&timerMux);

    // --- 3. Convert Velocity to Delay (integer math) ---
    uint32_t computed_delay = 0;
    int32_t abs_v_q = v_q >= 0 ? v_q : -v_q;
    if (abs_v_q < (ONE_Q / 10)) { // equivalent to <0.1 steps/sec
        computed_delay = 0;
    } else {
        // avoid overflow: (1e6 * ONE_Q) fits in 64-bit.
        uint64_t numer = (uint64_t)1000000ULL * (uint64_t)ONE_Q;
        computed_delay = (uint32_t)(numer / (uint64_t)abs_v_q);
    }

    // --- 4. Update Direction Bit ---
    bool logicalDir = (v_q > 0);
    if (mapDirection) logicalDir = !logicalDir;

    // Commit step delay and direction inside critical section
    portENTER_CRITICAL_ISR(&timerMux);
    isr_stepDelay_us = computed_delay;
    isr_dirState = logicalDir ? HIGH : LOW;
    portEXIT_CRITICAL_ISR(&timerMux);

    // --- 5. Kickstart Stepper if needed ---
    if (isr_stepDelay_us > 0 && !motion.isRunning) {
        // Serial.println("Kick Starting Step timer");
        motion.isRunning = true;
        // Direct register write instead of digitalWrite(), matching onStepTimer().
        if (isr_dirState) REG_WRITE(GPIO_OUT_W1TS_REG, DIR_MASK); // DIR = HIGH
        else              REG_WRITE(GPIO_OUT_W1TC_REG, DIR_MASK); // DIR = LOW
        // Schedule the first onStepTimer() call after half a period; onStepTimer()
        // re-arms itself for subsequent steps.
        uint32_t first_delay = isr_stepDelay_us / 2;
        if (first_delay < 20) first_delay = 20;
        armStepTimerOnce(first_delay);
    }
}

// -------------------- Step ISR (One-Shot) --------------------
// Fires when a step is due: emits one STEP pulse (rising edge, brief hold,
// falling edge) and re-arms for the remaining period.
void IRAM_ATTR onStepTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    if (isr_stepDelay_us == 0) {
        motion.isRunning = false;
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    // DIR pin
    if (isr_dirState) {
        REG_WRITE(GPIO_OUT_W1TS_REG, DIR_MASK);   // DIR = HIGH
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, DIR_MASK);   // DIR = LOW
    }

    // Rising edge: STEP = HIGH
    REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);

    // logical position update
    bool logicalUp = (isr_dirState == HIGH);
    if (mapDirection) logicalUp = !logicalUp;

    if (logicalUp) isr_currentPos++;
    else           isr_currentPos--;

    // Hold STEP high via a tight busy-wait.
    esp_rom_delay_us(STEP_PULSE_HIGH_US);

    // Falling edge: STEP = LOW
    REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);

    // Re-arm timer for the remainder of the step period
    uint32_t next_delay = (isr_stepDelay_us > STEP_PULSE_HIGH_US)
                               ? (isr_stepDelay_us - STEP_PULSE_HIGH_US)
                               : 1;
    if (next_delay < 20) next_delay = 20;

    armStepTimerOnce(next_delay);

    portEXIT_CRITICAL_ISR(&timerMux);
}

// -------------------- Helpers --------------------
void recalcMotionParams() {
    // Converts user-friendly Deg/s^2 into Steps/s^2 and stores Q-fixed
    float usteps = (float)microsteps;
    float revs = (float)stepsPerRev;
    float accel_steps = (accel / 360.0f) * revs * usteps;
    float decel_steps = (decel / 360.0f) * revs * usteps;

    // Position mode's max speed (posSpeed, deg/sec), also pre-converted to Q-fixed
    // steps/sec here -- see the comment on posSpeedStepsQ for why this moved out of
    // onMotionTimer() (now a hardware ISR, where float math isn't safe) and into here
    // instead (plain task context, called only when a dependency actually changes).
    float maxV_steps = (posSpeed / 360.0f) * revs * usteps;
    int32_t posSpeedQ = (int32_t)roundf(maxV_steps * (float)ONE_Q);
    if (posSpeedQ > MAX_STEP_RATE_Q) posSpeedQ = MAX_STEP_RATE_Q;
    else if (posSpeedQ < -MAX_STEP_RATE_Q) posSpeedQ = -MAX_STEP_RATE_Q;

    // store in Q-fixed
    portENTER_CRITICAL(&timerMux);
    motion.accelSteps_q = (int32_t)roundf(accel_steps * (float)ONE_Q);
    motion.decelSteps_q = (int32_t)roundf(decel_steps * (float)ONE_Q);
    posSpeedStepsQ = posSpeedQ;
    portEXIT_CRITICAL(&timerMux);

    // Does not touch motion.targetSpeed_q -- only used by Velocity mode, set directly
    // from the Set Velocity command (already integer math, see executeCommand() case 3).
}

// Rescales isr_currentPos/targetPos/posSetpoint and the ramp's speed state
// when microsteps/stepsPerRev changes, preserving the physical
// position/speed each one represents at the new resolution.
void rescaleStepCounts(unsigned int oldMicrosteps, unsigned int oldStepsPerRev) {
    if (oldMicrosteps == 0 || oldStepsPerRev == 0) return; // guard against div-by-zero (shouldn't happen)
    if (oldMicrosteps == microsteps && oldStepsPerRev == stepsPerRev) return; // nothing actually changed

    double ratio = ((double)stepsPerRev * (double)microsteps) /
                   ((double)oldStepsPerRev * (double)oldMicrosteps);

    portENTER_CRITICAL(&timerMux);
    isr_currentPos = (int64_t)llround((double)isr_currentPos * ratio);
    motion.targetPos = (int64_t)llround((double)motion.targetPos * ratio);
    motion.currentSpeed_q = (int32_t)llround((double)motion.currentSpeed_q * ratio);
    motion.targetSpeed_q = (int32_t)llround((double)motion.targetSpeed_q * ratio);
    if (isr_stepDelay_us != 0) {
        // Period is inversely proportional to steps/sec.
        isr_stepDelay_us = (uint32_t)llround((double)isr_stepDelay_us / ratio);
    }
    portEXIT_CRITICAL(&timerMux);

    posSetpoint = (int64_t)llround((double)posSetpoint * ratio);

    Serial.printf("Rescaled step/velocity counts for new resolution (x%.6f)\n", ratio);
}

float readInputVoltage() {
    const float SCALE = (47.0 + 4.7) / 4.7;   // ≈ 11.0
    int mv = analogReadMilliVolts(VBUS);      // calibrated ADC voltage (mV)
    return (mv / 1000.0) * SCALE;             // return actual input voltag
}

// -------------------- TMC2209 Power / Configuration --------------------
// Zeroes both currentSpeed_q and isr_stepDelay_us without touching
// targetPos/targetSpeed_q, so the motor restarts from standstill and ramps
// normally back up to any still-active setpoint.
void haltMotionKeepTarget() {
    portENTER_CRITICAL(&timerMux);
    motion.currentSpeed_q = 0; // internal: ramp/accel-decel state
    isr_stepDelay_us = 0;      // output: step pulse timing (0 = stopped)
    portEXIT_CRITICAL(&timerMux);
}

// Configures the TMC2209 from scratch over UART while TMC_EN stays high
// (disabled), then hardware-enables last so it never becomes live with
// stale settings. Must only run while VBUS is present.
void configureDriver() {
    // Holds at forced standstill for the whole function; released at the end
    // once the driver has settled.
    motionHold = true;
    haltMotionKeepTarget();

    stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
    stepper_driver.disable(); // force a known (off) chopper state before writing anything else
    stepper_driver.setRunCurrent(current);
    stepper_driver.setHoldCurrent(holdCurrent); // see MsgType 27
    stepper_driver.setMicrostepsPerStep(microsteps);
    stepper_driver.setStallGuardThreshold(stallThresh);
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.enableStealthChop(); // stealth chop needs to be enabled for stall detect
    stepper_driver.setCoolStepDurationThreshold(5000); // TCOOLTHRS - gates StallGuard, prevents false triggers at standstill

    if (standstillMode == 0){ stepper_driver.setStandstillMode(stepper_driver.NORMAL);}
    else if (standstillMode == 1){ stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);}
    else if (standstillMode == 2){ stepper_driver.setStandstillMode(stepper_driver.BRAKING);}
    else if (standstillMode == 3){ stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);}

    // Re-zero speed immediately before enabling, in case the ramp climbed
    // during the UART calls above.
    haltMotionKeepTarget();

    if (driverEnabled && !faultShutdownActive) {
        stepper_driver.enable();
        // Fully configured over UART, with the correct soft enable state
        // applied -- now safe to physically energize the driver.
        digitalWrite(TMC_EN, LOW);
        // Let current regulation settle at standstill before releasing motionHold,
        // so torque doesn't come up weak.
        delay(DRIVER_SETTLE_MS);
        motionHold = false; // now actually driving -- safe to resume ramping
    } else {
        stepper_driver.disable();
        digitalWrite(TMC_EN, LOW);
        // motionHold stays true if disabled or a shutdown fault is active, so
        // nothing ramps/reports movement while genuinely stopped.
    }
}

// Brings the TMC2209 up: waits for supplies to stabilize, then configures
// it from scratch over UART.
void powerUpDriver() {
    delay(100); // let the driver's internal supplies stabilize
    configureDriver();
    VbusState = 1;
    Serial.println("Driver configured from scratch and hardware enabled");
}

// Hardware-disables the TMC2209 immediately (no UART -- it has no power).
// Halts the motion ramp and latches motionHold so software doesn't keep
// "virtually" advancing while unpowered.
void powerDownDriver() {
    digitalWrite(TMC_EN, HIGH); // hardware disable
    motionHold = true;
    haltMotionKeepTarget();
    VbusState = 0;
    Serial.println("Driver hardware disabled");
}

float readNTCTemperature() {
    const float R_FIXED = 10000.0;   // 10k pull-up
    const float BETA = 3380.0;       // NTC beta value
    const float T0 = 298.15;         // 25°C in Kelvin
    const float R0 = 10000.0;        // 10k at 25°C

    int mv = analogReadMilliVolts(NTC);  // calibrated mV
    float v = mv / 1000.0;               // volts

    // Thermistor resistance from divider
    float r_ntc = (R_FIXED * v) / (3.3 - v);

    // Temperature in Kelvin (Steinhart approximation)
    float inv_T = (1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0);
    float T = 1.0 / inv_T;

    return T - 273.15;                   // return °C
}

void readEncoder() {
    static int prev_raw_counts = 0;
    static long revolutions = 0;
    int raw_counts = 0;

    Wire.beginTransmission(MT6701_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(MT6701_ADDRESS, 2);

    if (Wire.available() >= 2) {
        uint8_t angle_h = Wire.read();
        uint8_t angle_l = Wire.read();
        raw_counts = (angle_h << 6) | (angle_l >> 2);
    }

    if (prev_raw_counts > 12000 && raw_counts < 4000) revolutions++;
    else if (prev_raw_counts < 4000 && raw_counts > 12000) revolutions--;

    prev_raw_counts = raw_counts;
    total_encoder_counts = raw_counts + (16384L * revolutions) - encoder_offset; //account for "Zero encoder at boot" here.
}

// Converts an encoder count to microsteps at the current resolution,
// rounding to nearest instead of truncating.
int64_t encoderCountsToSteps(int64_t counts) {
    int64_t numerator = counts * (int64_t)stepsPerRev * (int64_t)microsteps;
    const int64_t denominator = 16384;
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    } else {
        return (numerator - denominator / 2) / denominator;
    }
}

void canSendTelemetry(uint8_t msgType, const void *data, uint8_t size) {
    if(size > 8) size = 8;  // CAN max payload

    CanFrame frame = {0};
    frame.identifier = (NODE_ID << 6) | msgType;
    frame.extd = 0;
    frame.data_length_code = size;

    memcpy(frame.data, data, size);

    ESP32Can.writeFrame(frame);
    forwardTelemetryToSerial(frame);
}

// -------------------- Execute Command (runs commands sent to this node) --------------------
void executeCommand(uint8_t targetNode, uint8_t msgType, const uint8_t *payload, uint8_t len, uint8_t rtr) {
    if(targetNode != NODE_ID && targetNode != 0) return; // don't execute unless it is for this node (or all nodes)

    if (rtr == 0){ //command 
      switch(msgType) {
          case 1: // Set Position (deg) (double)
              if(len >= 8) {
                  posControl = 1; // position control mode
                  double posAngle;
                  memcpy(&posAngle, payload, 8);
                  posSetpoint = (int64_t)round(posAngle * (stepsPerRev * (double)microsteps / 360.0));
                  Serial.printf("Executing Set Position: %.3f deg (%lld steps)\n", posAngle, (long long)posSetpoint);
  
                  portENTER_CRITICAL(&timerMux);
                  motion.targetPos = posSetpoint;
                  portEXIT_CRITICAL(&timerMux);
              }
              break;
  
          case 2: // Set Position (steps) (int64)
              if(len >= 8) {
                  posControl = 1; // position control mode
                  posSetpoint = 0;
                  memcpy(&posSetpoint, payload, 8);
                  Serial.printf("Executing Set Position: %lld steps\n", (long long)posSetpoint);
  
                  portENTER_CRITICAL(&timerMux);
                  motion.targetPos = posSetpoint;
                  portEXIT_CRITICAL(&timerMux);
              }
              break;
  
          case 3: // Set Velocity (deg/sec) (float)
              if(len >= 4) {
                  posControl = 0; // velocity control mode
                  memcpy(&velocitySetpoint, payload, 4);
                  Serial.printf("Executing Set Velocity: %.2f deg/sec\n", velocitySetpoint);
                  
                  // Convert to steps/sec for the motion loop then to Q-fixed
                  float velSteps = (velocitySetpoint / 360.0f) * stepsPerRev * microsteps;
                  int32_t velSteps_q = (int32_t)roundf(velSteps * (float)ONE_Q);
  
                  portENTER_CRITICAL(&timerMux);
                  motion.targetSpeed_q = velSteps_q;
                  portEXIT_CRITICAL(&timerMux);
              }
              break;
  
          case 4: // Set current (uint16)
              if(len >= 2) {
                  current = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Current set to: %u percent\n", current);
                  stepper_driver.setRunCurrent(current);
              }
              break;
  
          case 5: // Enable (bool)
              if(len >= 1) {
                  driverEnabled = payload[0] != 0;
                  Serial.printf("Executing Enable: %d\n", driverEnabled);
                  if(driverEnabled && faultShutdownActive) {
                      // Desired state recorded, but stays disabled while a PCB Over-Temp
                      // Shutdown fault is active.
                      Serial.println("Enable requested but PCB Over-Temp Shutdown is active; will resume once it clears");
                      stepper_driver.disable();
                      motionHold = true;
                      haltMotionKeepTarget();
                  } else if(driverEnabled) {
                      // Re-enabling always starts from standstill so the motor ramps cleanly to
                      // any already-active target.
                      motionHold = true;
                      haltMotionKeepTarget();
                      stepper_driver.enable();
                      // Let current regulation settle before resuming ramp.
                      delay(DRIVER_SETTLE_MS);
                      motionHold = false;
                  } else {
                      stepper_driver.disable();
                      // Disabling cancels any move and clears the target. motionHold also
                      // latches true so Position mode doesn't keep driving toward a stale target.
                      motionHold = true;
                      portENTER_CRITICAL(&timerMux);
                      motion.targetSpeed_q = 0;
                      motion.currentSpeed_q = 0;
                      isr_stepDelay_us = 0;
                      portEXIT_CRITICAL(&timerMux);
                  }
              }
              break;

          case 6: // Emergency Stop - any payload value (or none) triggers it. Re-enable with MsgType 5 afterwards.
              Serial.println("EMERGENCY STOP triggered");
              driverEnabled = false;
              stepper_driver.disable();
              motionHold = true; // see case 5's disable branch for why this is needed too
              portENTER_CRITICAL(&timerMux);
              motion.targetSpeed_q = 0;
              motion.currentSpeed_q = 0;
              isr_stepDelay_us = 0; // Immediate stop
              portEXIT_CRITICAL(&timerMux);
              break;
          
          case 7: // StallGuard action (uint8)
              // Sets behaviour when a stall is detected on the DIAG pin.
              //   0 = Do nothing (stall only sends MsgType 37 telemetry)
              //   1 = Instant stop
              //   2 = Instant stop and set current position as zero
              //   3 = Instant stop ONCE (resets to mode 0 after triggered)
              //   4 = Instant stop and set as zero ONCE (resets to mode 0 after triggered)
              if(len >= 1) {
                  sensorlessHomeMode = payload[0];
                  if (sensorlessHomeMode > 4) sensorlessHomeMode = 0; // sanitise
                  Serial.printf("StallGuard action set to: %d\n", sensorlessHomeMode);
              }
              break;

          case 8: // Zero Encoder at boot (bool)
              if(len >= 1) {
                  zeroEncAtBoot = payload[0] != 0;
                  Serial.printf("Zero Enc at boot set to: %d\n", zeroEncAtBoot);
              }
              break;
  
          case 9: // LED State's (0: LED1 State, 1: LED2 State)
              if(len >= 1) {
                  ledState = payload[0] != 0;
                  digitalWrite(LED1, ledState);
              }
              if(len >= 2) {
                  ledState2 = payload[1] != 0;
                  digitalWrite(LED2, ledState2);
              }
              Serial.printf("LED Command: LED1=%d LED2=%d\n", ledState, ledState2);
              break;
  
          case 10: // Set Steps per rev (uint16)
              if(len >= 2) {
                  unsigned int oldStepsPerRev = stepsPerRev;
                  stepsPerRev = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Steps per rev set to: %u\n", stepsPerRev);
                  // Preserve the physical position/target this change would
                  // otherwise silently shift -- see rescaleStepCounts().
                  rescaleStepCounts(microsteps, oldStepsPerRev);
                  recalcMotionParams();
              }
              break;

          case 11: // microsteps (uint16)
              if(len >= 2) {
                  unsigned int oldMicrosteps = microsteps;
                  microsteps = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Microsteps set to: %u\n", microsteps);
                  stepper_driver.setMicrostepsPerStep(microsteps);
                  // Preserve the physical position/target this change would
                  // otherwise silently shift -- see rescaleStepCounts().
                  rescaleStepCounts(oldMicrosteps, stepsPerRev);
                  recalcMotionParams();
              }
              break;

          case 12: // StallGuard Threshold (int16)
              // Negative values = more sensitive; positive = less sensitive
              // Stored in stallThresh (unsigned int) but cast through int16 to allow signed values
              if(len >= 2) {
                  int16_t thresh;
                  memcpy(&thresh, payload, 2);
                  stallThresh = (unsigned int)(int)thresh;
                  Serial.printf("StallGuard threshold set to: %d\n", (int16_t)stallThresh);
                  stepper_driver.setStallGuardThreshold(stallThresh);
              }
              break;
  
          case 13: // control type (uint16)
              if(len >= 2) {
                  controlType = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Control type set to: %u\n", controlType);
              }
              break;

          case 14: // Standstill Mode (uint16)
              if(len >= 2) {
                  standstillMode = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Standstill Mode set to: %u\n", standstillMode);
                  if (standstillMode == 0){ stepper_driver.setStandstillMode(stepper_driver.NORMAL);}
                  else if (standstillMode == 1){ stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);}
                  else if (standstillMode == 2){ stepper_driver.setStandstillMode(stepper_driver.BRAKING);}
                  else if (standstillMode == 3){ stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);}
              }
              break;
  
          case 15: // Toggle Map Direction (bool)
               if(len >= 1) {
                   mapDirection = payload[0] != 0;
                   Serial.printf("Map Direction set to: %d\n", mapDirection);
                   // Important: Reset current position to encoder position immediately
                   // to prevent a sudden jump when direction logic flips
                   readEncoder();
                   portENTER_CRITICAL(&timerMux);
                   int64_t encoderSteps = encoderCountsToSteps(total_encoder_counts);
                   isr_currentPos = encoderSteps;
                   motion.targetPos = encoderSteps; // Cancel any current move
                   portEXIT_CRITICAL(&timerMux);
               }
               break;
  
          case 16: // Position speed (deg/sec) (float) - Max Speed
              if(len >= 4) {
                  memcpy(&posSpeed, payload, 4);
                  Serial.printf("Position speed updated to: %.2f deg/sec\n", posSpeed);
                  // Update Q-fixed internal max speed
                  recalcMotionParams();
              }
              break;
              
          case 17: // Acceleration (deg/sec^2) (float)
               if(len >= 4) {
                  memcpy(&accel, payload, 4);
                  Serial.printf("Accel updated to: %.2f deg/s2\n", accel);
                  recalcMotionParams();
               }
               break;
               
          case 18: // Deceleration (deg/sec^2) (float)
               if(len >= 4) {
                  memcpy(&decel, payload, 4);
                  Serial.printf("Decel updated to: %.2f deg/s2\n", decel);
                  recalcMotionParams();
               }
               break;

          case 19: // Report Frequency
              if (len >= 4) {
                  uint16_t freq1 = payload[0] | (payload[1] << 8);
                  uint16_t freq2 = payload[2] | (payload[3] << 8);

                  // Optional sanity limits to avoid divide-by-zero or overload
                  if (freq1 >= 500) freq1 = 500;
                  if (freq2 >= 500) freq2 = 500;

                  reportFreq1 = freq1;
                  reportFreq2 = freq2;

                  Serial.printf(
                      "Report frequencies updated: Angle=%u Hz, Other=%u Hz\n",
                      reportFreq1,
                      reportFreq2
                  );
              }
              break;

          case 20: // Enabled/Disabled on Boot (bool)
              if(len >= 1) {
                  enableOnBoot = payload[0] != 0;
                  Serial.printf("Enable on Boot set to: %d\n", enableOnBoot);
              }
              break;

          case 21: // Enable (bool)
              if(len >= 1) {
                  LED3V3Disable = payload[0] != 0;
                  Serial.printf("LED3V3Disable set to: %d\n", LED3V3Disable);
                  if (LED3V3Disable) {
                    digitalWrite(DISABLE_3V3, LOW);
                    pinMode(DISABLE_3V3, OUTPUT); //low signal to disable LED
                  } else {
                    pinMode(DISABLE_3V3, INPUT); //high impedance to enable LED
                  }
              }
              break;

          case 22: // Incremental Move (deg) (float) -- relative to current target
              if(len >= 4) {
                  posControl = 1; // position control mode; works the same in open or closed loop
                  float deltaAngle;
                  memcpy(&deltaAngle, payload, 4);
                  int64_t deltaSteps = (int64_t)round((double)deltaAngle * (stepsPerRev * (double)microsteps / 360.0));
                  posSetpoint += deltaSteps;
                  Serial.printf("Executing Incremental Move: %.3f deg (%lld steps), new target %lld steps\n", deltaAngle, (long long)deltaSteps, (long long)posSetpoint);

                  portENTER_CRITICAL(&timerMux);
                  motion.targetPos = posSetpoint;
                  portEXIT_CRITICAL(&timerMux);
              }
              break;

          case 23: // Reset to Default
              resetToDefaults();
              break;

          case 24: // save config
              writeSettings();
              Serial.println("Config Saved");
              break;

          case 25: // Set Node ID (uint16)
              if(len >= 2) {
                  uint16_t newId = payload[0] | (payload[1] << 8);
                  if (newId >= 1 && newId <= MAX_CAN_ID) {
                      NODE_ID = newId;
                      Serial.printf("Node ID set to: %u\n", NODE_ID);
                      updateWifiHostname();
                      // MDNS.*/WiFi.* must only run on wifiTaskFn() (Core 0) -- flag it instead
                      // of touching MDNS directly; the WiFi task restarts it under the new name.
                      wifiHostnameDirty = true;
                  } else {
                      Serial.printf("Invalid Node ID requested: %u (must be 1-%d)\n", newId, MAX_CAN_ID);
                  }
              }
              break;

          case 26: // AUX Connector settings
              if (len >= 8) {

                  // Save the whole payload for EEPROM / boot replay
                  if (payload != storedAUXPayload) {
                      memset(storedAUXPayload, 0, 8);
                      memcpy(storedAUXPayload, payload, len);
                  }

                  // Extract per-pin function codes and values (little-endian uint16)
                  uint16_t newAUX1Function = payload[0] | (payload[1] << 8);
                  uint16_t newAUX1Value    = payload[2] | (payload[3] << 8);
                  uint16_t newAUX2Function = payload[4] | (payload[5] << 8);
                  uint16_t newAUX2Value    = payload[6] | (payload[7] << 8);

                  // --- Serial Control is a joint mode: both pins must be set to 1 ---
                  bool newSerial = (newAUX1Function == 1 && newAUX2Function == 1);
                  bool wasSerial = (AUX1Function == 1 && AUX2Function == 1);

                  // Teardown Serial1 if leaving serial mode
                  if (wasSerial && !newSerial) {
                      Serial1.end();
                      pinMode(AUX1, INPUT);
                      pinMode(AUX2, INPUT);
                      Serial.println("AUX: Serial1 stopped");
                  }

                  // Start Serial1 if entering serial mode
                  if (!wasSerial && newSerial) {
                      Serial1.begin(115200, SERIAL_8N1, AUX2, AUX1);
                      Serial.println("AUX: Serial1 started (115200 baud, AUX1=TX, AUX2=RX)");
                  }

                  // --- Configure AUX1 if its function has changed (and not in serial mode) ---
                  if (newAUX1Function != AUX1Function && !newSerial) {
                      Serial.printf("AUX1: Changing mode from %u to %u\n", AUX1Function, newAUX1Function);
                      switch (newAUX1Function) {
                          case 0: pinMode(AUX1, INPUT);        break; // High Impedance
                          case 3: pinMode(AUX1, OUTPUT);       break; // Digital Output
                          case 4: pinMode(AUX1, INPUT_PULLUP); break; // Digital Input
                          case 5: // Analog Input
                              pinMode(AUX1, ANALOG);
                              analogSetPinAttenuation(AUX1, ADC_11db);
                              break; // Analog Input
                          case 6: ledcAttach(AUX1, 50, 12);    break; // PWM Output
                      }
                  }

                  // --- Configure AUX2 if its function has changed (and not in serial mode) ---
                  if (newAUX2Function != AUX2Function && !newSerial) {
                      Serial.printf("AUX2: Changing mode from %u to %u\n", AUX2Function, newAUX2Function);
                      switch (newAUX2Function) {
                          case 0: pinMode(AUX2, INPUT);        break; // High Impedance
                          case 3: pinMode(AUX2, OUTPUT);       break; // Digital Output
                          case 4: pinMode(AUX2, INPUT_PULLUP); break; // Digital Input
                          case 5: // Analog Input
                              pinMode(AUX2, ANALOG);
                              analogSetPinAttenuation(AUX2, ADC_11db);
                              break; // Analog Input
                          case 6: ledcAttach(AUX2, 50, 12);    break; // PWM Output
                      }
                  }

                  // Store new function codes
                  AUX1Function = newAUX1Function;
                  AUX2Function = newAUX2Function;

                  // --- Apply output values (always, in case value changed without mode change) ---
                  if (!newSerial) {
                      if (AUX1Function == 3) { // Digital Output
                          digitalWrite(AUX1, newAUX1Value ? HIGH : LOW);
                          Serial.printf("AUX1: Digital = %u\n", newAUX1Value);
                      } else if (AUX1Function == 6) { // PWM Output
                          ledcWrite(AUX1, newAUX1Value);
                          Serial.printf("AUX1: PWM = %u\n", newAUX1Value);
                      }

                      if (AUX2Function == 3) { // Digital Output
                          digitalWrite(AUX2, newAUX2Value ? HIGH : LOW);
                          Serial.printf("AUX2: Digital = %u\n", newAUX2Value);
                      } else if (AUX2Function == 6) { // PWM Output
                          ledcWrite(AUX2, newAUX2Value);
                          Serial.printf("AUX2: PWM = %u\n", newAUX2Value);
                      }
                  }
              }
              break;

          case 27: // Set Hold Current (uint16, %)
              if(len >= 2) {
                  holdCurrent = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Hold current set to: %u percent\n", holdCurrent);
                  stepper_driver.setHoldCurrent(holdCurrent);
              }
              break;

          default:
              Serial.printf("Unknown command %d\n", msgType);
              break;
      }
    }

    //RTR = 1 means it is a request for a current value
    if (rtr == 1){ //reply with current value.
      switch(msgType) {
          case 0: { // Send empty reply (often used as a node detect/heartbeat)
              canSendTelemetry(0, 0, 0);
              break;
          }

          case 1: { // Send current set Position in deg
              double posAngle = (double)posSetpoint * (360.0 / (stepsPerRev * (double)microsteps)); //convert steps back to degrees
              canSendTelemetry(1, &posAngle, sizeof(posAngle));
              break;
          }

          case 2:
              canSendTelemetry(2, &posSetpoint, sizeof(posSetpoint)); 
              break;

          case 3:
              canSendTelemetry(3, &velocitySetpoint, sizeof(velocitySetpoint));
              break;

          case 4:
              canSendTelemetry(4, &current, sizeof(current));
              break;

          case 5:
              canSendTelemetry(5, &driverEnabled, sizeof(driverEnabled));
              break;

          case 7:
              canSendTelemetry(7, &sensorlessHomeMode, sizeof(sensorlessHomeMode));
              break;

          case 8:
              canSendTelemetry(8, &zeroEncAtBoot, sizeof(zeroEncAtBoot));
              break;

          case 9: {
              uint8_t ledPayload[2] = {(uint8_t)ledState, (uint8_t)ledState2};
              canSendTelemetry(9, ledPayload, sizeof(ledPayload));
              break;
          }

          case 10:
              canSendTelemetry(10, &stepsPerRev, sizeof(stepsPerRev));
              break;

          case 11:
              canSendTelemetry(11, &microsteps, sizeof(microsteps));
              break;

          case 12: { // Send current StallGuard threshold
              int16_t thresh = (int16_t)stallThresh;
              canSendTelemetry(12, &thresh, sizeof(thresh));
              break;
          }

          case 13: {
              unsigned int ct = controlType; //copy to new var as is used by ISR
              canSendTelemetry(13, &ct, sizeof(ct));
              break;
          }

          case 14:
              canSendTelemetry(14, &standstillMode, sizeof(standstillMode));
              break;

          case 15:
              canSendTelemetry(15, &mapDirection, sizeof(mapDirection));
              break;

          case 16:
              canSendTelemetry(16, &posSpeed, sizeof(posSpeed));
              break;

          case 17:
              canSendTelemetry(17, &accel, sizeof(accel));
              break;

          case 18:
              canSendTelemetry(18, &decel, sizeof(decel));
              break;
          
          case 19: {
              uint16_t freqs[2] = {reportFreq1, reportFreq2};
              canSendTelemetry(19, freqs, sizeof(freqs));
              break;
          }
          
          case 20:
              canSendTelemetry(20, &enableOnBoot, sizeof(enableOnBoot));
              break;

          case 21:
              canSendTelemetry(21, &LED3V3Disable, sizeof(LED3V3Disable));
              break;

          case 25: {
              uint16_t nid = NODE_ID;
              canSendTelemetry(25, &nid, sizeof(nid));
              break;
          }

          case 26: {
              // Build response from storedAUXPayload (contains function codes),
              // but substitute live pin readings for input modes.
              uint8_t rtrPayload[8];
              memcpy(rtrPayload, storedAUXPayload, 8);

              // AUX1 live read
              if (AUX1Function == 4) { // Digital Input
                  uint16_t val = digitalRead(AUX1) ? 1 : 0;
                  rtrPayload[2] = val & 0xFF;
                  rtrPayload[3] = (val >> 8) & 0xFF;
              } else if (AUX1Function == 5) { // Analog Input (mV)
                  uint16_t val = (uint16_t)analogReadMilliVolts(AUX1);
                  rtrPayload[2] = val & 0xFF;
                  rtrPayload[3] = (val >> 8) & 0xFF;
              }

              // AUX2 live read
              if (AUX2Function == 4) { // Digital Input
                  uint16_t val = digitalRead(AUX2) ? 1 : 0;
                  rtrPayload[6] = val & 0xFF;
                  rtrPayload[7] = (val >> 8) & 0xFF;
              } else if (AUX2Function == 5) { // Analog Input (mV)
                  uint16_t val = (uint16_t)analogReadMilliVolts(AUX2);
                  rtrPayload[6] = val & 0xFF;
                  rtrPayload[7] = (val >> 8) & 0xFF;
              }

              canSendTelemetry(26, rtrPayload, 8);
              break;
          }

          case 27:
              canSendTelemetry(27, &holdCurrent, sizeof(holdCurrent));
              break;

          // ---- Telemetry values requestable via RTR, computed/read on demand. ----

          case 33: // Angle (deg, double) — reuse the cached value refreshed every loop() iteration
              canSendTelemetry(33, &angle, sizeof(angle));
              break;

          case 34: { // Input voltage (float)
              float voltage = readInputVoltage();
              canSendTelemetry(34, &voltage, sizeof(voltage));
              break;
          }

          case 35: { // Button states (byte0=SW1, byte1=SW2, 1=pressed) — last debounced state
              uint8_t payload[2];
              payload[0] = lastSW1State ? 0 : 1; // LOW = pressed = 1
              payload[1] = lastSW2State ? 0 : 1;
              canSendTelemetry(35, payload, sizeof(payload));
              break;
          }

          case 36: { // StallGuard result (uint32, 10-bit, 0 = stall)
              uint32_t sgValue = stepper_driver.getStallGuardResult();
              canSendTelemetry(36, &sgValue, sizeof(sgValue));
              break;
          }

          case 37: { // Stall state — live DIAG pin read (1 = currently triggered)
              uint8_t stallState = digitalRead(DIAG);
              canSendTelemetry(37, &stallState, sizeof(stallState));
              break;
          }

          case 38: { // NTC temperature (float, °C)
              float NTCtemp = readNTCTemperature();
              canSendTelemetry(38, &NTCtemp, sizeof(NTCtemp));
              break;
          }

          case 39: // Fault Code (uint16 bitfield) -- see checkFaults() / FAULT_* definitions
              canSendTelemetry(39, &faultCode, sizeof(faultCode));
              break;

          case 40: // Firmware version (float)
              canSendTelemetry(40, &firmwareVersion, sizeof(firmwareVersion));
              break;

          case 41: { // ESP internal temperature (float, °C)
              float ESPtemp = temperatureRead();
              canSendTelemetry(41, &ESPtemp, sizeof(ESPtemp));
              break;
          }

          case 42: { // Current velocity (float, deg/sec)
              float currentVelocity = ((float)motion.currentSpeed_q / (float)ONE_Q) * (360.0f / (stepsPerRev * microsteps));
              canSendTelemetry(42, &currentVelocity, sizeof(currentVelocity));
              break;
          }

          default:
              Serial.printf("Unknown command %d\n", msgType);
              break;

      }
    }
            
}

// -------------------- Handle CAN Frame --------------------
void handleCANFrame(const CanFrame &frame) {
    int nodeId = (frame.identifier >> 6) & 0x1F;
    int msgType = frame.identifier & 0x3F;
    uint8_t rtr = frame.rtr;   // expect 0 or 1
    if (rtr != 1) rtr = 0;

    // Execute command if for this node
    if((msgType < 32) or (rtr == 1)) { // Command types (0-31)
        executeCommand(nodeId, msgType, frame.data, frame.data_length_code, rtr); 
    }

    // Forward all non-RTR messages onward (USB serial, AUX, and WebSocket).
    // NOTE: don't gate this on "&& Serial" -- forwardTelemetryToSerial() already checks
    // Serial internally before actually writing to it, but also unconditionally handles
    // the AUX and WebSocket forwarding. Gating the whole call on Serial being connected
    // meant other nodes' CAN telemetry never reached the WebSocket (only this node's own,
    // sent via canSendTelemetry()) whenever nothing was plugged into USB -- exactly the
    // "only 1 node found over WiFi" symptom.
    if(rtr == 0) {
        forwardTelemetryToSerial(frame);
    }
}

// ----------------- Forward telem data to serial to be parsed on a PC --------------------
void forwardTelemetryToSerial(const CanFrame &frame) {

    // Build the output once, write to whichever serial ports are active
    char buf[32];
    int pos = 0;

    // CAN ID (hex)
    pos += sprintf(buf + pos, "%lX ", frame.identifier);

    // RTR bit
    pos += sprintf(buf + pos, "%d ", frame.rtr ? 1 : 0);

    // Payload as hex (always 8 bytes padded)
    for (int i = 0; i < 8; i++) {
        uint8_t b = (i < frame.data_length_code) ? frame.data[i] : 0;
        pos += sprintf(buf + pos, "%02X", b);
    }
    buf[pos++] = '\n';
    buf[pos]   = '\0';

    if (Serial) {
        Serial.print(buf);
    }
    if (AUX1Function == 1) {
        Serial1.print(buf);
    }
    broadcastLineToWS(buf); // WS clients get the same line (trailing \n is harmless, GUI trims it)
}

// -------------------- Serial to CAN --------------------
// Handles both the normal <CAN_ID_HEX> <RTR> <PAYLOAD_HEX> frame protocol and the
// WIFI_SET / WIFI_STATUS? out-of-band text commands (serial-only -- see handleCommandLine()
// for the frame protocol itself, shared with the WebSocket input path in onWsEvent()).
void handleSerialInput() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("WIFI_SET,")) {
        handleWifiSetCommand(line);
        return;
    }
    if (line == "WIFI_STATUS?") {
        handleWifiStatusQuery(Serial);
        return;
    }

    handleCommandLine(line);
}

// Parses a "<CAN_ID_HEX> <RTR> <PAYLOAD_HEX>" line, executes it locally (if addressed to
// this node) and forwards it onto the CAN bus. Shared by USB serial and the WebSocket
// input path -- the two transports otherwise behave identically from the node's point of view.
void handleCommandLine(const String &line) {
    // Expect format:  <CAN_ID_HEX> <RTR> <PAYLOAD_HEX>
    int firstSpace = line.indexOf(' ');
    if (firstSpace < 0) {
        Serial.println("Invalid line (no CAN ID)");
        return;
    }

    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) {
        Serial.println("Invalid line (no RTR)");
        return;
    }

    // Parse CAN ID
    String headerHex = line.substring(0, firstSpace);
    unsigned long canId = strtoul(headerHex.c_str(), NULL, 16);

    // Extract NodeID (5 bits) and MsgType (6 bits)
    uint8_t targetNode = (canId >> 6) & 0x1F;
    uint8_t msgType    =  canId & 0x3F;

    // Parse RTR
    String rtrStr = line.substring(firstSpace + 1, secondSpace);
    uint8_t rtr = rtrStr.toInt();   // expect 0 or 1
    if (rtr > 1) rtr = 0;

    // Parse Payload (may be empty)
    String payloadHex = line.substring(secondSpace + 1);
    payloadHex.trim();

    uint8_t payload[8] = {0};
    int payloadLen = payloadHex.length() / 2;
    if (payloadLen > 8) payloadLen = 8;

    for (int i = 0; i < payloadLen; i++) {
        payload[i] = (uint8_t)strtoul(payloadHex.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
    }

    // Execute command locally if node matches (including RTR return)
    executeCommand(targetNode, msgType, payload, payloadLen, rtr);

    // Forward to CAN (with RTR)
    sendCANFrame(canId, payload, payloadLen, rtr);
}

// -------------------- AUX Serial to CAN (mirrors handleSerialInput but uses Serial1) --------------------
void handleAuxSerialInput() {
    if (!Serial1.available()) return;

    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Expect format:  <CAN_ID_HEX> <RTR> <PAYLOAD_HEX>
    int firstSpace = line.indexOf(' ');
    if (firstSpace < 0) {
        Serial1.println("Invalid line (no CAN ID)");
        return;
    }

    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) {
        Serial1.println("Invalid line (no RTR)");
        return;
    }

    // Parse CAN ID
    String headerHex = line.substring(0, firstSpace);
    unsigned long canId = strtoul(headerHex.c_str(), NULL, 16);

    // Extract NodeID (5 bits) and MsgType (6 bits)
    uint8_t targetNode = (canId >> 6) & 0x1F;
    uint8_t msgType    =  canId & 0x3F;

    // Parse RTR
    String rtrStr = line.substring(firstSpace + 1, secondSpace);
    uint8_t rtr = rtrStr.toInt();
    if (rtr > 1) rtr = 0;

    // Parse Payload (may be empty)
    String payloadHex = line.substring(secondSpace + 1);
    payloadHex.trim();

    uint8_t payload[8] = {0};
    int payloadLen = payloadHex.length() / 2;
    if (payloadLen > 8) payloadLen = 8;

    for (int i = 0; i < payloadLen; i++) {
        payload[i] = (uint8_t)strtoul(payloadHex.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
    }

    // Execute command locally if node matches (including RTR return)
    executeCommand(targetNode, msgType, payload, payloadLen, rtr);

    // Forward to CAN bus
    sendCANFrame(canId, payload, payloadLen, rtr);
}


void sendCANFrame(unsigned long canId, uint8_t *payload, uint8_t len, bool rtr) {
    CanFrame txFrame = {0};
    txFrame.identifier = canId; // full CAN ID already provided
    txFrame.extd = 0;
    txFrame.rtr = rtr;
    txFrame.data_length_code = len;
    memcpy(txFrame.data, payload, len);
    ESP32Can.writeFrame(txFrame);
}

// -------------------- WiFi / WebSocket (optional, serial-provisioned) --------------------
// Everything below is opt-in per node (wifiEnabled). Credentials are only ever accepted over
// USB serial (WIFI_SET, handled in handleSerialInput()) -- never over the CAN bus, which is
// shared by every node, and never over an existing WebSocket connection. See CAN Protocol.md.

// Queues a text line for wifiTaskFn() to broadcast to every connected WebSocket client --
// called from loop() (Core 1), so it must never touch wsServer directly (that's owned
// exclusively by the WiFi task on Core 0). Non-blocking: if the queue is momentarily full
// (WiFi task busy / disabled) this drops the line rather than stalling motion control.
void broadcastLineToWS(const char *buf) {
    if (wsOutQueue == nullptr) return; // not created yet (shouldn't happen past setup())
    WsLine line;
    strlcpy(line.text, buf, sizeof(line.text));
    xQueueSend(wsOutQueue, &line, 0);
}

// Builds the WIFI_STATUS reply line: WIFI_STATUS,<enabled>,<connected>,<hostname>,<ip>,<ssid>
// The password is intentionally never included. Called from both loop() (serial query) and
// wifiTaskFn() (WebSocket query), so the SSID/hostname buffers are read under wifiConfigMux.
String buildWifiStatusLine() {
    bool connected = (WiFi.status() == WL_CONNECTED); // WiFi.* calls are documented thread-safe (ESP-IDF)
    char hostCopy[sizeof(wifiHostname)];
    char ssidCopy[sizeof(wifiSsid)];
    portENTER_CRITICAL(&wifiConfigMux);
    strlcpy(hostCopy, wifiHostname, sizeof(hostCopy));
    strlcpy(ssidCopy, wifiSsid, sizeof(ssidCopy));
    portEXIT_CRITICAL(&wifiConfigMux);

    String line = "WIFI_STATUS,";
    line += (wifiEnabled ? "1" : "0"); line += ",";
    line += (connected ? "1" : "0"); line += ",";
    line += String(hostCopy) + ".local"; line += ",";
    line += (connected ? WiFi.localIP().toString() : String("")); line += ",";
    line += ssidCopy;
    return line;
}

void handleWifiStatusQuery(Print &out) {
    out.println(buildWifiStatusLine());
}

// WIFI_SET,<enabled 0|1>,<ssid>,<password>  -- serial-only, see handleSerialInput().
// SSID must not itself contain a comma; password is everything after the second comma,
// so it may contain commas freely. An empty password for the SAME ssid keeps the
// previously stored password (lets the GUI toggle "enabled" or change nothing else
// without having to resend the password).
void handleWifiSetCommand(const String &line) {
    int c1 = line.indexOf(',');           // right after "WIFI_SET"
    int c2 = (c1 < 0) ? -1 : line.indexOf(',', c1 + 1);  // right after <enabled>
    int c3 = (c2 < 0) ? -1 : line.indexOf(',', c2 + 1);  // right after <ssid>
    if (c1 < 0 || c2 < 0 || c3 < 0) {
        Serial.println("WIFI_SET_ERR malformed command");
        return;
    }

    String enabledStr = line.substring(c1 + 1, c2);
    String ssid       = line.substring(c2 + 1, c3);
    String password    = line.substring(c3 + 1); // rest of line -- may contain commas
    ssid.trim();

    bool newEnabled = enabledStr.toInt() != 0;
    if (newEnabled && ssid.length() == 0) {
        Serial.println("WIFI_SET_ERR SSID required when enabling WiFi");
        return;
    }
    if (ssid.length() > 32 || password.length() > 63) {
        Serial.println("WIFI_SET_ERR SSID/password too long (max 32/63 chars)");
        return;
    }

    // Snapshot the previous SSID (under the mutex -- wifiTaskFn() may be reading it right
    // now) before deciding whether to keep the existing password.
    char prevSsid[sizeof(wifiSsid)];
    portENTER_CRITICAL(&wifiConfigMux);
    strlcpy(prevSsid, wifiSsid, sizeof(prevSsid));
    portEXIT_CRITICAL(&wifiConfigMux);
    bool keepPassword = (password.length() == 0 && ssid.equals(String(prevSsid)));

    wifiEnabled = newEnabled;
    portENTER_CRITICAL(&wifiConfigMux);
    strlcpy(wifiSsid, ssid.c_str(), sizeof(wifiSsid));
    if (!keepPassword) {
        strlcpy(wifiPassword, password.c_str(), sizeof(wifiPassword));
    }
    portEXIT_CRITICAL(&wifiConfigMux);
    saveWifiCredentials();
    Serial.println("WIFI_SET_OK");

    // Actually bringing WiFi up/down (WiFi.*, wsServer.begin()) must only ever happen on
    // the dedicated WiFi task (see wifiTaskFn()) -- flag the request here; the task picks
    // it up on its next iteration (a couple of ms away at most, it isn't blocking).
    if (wifiEnabled && strlen(wifiSsid) > 0) {
        wifiConnectRequested = true;
    } else {
        wifiDisconnectRequested = true;
    }
}

void loadWifiCredentials() {
    wifiPreferences.begin("wifinet", false);
    wifiEnabled = wifiPreferences.getBool("enabled", false);
    wifiPreferences.getString("ssid", wifiSsid, sizeof(wifiSsid));
    wifiPreferences.getString("password", wifiPassword, sizeof(wifiPassword));
    wifiPreferences.end();
}

void saveWifiCredentials() {
    wifiPreferences.begin("wifinet", false);
    wifiPreferences.putBool("enabled", wifiEnabled);
    wifiPreferences.putString("ssid", wifiSsid);
    wifiPreferences.putString("password", wifiPassword);
    wifiPreferences.end();
}

void updateWifiHostname() {
    char newHost[sizeof(wifiHostname)];
    snprintf(newHost, sizeof(newHost), "canbus-stepper-%u", NODE_ID);
    portENTER_CRITICAL(&wifiConfigMux);
    strlcpy(wifiHostname, newHost, sizeof(wifiHostname));
    portEXIT_CRITICAL(&wifiConfigMux);
}

// Kicks off a non-blocking WiFi connect attempt. Progress/result is polled in
// handleWiFiStateMachine(). Only ever called from wifiTaskFn() (Core 0) -- at task entry,
// from its wifiConnectRequested handling, or from handleWiFiStateMachine()'s own
// reconnect/timeout branches -- never from loop().
void startWifiConnect() {
    char ssidCopy[sizeof(wifiSsid)];
    char passCopy[sizeof(wifiPassword)];
    char hostCopy[sizeof(wifiHostname)];
    portENTER_CRITICAL(&wifiConfigMux);
    strlcpy(ssidCopy, wifiSsid, sizeof(ssidCopy));
    strlcpy(passCopy, wifiPassword, sizeof(passCopy));
    strlcpy(hostCopy, wifiHostname, sizeof(hostCopy));
    portEXIT_CRITICAL(&wifiConfigMux);

    WiFi.mode(WIFI_STA);
    // Station-mode WiFi power-save ("modem sleep") is ON by default -- the radio
    // periodically sleeps between beacons to save power, and each wake-up involves an RF
    // resync that briefly stalls the chip. That stall doesn't respect task priorities or
    // core pinning, so it's a well-known cause of exactly this kind of periodic jitter in
    // time-critical sketches, independent of which core anything is running on. This node
    // is USB/externally powered, not battery, so there's no reason to pay for power-save.
    WiFi.setSleep(false);
    WiFi.setHostname(hostCopy);
    WiFi.begin(ssidCopy, passCopy);

    // Lazily start the WebSocket server's listen socket the first time we actually bring
    // up WiFi -- see the comment in setup() for why this can't happen any earlier.
    if (!wsServerStarted) {
        wsServer.begin();
        wsServerStarted = true;
    }

    wifiConnecting = true;
    wifiConnectStartMs = millis();
    Serial.printf("WiFi: connecting to \"%s\" as %s.local...\n", ssidCopy, hostCopy);
}

// Non-blocking WiFi state machine: notices a fresh connect (starts mDNS), notices a drop
// (reconnects), and retries if a connect attempt doesn't resolve within the timeout
// (wrong password / AP out of range). No-op whenever the feature is disabled. Only ever
// called from wifiTaskFn() (Core 0) -- never from loop().
void handleWiFiStateMachine() {
    if (!wifiEnabled) return;
    bool connected = (WiFi.status() == WL_CONNECTED);

    if (connected && !wifiWasConnected) {
        wifiWasConnected = true;
        wifiConnecting = false;
        char hostCopy[sizeof(wifiHostname)];
        portENTER_CRITICAL(&wifiConfigMux);
        strlcpy(hostCopy, wifiHostname, sizeof(hostCopy));
        portEXIT_CRITICAL(&wifiConfigMux);
        Serial.printf("WiFi: connected, IP %s, hostname %s.local\n", WiFi.localIP().toString().c_str(), hostCopy);
        MDNS.end();
        if (MDNS.begin(hostCopy)) {
            MDNS.addService("ws", "tcp", WS_PORT);
        } else {
            Serial.println("WiFi: mDNS responder failed to start");
        }
    } else if (!connected && wifiWasConnected) {
        wifiWasConnected = false;
        Serial.println("WiFi: connection lost, reconnecting...");
        startWifiConnect();
    } else if (!connected && wifiConnecting && (millis() - wifiConnectStartMs > WIFI_CONNECT_TIMEOUT_MS)) {
        Serial.println("WiFi: connect attempt timed out, retrying...");
        startWifiConnect();
    }
}

// WebSocket event callback (WebSocketsServer). Text frames carry the same line protocol as
// USB serial: a "<CAN_ID_HEX> <RTR> <PAYLOAD_HEX>" command, or a WIFI_STATUS? status query
// (read-only -- WIFI_SET is serial-only, intentionally not handled here). Runs inside
// wifiTaskFn() (called synchronously from wsServer.loop(), Core 0), so it must never touch
// CAN/motion state directly -- a command line is handed off to loop() via wsInQueue instead.
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            wsClientCount++;
            Serial.printf("WebSocket: client %u connected (%u total)\n", num, wsClientCount);
            break;

        case WStype_DISCONNECTED:
            if (wsClientCount > 0) wsClientCount--;
            Serial.printf("WebSocket: client %u disconnected (%u total)\n", num, wsClientCount);
            break;

        case WStype_TEXT: {
            // The library null-terminates TEXT payloads, so a plain char* constructor is safe.
            String line = String((char*)payload);
            line.trim();
            if (line.length() == 0) break;
            if (line == "WIFI_STATUS?") {
                String status = buildWifiStatusLine(); // sendTXT() takes a non-const String&, needs a named lvalue
                wsServer.sendTXT(num, status);
                break;
            }
            // Command line (CAN frame protocol) -- executeCommand()/sendCANFrame() touch
            // motion/CAN state that's only safe to touch from loop() (Core 1), so hand it
            // off via wsInQueue rather than calling handleCommandLine() here.
            if (wsInQueue != nullptr && line.length() < sizeof(WsLine::text)) {
                WsLine queued;
                strlcpy(queued.text, line.c_str(), sizeof(queued.text));
                xQueueSend(wsInQueue, &queued, 0);
            } else {
                Serial.println("WebSocket: command line dropped (too long or queue unavailable)");
            }
            break;
        }

        default:
            break;
    }
}

// Dedicated task for all WiFi/WebSocket/mDNS I/O -- pinned to Core 0 at creation (see
// setup()), kept off Core 1 where loop() and the hardware-interrupt-driven Motion Task /
// Step ISR now live. This task exclusively owns wsServer, WiFi.*, and MDNS.*;
// loop() (Core 1) talks to it only through wsInQueue/wsOutQueue and the small set of
// request flags (wifiConnectRequested/wifiDisconnectRequested/wifiHostnameDirty) -- see
// the WiFi globals comment above for the full picture.
void wifiTaskFn(void *pvParameters) {
    // wifiEnabled/wifiSsid were already loaded by loadWifiCredentials() in setup() before
    // this task was created. The mutex here is just cheap insurance against the (very
    // unlikely) case of a WIFI_SET serial command landing in the same instant.
    portENTER_CRITICAL(&wifiConfigMux);
    bool haveSsid = (strlen(wifiSsid) > 0);
    portEXIT_CRITICAL(&wifiConfigMux);
    if (wifiEnabled && haveSsid) {
        startWifiConnect();
    }

    WsLine outgoing;
    for (;;) {
        if (wifiDisconnectRequested) {
            wifiDisconnectRequested = false;
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnecting = false;
            wifiWasConnected = false;
        }
        if (wifiConnectRequested) {
            wifiConnectRequested = false;
            startWifiConnect();
        }

        if (wsServerStarted) wsServer.loop();
        handleWiFiStateMachine();

        if (wifiHostnameDirty) {
            wifiHostnameDirty = false;
            if (WiFi.status() == WL_CONNECTED) {
                char hostCopy[sizeof(wifiHostname)];
                portENTER_CRITICAL(&wifiConfigMux);
                strlcpy(hostCopy, wifiHostname, sizeof(hostCopy));
                portEXIT_CRITICAL(&wifiConfigMux);
                MDNS.end();
                if (MDNS.begin(hostCopy)) MDNS.addService("ws", "tcp", WS_PORT);
            }
        }

        // Drain outgoing telemetry lines queued by broadcastLineToWS() (loop(), Core 1)
        // and actually broadcast them -- this is the only place wsServer.broadcastTXT()
        // is ever called from.
        while (xQueueReceive(wsOutQueue, &outgoing, 0) == pdTRUE) {
            if (wsServerStarted && wsClientCount > 0) {
                wsServer.broadcastTXT((uint8_t*)outgoing.text, (size_t)strlen(outgoing.text));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // light yield -- keeps this task from spinning Core 0
    }
}

// -------------------- Button State Change Detection --------------------
// Sends MsgType 35 telemetry on any button state change.
// Byte 0 = SW1, Byte 1 = SW2. Value 1 = pressed, 0 = not pressed.
// Buttons read LOW when pressed, so pin state is inverted for the payload.
void checkButtons() {
    bool sw1 = digitalRead(SW1);
    bool sw2 = digitalRead(SW2);

    if (sw1 != lastSW1State || sw2 != lastSW2State) {
        // State change detected — start/reset debounce timer
        if (millis() - lastDebounceTime > DEBOUNCE_DELAY_MS) {
            lastDebounceTime = millis();

            lastSW1State = sw1;
            lastSW2State = sw2;

            uint8_t payload[2];
            payload[0] = sw1 ? 0 : 1; // LOW = pressed = 1
            payload[1] = sw2 ? 0 : 1;
            canSendTelemetry(35, payload, sizeof(payload));
        }
    }
}

// -------------------- Fault Monitoring (MsgType 39) --------------------
// Updates the faultCode bitfield every iteration, sends MsgType 39 on any
// change. PCB Over-Temp Shutdown also disables the driver and auto-resumes.
void checkFaults() {
    uint16_t newFaultCode = faultCode; // start from current state so hysteresis can hold bits set
    float pcbTemp = readNTCTemperature();

    // PCB over-temperature (NTC, same sensor as MsgType 38). Shutdown
    // threshold is always above Warning threshold, so Shutdown implies
    // Warning, but each bit is evaluated independently below.
    if (pcbTemp >= PCB_TEMP_WARNING_C) {
        newFaultCode |= FAULT_PCB_OVERTEMP_WARNING;
    } else if (pcbTemp < (PCB_TEMP_WARNING_C - PCB_TEMP_HYSTERESIS_C)) {
        newFaultCode &= ~FAULT_PCB_OVERTEMP_WARNING;
    }

    if (pcbTemp >= PCB_TEMP_SHUTDOWN_C) {
        newFaultCode |= FAULT_PCB_OVERTEMP_SHUTDOWN;
    } else if (pcbTemp < (PCB_TEMP_SHUTDOWN_C - PCB_TEMP_HYSTERESIS_C)) {
        newFaultCode &= ~FAULT_PCB_OVERTEMP_SHUTDOWN;
    }

    // Mirrors the TMC2209's live status registers, report-only. Cleared when
    // the driver isn't powered/answering rather than left stale.
    const uint16_t DRIVER_FAULT_BITS = FAULT_DRIVER_OVERTEMP_WARNING | FAULT_DRIVER_OVERTEMP_SHUTDOWN |
                                        FAULT_DRIVER_SHORT_TO_GROUND_A | FAULT_DRIVER_SHORT_TO_GROUND_B |
                                        FAULT_DRIVER_LOW_SIDE_SHORT_A | FAULT_DRIVER_LOW_SIDE_SHORT_B |
                                        FAULT_DRIVER_CHARGE_PUMP_UNDERVOLTAGE;
    if (stepper_driver.isSetupAndCommunicating()) {
        TMC2209::Status driverStatus = stepper_driver.getStatus();
        TMC2209::GlobalStatus globalStatus = stepper_driver.getGlobalStatus();

        if (driverStatus.over_temperature_warning) newFaultCode |= FAULT_DRIVER_OVERTEMP_WARNING;
        else newFaultCode &= ~FAULT_DRIVER_OVERTEMP_WARNING;

        if (driverStatus.over_temperature_shutdown) newFaultCode |= FAULT_DRIVER_OVERTEMP_SHUTDOWN;
        else newFaultCode &= ~FAULT_DRIVER_OVERTEMP_SHUTDOWN;

        if (driverStatus.short_to_ground_a) newFaultCode |= FAULT_DRIVER_SHORT_TO_GROUND_A;
        else newFaultCode &= ~FAULT_DRIVER_SHORT_TO_GROUND_A;

        if (driverStatus.short_to_ground_b) newFaultCode |= FAULT_DRIVER_SHORT_TO_GROUND_B;
        else newFaultCode &= ~FAULT_DRIVER_SHORT_TO_GROUND_B;

        if (driverStatus.low_side_short_a) newFaultCode |= FAULT_DRIVER_LOW_SIDE_SHORT_A;
        else newFaultCode &= ~FAULT_DRIVER_LOW_SIDE_SHORT_A;

        if (driverStatus.low_side_short_b) newFaultCode |= FAULT_DRIVER_LOW_SIDE_SHORT_B;
        else newFaultCode &= ~FAULT_DRIVER_LOW_SIDE_SHORT_B;

        if (globalStatus.uv_cp) newFaultCode |= FAULT_DRIVER_CHARGE_PUMP_UNDERVOLTAGE;
        else newFaultCode &= ~FAULT_DRIVER_CHARGE_PUMP_UNDERVOLTAGE;
    } else {
        newFaultCode &= ~DRIVER_FAULT_BITS;
    }

    if (newFaultCode != faultCode) {
        faultCode = newFaultCode;
        Serial.printf("Fault code changed: 0x%04X (PCB temp %.1f C)\n", faultCode, pcbTemp);
        canSendTelemetry(39, &faultCode, sizeof(faultCode)); // msgType 39, send immediately on change
    }

    // PCB Over-Temp Shutdown: disable the driver while the bit is set, and
    // bring it back once it clears, restoring whatever driverEnabled already
    // was (untouched here) rather than assuming the driver should re-enable.
    bool shutdownFaultNow = (faultCode & FAULT_PCB_OVERTEMP_SHUTDOWN) != 0;
    if (shutdownFaultNow && !faultShutdownActive) {
        faultShutdownActive = true;
        Serial.println("PCB Over-Temp Shutdown: disabling driver");
        stepper_driver.disable();
        motionHold = true; // see executeCommand() case 5's disable branch for why
        haltMotionKeepTarget();
    } else if (!shutdownFaultNow && faultShutdownActive) {
        faultShutdownActive = false;
        Serial.println("PCB Over-Temp Shutdown cleared");
        if (driverEnabled && VbusState == 1) { // VbusState mirrors driverPowered -- see powerUpDriver()/powerDownDriver()
            // Resume exactly like a fresh Enable command: hold at standstill,
            // soft-enable, then let current regulation settle before ramping
            // (see DRIVER_SETTLE_MS / configureDriver() for why this matters).
            motionHold = true;
            haltMotionKeepTarget();
            stepper_driver.enable();
            delay(DRIVER_SETTLE_MS);
            motionHold = false;
        }
    }
}

// Restores EEPROM-backed settings to protocol defaults and replays each
// through executeCommand() so side effects apply normally, then persists
// to EEPROM. NODE_ID is left untouched.
void resetToDefaults() {
    Serial.println("Resetting to default configuration...");

    // Safety: stop the motor before changing motion-related parameters
    stepper_driver.disable();
    motionHold = true; // see executeCommand() case 5's disable branch for why
    portENTER_CRITICAL(&timerMux);
    motion.targetSpeed_q = 0;
    motion.currentSpeed_q = 0;
    isr_stepDelay_us = 0;
    portEXIT_CRITICAL(&timerMux);
    driverEnabled = false;

    uint8_t p[8];

    // MsgType 4 - Set Current (uint16, %)
    memset(p, 0, 8); p[0] = DEFAULT_CURRENT & 0xFF; p[1] = (DEFAULT_CURRENT >> 8) & 0xFF;
    executeCommand(NODE_ID, 4, p, 2, 0);

    // MsgType 8 - Zero Encoder at Boot (bool)
    memset(p, 0, 8); p[0] = DEFAULT_ZERO_ENC_BOOT;
    executeCommand(NODE_ID, 8, p, 1, 0);

    // MsgType 9 - LED States (0: LED1, 1: LED2)
    memset(p, 0, 8); p[0] = DEFAULT_LED1_STATE; p[1] = DEFAULT_LED2_STATE;
    executeCommand(NODE_ID, 9, p, 2, 0);

    // MsgType 10 - Steps per Rev (uint16)
    memset(p, 0, 8); p[0] = DEFAULT_STEPS_PER_REV & 0xFF; p[1] = (DEFAULT_STEPS_PER_REV >> 8) & 0xFF;
    executeCommand(NODE_ID, 10, p, 2, 0);

    // MsgType 11 - Microsteps (uint16)
    memset(p, 0, 8); p[0] = DEFAULT_MICROSTEPS & 0xFF; p[1] = (DEFAULT_MICROSTEPS >> 8) & 0xFF;
    executeCommand(NODE_ID, 11, p, 2, 0);

    // MsgType 12 - Stallguard Threshold (int16)
    memset(p, 0, 8);
    { int16_t t = (int16_t)DEFAULT_STALL_THRESH; memcpy(p, &t, 2); }
    executeCommand(NODE_ID, 12, p, 2, 0);

    // MsgType 13 - Closed Loop Type (uint16)
    memset(p, 0, 8); p[0] = DEFAULT_CONTROL_TYPE & 0xFF; p[1] = (DEFAULT_CONTROL_TYPE >> 8) & 0xFF;
    executeCommand(NODE_ID, 13, p, 2, 0);

    // MsgType 14 - Standstill Mode (uint16)
    memset(p, 0, 8); p[0] = DEFAULT_STANDSTILL_MODE & 0xFF; p[1] = (DEFAULT_STANDSTILL_MODE >> 8) & 0xFF;
    executeCommand(NODE_ID, 14, p, 2, 0);

    // MsgType 15 - Map Direction (bool)
    memset(p, 0, 8); p[0] = DEFAULT_MAP_DIRECTION;
    executeCommand(NODE_ID, 15, p, 1, 0);

    // MsgType 16 - Position Speed (float, deg/sec)
    memset(p, 0, 8); memcpy(p, &DEFAULT_POS_SPEED, 4);
    executeCommand(NODE_ID, 16, p, 4, 0);

    // MsgType 17 - Acceleration (float, deg/sec^2)
    memset(p, 0, 8); memcpy(p, &DEFAULT_ACCEL, 4);
    executeCommand(NODE_ID, 17, p, 4, 0);

    // MsgType 18 - Deceleration (float, deg/sec^2)
    memset(p, 0, 8); memcpy(p, &DEFAULT_DECEL, 4);
    executeCommand(NODE_ID, 18, p, 4, 0);

    // MsgType 19 - Report Frequency (uint16 angle Hz, uint16 other Hz)
    memset(p, 0, 8);
    p[0] = DEFAULT_REPORT_FREQ1 & 0xFF; p[1] = (DEFAULT_REPORT_FREQ1 >> 8) & 0xFF;
    p[2] = DEFAULT_REPORT_FREQ2 & 0xFF; p[3] = (DEFAULT_REPORT_FREQ2 >> 8) & 0xFF;
    executeCommand(NODE_ID, 19, p, 4, 0);

    // MsgType 20 - Enabled/Disabled on Boot (bool)
    memset(p, 0, 8); p[0] = DEFAULT_ENABLE_ON_BOOT;
    executeCommand(NODE_ID, 20, p, 1, 0);

    // MsgType 21 - 3V3 LED Disable (bool)
    memset(p, 0, 8); p[0] = DEFAULT_LED3V3_DISABLE;
    executeCommand(NODE_ID, 21, p, 1, 0);

    // MsgType 26 - AUX Connector (both pins disabled)
    executeCommand(NODE_ID, 26, DEFAULT_AUX_PAYLOAD, 8, 0);

    // MsgType 27 - Hold Current (uint16, %)
    memset(p, 0, 8); p[0] = DEFAULT_HOLD_CURRENT & 0xFF; p[1] = (DEFAULT_HOLD_CURRENT >> 8) & 0xFF;
    executeCommand(NODE_ID, 27, p, 2, 0);

    // Persist everything above to EEPROM
    writeSettings();

    Serial.println("Reset to defaults complete.");
}

//Read saved settings from EEPROM
void readSettings(){
  preferences.begin("settings", false); //open the settings namespace

  NODE_ID = preferences.getUInt("NODE_ID", NODE_ID);
  microsteps = preferences.getUInt("microsteps", microsteps);
  current = preferences.getUInt("current", current);
  holdCurrent = preferences.getUInt("holdCurrent", holdCurrent);
  stallThresh = preferences.getUInt("stallThresh", stallThresh);
  stepsPerRev = preferences.getUInt("stepsPerRev", stepsPerRev);
  controlType = preferences.getUInt("controlType", controlType);
  standstillMode = preferences.getInt("standstillMode", standstillMode);
  mapDirection = preferences.getBool("mapDirection", mapDirection);
  posSpeed = preferences.getFloat("posSpeed", posSpeed);
  accel = preferences.getFloat("accel", accel);
  decel = preferences.getFloat("decel", decel);
  reportFreq1 = preferences.getUInt("reportFreq1", reportFreq1);
  reportFreq2 = preferences.getUInt("reportFreq2", reportFreq2);
  enableOnBoot = preferences.getBool("enableOnBoot", enableOnBoot);
  ledState = preferences.getBool("ledState", ledState);
  ledState2 = preferences.getBool("ledState2", ledState2);
  LED3V3Disable = preferences.getBool("LED3V3Disable", LED3V3Disable);
  zeroEncAtBoot = preferences.getBool("zeroEncAtBoot", zeroEncAtBoot);
  preferences.getBytes("storedAUX", storedAUXPayload, 8);

  preferences.end();
}

//save settings to flash
void writeSettings(){
  preferences.begin("settings", false);

  preferences.putUInt("NODE_ID", NODE_ID);
  preferences.putUInt("microsteps", microsteps);
  preferences.putUInt("current", current);
  preferences.putUInt("holdCurrent", holdCurrent);
  preferences.putUInt("stallThresh", stallThresh);
  preferences.putUInt("stepsPerRev", stepsPerRev);
  preferences.putUInt("controlType", controlType);
  preferences.putUInt("standstillMode", standstillMode);
  preferences.putBool("mapDirection", mapDirection);
  preferences.putFloat("posSpeed", posSpeed);
  preferences.putFloat("accel", accel);
  preferences.putFloat("decel", decel);
  preferences.putUInt("reportFreq1", reportFreq1);
  preferences.putUInt("reportFreq2", reportFreq2);
  preferences.putBool("enableOnBoot", enableOnBoot);
  preferences.putBool("ledState", ledState);
  preferences.putBool("ledState2", ledState2);
  preferences.putBool("LED3V3Disable", LED3V3Disable);
  preferences.putBool("zeroEncAtBoot", zeroEncAtBoot);
  preferences.putBytes("storedAUX", storedAUXPayload, 8);

  preferences.end();
}
