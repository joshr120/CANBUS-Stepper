/*
 * Multi-Node CAN Stepper Example
 * THINGS BY JOSH 2025
 * 
 * Each Node can receive commands over serial (USB) and CANBUS.
 * Messages received over USB serial will be forwarded onto the CAN bus (USB to CAN bridge).
 * See the CAN packet document for CAN and serial message formats
 * 
 *
 * Architecture:
 * 1. Motion Task (1000Hz): Calculates ramping and sets target delay. For speed reasons this uses Q16 fixed-point math (no floats)
 * 2. Step ISR (One-Shot): Generates pulses based on target delay. Direct register writes for speed.
 *
 *
 * TODO:
 * - max out I2C speed?
 * - Give error for invalid commands?
 * - Encoder calibration option?
 * - Add header file with CAN ID's instead of magic numbers!
 * - Better handle change of microsteps / accel / dir etc. better (update all values which rely on these when changed)
 * - When driver comes back online (VBUS rises) re-configure with current values better (as values may have been set when driver was not powered)
 *     - (driver setup function!)
 * - Verify accel/decel/speed timing
 * - Add Reset to defaults? (and better input sanitation)
 * - Add RTR for telem as well (for individual value request)!
 * - Disable on emergency STOP !!!!
 * - Error if another node with same ID exists
 * - Rounding errors on deg -> steps -> deg (when reporting, current setpos != the actual set pos)
 * - Add relative move command!
 * - Option to reset encoder pos on stall guard (sensorless homing)
 * - Check EEPROM settings being overwritten on flash? (need to decide preferred bahaviour).
 * - Rest of CAN msg types
 *    - Sensorless home
 *    - Stallguard
 *    - Reset to default
 *    - Set Node ID
 *    - Fault codes
 *    - AUX conn
 *       - I2C Slave Control? (removed from protocol)
 *       - Digital Input (RTR part)
 *       - Analog Input (RTR part)
 *       - AUX CONN STORE FUNTION IN EEPROM & SET AT BOOT!
 * 

 */


#include <ESP32-TWAI-CAN.hpp> //https://github.com/handmade0octopus/ESP32-TWAI-CAN
#include <TMC2209.h> //https://github.com/janelia-arduino/TMC2209
#include <Wire.h>
#include <Preferences.h>
#include "esp_timer.h"
#include <math.h> // used only in readNTCTemperature for log()

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
#define SW1 35
#define SW2 36
#define VBUS 4
#define NTC 7
#define DISABLE_3V3 38

const int MAX_CAN_ID = 31;

CanFrame rxFrame;
CanFrame txFrame;

// ---------------- FIRMWARE VERSION ----------------------------------------------------------
float firmwareVersion = 0.06;
// --------------------------------------------------------------------------------------------

// ---------------- Stepper Driver ---------------
TMC2209 stepper_driver;
HardwareSerial & serial_stream = Serial2;
const long SERIAL_BAUD_RATE = 115200;

// ---------------- Encoder ---------------
int64_t total_encoder_counts = 0;
double angle = 0.0; // degrees for telemetry (kept as float for telemetry payload)
int encoder_offset = 0; //initial offset (if enables)

// ---------------- Scheduling ---------------
unsigned long lastFreq1 = 0;
unsigned long lastFreq2 = 0;

// -------------------- Fixed-point (Q16) Motion Control Structures --------------------
// Q16 fixed point helpers
static const int32_t Q = 16;
static const int32_t ONE_Q = 1 << Q;            // 65536
static const int64_t ONE_Q64 = (int64_t)ONE_Q;

struct MotionState {
    // velocities/accels stored in Q16 fixed-point (steps/sec * 65536)
    volatile int32_t currentSpeed_q;     // Q16 steps/sec
    volatile int32_t targetSpeed_q;      // Q16 steps/sec
    volatile int32_t accelSteps_q;       // Q16 steps/sec^2
    volatile int32_t decelSteps_q;       // Q16 steps/sec^2

    volatile int64_t targetPos;          // Target Position in steps (for Position Mode)
    volatile bool isRunning;             // Is the step timer active?
};

volatile MotionState motion = {0,0,0,0,0,false};

// Position control (main-thread copies, used for UI/telemetry)
bool posControl = 1; // 0 = Velocity Mode, 1 = Position Mode

// ISR / Timer state (shared with ISR)
volatile int64_t isr_currentPos = 0;   // current logical step count (in microsteps)
volatile uint32_t isr_stepDelay_us = 0; // period between steps (0 = stopped)
volatile bool isr_stepToggle = false;  // step line toggle state
volatile bool isr_dirState = false;    // The specific direction pin state to write

// Adjustable deadband for position mode (avoids oscillation at target)
volatile int32_t isr_deadband = 2; 

// ---------------- Settings from EEPROM ---------------
unsigned int NODE_ID = 1;  // Change per node (0-31)

unsigned int microsteps = 16;
unsigned int current = 30;
unsigned int stallThresh = 10;
unsigned int stepsPerRev = 200;
volatile unsigned int controlType = 0; // 0=Open Loop, 1=Closed Loop
unsigned int standstillMode = 0;
bool mapDirection = 0;
float posSpeed = 1080.0f; // Max Speed (deg/sec) - human-friendly
float accel = 720.0f;    // Acceleration (deg/sec^2) - human-friendly
float decel = 720.0f;    // Deceleration (deg/sec^2) - human-friendly
unsigned int reportFreq1 = 10;
unsigned int reportFreq2 = 1;
bool enableOnBoot = 1;
bool LED3V3Disable = 0;
bool zeroEncAtBoot = 0;
uint8_t storedAUXPayload[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //AUX config (full CAN payload)

// ---------------- Variables for storing current values for RTR reporting & others ---------------
int64_t posSetpoint = 0;
float velocitySetpoint = 0;
bool driverEnabled = 0;
bool ledState = 0;

bool VbusState = 0;
uint16_t AUX1Function = 0;
uint16_t AUX2Function = 0;

// ---------------- Button state tracking (for change-on-event telemetry) ---------------
bool lastSW1State = HIGH; // HIGH = not pressed (buttons read LOW when pressed)
bool lastSW2State = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// ---------------- Timers and critical section ---------------
static esp_timer_handle_t stepTimer = nullptr;
static esp_timer_handle_t motionTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Forward declarations ---------------
void IRAM_ATTR onStepTimer(void* arg);   // One-Shot pulse generator
void IRAM_ATTR onMotionTimer(void* arg); // 1000Hz Ramp Calculator
void readEncoder(); 
void canSendTelemetry(uint8_t msgType, const void *data, uint8_t size);
void forwardTelemetryToSerial(const CanFrame &frame);
void executeCommand(uint8_t targetNode, uint8_t msgType, const uint8_t *payload, uint8_t len, uint8_t rtr);
void handleCANFrame(const CanFrame &frame);
void handleSerialInput();
void handleAuxSerialInput();
void sendCANFrame(unsigned long canId, uint8_t *payload, uint8_t len, bool rtr);
void readSettings();
void writeSettings();
void recalcMotionParams();
float readInputVoltage();
float readNTCTemperature();
void checkButtons();
void handleSerialInputFrom(Stream &port);

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

    // start I2C
    Wire.begin(SDA, SCL);

    // read from EEPROM
    readSettings();

    recalcMotionParams(); // Calculate initial acceleration in steps/s^2 (Q16)

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

    // Stepper setup (using values from EEPROM)
    stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
    stepper_driver.setRunCurrent(current);
    stepper_driver.setMicrostepsPerStep(microsteps);
    stepper_driver.setStallGuardThreshold(stallThresh);
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.enableStealthChop(); // stealth chop needs to be enabled for stall detect
    stepper_driver.setCoolStepDurationThreshold(5000); // TCOOLTHRS
    //set standstill mode
    if (standstillMode == 0){ stepper_driver.setStandstillMode(stepper_driver.NORMAL);}
    else if (standstillMode == 1){ stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);}
    else if (standstillMode == 2){ stepper_driver.setStandstillMode(stepper_driver.BRAKING);}
    else if (standstillMode == 3){ stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);}
    if (enableOnBoot){ 
      stepper_driver.enable(); 
      driverEnabled = 1;
      } else { 
        stepper_driver.disable(); 
        driverEnabled = 0;
        }

    // --- TIMERS SETUP ---

    // 1. Motion Profile Timer (Periodic 1000Hz)
    // This calculates the S-curve/Trapezoid ramp
    esp_timer_create_args_t motion_timer_args = {
        .callback = &onMotionTimer,
        .arg = NULL,
        .name = "motion_loop",
    };
    esp_timer_create(&motion_timer_args, &motionTimer);
    
    // Start Motion Loop (1ms interval = 1000Hz)
    esp_timer_start_periodic(motionTimer, 1000); 

    // 2. Step Pulse Timer (One-Shot)
    // This fires once per step, toggles pin, and re-arms itself
    esp_timer_create_args_t step_timer_args = {
        .callback = &onStepTimer,
        .arg = NULL,
        .name = "step_pulse",
    };
    esp_timer_create(&step_timer_args, &stepTimer);

    //ADC setup
    analogSetPinAttenuation(VBUS, ADC_11db);
    analogSetPinAttenuation(NTC, ADC_11db);

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
}

// -------------------- Main Loop --------------------
void loop() {

    // Send angle at chosen frequency
    if (((millis() - lastFreq1) > 1000/reportFreq1) & (reportFreq1 != 0)){
      readEncoder(); // may need to do this more often depending on report freq
      angle = ((double)total_encoder_counts) * 360.0 / 16384.0; // get degrees from encoder count
      canSendTelemetry(33, &angle, sizeof(angle));  // msgType 33
      lastFreq1 = millis();

      //send Current Velocity:
      float currentVelocity = ((float)motion.currentSpeed_q / 65536.0f) * (360.0f / (stepsPerRev * microsteps)); //convert from Q16 int to deg/s float
      canSendTelemetry(42, &currentVelocity, sizeof(currentVelocity));  // msgType 42

    }

    // Secondary periodic CAN ouptuts
    if (((millis() - lastFreq2) > 1000/reportFreq2) & (reportFreq2 != 0)){
      //Send voltage:
      float voltage = readInputVoltage();
      canSendTelemetry(34, &voltage, sizeof(voltage));  // msgType 34

      //send NTC temp:
      float NTCtemp = readNTCTemperature();
      canSendTelemetry(38, &NTCtemp, sizeof(NTCtemp));  // msgType 38

      //send ESP temp:
      float ESPtemp = temperatureRead();
      canSendTelemetry(41, &ESPtemp, sizeof(ESPtemp));  // msgType 41

      //send Firmware version:
      canSendTelemetry(40, &firmwareVersion, sizeof(firmwareVersion));  // msgType 40
      

      // //send Current Velocity:
      // float currentVelocity = ((float)motion.currentSpeed_q / 65536.0f) * (360.0f / (stepsPerRev * microsteps)); //convert from Q16 int to deg/s float
      // canSendTelemetry(42, &currentVelocity, sizeof(currentVelocity));  // msgType 42
      
      lastFreq2 = millis();
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

    // Check for button state changes and send telemetry on change
    checkButtons();

    //need to make this nicer
    if ((readInputVoltage() < 4.0) && (VbusState == 1)){ //disable driver as VBUS gone below threshold
      digitalWrite(TMC_EN, HIGH);
      VbusState = 0;
      Serial.println("Driver hardware disabled");
    } else if ((readInputVoltage() >= 4.0) && (VbusState == 0)) { //enable driver as VBUS has come back
      delay(100); //give time to stablize
      stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
      stepper_driver.setRunCurrent(current);
      stepper_driver.setMicrostepsPerStep(microsteps);
      stepper_driver.setStallGuardThreshold(stallThresh);
      stepper_driver.enableAutomaticCurrentScaling();
      stepper_driver.enableStealthChop(); // stealth chop needs to be enabled for stall detect
      stepper_driver.setCoolStepDurationThreshold(5000); // TCOOLTHRS
      if (standstillMode == 0){ stepper_driver.setStandstillMode(stepper_driver.NORMAL);}
      else if (standstillMode == 1){ stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);}
      else if (standstillMode == 2){ stepper_driver.setStandstillMode(stepper_driver.BRAKING);}
      else if (standstillMode == 3){ stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);}
      if (driverEnabled){
        stepper_driver.enable(); 
      } else {
        stepper_driver.disable();
      }
      digitalWrite(TMC_EN, LOW);
      VbusState = 1;
      Serial.println("Driver hardware enabled");
      
    }

    // -------------------- Closed-loop sync --------------------
    if (posControl && controlType == 1) { // closed-loop position mode
        // Read encoder
        readEncoder();
        // Convert encoder counts to microsteps
        int64_t encoderSteps = (int64_t)total_encoder_counts * stepsPerRev * microsteps / 16384;
        
        // Update ISR current position from encoder
        // CRITICAL: We only sync if there is a discrepancy to avoid jittering the ramp math
        portENTER_CRITICAL(&timerMux);
        int64_t diff = isr_currentPos - encoderSteps;
        if (llabs(diff) > 10) { 
             isr_currentPos = encoderSteps;
        }
        portEXIT_CRITICAL(&timerMux);
    }

    // Small yield to keep system responsive
    delay(1);
}

// -------------------- Motion Control Loop (1000Hz) --------------------
// Integer (Q16) implementation
void IRAM_ATTR onMotionTimer(void* arg) {
    // Use local copies outside critical section where possible
    int32_t v_q;          // Q16 current speed
    int32_t accel_q;
    int32_t decel_q;
    int32_t targetSpeed_q;
    int32_t local_posSpeed_steps_q; // Q16 max steps/sec
    int64_t targetPos_local;
    bool local_posControl;
    bool local_mapDirection;
    int32_t local_deadband;

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
    // compute maxV in Q16
    float maxV_steps = (posSpeed / 360.0f) * (float)stepsPerRev * (float)microsteps;
    local_posSpeed_steps_q = (int32_t)roundf(maxV_steps * (float)ONE_Q);
    portEXIT_CRITICAL_ISR(&timerMux);

    int32_t targetV_q = 0;

    // --- 1. Determine Target Velocity (Q16) ---
    if (local_posControl) {
        // position mode
        int64_t error = targetPos_local - isr_currentPos;
        int64_t absErr64 = llabs(error);

        if (absErr64 <= local_deadband) {
            targetV_q = 0;
        } else {
            // stopping distance = v^2 / (2*a), or 0 if decel is disabled (instant stop)
            // When accel is 0 the motor will instantly jump to full speed, so use
            // local_posSpeed_steps_q for the stop distance calculation rather than
            // the current v_q (which may still be 0), to avoid overshoot oscillation.
            int32_t v_for_stopdist = (accel_q == 0) ? local_posSpeed_steps_q : abs(v_q);
            int64_t v2 = (int64_t)v_for_stopdist * (int64_t)v_for_stopdist; // Q32
            int64_t denom_q = ((int64_t)decel_q << 1); // Q16
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
        // velocity mode: targetSpeed_q set by commands (already in Q16)
        targetV_q = targetSpeed_q;
    }

    // --- 2. Apply Acceleration/Deceleration (Q16 arithmetic) ---
    // If accel or decel is 0, velocity snaps instantly to target (no ramping).
    // dt_q = ONE_Q / 1000 for 1 kHz loop
    const int32_t dt_q = ONE_Q / 1000; // 65
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
        digitalWrite(DIR, isr_dirState); // Set Dir immediately
        // First event: schedule rising edge after half-period (we emulate previous behavior)
        // we rearm stepTimer with half delay (but ensure min 20us)
        uint32_t first_delay = isr_stepDelay_us / 2;
        if (first_delay < 20) first_delay = 20;
        esp_timer_start_once(stepTimer, first_delay);
    }
}

// -------------------- Step ISR (One-Shot) --------------------
// Fires when a step is due.
void IRAM_ATTR onStepTimer(void* arg) {
    portENTER_CRITICAL_ISR(&timerMux);

    if (isr_stepDelay_us == 0) {
        motion.isRunning = false;
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    isr_stepToggle = !isr_stepToggle;

    if (isr_stepToggle) {
        // Rising edge: STEP = HIGH

        // DIR pin
        if (isr_dirState) {
            REG_WRITE(GPIO_OUT_W1TS_REG, DIR_MASK);   // DIR = HIGH
        } else {
            REG_WRITE(GPIO_OUT_W1TC_REG, DIR_MASK);   // DIR = LOW
        }

        // STEP high
        REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);

        // logical position update
        bool logicalUp = (isr_dirState == HIGH);
        if (mapDirection) logicalUp = !logicalUp;

        if (logicalUp) isr_currentPos++;
        else           isr_currentPos--;

    } else {
        // Falling edge: STEP = LOW
        REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
    }

    // Re-arm timer
    uint64_t next_delay = isr_stepDelay_us / 2;
    if (next_delay < 20) next_delay = 20;

    esp_timer_start_once(stepTimer, next_delay);

    portEXIT_CRITICAL_ISR(&timerMux);
}

// -------------------- Helpers --------------------
void recalcMotionParams() {
    // Converts user-friendly Deg/s^2 into Steps/s^2 and stores Q16
    float usteps = (float)microsteps;
    float revs = (float)stepsPerRev;
    float accel_steps = (accel / 360.0f) * revs * usteps;
    float decel_steps = (decel / 360.0f) * revs * usteps;

    // store in Q16
    portENTER_CRITICAL(&timerMux);
    motion.accelSteps_q = (int32_t)roundf(accel_steps * (float)ONE_Q);
    motion.decelSteps_q = (int32_t)roundf(decel_steps * (float)ONE_Q);

    // Also compute posSpeed->targetSpeed_q (max steps/sec Q16)
    float maxV_steps = (posSpeed / 360.0f) * revs * usteps;
    motion.targetSpeed_q = (int32_t)roundf(maxV_steps * (float)ONE_Q);
    portEXIT_CRITICAL(&timerMux);
}

float readInputVoltage() {
    const float SCALE = (47.0 + 4.7) / 4.7;   // ≈ 11.0
    int mv = analogReadMilliVolts(VBUS);      // calibrated ADC voltage (mV)
    return (mv / 1000.0) * SCALE;             // return actual input voltag
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
                  
                  // Convert to steps/sec for the motion loop then to Q16
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
                  if(driverEnabled) {
                      stepper_driver.enable();
                  } else {
                      stepper_driver.disable();
                      // Stop motion
                      portENTER_CRITICAL(&timerMux);
                      motion.targetSpeed_q = 0;
                      motion.currentSpeed_q = 0;
                      isr_stepDelay_us = 0;
                      portEXIT_CRITICAL(&timerMux);
                  }
              }
              break;
  
          case 6: // Emergency Stop
              Serial.println("Executing Emergency Stop");
              stepper_driver.disable();
              portENTER_CRITICAL(&timerMux);
              motion.targetSpeed_q = 0;
              motion.currentSpeed_q = 0;
              isr_stepDelay_us = 0; // Immediate stop
              portEXIT_CRITICAL(&timerMux);
              break;
          
          case 8: // Zero Enc at boot (bool)
              if(len >= 1) {
                  zeroEncAtBoot = payload[0] != 0;
                  Serial.printf("Zero Enc at boot set to: %d\n", zeroEncAtBoot);
              }
              break;
  
          case 9: // LED control
              if(len >= 1) {
                  ledState = payload[0] != 0;
                  Serial.printf("LED Command: %d\n", ledState);
                  digitalWrite(LED1, ledState);
              }
              break;
  
          case 10: // Set Steps per rev (uint16)
              if(len >= 2) {
                  stepsPerRev = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Steps per rev set to: %u\n", stepsPerRev);
                  recalcMotionParams();
              }
              break;
  
          case 11: // microsteps (uint16)
              if(len >= 2) {
                  microsteps = payload[0] | (payload[1] << 8); // little endian
                  Serial.printf("Microsteps set to: %u\n", microsteps);
                  stepper_driver.setMicrostepsPerStep(microsteps);
                  recalcMotionParams();
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
                   int64_t encoderSteps = (int64_t)total_encoder_counts * stepsPerRev * microsteps / 16384;
                   isr_currentPos = encoderSteps;
                   motion.targetPos = encoderSteps; // Cancel any current move
                   portEXIT_CRITICAL(&timerMux);
               }
               break;
  
          case 16: // Position speed (deg/sec) (float) - Max Speed
              if(len >= 4) {
                  memcpy(&posSpeed, payload, 4);
                  Serial.printf("Position speed updated to: %.2f deg/sec\n", posSpeed);
                  // Update Q16 internal max speed
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
  
          case 24: // save config
              writeSettings();
              Serial.println("Config Saved");
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

          case 8:
              canSendTelemetry(8, &zeroEncAtBoot, sizeof(zeroEncAtBoot));
              break;

          case 9:
              canSendTelemetry(9, &ledState, sizeof(ledState));
              break;

          case 10:
              canSendTelemetry(10, &stepsPerRev, sizeof(stepsPerRev));
              break;

          case 11:
              canSendTelemetry(11, &microsteps, sizeof(microsteps));
              break;

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

    // Forward all non RTR messages to serial
    if(rtr == 0 && Serial) {
        // Only forward telemetry over serial if serial is connected
        forwardTelemetryToSerial(frame);     // ENABLE to pass telem over serial
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
}

// -------------------- Serial to CAN --------------------
void handleSerialInput() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

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

//Read saved settings from EEPROM
void readSettings(){
  preferences.begin("settings", false); //open the settings namespace

  NODE_ID = preferences.getUInt("NODE_ID", NODE_ID);
  microsteps = preferences.getUInt("microsteps", microsteps);
  current = preferences.getUInt("current", current);
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
  preferences.putBool("LED3V3Disable", LED3V3Disable);
  preferences.putBool("zeroEncAtBoot", zeroEncAtBoot);
  preferences.putBytes("storedAUX", storedAUXPayload, 8);

  preferences.end();
}
