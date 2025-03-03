// Include all necessary libraries
#include <Arduino.h>
#include "Display_ST7789.h"
#include "LVGL_Driver.h"
#include "ui.h"
#include "screens.h"
#include "RotaryEncoder.h"

// Include our motor driver abstraction
#include "StepperDriver.h"
#include "L298NDriver.h"
#include "DRV8825Driver.h"
#include "TimerStepperControl.h"

//===============================================
// DRIVER CONFIGURATION
//===============================================
// Set this to choose which driver to use
#define USE_L298N_DRIVER  0  // Set to 1 to use L298N
#define USE_DRV8825_DRIVER 1 // Set to 1 to use DRV8825

// L298N Pin definitions
#define L298N_PIN1 1      // Connected to IN1 on L298N
#define L298N_PIN2 4      // Connected to IN2 on L298N
#define L298N_PIN3 19     // Connected to IN3 on L298N
#define L298N_PIN4 20     // Connected to IN4 on L298N
#define L298N_ENABLE_A 18 // Connected to ENA on L298N
#define L298N_ENABLE_B 23 // Connected to ENB on L298N

// DRV8825 Pin definitions
#define DRV8825_ENABLE_PIN 9  // Enable pin
#define DRV8825_M0_PIN 18      // Microstep mode 0
#define DRV8825_M1_PIN 19      // Microstep mode 1
#define DRV8825_M2_PIN 20       // Microstep mode 2
#define DRV8825_SLEEP_RESET_PIN 23 // GPIO23 controls both SLEEP and RESET
#define DRV8825_STEP_PIN 12        // STEP
#define DRV8825_DIR_PIN 13         // Direction
#define DRV8825_SLEEP_PIN -1   // Optional - connect if needed
#define DRV8825_RESET_PIN -1   // Optional - connect if needed
#define DRV8825_FAULT_PIN -1   // Optional - connect if needed

//===============================================
// MOTOR CONFIGURATION
//===============================================
// Base motor characteristics
#define BASE_STEPS_PER_REVOLUTION 200    // Standard for NEMA 17 (1.8° per step)
#define DEFAULT_MICROSTEP_MODE 8         // Using 1/8 microstepping for smooth operation

// Gear ratio and transmission settings
float gearRatio = 5.0;                   // Default 5:1 gear ratio

//===============================================
// MOTION PARAMETERS
//===============================================
// Speed limits in RPM (at the output shaft after gearing)
#define MIN_RPM 0.1                      // Minimum speed in RPM
#define MAX_RPM 60.0                     // Maximum speed in RPM
#define DEFAULT_RPM 5.0                  // Default starting speed in RPM
#define RPM_FINE_ADJUST 0.1              // Fine adjustment increment in RPM
#define RPM_COARSE_ADJUST 1.0            // Coarse adjustment increment in RPM

// Step/rotation limits
#define MIN_ROTATION_PERCENT 1.0         // Minimum rotation (1% of a full turn)
#define MAX_ROTATION_PERCENT 1000.0      // Maximum rotation (10 full turns)
#define DEFAULT_ROTATION_PERCENT 100.0   // Default rotation (1 full turn)
#define ROTATION_FINE_ADJUST 1.0         // Fine adjustment increment (1% of rotation)
#define ROTATION_COARSE_ADJUST 5.0       // Coarse adjustment increment (5% of rotation)

// Acceleration and timing
#define DEFAULT_ACCELERATION 400         // Steps per second per second
#define MOTOR_IDLE_TIMEOUT_MS 5000       // 5 seconds before motor power saving

//===============================================
// ENCODER AND UI SETTINGS
//===============================================
// Encoder sensitivity
#define ENCODER_FINE_SENSITIVITY 1       // For fine adjustments
#define ENCODER_COARSE_SENSITIVITY 3     // For coarse adjustments
#define ENCODER_JOG_STEP_MULTIPLIER 4    // Multiplier for steps per encoder tick in jog mode

//===============================================
// ENCODER JOG CONFIGURATION
//===============================================
bool encoderJogMode = false;
long lastJogEncoderValue = 0;            // Track encoder position for jog mode

//===============================================
// GLOBAL VARIABLES
//===============================================
// Create the appropriate driver and controller
#if USE_L298N_DRIVER
    L298NDriver driver(L298N_PIN1, L298N_PIN2, L298N_PIN3, L298N_PIN4, L298N_ENABLE_A, L298N_ENABLE_B);
#elif USE_DRV8825_DRIVER
    DRV8825Driver driver(DRV8825_STEP_PIN, DRV8825_DIR_PIN, DRV8825_ENABLE_PIN, 
                         DRV8825_M0_PIN, DRV8825_M1_PIN, DRV8825_M2_PIN,
                         DRV8825_SLEEP_RESET_PIN);
#else
    #error "No driver selected! Set either USE_L298N_DRIVER or USE_DRV8825_DRIVER to 1"
#endif

// Create the timer-based controller with the selected driver
TimerStepperControl controller(&driver);

// Motor operation state
bool motorRunning = false;
bool continuousMode = false;
bool clockwiseDirection = true;
bool enableMotorPowerSave = true;
bool fineAdjustmentMode = true;          // Toggle between fine/coarse adjustment

// Current settings (these get converted to/from user-friendly units)
int targetSteps;                         // Target steps to move
int speedSetting;                        // Current speed in steps/sec

// Timing variables
unsigned long lastMotorActivityTime = 0;

// UI state tracking
bool valueAdjustmentMode = false;        // Whether we're in value adjustment mode
lv_obj_t *currentAdjustmentObject = NULL; // Currently selected UI element for adjustment
int adjustmentSensitivity = ENCODER_FINE_SENSITIVITY; // How much to change per encoder tick

// Forward declarations of event handlers
void on_move_steps_start_clicked();
void on_move_steps_direction_clicked();
void on_move_steps_speed_clicked();
void on_move_steps_steps_clicked();
void on_manual_jog_start_clicked();
void on_manual_jog_speed_clicked();
void on_continuous_rotation_start_clicked();
void on_continuous_rotation_direction_clicked();
void on_continuous_rotation_speed_clicked();
void on_back_clicked();

//===============================================
// MOTOR CONTROL FUNCTIONS
//===============================================
void startStepperMotion(int steps, bool clockwise, int speed) {
    // If motor is already running, stop it
    if (motorRunning) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_STOP_MOTOR;
        controller.sendCommand(&cmd);
        motorRunning = false;
        update_ui_labels();
        return;
    }
    
    // Wake the driver if it's using DRV8825
    #if USE_DRV8825_DRIVER
    controller.wake();
    #endif
    
    // Set direction and steps
    int relativeSteps = clockwise ? steps : -steps;
    
    // Send command to move
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_MOVE_STEPS;
    cmd.position = relativeSteps;
    cmd.speed = speed;
    cmd.direction = clockwise;
    controller.sendCommand(&cmd);
    
    // Remember state
    continuousMode = false;
    motorRunning = true;
    lastMotorActivityTime = millis();
    
    // Update UI to reflect motor running state
    update_ui_labels();
    
    Serial.print("Starting stepper motion: ");
    Serial.print(steps);
    Serial.print(" steps, direction: ");
    Serial.print(clockwise ? "clockwise" : "counterclockwise");
    Serial.print(", speed: ");
    Serial.println(speed);
}

void startContinuousRotation(bool clockwise, int speed) {
    // If motor is already running, stop it
    if (motorRunning) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_STOP_MOTOR;
        controller.sendCommand(&cmd);
        motorRunning = false;
        update_ui_labels();
        return;
    }
    
    // Wake the driver if it's using DRV8825
    #if USE_DRV8825_DRIVER
    controller.wake();
    #endif
    
    // Send command for continuous rotation
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_START_CONTINUOUS;
    cmd.speed = speed;
    cmd.direction = clockwise;
    controller.sendCommand(&cmd);
    
    // Remember state
    continuousMode = true;
    motorRunning = true;
    lastMotorActivityTime = millis();
    
    // Update UI to reflect motor running state
    update_ui_labels();
    
    Serial.print("Starting continuous rotation, direction: ");
    Serial.print(clockwise ? "clockwise" : "counterclockwise");
    Serial.print(", speed: ");
    Serial.println(speed);
}

void stopMotor() {
    // Send stop command
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_STOP_MOTOR;
    controller.sendCommand(&cmd);
    
    motorRunning = false;
    
    // Put driver to sleep to save power if power save is enabled
    if (enableMotorPowerSave) {
        #if USE_DRV8825_DRIVER
        controller.sleep();
        #endif
    }
    
    // Update UI to reflect motor stopped state
    update_ui_labels();
    
    Serial.println("Motor stopped");
}

// Helper functions for unit conversions
int getEffectiveStepsPerRevolution() {
    #if USE_DRV8825_DRIVER
    // Get current microstepping mode from the driver
    int microstepMode = driver.getMicrostepMode();
    return BASE_STEPS_PER_REVOLUTION * microstepMode;
    #else
    // For drivers without microstepping, use base steps
    return BASE_STEPS_PER_REVOLUTION;
    #endif
}

float stepsToRPM(int stepsPerSecond, float ratio) {
    // Convert steps/second to RPM at the output shaft
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (stepsPerSecond * 60.0) / (effectiveSteps * ratio);
}

float rpmToSteps(float rpm, float ratio) {
    // Convert RPM to steps/second
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (rpm * effectiveSteps * ratio) / 60.0f;
}

int safeRoundStepsPerSec(float stepsPerSec) {
    if (stepsPerSec <= 0) return 0;  // Handle zero/negative case
    return max(1, (int)round(stepsPerSec));  // Never round down to zero
}

float stepsToRotationPercent(int steps, float ratio) {
    // Convert steps to percentage of output shaft rotation
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (steps * 100.0) / (effectiveSteps * ratio);
}

int rotationPercentToSteps(float percent, float ratio) {
    // Convert percentage of rotation to steps
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (percent * effectiveSteps * ratio) / 100.0;
}

// Function to adjust values (speed, steps) when in adjustment mode
void adjustValueByEncoder(lv_obj_t* obj, int delta) {
    // Determine sensitivity based on fine/coarse mode
    float speedAdjustment = fineAdjustmentMode ? RPM_FINE_ADJUST : RPM_COARSE_ADJUST;
    float rotationAdjustment = fineAdjustmentMode ? ROTATION_FINE_ADJUST : ROTATION_COARSE_ADJUST;
    
    // Apply proper magnitude based on delta and sensitivity
    float speedDelta = delta * speedAdjustment;
    float rotationDelta = delta * rotationAdjustment;
    
    // Check which value is being adjusted and modify accordingly
    if (obj == objects.step_num) {
        // Work with percentage of rotation instead of steps
        float currentPercent = stepsToRotationPercent(targetSteps, gearRatio);
        currentPercent += rotationDelta;
        
        // Apply bounds
        if (currentPercent < MIN_ROTATION_PERCENT) {
            currentPercent = MIN_ROTATION_PERCENT;
        } else if (currentPercent > MAX_ROTATION_PERCENT) {
            currentPercent = MAX_ROTATION_PERCENT;
        }
        
        // Convert back to steps
        targetSteps = rotationPercentToSteps(currentPercent, gearRatio);
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Rot: %.1f%%", currentPercent);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        Serial.print("Rotation adjusted to: ");
        Serial.print(currentPercent);
        Serial.println("%");
    }
    else if (obj == objects.speed || obj == objects.speed_manual_jog || obj == objects.continuous_rotation_speed_button) {
        // Work with RPM instead of steps/second
        // Work with RPM instead of steps/second
        float currentRPM = stepsToRPM(speedSetting, gearRatio);
        
        // Apply delta to RPM
        currentRPM += speedDelta;
        // Apply bounds
        if (currentRPM < MIN_RPM) {
            currentRPM = MIN_RPM;
        } else if (currentRPM > MAX_RPM) {
            currentRPM = MAX_RPM;
        }
        
        // Convert back to steps/second
        speedSetting = safeRoundStepsPerSec(rpmToSteps(currentRPM, gearRatio));
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", currentRPM);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        // Update running speed if motor is already running in continuous mode
        if (motorRunning && continuousMode) {
            MotorCommand_t cmd;
            cmd.cmd_type = CMD_SET_SPEED;
            cmd.speed = speedSetting;
            controller.sendCommand(&cmd);
        }
        
        Serial.print("Speed adjusted to: ");
        Serial.print(currentRPM);
        Serial.println(" RPM");
    }
}

// Function to handle encoder jog mode
void checkEncoderJogMode() {
    // Update last activity time
    lastMotorActivityTime = millis();
  
    // Exit jog mode if encoder button is pressed
    if (buttonPressed) {
        buttonPressed = false;
        encoderJogMode = false;
        motorRunning = false;
        stopMotor();
        update_ui_labels();
        Serial.println("Exiting encoder jog mode via button press");
        return;
    }
  
    // Check for encoder movement
    if (encoderValue != lastJogEncoderValue) {
        // Calculate movement
        long delta = encoderValue - lastJogEncoderValue;
        lastJogEncoderValue = encoderValue;
        
        // Scale movement by sensitivity factor - use ENCODER_JOG_STEP_MULTIPLIER instead
        int moveSteps = delta * ENCODER_JOG_STEP_MULTIPLIER;
        
        // Apply direction setting from UI
        if (!clockwiseDirection) {
            moveSteps = -moveSteps;
        }
        
        // Only move if there's significant encoder change
        if (moveSteps != 0) {
            // Send command to move steps
            MotorCommand_t cmd;
            cmd.cmd_type = CMD_MOVE_STEPS;
            cmd.position = moveSteps;
            cmd.speed = speedSetting;
            controller.sendCommand(&cmd);
            
            Serial.print("Encoder jog: ");
            Serial.print(moveSteps);
            Serial.println(" steps");
        }
    }
}

//===============================================
// UI FUNCTIONS
//===============================================
// LVGL event handler function that routes to the appropriate handler
static void ui_event_handler(lv_event_t *e) {
    lv_obj_t *target = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        // Main Screen
        if (target == objects.move_steps) {
            loadScreen(SCREEN_ID_MOVE_STEPS_PAGE);
        } 
        else if (target == objects.manual_jog) {
            loadScreen(SCREEN_ID_MANUAL_JOG_PAGE);
        } 
        else if (target == objects.continuous) {
            loadScreen(SCREEN_ID_CONTINUOUS_ROTATION_PAGE);
        }
        
        // Move Steps Page
        else if (target == objects.back) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.start) {
            on_move_steps_start_clicked();
        }
        else if (target == objects.step_num) {
            on_move_steps_steps_clicked();
        }
        else if (target == objects.clockwise) {
            on_move_steps_direction_clicked();
        }
        else if (target == objects.speed) {
            on_move_steps_speed_clicked();
        }
        
        // Manual Jog Page
        else if (target == objects.back_1) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.start_1) {
            on_manual_jog_start_clicked();
        }
        else if (target == objects.speed_manual_jog) {
            on_manual_jog_speed_clicked();
        }
        
        // Continuous Rotation Page
        else if (target == objects.back_2) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.continuous_rotation_start_button) {
            on_continuous_rotation_start_clicked();
        }
        else if (target == objects.continuous_rotation_direction_button) {
            on_continuous_rotation_direction_clicked();
        }
        else if (target == objects.continuous_rotation_speed_button) {
            on_continuous_rotation_speed_clicked();
        }
    }
}

// Function to attach event handlers to UI elements
void attach_event_handlers() {
    // Main Screen
    lv_obj_add_event_cb(objects.move_steps, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.manual_jog, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Move Steps Page
    lv_obj_add_event_cb(objects.back, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.start, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.step_num, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.clockwise, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.speed, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Manual Jog Page
    lv_obj_add_event_cb(objects.back_1, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.start_1, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.speed_manual_jog, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Continuous Rotation Page
    lv_obj_add_event_cb(objects.back_2, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_start_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_direction_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_speed_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
}

// Function to update UI labels based on current settings
void update_ui_labels() {
    // Update direction button labels
    lv_obj_t *clockwise_label = lv_obj_get_child(objects.clockwise, 0);
    if (clockwise_label) {
        lv_label_set_text(clockwise_label, clockwiseDirection ? "Clockwise" : "Counter-CW");
    }
    
    lv_obj_t *direction_label = lv_obj_get_child(objects.continuous_rotation_direction_button, 0);
    if (direction_label) {
        lv_label_set_text(direction_label, clockwiseDirection ? "Clockwise" : "Counter-CW");
    }
    
    // Update step number button label - now showing as % of rotation
    lv_obj_t *steps_label = lv_obj_get_child(objects.step_num, 0);
    if (steps_label) {
        char buffer[20];
        float percentRotation = stepsToRotationPercent(targetSteps, gearRatio);
        snprintf(buffer, sizeof(buffer), "Rot: %.1f%%", percentRotation);
        lv_label_set_text(steps_label, buffer);
    }
    
    // Calculate RPM from current speed setting
    float rpm = stepsToRPM(speedSetting, gearRatio);
    
    // Update speed button labels - now showing as RPM
    lv_obj_t *speed_label = lv_obj_get_child(objects.speed, 0);
    if (speed_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_label, buffer);
    }
    
    lv_obj_t *speed_manual_label = lv_obj_get_child(objects.speed_manual_jog, 0);
    if (speed_manual_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_manual_label, buffer);
    }
    
    lv_obj_t *speed_cont_label = lv_obj_get_child(objects.continuous_rotation_speed_button, 0);
    if (speed_cont_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_cont_label, buffer);
    }
    
    // Update start button labels based on motor state
    lv_obj_t *start_label = lv_obj_get_child(objects.start, 0);
    if (start_label) {
        lv_label_set_text(start_label, motorRunning ? "Stop" : "Start");
    }
    
    lv_obj_t *start_manual_label = lv_obj_get_child(objects.start_1, 0);
    if (start_manual_label) {
        lv_label_set_text(start_manual_label, motorRunning ? "Stop" : "Start");
    }
    
    lv_obj_t *start_cont_label = lv_obj_get_child(objects.continuous_rotation_start_button, 0);
    if (start_cont_label) {
        lv_label_set_text(start_cont_label, motorRunning ? "Stop" : "Start");
    }
}

//===============================================
// UI EVENT HANDLERS (Connected to LVGL UI)
//===============================================
// When Move Steps screen buttons are pressed
void on_move_steps_start_clicked() {
    // Toggle between start and stop
    if (motorRunning) {
        stopMotor();
    } else {
        startStepperMotion(targetSteps, clockwiseDirection, speedSetting);
    }
}

void on_move_steps_direction_clicked() {
    clockwiseDirection = !clockwiseDirection;
    update_ui_labels();
    Serial.print("Direction changed to: ");
    Serial.println(clockwiseDirection ? "clockwise" : "counterclockwise");
}

void on_move_steps_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
}

void on_move_steps_steps_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (targetSteps >= 1000) {
        targetSteps = 50;
    } else {
        targetSteps *= 2;
    }
    update_ui_labels();
    Serial.print("Steps set to: ");
    Serial.println(targetSteps);
}

// When Manual Jog screen buttons are pressed
void on_manual_jog_start_clicked() {
    // If already in encoder jog mode, exit it
    if (encoderJogMode) {
        encoderJogMode = false;
        motorRunning = false;
        stopMotor();
        update_ui_labels();
        return;
    }
    
    // If motor is running in standard mode, stop it
    if (motorRunning) {
        stopMotor();
        return;
    }
    
    // Enter encoder jog mode
    encoderJogMode = true;
    motorRunning = true;
    lastJogEncoderValue = encoderValue; // Start from current encoder position
    
    // Update UI
    lv_obj_t *start_manual_label = lv_obj_get_child(objects.start_1, 0);
    if (start_manual_label) {
        lv_label_set_text(start_manual_label, "Jogging...");
    }
    
    // Enable the motor for jogging
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_START_JOG;
    cmd.speed = speedSetting;
    controller.sendCommand(&cmd);
    
    Serial.println("Entering encoder jog mode");
}

void on_manual_jog_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
    
    // Update running speed if motor is already running
    if (motorRunning && continuousMode) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_SET_SPEED;
        cmd.speed = speedSetting;
        controller.sendCommand(&cmd);
    }
}

// When Continuous Rotation screen buttons are pressed
void on_continuous_rotation_start_clicked() {
    // Toggle between start and stop
    if (motorRunning) {
        stopMotor();
    } else {
        startContinuousRotation(clockwiseDirection, speedSetting);
    }
}

void on_continuous_rotation_direction_clicked() {
    clockwiseDirection = !clockwiseDirection;
    update_ui_labels();
    Serial.print("Direction changed to: ");
    Serial.println(clockwiseDirection ? "clockwise" : "counterclockwise");
    
    // Update running direction if motor is already running
    if (motorRunning && continuousMode) {
        // Stop current movement
        MotorCommand_t stopCmd;
        stopCmd.cmd_type = CMD_STOP_MOTOR;
        controller.sendCommand(&stopCmd);
        
        // Start with new direction
        MotorCommand_t startCmd;
        startCmd.cmd_type = CMD_START_CONTINUOUS;
        startCmd.direction = clockwiseDirection;
        startCmd.speed = speedSetting;
        controller.sendCommand(&startCmd);
    }
}

void on_continuous_rotation_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
    
    // Update running speed if motor is already running
    if (motorRunning && continuousMode) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_SET_SPEED;
        cmd.speed = speedSetting;
        controller.sendCommand(&cmd);
    }
}

// Common back button handler
void on_back_clicked() {
    // Stop the motor when going back to main screen
    stopMotor();
}

//===============================================
// SETUP & LOOP
//===============================================
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("Stepper Motor Controller - Hardware Timer Version");
    
    // Convert RPM to steps/sec for initial values
    speedSetting = safeRoundStepsPerSec(rpmToSteps(DEFAULT_RPM, gearRatio));
    targetSteps = rotationPercentToSteps(DEFAULT_ROTATION_PERCENT, gearRatio);
    
    // Initialize our timer-based motor controller
    controller.init();
    
    // Set microstepping mode for DRV8825 if used
    #if USE_DRV8825_DRIVER
    driver.setMicrostepMode(DEFAULT_MICROSTEP_MODE);
    // Explicitly wake the driver
    controller.wake();
    #endif
    
    // Initialize the display and UI
    LCD_Init();
    Set_Backlight(50); // Set LCD backlight to 50%
    Lvgl_Init();
    ui_init();
    
    // Attach event handlers to UI elements
    attach_event_handlers();
    
    // Initialize the rotary encoder
    setupEncoder();
    
    // Update UI labels with initial values
    update_ui_labels();
    
    Serial.println("System ready!");
}

void loop() {
    // Current time tracking
    unsigned long currentMillis = millis();
    
    // Handle UI updates
    Timer_Loop();
    ui_tick();
    
    // Handle encoder input (includes UI navigation and value adjustment)
    handleEncoder();
    
    // Check if a long press was detected for toggling fine/coarse adjustment
    if (longPressDetected) {
        longPressDetected = false;
        
        // Only toggle if in value adjustment mode
        if (valueAdjustmentMode) {
            toggleAdjustmentPrecision();
        }
    }
    
    if (motorRunning && encoderJogMode) {
        checkEncoderJogMode();
    }
    
    // Check for motor idle timeout - automatic shutdown after inactivity
    if (enableMotorPowerSave && motorRunning && 
        !encoderJogMode && !continuousMode && 
        controller.isRunning() == false &&
        currentMillis - lastMotorActivityTime > MOTOR_IDLE_TIMEOUT_MS) {
        // Motor has been idle too long, disable it
        motorRunning = false;
        controller.sleep();
        update_ui_labels();
    }

    // Poll for motor status updates (completed movements)
    if (motorRunning && !encoderJogMode && !controller.isRunning()) {
        motorRunning = false;
        Serial.println("Motor stopped (reached target)");
        update_ui_labels();
    }
}