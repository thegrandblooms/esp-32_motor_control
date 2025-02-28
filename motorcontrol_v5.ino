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
#include "StepperController.h"

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

// Motor parameters - Adjust these as needed
#define DEFAULT_MAX_SPEED 500       // Maximum steps per second
#define DEFAULT_ACCELERATION 400    // Steps per second per second
#define DEFAULT_RUNNING_SPEED 300   // Normal running speed
#define STEPS_PER_REVOLUTION 200    // Standard for NEMA 17 (1.8° per step)
#define DEFAULT_MICROSTEP_MODE 1    // Full step mode (can be 1, 2, 4, 8, 16, 32)

//===============================================
// ENCODER JOG CONFIGURATION
//===============================================
bool encoderJogMode = false;
long lastJogEncoderValue = 0;
const int ENCODER_JOG_SENSITIVITY = 5; // Adjust for desired responsiveness

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

// Create the controller with the selected driver
StepperController controller(&driver);

// State variables
bool motorRunning = false;
bool continuousMode = false;
bool clockwiseDirection = true;
int targetSteps = 200;
int speedSetting = DEFAULT_RUNNING_SPEED;
unsigned long lastMotorActivityTime = 0;
const unsigned long MOTOR_IDLE_TIMEOUT = 5000; // 5 seconds of inactivity before power saving
bool enableMotorPowerSave = true;  // Enable power saving mode

// Speed and step adjustment bounds
const int speedLowerBound = 1;   // Minimum speed (rpm)
const int speedUpperBound = 1000; // Maximum speed (rpm)
const int stepsLowerBound = 1;    // Minimum steps
const int stepsUpperBound = 10000; // Maximum steps

// For value adjustment mode
bool valueAdjustmentMode = false;
lv_obj_t *currentAdjustmentObject = NULL;
int adjustmentSensitivity = 2; // How much to change per encoder tick when adjusting a setting

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
        stopMotor();
        update_ui_labels();
        return;
    }
    
    // Wake the driver if it's asleep
    #if USE_DRV8825_DRIVER
    driver.wake();
    #endif
    
    // Set direction and steps
    int relativeSteps = clockwise ? steps : -steps;
    
    // Configure and start the motor
    controller.setMaxSpeed(speed + 100); // Max speed slightly higher than running speed
    controller.setSpeed(speed);
    controller.move(relativeSteps);
    
    // Remember we're in step mode
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
        stopMotor();
        update_ui_labels();
        return;
    }
    
    // Wake the driver if it's asleep
    #if USE_DRV8825_DRIVER
    driver.wake();
    #endif
    
    // Configure and start the motor
    controller.setMaxSpeed(speed + 100);
    controller.setSpeed(speed);
    controller.startContinuous(clockwise, speed);
    
    // Remember we're in continuous mode
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
  controller.stop();
  motorRunning = false;
  
  // Put driver to sleep to save power if power save is enabled
  if (enableMotorPowerSave) {
      #if USE_DRV8825_DRIVER
      driver.sleep();
      #endif
  }
  
  // Update UI to reflect motor stopped state
  update_ui_labels();
  
  Serial.println("Motor stopped");
}

// Function to adjust values (speed, steps) when in adjustment mode
void adjustValueByEncoder(lv_obj_t* obj, int delta) {
    // Check which value is being adjusted and modify accordingly
    if (obj == objects.step_num) {
        // Adjust steps
        targetSteps += delta;
        
        // Apply bounds
        if (targetSteps < stepsLowerBound) {
            targetSteps = stepsLowerBound;
        } else if (targetSteps > stepsUpperBound) {
            targetSteps = stepsUpperBound;
        }
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Steps: %d", targetSteps);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        Serial.print("Steps adjusted to: ");
        Serial.println(targetSteps);
    }
    else if (obj == objects.speed || obj == objects.speed_manual_jog || obj == objects.continuous_rotation_speed_button) {
        // Adjust speed
        speedSetting += delta;
        
        // Apply bounds
        if (speedSetting < speedLowerBound) {
            speedSetting = speedLowerBound;
        } else if (speedSetting > speedUpperBound) {
            speedSetting = speedUpperBound;
        }
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %d", speedSetting);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        // Update running speed if motor is already running in continuous mode
        if (motorRunning && continuousMode) {
            controller.setSpeed(speedSetting);
        }
        
        Serial.print("Speed adjusted to: ");
        Serial.println(speedSetting);
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
        
        // Scale movement by sensitivity factor
        int moveSteps = delta * ENCODER_JOG_SENSITIVITY;
        
        // Apply direction setting from UI
        if (!clockwiseDirection) {
            moveSteps = -moveSteps;
        }
        
        // Only move if there's significant encoder change
        if (moveSteps != 0) {
            // Set motor to move relative to current position
            controller.move(moveSteps);
            
            // Use current speed setting for movement
            controller.setSpeed(speedSetting);
            
            Serial.print("Encoder jog: ");
            Serial.print(moveSteps);
            Serial.println(" steps");
        }
    }
    
    // Run the stepper
    controller.run();
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
    
    // Update step number button label
    lv_obj_t *steps_label = lv_obj_get_child(objects.step_num, 0);
    if (steps_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Steps: %d", targetSteps);
        lv_label_set_text(steps_label, buffer);
    }
    
    // Update speed button labels
    lv_obj_t *speed_label = lv_obj_get_child(objects.speed, 0);
    if (speed_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %d", speedSetting);
        lv_label_set_text(speed_label, buffer);
    }
    
    lv_obj_t *speed_manual_label = lv_obj_get_child(objects.speed_manual_jog, 0);
    if (speed_manual_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %d", speedSetting);
        lv_label_set_text(speed_manual_label, buffer);
    }
    
    lv_obj_t *speed_cont_label = lv_obj_get_child(objects.continuous_rotation_speed_button, 0);
    if (speed_cont_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %d", speedSetting);
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
    
    // Enable the motor
    driver.enable();
    
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
        controller.setSpeed(speedSetting);
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
        controller.stop();
        controller.startContinuous(clockwiseDirection, speedSetting);
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
        controller.setSpeed(speedSetting);
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
  Serial.println("Stepper Motor Controller - Modular Version with SLEEP/RESET control");
  
  // Initialize our motor driver and controller
  controller.init();
  controller.setMaxSpeed(DEFAULT_MAX_SPEED);
  controller.setAcceleration(DEFAULT_ACCELERATION);
  
  // Set microstepping mode for DRV8825 if used
  #if USE_DRV8825_DRIVER
  driver.setMicrostepMode(DEFAULT_MICROSTEP_MODE);
  // Explicitly wake the driver
  driver.wake();
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
    // Handle UI updates
    Timer_Loop();
    ui_tick();
    
    // Handle encoder input (includes UI navigation and value adjustment)
    handleEncoder();
    
    // Check for motor idle timeout
    if (enableMotorPowerSave && motorRunning && 
        !encoderJogMode && !continuousMode && 
        controller.distanceToGo() == 0 &&
        millis() - lastMotorActivityTime > MOTOR_IDLE_TIMEOUT) {
        // Motor has been idle too long, disable it
        motorRunning = false;
        driver.disable();
        update_ui_labels();
    }
    
    // Handle motor control
    if (motorRunning) {
        // Handle encoder jog mode
        if (encoderJogMode) {
            checkEncoderJogMode();
        }
        // Let the controller handle all other motor operations
        else {
            // Update activity time
            lastMotorActivityTime = millis();
            
            // Run the controller
            controller.run();
            
            // Check if motion has completed (in step mode)
            if (!continuousMode && !controller.isRunning()) {
                motorRunning = false;
                Serial.println("Target position reached");
                // Update UI to show motor has stopped
                update_ui_labels();
            }
        }
    }
}