// TimerStepperControl.h - Fixed version
#ifndef TIMER_STEPPER_CONTROL_H
#define TIMER_STEPPER_CONTROL_H

#include <Arduino.h>
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "StepperDriver.h"
#include "DRV8825Driver.h"  // For DRV8825-specific features

// Define command types for motor control
typedef enum {
    CMD_MOVE_TO,        // Move to absolute position
    CMD_MOVE_STEPS,     // Move relative number of steps
    CMD_SET_SPEED,      // Set speed
    CMD_START_JOG,      // Start manual jog mode
    CMD_STOP_JOG,       // Stop manual jog mode
    CMD_START_CONTINUOUS, // Start continuous rotation
    CMD_STOP_MOTOR       // Stop any motion
} MotorCommandType;

// Define command structure
typedef struct {
    MotorCommandType cmd_type;
    long position;           // Target position or steps to move
    int speed;               // Speed setting
    bool direction;          // Direction (true = clockwise)
    bool continuous;         // Whether in continuous mode
} MotorCommand_t;

// Timer control class
class TimerStepperControl {
public:
    // Constructor
    TimerStepperControl(StepperDriver* driver);
    
    // Initialize hardware timer and FreeRTOS components
    void init();
    
    // Send a command to the motor control task
    bool sendCommand(MotorCommand_t* cmd);
    
    // Check if motor is currently running
    bool isRunning();
    
    // Set current position
    void setCurrentPosition(long position);
    
    // Get current position
    long getCurrentPosition();
    
    // Public static method that will be called by the timer ISR
    static bool IRAM_ATTR timerCallback(gptimer_handle_t timer, 
                                        const gptimer_alarm_event_data_t *edata, 
                                        void *user_data);
    
    // For power management
    void sleep();
    void wake();
    
private:
    // Static pointer for ISR to access instance
    static TimerStepperControl* instance;
    
    // Motor driver
    StepperDriver* _driver;
    
    // Motor state
    volatile bool _isRunning;
    volatile bool _isContinuous;
    volatile bool _direction;
    volatile int _speed;
    volatile long _currentPosition;
    volatile long _targetPosition;
    
    // Hardware timer handle
    gptimer_handle_t _gptimer;
    
    // Command queue
    QueueHandle_t _commandQueue;
    
    // Task handle
    TaskHandle_t _motorTaskHandle;
    
    // Step timing variables
    unsigned long _minStepInterval; // Microseconds between steps
    unsigned long _lastStepTime;    // Time of last step
    
    // Static task function
    static void motorControlTask(void* pvParameters);
    
    // Internal method to process a single step
    void processStep();
    
    // Internal method to handle a command
    void handleCommand(MotorCommand_t* cmd);
};

#endif // TIMER_STEPPER_CONTROL_H