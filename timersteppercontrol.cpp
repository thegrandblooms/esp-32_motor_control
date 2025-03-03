// TimerStepperControl.cpp
#include "TimerStepperControl.h"

// Initialize static instance pointer
TimerStepperControl* TimerStepperControl::instance = nullptr;

// Constructor
TimerStepperControl::TimerStepperControl(StepperDriver* driver) :
    _driver(driver),
    _isRunning(false),
    _isContinuous(false),
    _direction(true),
    _speed(0),
    _currentPosition(0),
    _targetPosition(0),
    _gptimer(nullptr),
    _commandQueue(nullptr),
    _motorTaskHandle(nullptr),
    _minStepInterval(1000), // Default 1ms between steps
    _lastStepTime(0)
{
    // Store instance pointer for ISR
    instance = this;
}

// Initialize hardware timer and FreeRTOS components
void TimerStepperControl::init() {
    // Initialize the driver
    _driver->init();
    
    // Create command queue
    _commandQueue = xQueueCreate(10, sizeof(MotorCommand_t));
    
    // Create motor control task
    xTaskCreate(
        motorControlTask,
        "motor_task",
        4096,
        this,
        10, // Higher priority
        &_motorTaskHandle
    );
    
    // Configure timer
    gptimer_config_t timer_config;
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;  // 1MHz = 1us resolution
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_gptimer));
    
    // Configure timer alarm
    gptimer_alarm_config_t alarm_config;
    alarm_config.reload_count = 0;
    alarm_config.alarm_count = 1000; // 1ms intervals
    alarm_config.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(_gptimer, &alarm_config));
    
    // Register timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timerCallback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(_gptimer, &cbs, this));
    
    // Start the timer
    ESP_ERROR_CHECK(gptimer_enable(_gptimer));
    ESP_ERROR_CHECK(gptimer_start(_gptimer));
}

// Timer callback (ISR)
bool IRAM_ATTR TimerStepperControl::timerCallback(gptimer_handle_t timer, 
        const gptimer_alarm_event_data_t *edata, 
        void *user_data) {
    // Get instance pointer
    TimerStepperControl* obj = (TimerStepperControl*)user_data;

    // If running, process a step if needed
    if (obj->_isRunning) {
    obj->processStep();
    }

    // Return false to avoid waking up a high-priority task
    return false;
}

// Process a single step if needed
void TimerStepperControl::processStep() {
    // Get current time
    unsigned long currentTime = micros();
    
    // Check if it's time for the next step
    if (currentTime - _lastStepTime >= _minStepInterval) {
        // For continuous rotation, just keep stepping
        if (_isContinuous) {
            _driver->setDirection(_direction);
            _driver->step();
            if (_direction) {
                _currentPosition++;
            } else {
                _currentPosition--;
            }
            _lastStepTime = currentTime;
            return;
        }
        
        // For position control, check if we've reached the target
        if (_currentPosition == _targetPosition) {
            _isRunning = false;
            _driver->disable();
            return;
        }
        
        // Determine direction to move
        if (_currentPosition < _targetPosition) {
            _driver->setDirection(true); // clockwise
            _driver->step();
            _currentPosition++;
        } else {
            _driver->setDirection(false); // counterclockwise
            _driver->step();
            _currentPosition--;
        }
        
        _lastStepTime = currentTime;
    }
}

// Send a command to the motor control task
bool TimerStepperControl::sendCommand(MotorCommand_t* cmd) {
    // Send command to queue with timeout
    return xQueueSend(_commandQueue, cmd, pdMS_TO_TICKS(100)) == pdTRUE;
}

// Motor control task
void TimerStepperControl::motorControlTask(void* pvParameters) {
    TimerStepperControl* obj = (TimerStepperControl*)pvParameters;
    MotorCommand_t cmd;
    
    while (1) {
        // Check for new commands
        if (xQueueReceive(obj->_commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            obj->handleCommand(&cmd);
        }
        
        // Allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Handle a command
void TimerStepperControl::handleCommand(MotorCommand_t* cmd) {
    switch (cmd->cmd_type) {
        case CMD_MOVE_TO:
            _targetPosition = cmd->position;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _isRunning = true;
            _isContinuous = false;
            _driver->enable();
            break;
            
        case CMD_MOVE_STEPS:
            _targetPosition = _currentPosition + cmd->position;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _isRunning = true;
            _isContinuous = false;
            _driver->enable();
            break;
            
        case CMD_SET_SPEED:
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            break;
            
        case CMD_START_JOG:
            // Enable motor for jogging but don't change position targets
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _isRunning = true; // This allows the motor to be responsive to jog commands
            _isContinuous = false;
            _driver->enable();
            break;
            
        case CMD_START_CONTINUOUS:
            _direction = cmd->direction;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _isRunning = true;
            _isContinuous = true;
            _driver->setDirection(_direction);
            _driver->enable();
            break;
            
        case CMD_STOP_MOTOR:
            _isRunning = false;
            _isContinuous = false;
            _driver->disable();
            break;
            
        default:
            break;
    }
}

// Check if motor is currently running
bool TimerStepperControl::isRunning() {
    return _isRunning;
}

// Set current position
void TimerStepperControl::setCurrentPosition(long position) {
    _currentPosition = position;
    _targetPosition = position;
}

// Get current position
long TimerStepperControl::getCurrentPosition() {
    return _currentPosition;
}

// For power management
void TimerStepperControl::sleep() {
    _driver->disable();
    
    #if USE_DRV8825_DRIVER
    // If using DRV8825, we can safely call sleep directly
    DRV8825Driver* drv8825 = static_cast<DRV8825Driver*>(_driver);
    drv8825->sleep();
    #endif
}

void TimerStepperControl::wake() {
    #if USE_DRV8825_DRIVER
    // If using DRV8825, we can safely call wake directly
    DRV8825Driver* drv8825 = static_cast<DRV8825Driver*>(_driver);
    drv8825->wake();
    #endif
}