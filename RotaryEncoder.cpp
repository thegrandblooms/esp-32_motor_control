#include "RotaryEncoder.h"
#include "ui.h"
#include "screens.h"
#include "LVGL_Driver.h"

// Configuration options
const bool REVERSE_ENCODER_DIRECTION = true;  // Set to true to reverse encoder direction
const int encoderThreshold = 4;  // Sensitivity Adjustment 1 - required encoder events
const unsigned long encoderDebounceTime = 0; // Sensitivity Adjustment 2 - milliseconds between encoder events
const unsigned long SCREEN_PRE_RENDER_DELAY_MS = 5;    // Delay before LVGL rendering
const unsigned long SCREEN_POST_RENDER_DELAY_MS = 150; // Delay after LVGL rendering

// Navigation state variables
int8_t currentScreenIndex = 0;
int8_t currentFocusIndex = 0;
bool buttonPressed = false;
extern bool valueAdjustmentMode;
extern lv_obj_t *currentAdjustmentObject;
extern int adjustmentSensitivity;

// Encoder state tracking
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile long lastEncoderValue = 0;
volatile unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 200; // Debounce time in ms
unsigned long lastEncoderTime = 0;

// Screen navigation information
// Define the focusable objects for each screen
lv_obj_t** focusableObjects[4]; // Array for each screen
int focusableObjectsCount[4]; // Count for each screen

// Forward declaration
void setupFocusableObjects();
void setFocus(lv_obj_t* obj);

void setupFocusStyles() {
    // Apply to all button styles on focus state
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < focusableObjectsCount[i]; j++) {
        lv_obj_t* obj = focusableObjects[i][j];
        
        // Style for focused state - highlight with blue border or outline
        lv_obj_set_style_border_color(obj, lv_color_hex(0x2196F3), LV_STATE_FOCUSED);  // Blue border
        lv_obj_set_style_border_width(obj, 3, LV_STATE_FOCUSED);                       // Thicker border when focused
        lv_obj_set_style_border_opa(obj, 255, LV_STATE_FOCUSED);                       // Full opacity
        
        // Optional: You can also change the background color slightly
        lv_obj_set_style_bg_color(obj, lv_color_hex(0x808080), LV_STATE_FOCUSED);     // Darker background
        
        // Enable focus tracking on the object
        lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
      }
    }
  }

void IRAM_ATTR handleEncoderInterrupt() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;
  
  lastEncoded = encoded;
}

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonPress > debounceTime) {
    buttonPressed = true;
    lastButtonPress = currentTime;
  }
}

void setupEncoder() {
    // Configure pins
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON_PIN), handleButtonInterrupt, FALLING);
    
    // Set up focusable objects for each screen
    setupFocusableObjects();
    
    // Set up styles for focus states
    setupFocusStyles();
    
    // Set initial focus on the first item of the current screen
    if (focusableObjectsCount[currentScreenIndex] > 0) {
      setFocus(focusableObjects[currentScreenIndex][0]);
    }
  }

void setupFocusableObjects() {
  // Main screen objects
  static lv_obj_t* mainScreenObjects[] = {
    objects.move_steps,
    objects.manual_jog,
    objects.continuous
  };
  focusableObjects[0] = mainScreenObjects;
  focusableObjectsCount[0] = 3;
  
  // Move steps screen
  static lv_obj_t* moveStepsScreenObjects[] = {
    objects.back,
    objects.start,
    objects.step_num,
    objects.clockwise,
    objects.speed
  };
  focusableObjects[1] = moveStepsScreenObjects;
  focusableObjectsCount[1] = 5;
  
  // Manual jog screen
  static lv_obj_t* manualJogScreenObjects[] = {
    objects.back_1,
    objects.start_1,
    objects.speed_manual_jog
  };
  focusableObjects[2] = manualJogScreenObjects;
  focusableObjectsCount[2] = 3;
  
  // Continuous rotation screen
  static lv_obj_t* continuousRotationScreenObjects[] = {
    objects.back_2,
    objects.continuous_rotation_start_button,
    objects.continuous_rotation_speed_button,
    objects.continuous_rotation_direction_button
  };
  focusableObjects[3] = continuousRotationScreenObjects;
  focusableObjectsCount[3] = 4;
}

void handleEncoder() {
  unsigned long currentTime = millis();
  
  // Skip regular UI navigation when in jog mode
  if (encoderJogMode) {
      return;
  }
  
  if (encoderValue != lastEncoderValue) {
      // Handle value adjustment mode
      if (valueAdjustmentMode && currentAdjustmentObject != NULL) {
          // Only register a change if enough encoder ticks have accumulated
          if (abs(encoderValue - lastEncoderValue) >= 1) {
              // Determine direction based on encoder values
              int rawDirection = encoderValue > lastEncoderValue ? 1 : -1;
              int direction = REVERSE_ENCODER_DIRECTION ? -rawDirection : rawDirection;
              
              // Adjust the appropriate value based on which object is being adjusted
              adjustValueByEncoder(currentAdjustmentObject, direction * adjustmentSensitivity);
              
              // Update tracking variables
              lastEncoderValue = encoderValue;
              lastEncoderTime = currentTime;
          }
          return;
      }
      
      // Regular UI navigation (only when not in adjustment mode)
      // Only register a change if:
      // 1. Enough pulses have accumulated since last navigation
      // 2. Enough time has passed since the last navigation
      if ((abs(encoderValue - lastEncoderValue) >= encoderThreshold) && 
          (currentTime - lastEncoderTime > encoderDebounceTime)) {
          
          // Determine direction based on encoder values
          int rawDirection = encoderValue > lastEncoderValue ? 1 : -1;
          
          // Apply direction reversal if configured
          int direction = REVERSE_ENCODER_DIRECTION ? -rawDirection : rawDirection;
          
          navigateUI(direction);
          
          // Update tracking variables
          lastEncoderValue = encoderValue;
          lastEncoderTime = currentTime;
      }
  }
  
  if (buttonPressed) {
      selectCurrentItem();
      buttonPressed = false;
  }
}

void setFocus(lv_obj_t* obj) {
  // Remove focus from all objects
  for (int i = 0; i < focusableObjectsCount[currentScreenIndex]; i++) {
    lv_obj_clear_state(focusableObjects[currentScreenIndex][i], LV_STATE_FOCUSED);
  }
  
  // Set focus on new object
  lv_obj_add_state(obj, LV_STATE_FOCUSED);
}

void navigateUI(int8_t direction) {
  // Update focus index
  currentFocusIndex += direction;
  
  // Handle wrapping
  if (currentFocusIndex < 0) {
    currentFocusIndex = focusableObjectsCount[currentScreenIndex] - 1;
  } else if (currentFocusIndex >= focusableObjectsCount[currentScreenIndex]) {
    currentFocusIndex = 0;
  }
  
  // Focus the new object
  setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
}

void selectCurrentItem() {
  lv_obj_t *currentObj = focusableObjects[currentScreenIndex][currentFocusIndex];
  
  // Check if we're already in value adjustment mode
  if (valueAdjustmentMode) {
      // Exit value adjustment mode
      valueAdjustmentMode = false;
      currentAdjustmentObject = NULL;
      
      // Restore original style (remove highlight)
      lv_obj_set_style_bg_color(currentObj, lv_color_hex(0x656565), LV_PART_MAIN | LV_STATE_DEFAULT);
      
      return;
  }
  
  // Check if this is a value that can be adjusted (steps or speed buttons)
  bool isAdjustableValue = false;
  
  if (currentScreenIndex == 1) { // Move Steps Page
      isAdjustableValue = (currentObj == objects.step_num || currentObj == objects.speed);
  }
  else if (currentScreenIndex == 2) { // Manual Jog Page
      isAdjustableValue = (currentObj == objects.speed_manual_jog);
  }
  else if (currentScreenIndex == 3) { // Continuous Rotation Page
      isAdjustableValue = (currentObj == objects.continuous_rotation_speed_button);
  }
  
  // If this is an adjustable value, enter adjustment mode
  if (isAdjustableValue) {
      valueAdjustmentMode = true;
      currentAdjustmentObject = currentObj;
      
      // Highlight the button to indicate adjustment mode
      lv_obj_set_style_bg_color(currentObj, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT);
      
      return;
  }
  
  // For other buttons, proceed with normal button click processing
  // Before loading a new screen, clear all focus states to prevent style artifacts
  for (int i = 0; i < focusableObjectsCount[currentScreenIndex]; i++) {
    lv_obj_clear_state(focusableObjects[currentScreenIndex][i], LV_STATE_FOCUSED);
  }
  
  // Handle navigation between screens based on the selected button
  if (currentScreenIndex == 0) { // Main screen
    if (currentObj == objects.move_steps) {
      loadScreen(SCREEN_ID_MOVE_STEPS_PAGE);
      currentScreenIndex = 1;
      currentFocusIndex = 0;
      
      // Give LVGL time to load the screen and apply styles
      delay(SCREEN_PRE_RENDER_DELAY_MS);
      lv_timer_handler(); // Process any pending LVGL tasks
      delay(SCREEN_POST_RENDER_DELAY_MS);
      
      setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
    } else if (currentObj == objects.manual_jog) {
      loadScreen(SCREEN_ID_MANUAL_JOG_PAGE);
      currentScreenIndex = 2;
      currentFocusIndex = 0;
      
      // Give LVGL time to load the screen and apply styles
      delay(SCREEN_PRE_RENDER_DELAY_MS);
      lv_timer_handler(); // Process any pending LVGL tasks
      delay(SCREEN_POST_RENDER_DELAY_MS);
      
      setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
    } else if (currentObj == objects.continuous) {
      loadScreen(SCREEN_ID_CONTINUOUS_ROTATION_PAGE);
      currentScreenIndex = 3;
      currentFocusIndex = 0;
      
      // Give LVGL time to load the screen and apply styles
      delay(SCREEN_PRE_RENDER_DELAY_MS);
      lv_timer_handler(); // Process any pending LVGL tasks
      delay(SCREEN_POST_RENDER_DELAY_MS);
      
      setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
    }
  } else {
    // Handle back buttons on sub-screens
    if ((currentScreenIndex == 1 && currentObj == objects.back) ||
        (currentScreenIndex == 2 && currentObj == objects.back_1) ||
        (currentScreenIndex == 3 && currentObj == objects.back_2)) {
      loadScreen(SCREEN_ID_MAIN);
      currentScreenIndex = 0;
      currentFocusIndex = 0;
      
      // Give LVGL time to load the screen and apply styles
      delay(SCREEN_PRE_RENDER_DELAY_MS);
      lv_timer_handler(); // Process any pending LVGL tasks
      delay(SCREEN_POST_RENDER_DELAY_MS);
      
      setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
    } else {
      // Simulate a button click event for other buttons
      lv_event_send(currentObj, LV_EVENT_CLICKED, NULL);
    }
  }
}