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

// Long press settings
#define LONG_PRESS_DURATION 800  // 800ms for long press
volatile unsigned long buttonPressStartTime = 0;
volatile bool longPressDetected = false;

// Navigation state variables
int8_t currentScreenIndex = 0;
int8_t currentFocusIndex = 0;
bool buttonPressed = false;
extern bool valueAdjustmentMode;
extern lv_obj_t *currentAdjustmentObject;
extern int adjustmentSensitivity;

// Encoder state tracking
volatile bool buttonCurrentlyPressed = false;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile long lastEncoderValue = 0;
volatile unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 200; // Debounce time in ms
unsigned long lastEncoderTime = 0;

// Screen navigation information
// Define the focusable objects for each screen
lv_obj_t** focusableObjects[7]; // Array for each screen
int focusableObjectsCount[7]; // Count for each screen

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

void toggleAdjustmentPrecision() {
  fineAdjustmentMode = !fineAdjustmentMode;
  
  // Show feedback
  showModeChangeIndicator();
  
  Serial.print("Switched to ");
  Serial.println(fineAdjustmentMode ? "fine adjustment mode" : "coarse adjustment mode");
}

void showModeChangeIndicator() {
  // For now, just print to serial
  Serial.println(fineAdjustmentMode ? "MODE: FINE ADJUSTMENT" : "MODE: COARSE ADJUSTMENT");
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
  // This gets called on both rising and falling edges
  bool buttonState = digitalRead(ENCODER_BUTTON_PIN);
  unsigned long currentTime = millis();
  
  // Button pressed (LOW since it's pulled up)
  if (buttonState == LOW && !buttonCurrentlyPressed) {
    buttonCurrentlyPressed = true;
    buttonPressStartTime = currentTime;
  }
  // Button released (HIGH)
  else if (buttonState == HIGH && buttonCurrentlyPressed) {
    buttonCurrentlyPressed = false;
    
    // Check if it was a long press
    if (currentTime - buttonPressStartTime > LONG_PRESS_DURATION) {
      longPressDetected = true;
    } 
    // Otherwise it's a normal press if it's past debounce time
    else if (currentTime - buttonPressStartTime > 20) { // 20ms debounce
      buttonPressed = true;
    }
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON_PIN), handleButtonInterrupt, CHANGE);
  
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
  // Main screen objects - ADD auto_button and settings_button to focusable list
  static lv_obj_t* mainScreenObjects[] = {
      objects.move_steps,
      objects.manual_jog,
      objects.continuous,
      objects.auto_button,      // Added sequence button
      objects.settings_button   // Added settings button
  };
  focusableObjects[0] = mainScreenObjects;
  focusableObjectsCount[0] = 5;  // Updated count to 5
  
  // Move steps screen - no changes needed
  static lv_obj_t* moveStepsScreenObjects[] = {
      objects.back,
      objects.start,
      objects.step_num,
      objects.clockwise,
      objects.speed
  };
  focusableObjects[1] = moveStepsScreenObjects;
  focusableObjectsCount[1] = 5;
  
  // Manual jog screen - no changes needed
  static lv_obj_t* manualJogScreenObjects[] = {
      objects.back_1,
      objects.start_1,
      objects.speed_manual_jog
  };
  focusableObjects[2] = manualJogScreenObjects;
  focusableObjectsCount[2] = 3;
  
  // Continuous rotation screen - no changes needed
  static lv_obj_t* continuousRotationScreenObjects[] = {
      objects.back_2,
      objects.continuous_rotation_start_button,
      objects.continuous_rotation_speed_button,
      objects.continuous_rotation_direction_button
  };
  focusableObjects[3] = continuousRotationScreenObjects;
  focusableObjectsCount[3] = 4;
  
  // ADD NEW SCREENS - Sequence screen
  static lv_obj_t* sequenceScreenObjects[] = {
      objects.back_4,
      objects.continuous_rotation_start_button_1,
      objects.sequence_positions_button,
      objects.sequence_speed_button,
      objects.sequence_direction_button
  };
  focusableObjects[4] = sequenceScreenObjects;
  focusableObjectsCount[4] = 5;
  
  // ADD NEW SCREENS - Sequence positions screen
  static lv_obj_t* sequencePositionsScreenObjects[] = {
      objects.back_5,
      objects.sequence_position_0_button,
      objects.sequence_position_1_button,
      objects.sequence_position_2_button,
      objects.sequence_position_3_button
  };
  focusableObjects[5] = sequencePositionsScreenObjects;
  focusableObjectsCount[5] = 5;
  
  // ADD NEW SCREENS - Settings screen
  static lv_obj_t* settingsScreenObjects[] = {
      objects.back_3,
      objects.acceleration_button,
      objects.microstepping_button
  };
  focusableObjects[6] = settingsScreenObjects;
  focusableObjectsCount[6] = 3;
}

void handleEncoder() {
  unsigned long currentTime = millis();
  
  // Check for long press to toggle adjustment mode
  if (longPressDetected) {
    longPressDetected = false;
    
    // Only toggle if in value adjustment mode
    if (valueAdjustmentMode) {
      toggleAdjustmentPrecision();
    }
    return;
  }

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
  
  // Check if this is a value that can be adjusted
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
  // ADD NEW ADJUSTABLE VALUES
  else if (currentScreenIndex == 4) { // Sequence Page
      isAdjustableValue = (currentObj == objects.sequence_speed_button);
  }
  else if (currentScreenIndex == 5) { // Sequence Positions Page
      isAdjustableValue = (currentObj == objects.sequence_position_1_button || 
                          currentObj == objects.sequence_position_2_button ||
                          currentObj == objects.sequence_position_3_button);
  }
  else if (currentScreenIndex == 6) { // Settings Page
      isAdjustableValue = (currentObj == objects.acceleration_button);
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
  
  // Special handling for main screen buttons
  if (currentScreenIndex == 0) {
      if (currentObj == objects.auto_button) {
          Serial.println("Sequence button pressed");
          loadScreen(SCREEN_ID_SEQUENCE_PAGE);
          currentScreenIndex = 4;
          currentFocusIndex = 0;
          // After short delay, set focus on the first item
          delay(200);
          setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
          return;
      }
      else if (currentObj == objects.settings_button) {
          Serial.println("Settings button pressed");
          loadScreen(SCREEN_ID_SETTINGS_PAGE);
          currentScreenIndex = 6;
          currentFocusIndex = 0;
          // After short delay, set focus on the first item
          delay(200);
          setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
          return;
      }
  }
  
  // Forward the click event to the UI event handler
  lv_event_send(currentObj, LV_EVENT_CLICKED, NULL);
}