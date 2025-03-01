#pragma once
#include <Arduino.h>
#include "screens.h"

#define ENCODER_PIN_A 5      // CLK pin
#define ENCODER_PIN_B 4      // DT pin
#define ENCODER_BUTTON_PIN 3 // SW pin
#define LONG_PRESS_DURATION 800 // 800ms for long press

// Navigation states
extern int8_t currentScreenIndex;
extern int8_t currentFocusIndex;
extern bool buttonPressed;
extern bool encoderJogMode;
extern volatile long encoderValue;
extern volatile long lastEncoderValue;
extern bool valueAdjustmentMode;
extern lv_obj_t *currentAdjustmentObject;
extern int adjustmentSensitivity;
extern bool fineAdjustmentMode;
extern volatile bool longPressDetected;  // Declare as extern

// Forward declarations for functions
void adjustValueByEncoder(lv_obj_t* obj, int delta);
void toggleAdjustmentPrecision();
void showModeChangeIndicator();

// Forward declarations for all functions
void setupEncoder();
void setupFocusableObjects();
void handleEncoder();
void navigateUI(int8_t direction);
void setFocus(lv_obj_t* obj);
void selectCurrentItem();
void setupFocusStyles();