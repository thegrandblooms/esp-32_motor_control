# ESP32-C6 Motor Controller

This system takes in user input from an EC11 Rotary Encoder with a push switch and has a UI display built in LVGL for this ESP32 package with a screen from waveshare: https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47

This code is modular, and can interface with both DRV8825 and LN298N motor controllers. Stepper timing is handled using the onboard hardware timer.


Wiring Diagram:
![image](https://github.com/user-attachments/assets/9b5da787-d41b-40db-b8d2-8a0d870375fb)
