// lib/hw/pins.h
#pragma once
#include <Arduino.h>

// -------------------- Pin assignments --------------------
#define HIN1 25
#define LIN1 21
#define HIN2 26
#define LIN2 22
#define HIN3 27
#define LIN3 23
#define ENABLE 4
#define LED_BUILTIN 2

// Optional: test LED strip pins on master (DotStar)
#define MASTER_DATAPIN  13
#define MASTER_CLOCKPIN 14

// Actuator (DRV8833-style) pins
#define AIN1 5
#define AIN2 18

// Zero-cross inputs (if/when used)
#define PHASE3_ZC_PIN 32
#define PHASE2_ZC_PIN 34
#define PHASE1_ZC_PIN 35

// -------------------- LEDC channels ----------------------
// BLDC High/Low side channels
#define HIN1_CH 0
#define HIN2_CH 1
#define HIN3_CH 2
#define LIN1_CH 3
#define LIN2_CH 4
#define LIN3_CH 5

// Actuator channels
#define AIN1_CH 6
#define AIN2_CH 7

// Optional visible LED channel (if you drive a simple LED via PWM)
#define LED_STRIP_CHANNEL 8

// -------------------- PWM base params --------------------
#define PWM_FREQ        20
#define PWM_RESOLUTION  8
#define PWM_DEFAULT_DUTY 150
#define PWM_MAX         255
