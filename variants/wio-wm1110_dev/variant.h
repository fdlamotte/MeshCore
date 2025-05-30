/*
 * variant.h
 * Copyright (C) 2023 Seeed K.K.
 * MIT License
 */

#pragma once

#include "WVariant.h"

////////////////////////////////////////////////////////////////////////////////
// Low frequency clock source

#define USE_LFXO    // 32.768 kHz crystal oscillator
#define VARIANT_MCK (64000000ul)
// #define USE_LFRC    // 32.768 kHz RC oscillator

////////////////////////////////////////////////////////////////////////////////
// Power

#define PIN_3V3_EN              (0 + 7)            // P0.7 Power to Sensors

////////////////////////////////////////////////////////////////////////////////
// Number of pins

#define PINS_COUNT              (48)
#define NUM_DIGITAL_PINS        (48)
#define NUM_ANALOG_INPUTS       (6)
#define NUM_ANALOG_OUTPUTS      (0)

////////////////////////////////////////////////////////////////////////////////
// UART pin definition

#define PIN_SERIAL1_RX          (22)             // P0.22
#define PIN_SERIAL1_TX          (24)             // P0.24

#define PIN_SERIAL2_RX          (6)             // P0.6
#define PIN_SERIAL2_TX          (8)             // P0.8

////////////////////////////////////////////////////////////////////////////////
// I2C pin definition

#define HAS_WIRE                (1)
#define WIRE_INTERFACES_COUNT   (1)

#define PIN_WIRE_SDA            (27)             // P0.27
#define PIN_WIRE_SCL            (26)             // P0.26

////////////////////////////////////////////////////////////////////////////////
// SPI pin definition

#define SPI_INTERFACES_COUNT    (1)

#define PIN_SPI_MISO            (32 + 15)            // P1.15
#define PIN_SPI_MOSI            (32 + 14)            // P1.14
#define PIN_SPI_SCK             (32 + 13)            // P1.13
#define PIN_SPI_NSS             (32 + 12)            // P1.12

////////////////////////////////////////////////////////////////////////////////
// Builtin LEDs

#define PIN_LED1                (13)
#define PIN_LED2                (14)
#define LED_BLUE                (-1)
#define LED_GREEN               (-1)
#define LED_PIN                 PIN_LED1
#define LED_BUILTIN             PIN_LED1
#define LED_STATE_ON            HIGH
////////////////////////////////////////////////////////////////////////////////
// Builtin buttons

#define PIN_BUTTON1             (23)             // P0.6
#define BUTTON_PIN              PIN_BUTTON1

////////////////////////////////////////////////////////////////////////////////
// LR1110

#define LORA_DIO_1              (32 + 8)            // P1.8
#define LORA_RESET              (32 + 10)            // P1.10
#define LORA_BUSY               (32 + 11)             // P1.11
#define LORA_SCLK               (PIN_SPI_SCK)   // P0.11
#define LORA_MISO               (PIN_SPI_MISO)  // P1.8
#define LORA_MOSI               (PIN_SPI_MOSI)  // P0.9
#define LORA_NSS                (PIN_SPI_NSS)   // P0.12
 
#define LR11X0_DIO_AS_RF_SWITCH    true
#define LR11X0_DIO3_TCXO_VOLTAGE   1.6

////////////////////////////////////////////////////////////////////////////////
// GPS

// #define HAS_GPS                 1
// #define GPS_RX_PIN              PIN_SERIAL1_RX
// #define GPS_TX_PIN              PIN_SERIAL1_TX

// #define GPS_EN                  (43)            // P1.11
// #define GPS_RESET               (47)            // P1.15

// #define GPS_VRTC_EN             (8)             // P0.8
// #define GPS_SLEEP_INT           (44)            // P1.12
// #define GPS_RTC_INT             (15)            // P0.15
// #define GPS_RESETB              (46)            // P1.14

////////////////////////////////////////////////////////////////////////////////
// Temp+Lux Sensor

// #define SENSOR_EN               (4)             // P0.4
// #define TEMP_SENSOR             (31)            // P0.31/AIN7
// #define LUX_SENSOR              (29)            // P0.29/AIN5

////////////////////////////////////////////////////////////////////////////////
// Accelerometer (I2C addr : ??? )

// #define PIN_3V3_ACC_EN          (39)            // P1.7
