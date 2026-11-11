#pragma once

// goosebrain (rev 2): RP2354A, 2 MB flash.
//
// Only the six S ports on J5 (S1-S6 = GPIO0, 1, 2, 3, 8, 9) are free for
// sketches; the other GPIOs drive the motors, the LED stick, the battery
// sense and the USB-A host. There's no LED_BUILTIN: GPIO25, the Pico's LED,
// is the USB-A port's D+ here, and driving it high without a pack
// back-powers the USB-A port.

#include <stdint.h>

#define PICO_RP2350A __PICO_RP2350A

#define PINS_COUNT          (30u)
#define NUM_DIGITAL_PINS    (30u)
#define NUM_ANALOG_INPUTS   (4u)
#define NUM_ANALOG_OUTPUTS  (0u)
#define ADC_RESOLUTION      (12u)

// Serial1 on S1/S2, Serial2 on S5/S6
#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)
#define PIN_SERIAL2_TX (8u)
#define PIN_SERIAL2_RX (9u)

// SPI on S1-S4. SPI1 has no free SCK/MOSI pins; it's here because the core
// expects two.
#define PIN_SPI0_MISO  (0u)
#define PIN_SPI0_SS    (1u)
#define PIN_SPI0_SCK   (2u)
#define PIN_SPI0_MOSI  (3u)
#define PIN_SPI1_MISO  (8u)
#define PIN_SPI1_SS    (9u)
#define PIN_SPI1_SCK   (10u)
#define PIN_SPI1_MOSI  (11u)

// Wire on S1/S2, Wire1 on S3/S4
#define PIN_WIRE0_SDA  (0u)
#define PIN_WIRE0_SCL  (1u)
#define PIN_WIRE1_SDA  (2u)
#define PIN_WIRE1_SCL  (3u)

#define SERIAL_HOWMANY (3u)
#define SPI_HOWMANY    (2u)
#define WIRE_HOWMANY   (2u)
#define WIRE_INTERFACES_COUNT (WIRE_HOWMANY)

static const uint8_t D0 = 0u, D1 = 1u, D2 = 2u, D3 = 3u, D4 = 4u, D5 = 5u, D6 = 6u, D7 = 7u;
static const uint8_t D8 = 8u, D9 = 9u, D10 = 10u, D11 = 11u, D12 = 12u, D13 = 13u, D14 = 14u, D15 = 15u;
static const uint8_t D16 = 16u, D17 = 17u, D18 = 18u, D19 = 19u, D20 = 20u, D21 = 21u, D22 = 22u, D23 = 23u;
static const uint8_t D24 = 24u, D25 = 25u, D26 = 26u, D27 = 27u, D28 = 28u, D29 = 29u;

static const uint8_t A0 = 26u, A1 = 27u, A2 = 28u, A3 = 29u;

static const uint8_t SS = PIN_SPI0_SS;
static const uint8_t MOSI = PIN_SPI0_MOSI;
static const uint8_t MISO = PIN_SPI0_MISO;
static const uint8_t SCK = PIN_SPI0_SCK;

static const uint8_t SDA = PIN_WIRE0_SDA;
static const uint8_t SCL = PIN_WIRE0_SCL;
