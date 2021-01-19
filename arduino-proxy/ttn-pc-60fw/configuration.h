#pragma once

#include <Arduino.h>
#include <lmic.h>
void ttn_register(void (*callback)(uint8_t message));

#define USE_ABP

/* The 3 values below come from the Example code on the TTN devices screen. For example if the example code shows the following:

const char *devAddr = "12345678";
const char *nwkSKey = "00112233445566778899AABBCCDDEEFF";
const char *appSKey = "A0A1A2A3B4B5B6B7C8C9CACBDCDDDEDF";

...then the variables should be set as below.

*/

// LoRaWAN NwkSKey, network session key
static const u1_t PROGMEM NWKSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xB4, 0xB5, 0xB6, 0xB7, 0xC8, 0xC9, 0xCA, 0xCB, 0xDC, 0xDD, 0xDE, 0xDF };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x12345678;

// Select which T-Beam board is being used. Only uncomment one.
// #define T_BEAM_V07  // AKA Rev0 (first board released)
#define T_BEAM_V10  // AKA Rev1 (second board released)

// If using a single-channel gateway, uncomment this next option and set to your gateway's channel
//#define SINGLE_CHANNEL_GATEWAY  0

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             5

#define SEND_INTERVAL           (20 * 1000)     // Sleep for these many millis
#define MESSAGE_TO_SLEEP_DELAY  5000            // Time after message before going to sleep
#define LORAWAN_PORT            10              // Port the messages will be sent to
#define LORAWAN_CONFIRMED_EVERY 0               // Send confirmed message every these many messages (0 means never)
#define LORAWAN_SF              DR_SF7         // Spreading factor (recommended DR_SF7 for ttn network map purposes, DR_SF10 works for slow moving trackers)
#define LORAWAN_ADR             0               // Enable ADR

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED       100
#define EV_PENDING      101
#define EV_ACK          102
#define EV_RESPONSE     103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------

#define I2C_SDA         21
#define I2C_SCL         22

#if defined(T_BEAM_V07)
#define LED_PIN         14
#define BUTTON_PIN      39
#elif defined(T_BEAM_V10)
#define BUTTON_PIN      38
#endif

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#if defined(T_BEAM_V10)
#define RESET_GPIO      14
#else
#define RESET_GPIO      23
#endif
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

#define GPS_POWER_CTRL_CH     3
#define LORA_POWER_CTRL_CH    2
#define PMU_IRQ               35
