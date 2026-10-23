#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// GPIO Definitions
#define B1 GPIO_NUM_4
#define B2 GPIO_NUM_5
#define B3 GPIO_NUM_6
#define B4 GPIO_NUM_7

#define L1 GPIO_NUM_8
#define L2 GPIO_NUM_9
#define L3 GPIO_NUM_10
#define L4 GPIO_NUM_11
#define L5 GPIO_NUM_12

// DIP Switch GPIOs
#define SW1 GPIO_NUM_13  // 1
#define SW2 GPIO_NUM_14  // 2
#define SW3 GPIO_NUM_15  // 4
#define SW4 GPIO_NUM_16  // 8
#define SW5 GPIO_NUM_17  // 16
#define SW6 GPIO_NUM_18  // 32
#define SW7 GPIO_NUM_21  // 64
#define SW8 GPIO_NUM_38  // 128

// Variables
static const char *TAG = "BUTTON_LED_TIMER";
static int64_t timer_start = 0;
static int64_t timer_duration = 0;
static bool timer_active = false;
static int active_button = 0;
static int timer_value = 0;

// Button states for debouncing
static bool last_b1_state = true;
static bool last_b2_state = true;
static bool last_b3_state = true;
static bool last_b4_state = true;
static int64_t last_debounce_time = 0;
static const int debounce_delay_ms = 50;

// Function prototypes
void turnOffAllLEDs(void);
void handleButtonPress(int button);
void readDIPSwitch(void);
bool debounce(void);

void turnOffAllLEDs(void) {
    gpio_set_level(L1, 0);
    gpio_set_level(L2, 0);
    gpio_set_level(L3, 0);
    gpio_set_level(L4, 0);
    gpio_set_level(L5, 0);
}

void handleButtonPress(int button) {
    timer_duration = (int64_t)timer_value * 1000000; // Convert seconds to microseconds
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = button;
    
    // Turn off all LEDs first
    turnOffAllLEDs();
    
    // Turn on appropriate LEDs based on button 
    switch(button) {
        case 1: // B1 → L3 + L5
            gpio_set_level(L3, 1);
            gpio_set_level(L5, 1);
            ESP_LOGI(TAG, "B1 pressed - L3 and L5 ON for %d seconds", timer_value);
            ESP_LOGI(TAG, "B3 and B4 blocked during this time");
            break;
            
        case 2: // B2 → L4 + L5
            gpio_set_level(L4, 1);
            gpio_set_level(L5, 1);
            ESP_LOGI(TAG, "B2 pressed - L4 and L5 ON for %d seconds", timer_value);
            ESP_LOGI(TAG, "B3 and B4 blocked during this time");
            break;
            
        case 3: // B3 → L1 + L5
            gpio_set_level(L1, 1);
            gpio_set_level(L5, 1);
            ESP_LOGI(TAG, "B3 pressed - L1 and L5 ON for %d seconds", timer_value);
            ESP_LOGI(TAG, "B1 and B2 blocked during this time");
            break;
            
        case 4: // B4 → L2 + L5
            gpio_set_level(L2, 1);
            gpio_set_level(L5, 1);
            ESP_LOGI(TAG, "B4 pressed - L2 and L5 ON for %d seconds", timer_value);
            ESP_LOGI(TAG, "B1 and B2 blocked during this time");
            break;
    }
}

void readDIPSwitch(void) {
    int value = 0;
    
    if (gpio_get_level(SW1) == 0) value += 1;
    if (gpio_get_level(SW2) == 0) value += 2;
    if (gpio_get_level(SW3) == 0) value += 4;
    if (gpio_get_level(SW4) == 0) value += 8;
    if (gpio_get_level(SW5) == 0) value += 16;
    if (gpio_get_level(SW6) == 0) value += 32;
    if (gpio_get_level(SW7) == 0) value += 64;
    if (gpio_get_level(SW8) == 0) value += 128;
    
    // Only update if value changed
    if (value != timer_value) {
        timer_value = value;
        ESP_LOGI(TAG, "Timer set to %d seconds", timer_value);
    }
}

bool debounce(void) {
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (current_time - last_debounce_time > debounce_delay_ms) {
        last_debounce_time = current_time;
        return true;
    }
    return false;
}

void app_main(void) {
    ESP_LOGI(TAG, "System Starting...");
    
    // Configure Button pins with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << B1) | (1ULL << B2) | (1ULL << B3) | (1ULL << B4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Configure LED pins
    io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << L1) | (1ULL << L2) | (1ULL << L3) | (1ULL << L4) | (1ULL << L5),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Configure DIP Switch pins with internal pull-up
    io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << SW1) | (1ULL << SW2) | (1ULL << SW3) | (1ULL << SW4) | 
                        (1ULL << SW5) | (1ULL << SW6) | (1ULL << SW7) | (1ULL << SW8),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Turn off all LEDs initially
    turnOffAllLEDs();
    
    ESP_LOGI(TAG, "System Ready!");
    
    while (1) {
        // Read DIP switch value
        readDIPSwitch();
        
        // Check if timer has expired
        if (timer_active) {
            int64_t current_time = esp_timer_get_time();
            if (current_time - timer_start >= timer_duration) {
                turnOffAllLEDs();
                timer_active = false;
                active_button = 0;
                ESP_LOGI(TAG, "Timer expired - LEDs turned off");
            }
        }
        
        // Read button states
        bool current_b1 = gpio_get_level(B1) == 0;
        bool current_b2 = gpio_get_level(B2) == 0;
        bool current_b3 = gpio_get_level(B3) == 0;
        bool current_b4 = gpio_get_level(B4) == 0;
        
        // Check for button presses (LOW because of pull-up)
        if (!timer_active) {
            // B1 pressed
            if (current_b1 && !last_b1_state) {
                if (debounce()) {
                    handleButtonPress(1);
                }
            }
            // B2 pressed
            else if (current_b2 && !last_b2_state) {
                if (debounce()) {
                    handleButtonPress(2);
                }
            }
            // B3 pressed
            else if (current_b3 && !last_b3_state) {
                if (debounce()) {
                    handleButtonPress(3);
                }
            }
            // B4 pressed
            else if (current_b4 && !last_b4_state) {
                if (debounce()) {
                    handleButtonPress(4);
                }
            }
        } else {
            // Check if blocked buttons are pressed (B3/B4 for B1/B2, B1/B2 for B3/B4)
            if (active_button == 1 || active_button == 2) {
                if (current_b3 || current_b4) {
                    ESP_LOGW(TAG, "Button B%d blocked during B%d timer", 
                            current_b3 ? 3 : 4, active_button);
                }
            } else if (active_button == 3 || active_button == 4) {
                if (current_b1 || current_b2) {
                    ESP_LOGW(TAG, "Button B%d blocked during B%d timer", 
                            current_b1 ? 1 : 2, active_button);
                }
            }
        }
        
        // Update last states
        last_b1_state = current_b1;
        last_b2_state = current_b2;
        last_b3_state = current_b3;
        last_b4_state = current_b4;
        
        // Small delay to reduce CPU usage
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
