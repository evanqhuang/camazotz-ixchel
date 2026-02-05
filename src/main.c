/**
 * @file main.c
 * @brief Minimal RP2350 firmware entry point - LED blink verification
 */

#include "pico/stdlib.h"
#include <stdio.h>

#ifndef PICO_DEFAULT_LED_PIN
#error "PICO_DEFAULT_LED_PIN not defined - board configuration missing"
#endif

int main(void) {
    // Initialize USB stdio
    stdio_init_all();

    // Initialize LED pin
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("RP2350 Mapper Firmware - Build: %s %s\n", __DATE__, __TIME__);
    printf("Board: %s\n", PICO_BOARD);
    printf("SDK Version: %s\n", PICO_SDK_VERSION_STRING);

    // Main loop - blink LED at 1Hz
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        printf("LED ON\n");
        sleep_ms(500);

        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        printf("LED OFF\n");
        sleep_ms(500);
    }

    return 0;
}
