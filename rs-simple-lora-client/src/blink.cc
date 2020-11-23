//
// Created by James Gallagher on 2/1/20.
//

#include "blink.h"
#include <Arduino.h>

#define one_second 1000
#define quarter_second 250
#define eigth_second 125

/**
 * @brief One Second on, 1/4s off/on for N times. Repeat forever
 *
 * Signal various error states. The long (1s) one period makes
 * it easier to recognize the following set of flashes, which can be varied
 * in number to indicate various conditions.
 *
 * @param pin Toggle the state of this pin
 * @param N One cycle should flash this many times
 * @return Never returns
 */
void error_blink(int pin, int N) {
    blink(pin, N, 0);
}

/**
 * @brief One Second on, 1/4s off/on for N times. Repeat M times
 *
 * Signal various states, including errors. The long (1s) one period makes
 * it easier to recognize the following set of flashes, which can be varied
 * in number to indicate various conditions. Use 2-3 cycles to indicate
 * various initialization operations and infinite cycles to indicate an error.
 *
 * @param pin Toggle the state of this pin
 * @param N One cycle should flash this many times
 * @param M Number of cycles. If 0, cycle forever
 */
void blink(int pin, int N, int M) {
    int count = 0;
    while ((M == 0) ? 1 : count < M) {
        count++;
        digitalWrite(pin, HIGH);
        delay(one_second);
        for (int i = 0; i < N; ++i) {
            digitalWrite(pin, LOW);
            delay(quarter_second);
            digitalWrite(pin, HIGH);
            delay(quarter_second);
        }
        digitalWrite(pin, LOW);
        delay(one_second);
    }
}
