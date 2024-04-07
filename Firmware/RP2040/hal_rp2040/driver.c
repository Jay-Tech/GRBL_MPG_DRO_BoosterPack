/*
 * driver.c - HAL driver for Raspberry RP2040 ARM processor
 *
 * Part of MPG/DRO for grbl on a secondary processor
 *
 * v0.0.3 / 2022-01-29 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021-2022, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "encoder.pio.h"

#include "../src/grbl.h"
#include "../src/config.h"
#include "../src/interface.h"
#include "../src/keypad.h"

#include "i2c_nb.h"
#include "driver.h"

static qei_t qei = {0}, qei_mpg;
static mpg_t mpg = {0};
static mpg_axis_t *mpg_axis;
static leds_t leds_state = {
    .value = 255
};
static bool keyDown = false;
static void mpg_int_handler (void);
static void mpgMode_sw_int_handler (uint gpio, uint32_t events);
static void axis_toggle_int_handler (uint gpio, uint32_t events);
static void jog_rate_int_handler (uint gpio, uint32_t events);
static void gpio_int_handler (uint gpio, uint32_t events);

static int enc_sm, mpg_sm;

#define LAST_STATE(state)  ((state) & 0b0011)
#define CURR_STATE(state)  (((state) & 0b1100) >> 2)

static const uint32_t STATE_A_MASK      = 0x80000000;
static const uint32_t STATE_B_MASK      = 0x40000000;
static const uint32_t STATE_A_LAST_MASK = 0x20000000;
static const uint32_t STATE_B_LAST_MASK = 0x10000000;

static const uint8_t Maxium = 3;
uint_fast8_t current = 0;

#define STATES_MASK (STATE_A_MASK | STATE_B_MASK | STATE_A_LAST_MASK | STATE_B_LAST_MASK);

static const uint32_t TIME_MASK   = 0x0fffffff;

static const uint8_t MICROSTEP_0  = 0b00;
static const uint8_t MICROSTEP_1  = 0b10;
static const uint8_t MICROSTEP_2  = 0b11;
static const uint8_t MICROSTEP_3  = 0b01;

void hal_init (void)
{
    int offset;

    mpg_axis = &mpg.x;

    gpio_set_irq_enabled_with_callback(MPGMODE_SW_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_int_handler);
    gpio_pull_up(MPGMODE_SW_PIN);
    gpio_set_irq_enabled(MPGMODE_SW_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled_with_callback(JOG_RATE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, gpio_int_handler);
    gpio_pull_up(JOG_RATE_PIN);
    gpio_set_irq_enabled(JOG_RATE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    gpio_set_irq_enabled_with_callback(AXIS_TOGGLE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, gpio_int_handler);
    gpio_pull_up(AXIS_TOGGLE_PIN);
    gpio_set_irq_enabled(AXIS_TOGGLE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_init(KEYFWD_PIN);
    gpio_set_oeover(KEYFWD_PIN, GPIO_OVERRIDE_LOW); // > to OD // GPIO_OVERRIDE_INVERT does not work!
    gpio_set_dir(KEYFWD_PIN, GPIO_OUT);
    gpio_set_pulls(KEYFWD_PIN, false, false);
    gpio_put(KEYFWD_PIN, 0); // > to OD



    mpg_sm = pio_claim_unused_sm(pio1, true);
    offset = pio_add_program(pio1, &encoder_program);
    encoder_program_init(pio1, mpg_sm, offset, MPG_A, MPG_B, 250);
    hw_set_bits(&pio1->inte0, PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << mpg_sm);
    encoder_program_start(pio1, mpg_sm, gpio_get(MPG_A), gpio_get(MPG_B));

    irq_set_exclusive_handler(PIO1_IRQ_0, mpg_int_handler);
    irq_set_enabled(PIO1_IRQ_0, true);

 // Boot MSP430 keypad controller (the RP2040 does not support open drain outputs?)
    // gpio_set_pulls(SWD_RESET, false, false);
    // gpio_set_dir(SWD_RESET, GPIO_IN);
    // delay(5); // Wait for keypad controller startup

  }


void leds_setState (leds_t leds)
{
    // if(leds_state.value != leds.value) {
    //     leds_state.value = leds.value;
    //     i2c_nb_send(KEYPAD_I2CADDR, leds.value);
    // }
}

leds_t leds_getState (void)
{
    return leds_state;
}

void signal_setFeedHold (bool on)
{
    if(on)
        gpio_set_oeover(CYCLESTART_PIN, GPIO_OVERRIDE_LOW);
 
    gpio_set_oeover(FEEDHOLD_PIN, on ? GPIO_OVERRIDE_HIGH : GPIO_OVERRIDE_LOW);
}

void signal_setCycleStart (bool on)
{
    // if(on)
    //     gpio_set_oeover(FEEDHOLD_PIN, GPIO_OVERRIDE_LOW);

    // gpio_set_oeover(CYCLESTART_PIN, on ? GPIO_OVERRIDE_HIGH : GPIO_OVERRIDE_LOW);
}

void signal_setMPGMode (bool on)
{
    // static bool initOk = false;

    // if(!initOk) {
    //     initOk = true;
    //     gpio_set_dir(MPG_MODE_PIN, GPIO_OUT);
    //     gpio_set_pulls(MPG_MODE_PIN, false, false);
    //     gpio_set_oeover(MPG_MODE_PIN, GPIO_OVERRIDE_LOW); // > to OD
    //     gpio_put(MPG_MODE_PIN, 0);
    // }

    // // If last mode change request was not honoured then output a 1us pulse to reset
    // // grbl interrupt that may have become out of sync.
    // if(gpio_get(MPG_MODE_PIN) == on) {
    //     gpio_set_oeover(MPG_MODE_PIN, on ? GPIO_OVERRIDE_HIGH : GPIO_OVERRIDE_LOW);
    //     sleep_us(1);
    //}
}

bool signal_getMPGMode (void)
{
#ifdef UART_MODE
    return false;
#else
    return !gpio_get(MPG_MODE_PIN);
#endif
}

static uint32_t qei_xPos = 0, ymax = 0;

void navigator_setLimits (int16_t min, int16_t max)
{
    ymax = max;
}

bool NavigatorSetPosition (uint32_t xPos, uint32_t yPos, bool callback)
{
    qei_xPos = xPos;
    qei.count = yPos;

    if(callback && interface.on_navigator_event)
        interface.on_navigator_event(WIDGET_MSG_PTR_MOVE, qei_xPos, qei.count);
}

extern uint32_t NavigatorGetYPosition (void)
{
    return qei.count >= 0 ? (uint32_t)qei.count : 0;
}

extern void NavigatorSetEventHandler (on_navigator_event_ptr on_navigator_event)
{
    interface.on_navigator_event = on_navigator_event;
}

void mpg_setActiveAxis (uint_fast8_t axis)
{
    switch(axis) {

        case 0:
            mpg_axis = &mpg.x;
            break;

        case 1:
            mpg_axis = &mpg.y;
            break;

        case 2:
            mpg_axis = &mpg.z;
            break;

        default:
            break;
    }
   mpg_ActiveAxisUpdated(axis);
}

mpg_t *mpg_getPosition (void)
{
    static mpg_t mpg_cur;

    memcpy(&mpg_cur, &mpg, sizeof(mpg_t));

    return &mpg_cur;
}

void mpg_reset (void)
{
    mpg.x.position =
    mpg.y.position =
    mpg.z.position =
    qei_mpg.count = 0;
}

void mpg_setCallback (on_mpgChanged_ptr fn)
{
    interface.on_mpgChanged = fn;
}

//For now...
static void gpio_int_handler (uint gpio, uint32_t events)
{
    switch(gpio) {

        case MPGMODE_SW_PIN:
            mpgMode_sw_int_handler(gpio, events);
            break;
            
        case JOG_RATE_PIN:
            jog_rate_int_handler(gpio, events);
            break;

        case AXIS_TOGGLE_PIN:
            axis_toggle_int_handler(gpio, events);
            break;

        default:
            break;
    }

}

static int64_t debounceMode_callback (alarm_id_t id, void *pin)
{
    gpio_set_irq_enabled(*(uint *)pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    return 0;
}

static void mpgMode_sw_int_handler (uint gpio, uint32_t events)
{
   
    if (!gpio_get(gpio))
    {
          serial_putC(CMD_MPG_MODE_TOGGLE);
    }
    
}
static void jog_rate_int_handler (uint gpio, uint32_t events)
{

    if (!gpio_get(gpio))
    {
         current = current == Maxium ? 0 : current + 1;
         mpg_JogRateToggle(current);
    }
   
}

static void axis_toggle_int_handler (uint gpio, uint32_t events)
{
    if (!gpio_get(gpio))
    {
        if(mpg_axis  == &mpg.x)
        {
        mpg_setActiveAxis(1);
        }
        else if(mpg_axis == &mpg.y)
        {
         mpg_setActiveAxis(2);
        }
        else{
        mpg_setActiveAxis(0);
        }
    }
}



const uint8_t encoder_valid_state[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};



static void mpg_int_handler (void)
{
    static uint32_t microstep_time = 0;

    uint32_t received, idx;
    qei_state_t state = {0};

    while(pio1->ints0 & (PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS << mpg_sm)) {

        received = pio_sm_get(pio1, mpg_sm);

        state.a = !!(received & STATE_A_MASK);
        state.b = !!(received & STATE_B_MASK);

        int32_t time_received = (received & TIME_MASK) + ENC_DEBOUNCE_TIME;

        // For rotary encoders, only every fourth transition is cared about, causing an inaccurate time value
        // To address this we accumulate the times received and zero it when a transition is counted
        if(true) {
            if(time_received + microstep_time < time_received)  //Check to avoid integer overflow
                time_received = INT32_MAX;
            else
                time_received += microstep_time;
            microstep_time = time_received;
        }

        idx = (((qei_mpg.state << 2) & 0x0F) | state.pins);

        if(encoder_valid_state[idx]) {

            qei_mpg.state = ((qei_mpg.state << 4) | idx) & 0xFF;

            if (qei_mpg.state == 0x42 || qei_mpg.state == 0xD4 || qei_mpg.state == 0x2B || qei_mpg.state == 0xBD) {
                qei_mpg.count--;
            } else if(qei_mpg.state == 0x81 || qei_mpg.state == 0x17 || qei_mpg.state == 0xE8 || qei_mpg.state == 0x7E) {
                qei_mpg.count++;
            }

            if(mpg_axis->position != qei_mpg.count) {
                mpg_axis->position = qei_mpg.count;
                mpg_axis->velocity = 100;
                if(interface.on_mpgChanged)
                {
                        interface.on_mpgChanged(mpg);
                }
                   
            }
        }
    }
}
