/***************************************************************************//**
 * @file app.c
 * @brief main app.c for ZigbeeMinimalRTOS
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include PLATFORM_HEADER

#include "app/framework/include/af.h"

#include "sl_led.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "network-steering.h"

extern void sl_zigbee_wakeup_app_framework_task(void);

//#define led_turn_on(led) sl_led_turn_on(led)
#define led_turn_on(led) sl_led_turn_on(led)
#define led_turn_off(led) sl_led_turn_off(led)
#define led_toggle(led) sl_led_toggle(led)
#define COMMISSIONING_STATUS_LED (&sl_led_network_status)

#define BASE_ENDPOINT (1)

#define LED_BLINK_ONTIME_MS      (1000)

static uint32_t tick_rate;

static sl_zigbee_af_event_t button_isr_event;
static const uint32_t BUTTON_LONG_PRESS_SECS = 3;
static uint32_t button_pressed_time;

// Event use for periodic network monitoring
static sl_zigbee_af_event_t network_event;
static const uint32_t NETWORK_MONITOR_PERIOD_SECS = 60;

// LED indication is as follows
// - pulse once if commission button is pressed and network already up
// - pulse twice upon network up callback
// - pulse three times upon commission button long press to leave the network
static sl_zigbee_af_event_t led_event;
static bool led_on = false;
static uint32_t led_pulse_cnt = 0;

extern void water_detector_init(uint32_t _endpoint);
extern void battery_init(uint32_t _endpoint, uint32_t _nominal_mv, uint32_t _refresh_minutes);

// Event handler that pulses the led_pulse_cnt times
static void led_event_handler(sl_zigbee_af_event_t *event)
{
    if (led_on)
    {
        led_on = false;
        led_turn_off(COMMISSIONING_STATUS_LED);
        if (led_pulse_cnt > 0)
            sl_zigbee_af_event_set_delay_ms(&led_event, LED_BLINK_ONTIME_MS);
        return;
    }

    if (led_pulse_cnt > 0)
    {
        led_pulse_cnt--;
        // start of an on/off cycle
        led_on = true;
        led_turn_on(COMMISSIONING_STATUS_LED);
        sl_zigbee_af_event_set_delay_ms(&led_event, LED_BLINK_ONTIME_MS);
    }
}

static void network_event_handler(sl_zigbee_af_event_t *event)
{
    sl_zigbee_network_status_t status = sl_zigbee_af_network_state();
    sl_zigbee_app_debug_println("Network state: 0x%02X", status);
    sl_zigbee_af_event_set_delay_ms(&network_event, NETWORK_MONITOR_PERIOD_SECS * tick_rate);
}

static void start_commissioning()
{
    if (sl_zigbee_af_network_state() == SL_ZIGBEE_JOINED_NETWORK) {
        sl_zigbee_app_debug_println("Already joined network");

        // pulse led once
        led_pulse_cnt = 1;
        sl_zigbee_af_event_set_active(&led_event);

    } else {
        sl_status_t status = sl_zigbee_af_network_steering_start();
        sl_zigbee_app_debug_println("Join network start status: 0x%02X", status);
    }
}

static void leave_network()
{
    sl_status_t status = sl_zigbee_leave_network(SL_ZIGBEE_LEAVE_NWK_WITH_NO_OPTION);
    sl_zigbee_app_debug_println("Leave network status: 0x%02X", status);

    // pulse led 3 times
    led_pulse_cnt = 3;
    sl_zigbee_af_event_set_active(&led_event);
}

static void button_isr_event_handler(sl_zigbee_af_event_t *event)
{
    switch(sl_button_get_state(&sl_button_network))
    {
    case SL_SIMPLE_BUTTON_PRESSED:
        button_pressed_time = osKernelGetTickCount();
        break;
    case SL_SIMPLE_BUTTON_RELEASED:
        uint32_t dt = (osKernelGetTickCount() - button_pressed_time) / osKernelGetTickFreq();
        if (dt >= BUTTON_LONG_PRESS_SECS)
            //sl_zigbee_app_debug_println("Long press");
            leave_network();
        else
            //sl_zigbee_app_debug_println("Short press");
            start_commissioning();
        break;
    default:
        break;
    }
}

void sli_zigbee_app_rtos_task_init_cb(void)
{
    water_detector_init(1);
    battery_init(1, 4000, 1);

    tick_rate = osKernelGetTickFreq();

    sl_zigbee_af_isr_event_init(&button_isr_event, button_isr_event_handler);
    sl_zigbee_af_event_init(&led_event, led_event_handler);
    //sl_zigbee_af_event_init(&network_event, network_event_handler);
    //sl_zigbee_af_event_set_delay_ms(&network_event, NETWORK_MONITOR_PERIOD_SECS * tick_rate);
}

//----------------------
// Implemented Callbacks

/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action. The framework
 * will always process the stack status after the callback returns.
 */
void sl_zigbee_af_stack_status_cb(sl_status_t status)
{
    // Note, the ZLL state is automatically updated by the stack and the plugin.
    if (status == SL_STATUS_NETWORK_UP) {
        sl_zigbee_app_debug_println("Stack status up");

        // pulse led twice
        led_pulse_cnt = 2;
        sl_zigbee_af_event_set_active(&led_event);
    }
    else
    {
        sl_zigbee_app_debug_println("Stack status 0x%02X", status);
    }
}

/** @brief Complete network steering.
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt.
 *
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID.
 *
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network.
 *
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete.
 */
void sl_zigbee_af_network_steering_complete_cb(sl_status_t status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{
    sl_zigbee_app_debug_println("Join network complete: 0x%02X", status);
}

/** @brief Post Attribute Change
 *
 * This function is called by the application framework after it changes an
 * attribute value. The value passed into this callback is the value to which
 * the attribute was set by the framework.
 */
void sl_zigbee_af_post_attribute_change_cb(uint8_t endpoint,
                                           sl_zigbee_af_cluster_id_t clusterId,
                                           sl_zigbee_af_attribute_id_t attributeId,
                                           uint8_t mask,
                                           uint16_t manufacturerCode,
                                           uint8_t type,
                                           uint8_t size,
                                           uint8_t* value)
{
}

/** @brief On/off Cluster Server Post Init
 *
 * Following resolution of the On/Off state at startup for this endpoint, perform any
 * additional initialization needed; e.g., synchronize hardware state.
 *
 * @param endpoint Endpoint that is being initialized
 */
void sl_zigbee_af_on_off_cluster_server_post_init_cb(uint8_t endpoint)
{
}

/** @brief
 *
 * Application framework equivalent of ::emberRadioNeedsCalibratingHandler
 */
void sl_zigbee_af_radio_needs_calibrating_cb(void)
{
#ifndef EZSP_HOST
    sl_mac_calibrate_current_channel();
#endif
}

/***************************************************************************//**
 * A callback called in interrupt context whenever a button changes its state.
 *
 * @remark Can be implemented by the application if required. This function
 * can contain the functionality to be executed in response to changes of state
 * in each of the buttons, or callbacks to appropriate functionality.
 *
 * @note The button state should not be updated in this function, it is updated
 * by specific button driver prior to arriving here
 *
   @param[out] handle             Pointer to button instance
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
    if (handle != &sl_button_network)
      return;

    sl_zigbee_af_event_set_active(&button_isr_event);
    sl_zigbee_wakeup_app_framework_task();

#ifdef SL_CATALOG_ZIGBEE_FORCE_SLEEP_AND_WAKEUP_PRESENT
    sl_zigbee_app_framework_force_wakeup();
#endif //SL_CATALOG_ZIGBEE_FORCE_SLEEP_AND_WAKEUP_PRESENT
}

#ifdef SL_CATALOG_ZIGBEE_FORCE_SLEEP_AND_WAKEUP_PRESENT
void sli_zigbee_app_framework_force_sleep_callback(void)
{
    // Do other things like turn off LEDs etc
    sl_led_turn_off(&sl_led_led0);
}
#endif // SL_CATALOG_ZIGBEE_FORCE_SLEEP_AND_WAKEUP_PRESENT

