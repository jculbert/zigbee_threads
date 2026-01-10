/*
 * water_detector.c
 *
 *  Created on: Jun. 9, 2025
 *      Author: jeff
 */

#include PLATFORM_HEADER
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "app/framework/util/attribute-table.h"
#include "app/framework/plugin/reporting/reporting.h"
#include "ias-zone-server.h"

#define ACMP_INPUT_PORT       gpioPortA
#define ACMP_INPUT_PIN        4
#define ACMP_INPUT_PORT_PIN   acmpInputPA4

static uint32_t endpoint;
static osThreadId_t thread_id;

static const uint32_t INTERRUPT_FLAG = 1;

static void init_ACMP(void)
{
    CMU_ClockEnable(cmuClock_ACMP0, true);

    // Initialize with default settings
    ACMP_Init_TypeDef init = ACMP_INIT_DEFAULT;
    init.vrefDiv = 4; // Divide reference by 2
    ACMP_Init(ACMP0, &init);

    // Allocate BODD0 to ACMP0 to be able to use the input
    // Note this is A or B and odd or even GPIO specific
    GPIO->ABUSALLOC = GPIO_ABUSALLOC_AEVEN0_ACMP0;

    /*
    * Configure ACMP0 to compare the specified input pin against the
    * selected and divided reference (which defaults to divide-by-1 if
    * the ACMP_INIT_DEFAULT settings are not overridden).
    *
    * In this example, the internal 1.25 V reference is used undivided
    * so that when the input is lower than 1.25 V, the ACMP output is 0,
    * and when it's higher than 1.25 V, the ACMP output is 1.
    */
    ACMP_ChannelSet(ACMP0, acmpInputVREFDIV1V25, ACMP_INPUT_PORT_PIN);

    // Wait for warm-up
    while (!(ACMP0->IF & ACMP_IF_ACMPRDY));

    // Clear pending ACMP interrupts
    NVIC_ClearPendingIRQ(ACMP0_IRQn);
    ACMP_IntClear(ACMP0, ACMP_IF_RISE | ACMP_IF_FALL);

    // Enable ACMP interrupts
    NVIC_EnableIRQ(ACMP0_IRQn);
    ACMP_IntEnable(ACMP0, ACMP_IEN_RISE | ACMP_IEN_FALL);
}

void ACMP0_IRQHandler(void)
{
    // need to use sl_zigbee_af_isr_event_init for events called from ISR context
    //
    if(ACMP0->IF & ACMP_IF_RISE)
    {
        ACMP_IntClear(ACMP0, ACMP_IF_RISE);
    }
    if(ACMP0->IF & ACMP_IF_FALL)
    {
        ACMP_IntClear(ACMP0, ACMP_IF_FALL);
    }

    osThreadFlagsSet(thread_id, INTERRUPT_FLAG);
}

static void update_attribute()
{
    uint16_t status = ACMP_OutputGet(ACMP0) ? 1: 0;
    sl_zigbee_af_status_t result = sl_zigbee_af_ias_zone_server_update_zone_status(endpoint, status, 0);
    sl_zigbee_app_debug_println("Water update_status result=%d, status=%d", result, status);
}

static void thread(void *p_arg)
{
    uint32_t refresh_period = (5*60) * osKernelGetTickFreq();
    uint32_t debounce_period = osKernelGetTickFreq() / 3;
    uint32_t timeout = refresh_period;

    // Refresh the state at startup after a delay
    osDelay(5 * osKernelGetTickFreq());
    update_attribute();

    while(true)
    {
        osStatus_t flags = osThreadFlagsWait(INTERRUPT_FLAG, osFlagsWaitAny, timeout);
        if (flags == osErrorTimeout) {
            // Woke up due to timeout
            timeout = refresh_period;
            update_attribute();
        } else {
            // Woke up due to state change
            // Keep waiting for debounce period until no further interrupt
            timeout = debounce_period;
        }
    }
}

void water_detector_init(uint32_t _endpoint)
{
    endpoint = _endpoint;
    static osThreadAttr_t thread_attr;
    __ALIGNED(8) static uint8_t stack[2000];
    __ALIGNED(4) static uint8_t task_cb[osThreadCbSize];

    thread_attr.name = "Water Detector";
    thread_attr.stack_mem = stack;
    thread_attr.stack_size = sizeof(stack);
    thread_attr.cb_mem = task_cb;
    thread_attr.cb_size = osThreadCbSize;
    thread_attr.priority = osPriorityNormal;
    thread_attr.attr_bits = 0;
    thread_attr.tz_module = 0;

    thread_id = osThreadNew(thread, NULL, &thread_attr);
    assert(thread_id != NULL);

    init_ACMP();
}

