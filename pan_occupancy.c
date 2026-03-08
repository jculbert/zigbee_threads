
#include "app_config.h"

#ifdef PAN_OCCUPANCY_ENDPOINT

#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_emlib_gpio_init_motion_input_config.h"

#include "app/framework/util/attribute-table.h"
#include "app/framework/plugin/reporting/reporting.h"

#include "sl_udelay.h"

static uint32_t endpoint;
static osThreadId_t thread_id;

enum {STATE_IDLE, STATE_BLANKING, STATE_DELAY} state;
uint32_t motion_timeout; // period of no motion before sending state off
uint32_t motion_blanking_time; // period for disabling interrupts after motion detected

static const uint32_t INTERRUPT_FLAG = 1;

static void int_callback(unsigned char intNo)
{
    GPIO_IntDisable(1<<SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN);
    osThreadFlagsSet(thread_id, INTERRUPT_FLAG);
}

static void set_reporting_table()
{
    // Override min report interval if not set to 0
    // Zigbee2mqtt sets it to 10
    sl_zigbee_af_plugin_reporting_entry_t entry;
    for(int i = 0; i < REPORT_TABLE_SIZE; i++){
        sli_zigbee_af_reporting_get_entry(i, &entry);
        if(entry.endpoint == endpoint && entry.clusterId == ZCL_OCCUPANCY_SENSING_CLUSTER_ID)
        {
            //sl_zigbee_app_debug_println("r %d %d", entry.attributeId, entry.data.reported.minInterval);
            if (entry.data.reported.minInterval != 0)
            {
                sl_zigbee_app_debug_println("updating rep int to 1");
                entry.data.reported.minInterval = 0;
                sli_zigbee_af_reporting_set_entry(i, &entry);
            }

            break;
        }
    }
}

void update_occupancy(bool _occupancy)
{
    uint8_t occupancy = _occupancy ? 1: 0;
    sl_zigbee_af_status_t result = sl_zigbee_af_write_server_attribute(endpoint, ZCL_OCCUPANCY_SENSING_CLUSTER_ID, ZCL_OCCUPANCY_ATTRIBUTE_ID,
        &occupancy, ZCL_BITMAP8_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_println("update_occupancy result=%d, state=%d", result, occupancy);
    sl_zigbee_wakeup_app_framework_task();
}

static void thread(void *p_arg)
{
    state = STATE_IDLE;
    // Since timeout=0 returns immediately, use a long timeout value for the "0" case
    const uint32_t long_timeout = 30*60*osKernelGetTickFreq();
    uint32_t timeout = long_timeout;

    set_reporting_table();

    while(true)
    {
        osStatus_t flags = osThreadFlagsWait(INTERRUPT_FLAG, osFlagsWaitAny, timeout);
        if (flags == osErrorTimeout)
        {
            // Woke up due to timeout
            switch (state)
            {
            case STATE_BLANKING:
                // End of blanking period
                GPIO_IntClear(1<<SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN); // Clear any spurious interrupt
                GPIO_IntEnable(1<<SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN);
                state = STATE_DELAY;
                sl_zigbee_app_debug_println("PIR Delay");
                timeout = motion_timeout - motion_blanking_time; // Delay state until the end of delay period
                break;
            case STATE_DELAY:
                state = STATE_IDLE;
                update_occupancy(false);
                timeout = long_timeout;
                break;
            case STATE_IDLE:
            default:
                timeout = long_timeout;
                break;
            }
        }
        else
        {
            // Woke up due to motion
            if (state == STATE_IDLE)
            {
                state = STATE_BLANKING;
                update_occupancy(true);
            }
            else
                state = STATE_BLANKING;

            sl_zigbee_app_debug_println("PIR Blanking");
            timeout = motion_blanking_time; // Start/restart blanking timeout, leave interrupts disabled
        }
    }
}

void pan_occupancy_init(uint32_t _endpoint, uint32_t motion_timeout_secs)
{
    motion_timeout = motion_timeout_secs * osKernelGetTickFreq();
    motion_blanking_time = 3 * motion_timeout / 4;

    endpoint = _endpoint;
    static osThreadAttr_t thread_attr;
    __ALIGNED(8) static uint8_t stack[2000];
    __ALIGNED(4) static uint8_t task_cb[osThreadCbSize];

    thread_attr.name = "Panocc";
    thread_attr.stack_mem = stack;
    thread_attr.stack_size = sizeof(stack);
    thread_attr.cb_mem = task_cb;
    thread_attr.cb_size = osThreadCbSize;
    thread_attr.priority = osPriorityNormal;
    thread_attr.attr_bits = 0;
    thread_attr.tz_module = 0;

    thread_id = osThreadNew(thread, NULL, &thread_attr);
    assert(thread_id != NULL);

    GPIOINT_CallbackRegister(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, int_callback);
    GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PORT, SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN,
        SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, 1, 0, true); // rising edge only, interrupt enabled
}


#endif // EXCEL_OCCUPANCY_ENDPOINT
