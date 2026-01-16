
#include "app_config.h"

#ifdef EXCEL_OCCUPANCY_ENDPOINT

#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_emlib_gpio_init_serin_config.h"
#include "sl_emlib_gpio_init_motion_input_config.h"

#include "app/framework/util/attribute-table.h"

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

static void inline set_serin()
{
    GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_SERIN_PORT, SL_EMLIB_GPIO_INIT_SERIN_PIN);
}

static void inline clear_serin()
{
    GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_SERIN_PORT, SL_EMLIB_GPIO_INIT_SERIN_PIN);
}

static void clear_direct_link()
{
    // Pull the detector interrupt output low to clear interrupt state
    GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PORT,
        SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, gpioModePushPull, 0);

    // Restore for interrupt input
    GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PORT,
        SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, SL_EMLIB_GPIO_INIT_MOTION_INPUT_MODE,
        SL_EMLIB_GPIO_INIT_MOTION_INPUT_DOUT);
}

static void init_detector()
{
    uint32_t config25bits = 0;

    // BPF threshold 8 bits, 24:17
    config25bits |= 100 << 17;

    // Blind Time, 4 bits, 16:13, 0.5 s and 8 s in steps of 0.5 s.
    config25bits |= 0x01 << 13; // Small value, not needed, we implement blanking logic

    // Pulse counter, 2 bits, 12:11, 1 to 4 pulses to trigger detection
    config25bits |= 1 << 11;

    // Window time, 2 bits, 10:9, 2 s up to 8 s in intervals of 2 s
    //config25bits |= 0; // min window for now

    // Operation Modes, 2 bits, 8:7
     config25bits |= 2 << 7; // 2 for Wake Up, interrupt, mode

    // Signal Source, 2 bits, 6:5,
    //config25bits |= 0; // PIR BPF

    // Reserved, 2 bits, 4:3, must be set to 2
    config25bits |= 2 << 3;

    // HPF Cut-Off, 1 bit, bit 2
    config25bits |= 0; // 0 for 0.4 Hz

    // Reserver, 1 bit, bit 1, must be set to 0
    //config25bits |= 0;

    // Count Mode, 1 bit, bit 0
    config25bits |= 1; // no zero crossing required

    // Send the 25 bit bit train in a loop
    // Start of each bit is marked by low to high
    // for a 0 value, output returns to zero after start pulse
    // for a 1 value, output is held for 80 usec or more
    clear_serin();
    sl_udelay_wait(1000); // Start output low for 1 ms

    const int32_t num_bits = 25;
    uint32_t mask = 1 << 24;
    for (int32_t i = 0; i < num_bits; i++)
    {
        clear_serin();
        sl_udelay_wait(5);
        set_serin();
        sl_udelay_wait(5);

        if ((config25bits & mask) == 0)
        {
            // bit value is zero
            clear_serin();
        }
        sl_udelay_wait(100); // hold bit value
        mask >>= 1;
    }

    // Latch the data
    clear_serin();
    sl_udelay_wait(1000);
    sl_zigbee_app_debug_println("PIR init done");
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
}

static void thread(void *p_arg)
{
    state = STATE_IDLE;
    uint32_t timeout = 0;

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
                clear_direct_link();
                GPIO_IntClear(1<<SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN); // Clear any spurious interrupt
                GPIO_IntEnable(1<<SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN);
                state = STATE_DELAY;
                log_debug("PIR Delay");
                timeout = motion_timeout - motion_blanking_time; // Delay state until the end of delay period
                break;
            case STATE_DELAY:
                state = STATE_IDLE;
                update_occupancy(false);
                timeout = 0;
                break;
            case STATE_IDLE:
            default:
                timeout = 0;
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

void excel_occupancy_init(uint32_t _endpoint, uint32_t motion_timeout_secs)
{
    motion_timeout = timeout_secs * osKernelGetTickFreq();
    motion_blanking_time = 3 * motion_timeout / 4;

    endpoint = _endpoint;
    static osThreadAttr_t thread_attr;
    __ALIGNED(8) static uint8_t stack[2000];
    __ALIGNED(4) static uint8_t task_cb[osThreadCbSize];

    thread_attr.name = "Excel";
    thread_attr.stack_mem = stack;
    thread_attr.stack_size = sizeof(stack);
    thread_attr.cb_mem = task_cb;
    thread_attr.cb_size = osThreadCbSize;
    thread_attr.priority = osPriorityNormal;
    thread_attr.attr_bits = 0;
    thread_attr.tz_module = 0;

    thread_id = osThreadNew(thread, NULL, &thread_attr);
    assert(thread_id != NULL);

    init_detector();

    // Note, the input GPIO has been setup by app config to input no pull
    GPIOINT_CallbackRegister(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, int_callback);
    GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_MOTION_INPUT_PORT, SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN,
        SL_EMLIB_GPIO_INIT_MOTION_INPUT_PIN, 1, 0, true); // rising edge only, interrupt enabled
    clear_direct_link();
}


#endif // EXCEL_OCCUPANCY_ENDPOINT
