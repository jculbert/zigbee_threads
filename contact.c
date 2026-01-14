#include PLATFORM_HEADER
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif

#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_i2cspm_instances.h"
#include "sl_si7210.h"

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "sl_emlib_gpio_init_hall_input_config.h"

#include "app/framework/util/attribute-table.h"
#include "app/framework/plugin/reporting/reporting.h"
#include "ias-zone-server.h"

static uint32_t endpoint;
static osThreadId_t thread_id;

static const uint32_t INTERRUPT_FLAG = 1;

static void int_callback(unsigned char intNo)
{
    osThreadFlagsSet(thread_id, INTERRUPT_FLAG);
}

static sl_status_t hall_init()
{
    sl_status_t status = sl_si7210_init(sl_i2cspm_inst0);
    sl_zigbee_app_debug_println("sl_si7210_init: status %d", status);
    if (status == SL_STATUS_OK)
    {
        sl_si7210_configure_t config = {};
        config.threshold = 0.5;
        config.hysteresis = config.threshold / 5.0;

        // Configure sets threshold and hysteresis and enables sleep with periodic measurements
        status = sl_si7210_configure(sl_i2cspm_inst0, &config);
        sl_zigbee_app_debug_println("sl_si7210_configure: status %d", status);
    }
    return status;
}

#if 0
static void read_hall()
{
  float value = 0.123;
  sl_status_t status = sl_si7210_measure(sl_i2cspm_inst0, 1000, &value);
  task->log_debug("read_hall: status %d, value %d", status, (int32_t)(value * 1000));

  // Had trouble getting periodic reading to work and calling
  // measure to we completely init hall after a reading
  hall_init();
}
#endif

static void update_attribute()
{
    uint8_t state = (uint8_t) GPIO_PinInGet(SL_EMLIB_GPIO_INIT_HALL_INPUT_PORT, SL_EMLIB_GPIO_INIT_HALL_INPUT_PIN);
    sl_zigbee_af_status_t result = sl_zigbee_af_write_server_attribute(endpoint, ZCL_OCCUPANCY_SENSING_CLUSTER_ID, ZCL_OCCUPANCY_ATTRIBUTE_ID,
        &state, ZCL_BITMAP8_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_println("update state result=%d, state=%d", result, state);
}

static void set_reporting_table()
{
    // Override min report interval if not set to 1
    // Zigbee2mqtt sets it to 10
    sl_zigbee_af_plugin_reporting_entry_t entry;
    for(int i = 0; i < REPORT_TABLE_SIZE; i++){
        sli_zigbee_af_reporting_get_entry(i, &entry);
        if(entry.endpoint == endpoint && entry.clusterId == ZCL_OCCUPANCY_SENSING_CLUSTER_ID)
        {
            //sl_zigbee_app_debug_println("r %d %d", entry.attributeId, entry.data.reported.minInterval);
            if (entry.data.reported.minInterval != 1)
            {
                sl_zigbee_app_debug_println("updating rep int to 1");
                entry.data.reported.minInterval = 1;
                sli_zigbee_af_reporting_set_entry(i, &entry);
            }

            break;
        }
    }
}

static void thread(void *p_arg)
{
    uint32_t refresh_period = (5*60) * osKernelGetTickFreq();

    // Refresh the state at startup after a delay
    osDelay(5 * osKernelGetTickFreq());
    set_reporting_table();
    update_attribute();

    while(true)
    {
        osThreadFlagsWait(INTERRUPT_FLAG, osFlagsWaitAny, refresh_period);
        set_reporting_table();
        update_attribute();
        sl_zigbee_wakeup_app_framework_task();
    }
}

void contact_init(uint32_t _endpoint)
{
    endpoint = _endpoint;
    static osThreadAttr_t thread_attr;
    __ALIGNED(8) static uint8_t stack[2000];
    __ALIGNED(4) static uint8_t task_cb[osThreadCbSize];

    thread_attr.name = "Contact";
    thread_attr.stack_mem = stack;
    thread_attr.stack_size = sizeof(stack);
    thread_attr.cb_mem = task_cb;
    thread_attr.cb_size = osThreadCbSize;
    thread_attr.priority = osPriorityNormal;
    thread_attr.attr_bits = 0;
    thread_attr.tz_module = 0;

    thread_id = osThreadNew(thread, NULL, &thread_attr);
    assert(thread_id != NULL);

    const int INT_NUM = 4; // Note, valid values depends on pin number (see GPIO_ExtIntConfig)
    GPIOINT_CallbackRegister(INT_NUM, int_callback);
    GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_HALL_INPUT_PORT, SL_EMLIB_GPIO_INIT_HALL_INPUT_PIN, INT_NUM, 1, 1, true); // rising and falling edge, interrupt enabled

    hall_init();
}



