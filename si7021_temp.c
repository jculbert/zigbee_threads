#include "app_config.h"

#if defined(SI7021_ENDPOINT)

#include PLATFORM_HEADER

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "app/framework/util/attribute-table.h"

#include "sl_i2cspm_instances.h"
#include "sl_si70xx.h"

static const uint32_t INTERRUPT_FLAG = 1;
static uint32_t endpoint;
static uint32_t period_minutes;
static osThreadId_t thread_id;

static void thread(void *p_arg)
{
    const period_ms = period_minutes * 60 * osKernelGetTickFreq();

    // Trigger a reading soon after startup
    uint32_t wait_ms = 10 * osKernelGetTickFreq();

    while(true)
    {
        osDelay(wait_ms);
        wait_ms = period_ms;

        uint32_t rh_data, temp_data;
        sl_si70xx_measure_rh_and_temp(sl_i2cspm_inst0, SI7021_ADDR, &rh_data, &temp_data);

        int16_t zigbee_temp = (int16_t) (temp_data / 10);
        sl_zigbee_af_status_t result = sl_zigbee_af_write_server_attribute(endpoint,
            ZCL_TEMP_MEASUREMENT_CLUSTER_ID, ZCL_TEMP_MEASURED_VALUE_ATTRIBUTE_ID,
            (uint8_t *) &zigbee_temp, ZCL_INT16S_ATTRIBUTE_TYPE);
        sl_zigbee_wakeup_app_framework_task();
        sl_zigbee_app_debug_println("Temperature update_state result=%d, zigbee_temp=%d", result, zigbee_temp);
    }
}

void si7020_temp_battery_init(uint32_t _endpoint, uint32_t _period_minutes)
{
    endpoint = _endpoint;
    period_minutes = _period_minutes;

    static osThreadAttr_t thread_attr;
    __ALIGNED(8) static uint8_t stack[2000];
    __ALIGNED(4) static uint8_t task_cb[osThreadCbSize];

    thread_attr.name = "Temperature";
    thread_attr.stack_mem = stack;
    thread_attr.stack_size = sizeof(stack);
    thread_attr.cb_mem = task_cb;
    thread_attr.cb_size = osThreadCbSize;
    thread_attr.priority = osPriorityNormal;
    thread_attr.attr_bits = 0;
    thread_attr.tz_module = 0;

    thread_id = osThreadNew(thread, NULL, &thread_attr);
    assert(thread_id != NULL);

    sl_status_t status = sl_si70xx_init(sl_i2cspm_inst0, SI7021_ADDR);
    sl_zigbee_app_debug_println("sl_si7021_init status = %d\n", status);
}

#endif // SI7021_ENDPOINT
