/*
 * battery.c
 *
 *  Created on: Nov 16, 2025
 *      Author: jeff
 */

// IADC code copied from https://github.com/SiliconLabs/peripheral_examples/blob/master/series2/iadc/iadc_single_interrupt/src/main_single_interrupt.c

#include PLATFORM_HEADER

#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "app/framework/util/attribute-table.h"

static uint32_t endpoint;
static uint32_t nominal_mv;
static uint32_t refresh_minutes;
static osThreadId_t thread_id;
static const uint32_t INTERRUPT_FLAG = 1;
static bool battery_type_lithium = true;

void IADC_IRQHandler(void)
{
    /*
    * Clear the single conversion complete interrupt.  Reading FIFO
    * results does not do this automatically.
    */
    IADC_clearInt(IADC0, IADC_IF_SINGLEDONE);

    osThreadFlagsSet(thread_id, INTERRUPT_FLAG);
}

static void init_iadc(void)
{
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;

    CMU_ClockEnable(cmuClock_IADC0, true);

    // Use the FSRC0 as the IADC clock so it can run in EM2
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

#if 0
    // Set the prescaler needed for the intended IADC clock frequency
    // Set CLK_ADC to 10 MHz
    const uint32_t CLK_SRC_ADC_FREQ = 20000000;
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);
#endif

    // Set the prescaler needed for the intended IADC clock frequency
    //const uint32_t CLK_ADC_FREQ = 10000000;  // CLK_ADC - 10 MHz max in normal mode
    const uint32_t CLK_ADC_FREQ = 10000000;  // CLK_ADC - 10 MHz max in normal mode
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_ADC_FREQ, 0);

    // Shutdown between conversions to reduce current
    init.warmup = iadcWarmupNormal;

    /*
     * Configuration 0 is used by both scan and single conversions by
     * default.  Use internal bandgap as the reference and specify the
     * reference voltage in mV.
     *
     * Resolution is not configurable directly but is based on the
     * selected oversampling ratio (osrHighSpeed), which defaults to
     * 2x and generates 12-bit results.
     */
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].vRef = 1210;
    initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;

    /*
     * CLK_SRC_ADC must be prescaled by some value greater than 1 to
     * derive the intended CLK_ADC frequency.
     *
     * Based on the default 2x oversampling rate (OSRHS)...
     *
     * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
     *
     * ...which results in a maximum sampling rate of 833 ksps with the
     * 2-clock input multiplexer switching time is included.
     */
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
        CLK_ADC_FREQ, 0, iadcCfgModeNormal, init.srcClkPrescale);

    /*
     * Specify the input channel.  When negInput = iadcNegInputGnd, the
     * conversion is single-ended.
     */
    singleInput.posInput   = iadcPosInputAvdd;
    singleInput.negInput   = iadcNegInputGnd;

    // Allocate the analog bus for ADC0 inputs
    //GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC; // Needed?

    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize a single-channel conversion
    IADC_initSingle(IADC0, &initSingle, &singleInput);

    // Clear any previous interrupt flags
    IADC_clearInt(IADC0, _IADC_IF_MASK);

    // Enable single-channel done interrupts
    IADC_enableInt(IADC0, IADC_IEN_SINGLEDONE);

    // Enable IADC interrupts
    NVIC_ClearPendingIRQ(IADC_IRQn);
    NVIC_EnableIRQ(IADC_IRQn);
}

// Returns battery percent in zigbee units (int percent * 2)
// battery percent is in 1/2 percent units, so 200 is 100 percent
static uint8_t get_zigbee_battery_percent(uint32_t vbat_mv)
{
    typedef struct { float v; int p; } vp_t;

    // Curve for NiMH
    static const vp_t curve_nimh[] = {
        {1.45f, 100},
        {1.35f,  90},
        {1.30f,  80},
        {1.25f,  65},
        {1.20f,  50},
        {1.15f,  30},
        {1.10f,  12},
        {1.05f,   3},
        {1.00f,   0}
    };

    // Curve for Lithium
    static const vp_t curve_lithium[] = {
        {3.00f, 100},
        {2.95f,  90},
        {2.90f,  80},
        {2.85f,  60},
        {2.80f,  40},
        {2.75f,  20},
        {2.70f,  10},
        {2.50f,   0},
    };

    uint32_t curve_len;
    vp_t *curve;
    float vbat;
    if (battery_type_lithium) {
        curve = curve_lithium;
        curve_len = sizeof(curve_lithium) / sizeof(curve_lithium[0]);
        vbat = ((float)vbat_mv) / 1000.0f;
    } else {
        curve = curve_nimh;
        curve_len = sizeof(curve_nimh) / sizeof(curve_nimh[0]);
        vbat = ((float)vbat_mv) / 2000.0f; // vbat_mv is 2x battery mv for nimh
    }

    if (vbat >= curve[0].v) return 200;
    if (vbat <= curve[curve_len-1].v) return 0;
    for (uint32_t i = 0; i < curve_len; ++i) {
        float v1 = curve[i].v, v2 = curve[i+1].v;
        if (vbat <= v1 && vbat >= v2) {
            // interpolate between the two points
            float p1 = (float)curve[i].p, p2 = (float)curve[i+1].p;
            float t = (v1 - vbat) / (v1 - v2);
            return (uint8_t) (2.0f * (p1 - t * (p1 - p2) + 0.5f));
        }
    }
    return 0; // should be impossible
}

static void thread(void *p_arg)
{
    uint32_t refresh_period = refresh_minutes * 60 * osKernelGetTickFreq();

    // Trigger a reading soon after startup
    osDelay(10 * osKernelGetTickFreq());
    init_iadc();
    IADC_command(IADC0, iadcCmdStartSingle);

    while(true)
    {
        osStatus_t flags = osThreadFlagsWait(INTERRUPT_FLAG, osFlagsWaitAny, refresh_period);
        if (flags == osErrorTimeout) {
            // timeout means time for another reading
            init_iadc();
            IADC_command(IADC0, iadcCmdStartSingle);
            continue;
        }

        // ADC sample completed
        // Read a result from the FIFO
        IADC_Result_t sample = IADC_pullSingleFifoResult(IADC0);

        // 1210 for the bandgap reference, and 4 because we convert Vdd/4
        uint32_t vbat_mv = (sample.data * 4 * 1210) / 0xFFF;

        // reset ADC and disable clock between measurements to reduce power
        IADC_reset(IADC0);
        CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_Disabled);
        CMU_ClockEnable(cmuClock_IADC0, false);

        uint8_t battery_percent = get_zigbee_battery_percent(vbat_mv);

        sl_zigbee_af_status_t result = sl_zigbee_af_write_server_attribute(endpoint,
            ZCL_POWER_CONFIG_CLUSTER_ID, ZCL_BATTERY_PERCENTAGE_REMAINING_ATTRIBUTE_ID,
            &battery_percent, ZCL_INT8U_ATTRIBUTE_TYPE);
        sl_zigbee_app_debug_println("Battery update_state result=%d, batt_percent=%d", result, battery_percent);
    }
}

void battery_init(uint32_t _endpoint, uint32_t _nominal_mv, uint32_t _refresh_minutes)
{
    endpoint = _endpoint;
    nominal_mv = _nominal_mv;
    refresh_minutes = _refresh_minutes;

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
}

# if 0
// CR2032
typedef struct {
    float voltage;
    int percent;
} vp_t;

static const vp_t curve[] = {
    {3.00, 100},
    {2.95,  90},
    {2.90,  80},
    {2.85,  60},
    {2.80,  40},
    {2.75,  20},
    {2.70,  10},
    {2.50,   0},
};

int battery_percent(float v)
{
    if (v >= curve[0].voltage) return 100;
    if (v <= curve[7].voltage) return 0;

    for (int i = 0; i < 7; i++) {
        float v1 = curve[i].voltage;
        float v2 = curve[i+1].voltage;

        if (v <= v1 && v >= v2) {
            float p1 = curve[i].percent;
            float p2 = curve[i+1].percent;

            float t = (v1 - v) / (v1 - v2);
            return (int)(p1 + t * (p2 - p1));
        }
    }

    return 0;
}

// AAA Rechargable
typedef struct { float v; int p; } vp_t;

static const vp_t curve[] = {
    {1.45f, 100},
    {1.35f,  90},
    {1.30f,  80},
    {1.25f,  65},
    {1.20f,  50},
    {1.15f,  30},
    {1.10f,  12},
    {1.05f,   3},
    {1.00f,   0}
};

int aaa_percent(float v) {
    if (v >= curve[0].v) return 100;
    if (v <= curve[8].v) return 0;
    for (int i = 0; i < 8; ++i) {
        float v1 = curve[i].v, v2 = curve[i+1].v;
        if (v <= v1 && v >= v2) {
            float p1 = (float)curve[i].p, p2 = (float)curve[i+1].p;
            float t = (v1 - v) / (v1 - v2);
            return (int) (p1 + t * (p2 - p1) + 0.5f);
        }
    }
    return 0;
}

#endif
