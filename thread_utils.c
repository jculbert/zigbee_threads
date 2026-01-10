/*
 * task_utils.c
 *
 *  Created on: Jun. 9, 2025
 *      Author: jeff
 */


#if 0
task_create()
{
    static osThreadAttr_t zigbee_app_framework_task_attr;

    zigbee_app_framework_task_attr.name = "Zigbee AppFramework";
    zigbee_app_framework_task_attr.stack_mem = &zigbee_app_framework_task_stack[0];
    zigbee_app_framework_task_attr.stack_size = sizeof(zigbee_app_framework_task_stack);
    zigbee_app_framework_task_attr.cb_mem = zigbee_app_framework_task_cb;
    zigbee_app_framework_task_attr.cb_size = osThreadCbSize;
    zigbee_app_framework_task_attr.priority = (osPriority_t)SL_ZIGBEE_APP_FRAMEWORK_RTOS_TASK_PRIORITY;
    zigbee_app_framework_task_attr.attr_bits = 0;
    zigbee_app_framework_task_attr.tz_module = 0;

    zigbee_app_framework_task_id = osThreadNew(zigbee_app_framework_task,
                                             NULL,
                                             &zigbee_app_framework_task_attr);
    assert(zigbee_app_framework_task_id != NULL);

    zigbee_app_framework_task_semaphore_id =   osSemaphoreNew(ZIGBEE_TASK_SEMAPHORE_MAX_COUNT,
                                                            ZIGBEE_TASK_SEMAPHORE_INITIAL_COUNT,
                                                            &zigbee_app_framework_task_semaphore_attr);
    assert(zigbee_app_framework_task_semaphore_id != NULL);
}
#endif

