#include "cmsis_os.h"
#include "cpu_task.h"
#include "scale_task.h"
#include "tasks_init.h"
#include "adc_task.h"
#include "lock_task.h"
#include "temperature_task.h"
#include "compressor_task.h"
#include "communication_task.h"
#include "firmware_version.h"
#include "log.h"

/*
* @brief 任务初始化
* @param 无
* @return 无
* @note
*/

void tasks_init(void)
{

    /*通信消息队列*/
    osMessageQDef(communication_task_msg_q,2,uint32_t);
    communication_task_msg_q_id = osMessageCreate(osMessageQ(communication_task_msg_q),0);
    log_assert(communication_task_msg_q_id);

    /*温度消息队列*/
    osMessageQDef(temperature_task_msg_q,2,uint32_t);
    temperature_task_msg_q_id = osMessageCreate(osMessageQ(temperature_task_msg_q),0);
    log_assert(temperature_task_msg_q_id);

    /*压缩机消息队列*/
    osMessageQDef(compressor_task_msg_q,2,uint32_t);
    compressor_task_msg_q_id = osMessageCreate(osMessageQ(compressor_task_msg_q),0);
    log_assert(compressor_task_msg_q_id);

    /*锁控消息队列*/
    osMessageQDef(lock_task_msg_q,2,uint32_t);
    lock_task_msg_q_id = osMessageCreate(osMessageQ(lock_task_msg_q),0);
    log_assert(lock_task_msg_q_id);

    /*cpu任务*/
    osThreadDef(cpu_task, cpu_task, osPriorityNormal, 0, 128);
    cpu_task_hdl = osThreadCreate(osThread(cpu_task), NULL);
    log_assert(cpu_task_hdl);

    /*锁控任务*/
    osThreadDef(lock_task, lock_task, osPriorityNormal, 0, 256);
    lock_task_hdl = osThreadCreate(osThread(lock_task), NULL);
    log_assert(lock_task_hdl);

    /*压缩机任务*/
    osThreadDef(compressor_task, compressor_task, osPriorityNormal, 0, 256);
    compressor_task_hdl = osThreadCreate(osThread(compressor_task), NULL);
    log_assert(compressor_task_hdl);

    /*ADC任务*/
    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, 256);
    adc_task_hdl = osThreadCreate(osThread(adc_task), NULL);
    log_assert(adc_task_hdl);

    /*温度任务*/
    osThreadDef(temperature_task, temperature_task, osPriorityNormal, 0, 256);
    temperature_task_hdl = osThreadCreate(osThread(temperature_task), NULL);
    log_assert(temperature_task_hdl);

    /*主控器通信任务*/
    osThreadDef(communication_task, communication_task, osPriorityNormal, 0, 256);
    communication_task_hdl = osThreadCreate(osThread(communication_task), NULL);
    log_assert(communication_task_hdl);


    log_info("firmware version: %s.\r\n",FIRMWARE_VERSION_STR);

}

