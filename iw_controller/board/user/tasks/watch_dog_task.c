#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "watch_dog_task.h"
#include "tasks_init.h"
#include "log.h"

osThreadId   watch_dog_task_hdl;

/*
* @brief 看门狗任务
* @param argument 任务参数
* @param
* @return 无
* @note
*/
void watch_dog_task(void const * argument)
{

    while (1) {
        osDelay(WATCH_DOG_TASK_INTERVAL);
    }
}  
  