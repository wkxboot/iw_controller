#ifndef  __DOOR_LOCK_TASK_H__
#define  __DOOR_LOCK_TASK_H__



extern osThreadId   door_lock_task_hdl;
extern osMessageQId door_lock_task_msg_q_id;
void door_lock_task(void const * argument);


#define  DOOR_LOCK_TASK_MSG_WAIT_TIMEOUT_VALUE       osWaitForever
#define  DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE        5
#define  DOOR_LOCK_TASK_LOCK_TIMEOUT_VALUE           1700
#define  DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE   10
/*状态稳定时间*/
#define  LOCK_DOOR_TASK_STATUS_STABLE_TIME           100


/*锁任务状态和结果定义*/
#define  LOCK_TASK_STATUS_DOOR_OPEN                  1
#define  LOCK_TASK_STATUS_DOOR_CLOSE                 2

#define  LOCK_TASK_STATUS_LOCK_LOCKED                3
#define  LOCK_TASK_STATUS_LOCK_UNLOCKED              4

#define  LOCK_TASK_RESULT_SUCCESS                    5
#define  LOCK_TASK_RESULT_FAIL                       6

/*锁任务消息*/

enum
{
    LOCK_TASK_MSG_TYPE_REQ_LOCK_LOCK,
    LOCK_TASK_MSG_TYPE_REQ_UNLOCK_LOCK,
    LOCK_TASK_MSG_TYPE_QUERY_LOCK_STATUS,
    LOCK_TASK_MSG_TYPE_QUERY_DOOR_STATUS,
    LOCK_TASK_MSG_TYPE_RSP_LOCK_LOCK_RESULT,
    LOCK_TASK_MSG_TYPE_RSP_UNLOCK_LOCK_RESULT,
    LOCK_TASK_MSG_TYPE_RSP_QUERY_LOCK_STATUS,
    LOCK_TASK_MSG_TYPE_RSP_QUERY_DOOR_STATUS
};

typedef struct
{
    union 
    {
    struct 
    {  
        uint8_t type;/*请求消息类型*/
        osMessageQId rsp_message_queue_id;/*回应的消息队列id*/
    }request;
    struct
    {
        uint8_t type;/*回应的消息类型*/
        uint8_t result;/*回应的操作结果*/
        uint8_t status;/*回应的状态值*/
    }response;
    };
}lock_task_message_t;/*锁任务任务消息体*/






#endif