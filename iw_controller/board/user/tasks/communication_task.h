#ifndef  __COMMUNICATION_H__
#define  __COMMUNICATION_H__


extern osThreadId   controller_task_hdl;
extern osMessageQId controller_task_msg_q_id;
void controller_task(void const * argument);



#define  COMMUNICATION_TASK_SERIAL_PORT                 0
#define  COMMUNICATION_TASK_SERIAL_BAUDRATES            115200
#define  COMMUNICATION_TASK_SERIAL_DATABITS             8
#define  COMMUNICATION_TASK_SERIAL_STOPBITS             1


#define  COMMUNICATION_TASK_RX_BUFFER_SIZE              64
#define  COMMUNICATION_TASK_TX_BUFFER_SIZE              64

#define  COMMUNICATION_TASK_COMMUNICATION_ADDR          1




#define  SCALE_CNT_MAX                                  8
#define  COMMUNICATION_TASK_SCALE_DEFAULT_ADDR          1

typedef struct
{
    uint8_t cnt;
    uint8_t value[SCALE_CNT_MAX];
}scale_addr_configration_t;

/*电子秤任务上下文*/
typedef struct
{
    int handle;
    uint8_t port;
    uint32_t baud_rates;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t internal_addr;
    uint8_t phy_addr;
    uint32_t flag;
    osMessageQId msg_q_id;
    osThreadId   task_hdl;
}scale_task_contex_t;

/*通信任务上下文*/
typedef struct
{
    uint8_t cnt;
    bool initialized;
    scale_task_contex_t scale_task_contex[SCALE_CNT_MAX];
    osMessageQId net_weight_rsp_msg_q_id;
    osMessageQId remove_tare_rsp_msg_q_id;
    osMessageQId calibration_zero_rsp_msg_q_id;
    osMessageQId calibration_full_rsp_msg_q_id;
    osMessageQId query_door_status_rsp_msg_q_id;
    osMessageQId query_lock_status_rsp_msg_q_id;
    osMessageQId lock_lock_rsp_msg_q_id;
    osMessageQId unlock_lock_rsp_msg_q_id;
    osMessageQId query_temperature_rsp_msg_q_id;
    osMessageQId set_temperature_rsp_msg_q_id;
}communication_task_contex_t;
    

#endif