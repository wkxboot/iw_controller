#include "cmsis_os.h"
#include "tasks_init.h"
#include "board.h"
#include "compressor_task.h"
#include "log.h"

/*任务句柄*/
osThreadId   compressor_task_hdl;
/*消息句柄*/
osMessageQId compressor_task_msg_q_id;

/*定时器句柄*/
osTimerId    compressor_work_timer_id;
osTimerId    compressor_wait_timer_id;
osTimerId    compressor_rest_timer_id;
osTimerId    compressor_pwr_wait_timer_id;


typedef enum
{
    COMPRESSOR_STATUS_INIT = 0,     /*上电后的状态*/
    COMPRESSOR_STATUS_RDY,          /*正常关机后的就绪状态*/
    COMPRESSOR_STATUS_RDY_CONTINUE, /*长时间工作停机后的就绪状态*/
    COMPRESSOR_STATUS_REST,         /*长时间工作后的停机状态*/
    COMPRESSOR_STATUS_WORK,         /*压缩机工作状态*/
    COMPRESSOR_STATUS_WAIT,         /*两次开机之间的状态*/
    COMPRESSOR_STATUS_WAIT_CONTINUE /*异常时两次开机之间的状态*/
}compressor_status_t;

typedef struct
{
    int8_t stop;
    int8_t work;
}compressor_temperature_level_t;

typedef struct
{
    compressor_status_t status;
    int8_t temperature_int;
    float temperature_float;
    uint8_t level;
    int8_t temperature_stop;
    int8_t temperature_work;
    bool   temperature_err;
    compressor_temperature_level_t temperature_level[COMPRESSOR_TASK_TEMPERATURE_LEVEL_CNT];
}compressor_t;

/*压缩机对象实体*/
static compressor_t compressor ={
.status = COMPRESSOR_STATUS_INIT,
.temperature_level[0] = {4, 8},
.temperature_level[1] = {9,13},
.temperature_level[2] = {14,18},
.temperature_level[3] = {19,23}
};

static void compressor_work_timer_init(void);
static void compressor_wait_timer_init(void);
static void compressor_rest_timer_init(void);

static void compressor_work_timer_start(void);
static void compressor_wait_timer_start(void);
static void compressor_rest_timer_start(void);


static void compressor_work_timer_stop(void);
/*
static void compressor_wait_timer_stop(void);
static void compressor_rest_timer_stop(void);
*/

static void compressor_work_timer_expired(void const *argument);
static void compressor_wait_timer_expired(void const *argument);
static void compressor_rest_timer_expired(void const *argument);
static void compressor_pwr_wait_timer_expired(void const *argument);

static void compressor_pwr_turn_on();
static void compressor_pwr_turn_off();


/*
* @brief 工作时间定时器
* @param 无
* @return 无
* @note
*/
static void compressor_work_timer_init(void)
{
    osTimerDef(compressor_work_timer,compressor_work_timer_expired);
    compressor_work_timer_id=osTimerCreate(osTimer(compressor_work_timer),osTimerOnce,0);
    log_assert(compressor_work_timer_id);
}
/*
* @brief 工作时间定时器启动
* @param 无
* @return 无
* @note
*/
static void compressor_work_timer_start(void)
{
    osTimerStart(compressor_work_timer_id,COMPRESSOR_TASK_WORK_TIMEOUT);  
}
/*
* @brief 工作时间定时器停止
* @param 无
* @return 无
* @note
*/
static void compressor_work_timer_stop(void)
{
    osTimerStop(compressor_work_timer_id);  
}
/*
* @brief 工作时间定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void compressor_work_timer_expired(void const *argument)
{
    osStatus status;
    static compressor_task_message_t msg;
    /*工作超时 发送消息*/
    msg.request.type = COMPRESSOR_TASK_MSG_TYPE_WORK_TIMEOUT;
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
    if (status !=osOK) {
        log_error("compressor put work timeout msg error:%d\r\n",status);
    } 
}

/*
* @brief 两次开机等待定时器
* @param 无
* @return 无
* @note
*/
static void compressor_wait_timer_init(void)
{
    osTimerDef(compressor_wait_timer,compressor_wait_timer_expired);
    compressor_wait_timer_id = osTimerCreate(osTimer(compressor_wait_timer),osTimerOnce,0);
    log_assert(compressor_wait_timer_id);
}

/*
* @brief 两次开机等待定时器启动
* @param 无
* @return 无
* @note
*/
static void compressor_wait_timer_start(void)
{
    osTimerStart(compressor_wait_timer_id,COMPRESSOR_TASK_WAIT_TIMEOUT);  
}

/*
* @brief 两次开机等待定时器停止
* @param 无
* @return 无
* @note
*/
/*
static void compressor_wait_timer_stop(void)
{
    osTimerStop(compressor_wait_timer_id);  
}
*/
/*
* @brief 两次开机等待定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void compressor_wait_timer_expired(void const *argument)
{
    osStatus status;  
    static compressor_task_message_t msg;
    /*工作超时 发送消息*/
    msg.request.type = COMPRESSOR_TASK_MSG_TYPE_WAIT_TIMEOUT;
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
    if (status !=osOK) {
        log_error("compressor put wait timeout msg error:%d\r\n",status);
    } 
}

/*
* @brief 上电等待定时器
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_wait_timer_init(void)
{
    osTimerDef(compressor_pwr_wait_timer,compressor_pwr_wait_timer_expired);
    compressor_pwr_wait_timer_id=osTimerCreate(osTimer(compressor_pwr_wait_timer),osTimerOnce,0);
    log_assert(compressor_pwr_wait_timer_id);
}

/*
* @brief 上电等待定时器启动
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_wait_timer_start(void)
{
    osTimerStart(compressor_pwr_wait_timer_id,COMPRESSOR_TASK_PWR_WAIT_TIMEOUT);   
}
/*
* @brief 工作时间定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void compressor_pwr_wait_timer_expired(void const *argument)
{
    osStatus status;
    static compressor_task_message_t msg;
    /*工作超时 发送消息*/
    msg.request.type = COMPRESSOR_TASK_MSG_TYPE_PWR_WAIT_TIMEOUT;
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
    if (status != osOK) {
        log_error("compressor put pwr wait timeout msg error:%d\r\n",status);
    } 
}

/*
* @brief 休息定时器
* @param 无
* @return 无
* @note
*/
static void compressor_rest_timer_init(void)
{
    osTimerDef(compressor_rest_timer,compressor_rest_timer_expired);
    compressor_rest_timer_id = osTimerCreate(osTimer(compressor_rest_timer),osTimerOnce,0);
    log_assert(compressor_rest_timer_id);
}
/*
* @brief 休息定时器启动
* @param 无
* @return 无
* @note
*/
static void compressor_rest_timer_start(void)
{
    osTimerStart(compressor_rest_timer_id,COMPRESSOR_TASK_REST_TIMEOUT);
}

/*
* @brief 休息定时器停止
* @param 无
* @return 无
* @note
*/
/*
static void compressor_rest_timer_stop(void)
{
    osTimerStop(compressor_rest_timer_id);
}
*/
/*
* @brief 休息定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void compressor_rest_timer_expired(void const *argument)
{
    osStatus status;
    static compressor_task_message_t msg;
    /*工作超时 发送消息*/
    msg.request.type = COMPRESSOR_TASK_MSG_TYPE_REST_TIMEOUT;
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
    if(status != osOK) {
        log_error("compressor put rest timeout msg error:%d\r\n",status);
    } 
}

/*
* @brief 打开压缩机
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_turn_on(void)
{
    bsp_compressor_ctrl_pwr_on(); 
}

/*
* @brief 关闭压缩机
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_turn_off()
{
    bsp_compressor_ctrl_pwr_off();  
}


static int nv_flash_save(uint8_t *value,uint8_t cnt)
{

    return 0;
}
/*
* @brief 压缩机任务
* @param argument 任务参数
* @return 无
* @note
*/
void compressor_task(void const *argument)
{
    int rc;
    osEvent  os_event;
    osStatus status;
    uint8_t level;
    compressor_task_message_t req_msg,req_update_msg,rsp_setting_msg;

    /*上电先关闭压缩机*/
    compressor_pwr_turn_off();
    /*定时器初始化*/
    compressor_work_timer_init();
    compressor_wait_timer_init();
    compressor_rest_timer_init();
    compressor_pwr_wait_timer_init();
    /*上电等待*/
    compressor_pwr_wait_timer_start();
  
    while (1) {
    os_event = osMessageGet(compressor_task_msg_q_id,osWaitForever);
    if (os_event.status == osEventMessage) {     
        req_msg = *(compressor_task_message_t*)os_event.value.v;
  
        /*配置压缩机工作温度区间消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_SET_TEMPERATURE_LEVEL){      
            if (req_msg.request.temperature_setting < COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN ||\
                rsp_setting_msg.request.temperature_setting > COMPRESSOR_TASK_TEMPERATURE_SETTING_MAX) {
                rsp_setting_msg.response.result = COMPRESSOR_TASK_FAIL;
                log_error("new temperature setting:%d invalid.min:%d max:%d.\r\n.",
                            rsp_setting_msg.request.temperature_setting,
                            COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN,
                            COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN);
             } else {
                for (level = 0;level < COMPRESSOR_TASK_TEMPERATURE_LEVEL_CNT;level++) {
                    /*寻找温度区间*/
                    if (req_msg.request.temperature_setting >= compressor.temperature_level[level].stop &&\
                        req_msg.request.temperature_setting <= compressor.temperature_level[level].work) {
                        /*复制当前温度区间到缓存*/
                        compressor.temperature_work = compressor.temperature_level[level].work;
                        compressor.temperature_stop = compressor.temperature_level[level].stop;
                        break;
                    }
                }
                rsp_setting_msg.response.result = COMPRESSOR_TASK_SUCCESS;
                if (compressor.level != level) {
                    compressor.level = level;
                    rc = nv_flash_save(&level,1);
                    if (rc != 0) {
                        rsp_setting_msg.response.result = COMPRESSOR_TASK_FAIL;
                        log_error("nv flash save temperature level fail.\r\n");
                    } else {                                
                        log_debug("temperature setting level:%d ok.from %d to %d.\r\n",level,compressor.temperature_level[level].stop,compressor.temperature_level[level].work);
                        /*发送消息更新压缩机工作状态*/
                        req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
                        status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
                        if (status != osOK) {
                            log_error("compressor put update msg timeout error:%d\r\n",status);
                        }                     
                    }
                } else {
                    log_debug("new temperature setting is same with old.skip.\r\n");
                }
            }
            /*发送消息给通信任务*/
            rsp_setting_msg.response.type = COMPRESSOR_TASK_MSG_TYPE_RSP_SET_TEMPERATURE_LEVEL;     
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_setting_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put rsp_setting msg timeout error:%d\r\n",status);
            }                                    
        }
                   
        /*压缩机上电等待完毕消息处理*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_PWR_WAIT_TIMEOUT){ 
            compressor.status = COMPRESSOR_STATUS_RDY_CONTINUE;
            /*发送消息更新压缩机工作状态*/
            req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
            status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put update msg timeout error:%d\r\n",status);
            }          
        } 
      
        /*温度更新消息处理*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_UPDATE){ 
            /*缓存温度值*/
            compressor.temperature_int = req_msg.request.temperature_int; 
            compressor.temperature_float = req_msg.request.temperature_float; 
            /*发送消息更新压缩机工作状态*/
            req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
            status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put update msg timeout error:%d\r\n",status);
            }   
        }

        /*压缩机工作状态更新*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS){ 
            /*只有在没有温度错误和等待上电完毕后才处理压缩机的启停*/
            if (compressor.temperature_err == false && compressor.status != COMPRESSOR_STATUS_INIT) {
                if (compressor.temperature_int <= compressor.temperature_stop  && compressor.status == COMPRESSOR_STATUS_WORK){
                    log_info("温度:%d C低于关机温度:%d C.\r\n",compressor.temperature_int,compressor.temperature_stop );
                    compressor.status = COMPRESSOR_STATUS_WAIT;
                    log_info("compressor change status to wait.\r\n");  
                    log_info("关压缩机.\r\n");
                    /*关闭压缩机和工作定时器*/
                    compressor_pwr_turn_off(); 
                    compressor_work_timer_stop();
                    /*打开等待定时器*/ 
                    compressor_wait_timer_start(); 
                } else if ((compressor.temperature_int >= compressor.temperature_work  && compressor.status == COMPRESSOR_STATUS_RDY )  || \
                          (compressor.temperature_int > compressor.temperature_stop   && compressor.status == COMPRESSOR_STATUS_RDY_CONTINUE )){
                    /*温度大于开机温度，同时是正常关机状态时，开机*/
                    /*或者温度大于关机温度，同时是超时关机或者异常关机状态时，继续开机*/
                    log_info("温度:%d C高于开机温度1:%d C 温度2:%d C.\r\n",compressor.temperature_int,compressor.temperature_work,compressor.temperature_stop);
                    log_info("开压缩机.\r\n");
                    compressor.status = COMPRESSOR_STATUS_WORK;
                    log_info("compressor change status to work.\r\n");  
                    /*打开压缩机*/
                    compressor_pwr_turn_on();
                    /*打开工作定时器*/ 
                    compressor_work_timer_start(); 
                }else{
                    log_info("压缩机状态:%d 无需处理.\r\n",compressor.status);     
                } 
            } else {
                log_info("压缩机正在上电等待或者温度错误 跳过.\r\n");
            }
        }

        /*温度错误消息处理*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_ERR) { 
            compressor.temperature_err = true;
            if (compressor.status == COMPRESSOR_STATUS_WORK){
                /*温度异常时，如果在工作,就变更为wait continue状态*/
                compressor.status = COMPRESSOR_STATUS_WAIT_CONTINUE;
                log_info("温度错误.\r\n"); 
                log_info("compressor change status to wait continue.\r\n");  
                log_info("关压缩机.\r\n");
                /*关闭压缩机和工作定时器*/
                compressor_pwr_turn_off(); 
                compressor_work_timer_stop();
                /*打开等待定时器*/ 
                compressor_wait_timer_start();  
            } else {
                log_info("压缩机状态:%d 无需处理.\r\n",compressor.status);  
            }
        }
 
  
        /*压缩机工作超时消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_WORK_TIMEOUT) {
            if (compressor.status == COMPRESSOR_STATUS_WORK) {
                log_info("关压缩机.\r\n");
                compressor.status = COMPRESSOR_STATUS_REST;
                log_info("compressor change status to rest.\r\n");  
                /*关闭压缩机和工作定时器*/
                compressor_pwr_turn_off();  
                /*打开休息定时器*/ 
                compressor_rest_timer_start(); 
            } else {
                log_info("压缩机状态:%d 无需处理.\r\n",compressor.status);  
            }
        }
 
        /*压缩机等待超时消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_WAIT_TIMEOUT) {
            if (compressor.status == COMPRESSOR_STATUS_WAIT){
                compressor.status = COMPRESSOR_STATUS_RDY;
                log_info("compressor change status to rdy.\r\n");  
            } else if (compressor.status == COMPRESSOR_STATUS_WAIT_CONTINUE) {
                compressor.status = COMPRESSOR_STATUS_RDY_CONTINUE;
                log_info("compressor change status to rdy continue.\r\n");  
            } else {
                log_info("压缩机状态:%d 无需处理.\r\n",compressor.status);     
            }
            /*发送消息更新压缩机工作状态*/
            req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
            status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put update msg timeout error:%d\r\n",status);
            }    
        }
 
        /*压缩机休息超时消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_REST_TIMEOUT) {
            if (compressor.status == COMPRESSOR_STATUS_REST){
                compressor.status = COMPRESSOR_STATUS_RDY_CONTINUE;
                log_info("compressor change status to rdy continue.\r\n");
        
                /*发送消息更新压缩机工作状态*/
                req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
                status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("compressor put update msg timeout error:%d\r\n",status);
                } 
            } else {
                log_info("压缩机状态:%d 无需处理.\r\n",compressor.status);     
            }
        }


    }
    }
 }


