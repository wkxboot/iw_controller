#include "board.h"
#include "cmsis_os.h"
#include "utils.h"
#include "serial.h"
#include "firmware_version.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "lock_task.h"
#include "temperature_task.h"
#include "compressor_task.h"
#include "communication_task.h"
#include "log.h"

osThreadId   communication_task_hdl;
osMessageQId communication_task_msg_q_id;
int communication_serial_handle;

extern serial_hal_driver_t nxp_serial_uart_hal_driver;
extern void nxp_serial_uart_hal_isr(int handle);
/*通信任务上文实体*/
static communication_task_contex_t communication_task_contex;


/*协议定义*/
#define  ADU_SIZE_MAX                               60
/*地址域*/
#define  ADU_ADDR_REGION_OFFSET                     0
#define  ADU_ADDR_REGION_SIZE                       1
#define  ADU_ADDR                                   0x01
/*命令码域*/
#define  ADU_CODE_REGION_OFFSET                     1
#define  ADU_CODE_REGION_SIZE                       1
#define  CODE_REMOVTE_TARE                          0x01     
#define  CODE_CALIBRATION                           0x02  
#define  CODE_QUERY_NET_WEIGHT                      0x03  
#define  CODE_QUERY_SCALE_CNT                       0x04  
#define  CODE_QUERY_DOOR_STATUS                     0x11  
#define  CODE_UNLOCK_LOCK                           0x21   
#define  CODE_LOCK_LOCK                             0x22  
#define  CODE_QUERY_LOCK_STATUS                     0x23  
#define  CODE_QUERY_TEMPERATURE                     0x41  
#define  CODE_SET_TEMPERATURE                       0x0A 
#define  CODE_QUERY_MANUFACTURER                    0x51 
/*数据域*/
#define  ADU_DATA_REGION_OFFSET                     2
#define  ADU_DATA_REGION_REMOVE_TARE_SIZE           1 
#define  ADU_DATA_REGION_CALIBRATION_SIZE           3 
#define  ADU_DATA_REGION_QUERY_NET_WEIGHT_SIZE      1 
#define  ADU_DATA_REGION_QUERY_SCALE_CNT_SIZE       0 
#define  ADU_DATA_REGION_QUERY_DOOR_STATUS_SIZE     0
#define  ADU_DATA_REGION_LOCK_LOCK_SIZE             0
#define  ADU_DATA_REGION_UNLOCK_LOCK_SIZE           0
#define  ADU_DATA_REGION_QUERY_LOCK_STATUS_SIZE     0
#define  ADU_DATA_REGION_QUERY_TEMPERATURE_SIZE     0
#define  ADU_DATA_REGION_SET_TEMPERATURE_SIZE       1
#define  ADU_DATA_REGION_QUERY_MANUFACTURER_SIZE    0


#define  DATA_REGION_SCALE_ADDR_OFFSET              0
#define  DATA_REGION_CALIBRATION_WEIGHT_OFFSET      1
#define  DATA_REGION_SCALE_CNT_OFFSET               0
#define  DATA_REGION_TEMPERATURE_OFFSET             0
#define  DATA_REGION_STATUS_OFFSET                  0

/*协议操作值定义*/
#define  DATA_NET_WEIGHT_ERR_VALUE                  0xFFFF
#define  DATA_TEMPERATURE_ERR_VALUE                 0x7F
#define  DATA_STATUS_DOOR_OPEN                      0x01
#define  DATA_STATUS_DOOR_CLOSE                     0x00
#define  DATA_STATUS_DOOR_ERR                       0xFF
#define  DATA_STATUS_LOCK_UNLOCKED                  0x01
#define  DATA_STATUS_LOCK_LOCKED                    0x00
#define  DATA_STATUS_LOCK_ERR                       0xFF
#define  DATA_RESULT_LOCK_SUCCESS                   0x01
#define  DATA_RESULT_LOCK_FAIL                      0x00
#define  DATA_RESULT_UNLOCK_SUCCESS                 0x01
#define  DATA_RESULT_UNLOCK_FAIL                    0x00
#define  DATA_RESULT_REMOVE_TARE_SUCCESS            0x01
#define  DATA_RESULT_REMOVE_TARE_FAIL               0x00
#define  DATA_RESULT_CALIBRATION_SUCCESS            0x01
#define  DATA_RESULT_CALIBRATION_FAIL               0x00
#define  DATA_RESULT_SET_TEMPERATURE_SUCCESS        0x01
#define  DATA_RESULT_SET_TEMPERATURE_FAIL           0x00
#define  DATA_MANUFACTURER_CHANGHONG_ID             0x1101
/*CRC16域*/
#define  ADU_CRC_SIZE                               2


/*协议时间*/
#define  ADU_WAIT_TIMEOUT                           osWaitForever
#define  ADU_FRAME_TIMEOUT                          3
#define  ADU_RSP_TIMEOUT                            200
#define  ADU_LOCK_RSP_TIMEOUT                       600
#define  ADU_SCALE_CNT_MAX                          20
#define  ADU_SEND_TIMEOUT                           5



/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/*
* @brief 计算接收的数据CRC
* @param adu 接收缓存
* @param buffer_length 数据缓存长度
* @return CRC
* @note
*/

static uint16_t calculate_crc16(uint8_t *adu, uint16_t size)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint32_t i; /* will index into CRC lookup */

   /* calculate the CRC  */
    while (size --) {
        i = crc_hi ^ *adu++; 
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}
/*
* @brief 计算发送缓存CRC并填充到发送缓存
* @param adu 回应缓存
* @param size 当前数据缓存长度
* @return 加上crc后的数据长度
* @note
*/

static uint8_t adu_add_crc16(uint8_t *adu,uint8_t size)
{
    uint16_t crc16;
    crc16 = calculate_crc16(adu,size);

    adu[size ++] = (crc16 >> 8) & 0xFF ;
    adu[size ++] = crc16 & 0xFF;
    return size;
}


/*
* @brief 读取电子称地址配置
* @param config 硬件配置指针
* @return 0 成功
* @return -1 失败
* @note
*/
static int communication_read_scale_addr_configration(scale_addr_configration_t *addr)
{
    addr->cnt = 4;
    addr->value[0] = 1;
    addr->value[1] = 2;
    addr->value[2] = 3;
    addr->value[3] = 4;
    
    return 0;
}
/*
* @brief 查找地址在配置表中对应的标号
* @param addr 传感器地址
* @return -1 失败
* @return >=0 对应的标号
* @note
*/
static int find_scale_task_contex_index(const communication_task_contex_t *contex,uint8_t addr)
{   
    /*指定电子秤任务*/
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        if (addr == contex->scale_task_contex[i].internal_addr) {
            return i;
        }
    }

    return -1;
}


/*
* @brief 请求电子秤数量
* @param contex 电子秤上下文
* @return -1 失败
* @return  > 0 配置电子秤数量
* @note
*/
static int query_scale_cnt(communication_task_contex_t *contex)
{
    return contex->cnt;
}

/*
* @brief 请求净重值
* @param contex 通信任务任务上下文
* @param addr 电子秤地址
* @param value 净重量值指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_net_weight(const communication_task_contex_t *contex,const uint8_t addr,int16_t *value)
{
    int rc;
    uint32_t flags = 0;
    uint8_t cnt;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    utils_timer_t timer;

    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) {
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_NET_WEIGHT;
            req_msg[i].request.rsp_message_queue_id = contex->net_weight_rsp_msg_q_id; 
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],utils_timer_value(&timer));
            log_assert(status == osOK);
        }
        cnt = contex->cnt;
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }    
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_NET_WEIGHT;
        req_msg[0].request.rsp_message_queue_id = contex->net_weight_rsp_msg_q_id;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],utils_timer_value(&timer));
        log_assert(status == osOK);
        cnt = 1;
    }
    /*等待消息*/
    while (flags != 0 && utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->net_weight_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_NET_WEIGHT) {     
                log_error("comm net weight rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            value[rsp_msg.response.index] = rsp_msg.response.weight;
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("net weight query err.fags:%d.\r\n",flags);
        return -1;
    }

    return cnt;
}


/*
* @brief 去除皮重
* @param contex 电子秤任务上下文
* @param addr 电子秤地址
* @return -1 失败
* @return  0 成功
* @note
*/
static int remove_tare_weight(const communication_task_contex_t *contex,const uint8_t addr)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    utils_timer_t timer;
    bool success = true;

    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT;
            req_msg[i].request.rsp_message_queue_id = contex->remove_tare_rsp_msg_q_id;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],utils_timer_value(&timer));
            log_assert(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }   
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT;
        req_msg[0].request.rsp_message_queue_id = contex->remove_tare_rsp_msg_q_id; 
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->remove_tare_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_REMOVE_TARE_WEIGHT) {     
                log_error("comm remove tare weight rsp type:%d err.\r\n",rsp_msg.response.type);
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm remove tare weight err.fags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;
}

/*
* @brief 0点校准
* @param contex 通信任务上下文
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int calibration_zero(const communication_task_contex_t *contex,const uint8_t addr,const int16_t weight)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    utils_timer_t timer;
    bool success = true;

    if (weight != 0) {
        log_error("calibration zero weight:%d != 0 err.\r\n",weight);
        return -1;
    }
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT;    
            req_msg[i].request.rsp_message_queue_id = contex->calibration_zero_rsp_msg_q_id;
            req_msg[i].request.weight = weight;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],utils_timer_value(&timer));
            log_assert(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }    
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT;    
        req_msg[0].request.rsp_message_queue_id = contex->calibration_zero_rsp_msg_q_id;
        req_msg[0].request.weight = weight;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->calibration_zero_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_ZERO_WEIGHT) {     
                log_error("comm calibration zero weight rsp type:%d err.\r\n",rsp_msg.response.type);
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm calibration zero internal timeout err.fags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;
}

/*
* @brief 增益校准
* @param contex 通信任务上下文
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int calibration_full(const communication_task_contex_t *contex,const uint8_t addr,const int16_t weight)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    utils_timer_t timer;
    bool success = true;

    if (weight <= 0) {
        log_error("calibration full weight:%d <= 0 err.\r\n",weight);
        return -1;
    }
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT;    
            req_msg[i].request.rsp_message_queue_id = contex->calibration_full_rsp_msg_q_id;
            req_msg[i].request.weight = weight;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],utils_timer_value(&timer));
            log_assert(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }  
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT;    
        req_msg[0].request.rsp_message_queue_id = contex->calibration_full_rsp_msg_q_id;  
        req_msg[0].request.weight = weight;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->calibration_full_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_FULL_WEIGHT) {     
                log_error("comm calibration full weight rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm calibration full internal timeout err.fags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;       
}

/*
* @brief 查询门状态
* @param contex 通信任务上下文
* @param door_status 门状态指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int query_door_status(communication_task_contex_t *contex,uint8_t *door_status)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_DOOR_STATUS;    
    req_msg.request.rsp_message_queue_id = contex->query_door_status_rsp_msg_q_id;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_door_status_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_DOOR_STATUS) {     
                log_error("comm query door status rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            *door_status = rsp_msg.response.status;
            return 0;
        }
    }
        
    log_error("comm query door status timeout err.\r\n");
    return -1;
}

/*
* @brief 查询锁状态
* @param contex 通信任务上下文
* @param lock_status 锁状态指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_lock_status(communication_task_contex_t *contex,uint8_t *lock_status)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_LOCK_STATUS;    
    req_msg.request.rsp_message_queue_id = contex->query_lock_status_rsp_msg_q_id;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_lock_status_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_LOCK_STATUS) {     
                log_error("comm query lock status rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            *lock_status = rsp_msg.response.status;
            return 0;
        }
    }
        
    log_error("comm query door status timeout err.\r\n");
    return -1;
}

/*
* @brief 开锁
* @param contex 通信任务上下文
* @param result 结果指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int unlock_lock(communication_task_contex_t *contex)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_UNLOCK_LOCK;    
    req_msg.request.rsp_message_queue_id = contex->unlock_lock_rsp_msg_q_id;
    utils_timer_init(&timer,ADU_LOCK_RSP_TIMEOUT,false);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->unlock_lock_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_UNLOCK_LOCK_RESULT) {     
                log_error("comm unlock lock rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            return rsp_msg.response.result == LOCK_TASK_SUCCESS ? 0 : -1;
            
        }
    }
        
    log_error("comm unlock lock timeout err.\r\n");
    return -1;
}

/*
* @brief 关锁
* @param contex 通信任务上下文
* @return -1 失败
* @return  0 成功
* @note
*/
static int lock_lock(communication_task_contex_t *contex)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_LOCK_LOCK;    
    req_msg.request.rsp_message_queue_id = contex->lock_lock_rsp_msg_q_id;
    utils_timer_init(&timer,ADU_LOCK_RSP_TIMEOUT,false);
    
    /*发送消息*/

    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->lock_lock_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_LOCK_LOCK_RESULT) {     
                log_error("comm lock lock rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            return rsp_msg.response.result == LOCK_TASK_SUCCESS ? 0 : -1;
        }
    }
        
    log_error("comm lock lock timeout err.\r\n");
    return -1;
}

/*
* @brief 查询温度值
* @param contex 通信任务上下文
* @param temperature 温度指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_temperature(communication_task_contex_t *contex,int8_t *temperature)
{
    osStatus status;
    osEvent os_event;

    temperature_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = TEMPERATURE_TASK_MSG_TYPE_TEMPERATURE;    
    req_msg.request.rsp_message_queue_id = contex->query_temperature_rsp_msg_q_id;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    
    /*发送消息*/

    status = osMessagePut(temperature_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_temperature_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(temperature_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != TEMPERATURE_TASK_MSG_TYPE_RSP_TEMPERATURE) {     
                log_error("comm query temperature rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            if (rsp_msg.response.err == false) {
                *temperature = rsp_msg.response.temperature_int;
            } else {
                *temperature = DATA_TEMPERATURE_ERR_VALUE;
            }
            return 0;
        }
    }
        
    log_error("comm query temperature timeout err.\r\n");
    return -1;
}


/*
* @brief 设置压缩机温度控制区间
* @param contex 通信任务上下文
* @param temperature 温度
* @return -1 失败
* @return  0 成功
* @note
*/
static int set_temperature_level(communication_task_contex_t *contex,uint8_t temperature)
{
    osStatus status;
    osEvent os_event;

    compressor_task_message_t req_msg,rsp_msg;
    utils_timer_t timer;

    req_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_SET_TEMPERATURE_LEVEL;    
    req_msg.request.rsp_message_queue_id = contex->set_temperature_level_rsp_msg_q_id;
    req_msg.request.temperature_setting = temperature;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);
    
    /*发送消息*/
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_msg,utils_timer_value(&timer));
    log_assert(status == osOK);

    /*等待消息*/
    while (utils_timer_value(&timer) > 0) {
        os_event = osMessageGet(contex->set_temperature_level_rsp_msg_q_id,utils_timer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(compressor_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != COMPRESSOR_TASK_MSG_TYPE_RSP_SET_TEMPERATURE_LEVEL) {     
                log_error("comm set temperature level rsp type:%d err.\r\n",rsp_msg.response.type);
                return -1;
            }
            return rsp_msg.response.result == COMPRESSOR_TASK_SUCCESS ? 0 : -1;
        }
    }
        
    log_error("comm set temperature level timeout err.\r\n");
    return -1;
}

/*
* @brief 查询厂家ID
* @param contex 通信任务上下文
* @param manufacturer 厂家id指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int query_manufacturer(communication_task_contex_t *contex,uint16_t *manufacturer)
{
    *manufacturer = contex->manufacturer_id;
    return 0;
}

/*
* @brief 串口接收主机ADU
* @param handle 串口句柄
* @param adu 数据缓存指针
* @param timeout 等待超时时间
* @return -1 失败
* @return  > 0 成功接收的数据量
* @note
*/
static int receive_adu(int handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    int rc;
    int read_size,read_size_total = 0;

    while (read_size_total < size) {
        rc = serial_select(handle,timeout);
        if (rc == -1) {
            log_error("adu select error.read total:%d.\r\n",read_size_total);
            return -1;
        }
        /*读完了一帧数据*/
        if (rc == 0) {
            log_debug("adu recv over.read size total:%d.\r\n",read_size_total);
            return read_size_total;
        }
        /*数据溢出*/
        if (rc > size - read_size_total) {
            log_error("adu size:%d too large than buffer size:%d .error.\r\n",read_size_total + rc,size);
            return -1;
        }
        read_size = rc;
        rc = serial_read(handle,(char *)adu + read_size_total,read_size);
        if (rc == -1) {
            log_error("adu read error.read total:%d.\r\n",read_size_total);
            return -1;
        }
   
        /*打印接收的数据*/
        for (int i = 0;i < rc;i++){
            log_array("<%2X>\r\n", adu[read_size_total + i]);
        }
        /*更新接收数量*/
        read_size_total += rc;
        /*超时时间变为帧超时间*/
        timeout = ADU_FRAME_TIMEOUT;
    }

    log_error("adu recv unknow err.\r\n");
    return -1;
}


/*
* @brief 解析adu
* @param adu 数据缓存指针
* @param size 数据大小
* @param rsp 回应的数据缓存指针
* @return -1 失败 
* @return  > 0 回应的adu大小
* @note
*/

static int parse_adu(uint8_t *adu,uint8_t size,uint8_t *rsp)
{
    int rc;
    uint8_t communication_addr;
    uint8_t code;
    uint8_t scale_addr;
    uint8_t temperature_setting;
    uint16_t manufacturer_id;
    uint16_t calibration_weight;
    uint16_t crc_received,crc_calculated;

    uint8_t status;
    uint8_t scale_cnt;
    int8_t  temperature;
    int16_t net_weight[SCALE_CNT_MAX];
    uint8_t rsp_size = 0;
    uint8_t rsp_offset = 0;

    if (size < ADU_ADDR_REGION_SIZE + ADU_CODE_REGION_SIZE + ADU_CRC_SIZE) {
        log_error("adu size:%d < %d err.\r\n",size,ADU_ADDR_REGION_SIZE + ADU_CODE_REGION_SIZE + ADU_CRC_SIZE);
        return -1;
    }
    /*校验CRC*/
    crc_received = (uint16_t)adu[size - 2] << 8 | adu[size - 1];
    crc_calculated = calculate_crc16(adu,size - ADU_CRC_SIZE);
    if (crc_received != crc_calculated) {
        log_error("crc err.claculate:%d receive:%d.\r\n",crc_calculated,crc_received);
        return -1;
    }
    /*校验通信地址*/
    communication_addr = adu[ADU_ADDR_REGION_OFFSET];
    if (communication_addr != ADU_ADDR) {
        log_error("communication addr:%d != %d err.\r\n",communication_addr,ADU_ADDR);
        return -1;
    }
    rsp[rsp_offset ++] = ADU_ADDR;
    /*数据域长度*/
    size -= ADU_ADDR_REGION_SIZE + ADU_CODE_REGION_SIZE + ADU_CRC_SIZE;
    /*校验命令码*/
    code = adu[ADU_CODE_REGION_OFFSET];
    /*回应adu构建code*/
    rsp[rsp_offset ++] = code;
    switch (code) {
        case CODE_REMOVTE_TARE:/*去皮*/
            if (size != ADU_DATA_REGION_REMOVE_TARE_SIZE) {
                log_error("remove tare data size:%d != %d err.\r\n",size,ADU_DATA_REGION_REMOVE_TARE_SIZE);
                return -1;
            }
            scale_addr = adu[ADU_DATA_REGION_OFFSET + DATA_REGION_SCALE_ADDR_OFFSET];
            log_debug("scale addr:%d remove tare weight...\r\n",scale_addr);
            rc = remove_tare_weight(&communication_task_contex,scale_addr);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_REMOVE_TARE_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_REMOVE_TARE_FAIL;
            }
            break;
        case CODE_CALIBRATION:/*校准*/
            if (size != ADU_DATA_REGION_CALIBRATION_SIZE) {
                log_error("calibration data size:%d != %d err.\r\n",size,ADU_DATA_REGION_CALIBRATION_SIZE);
                return -1;
            }
            scale_addr = adu[ADU_DATA_REGION_OFFSET + DATA_REGION_SCALE_ADDR_OFFSET];
            calibration_weight = (uint16_t)adu[ADU_DATA_REGION_OFFSET + DATA_REGION_CALIBRATION_WEIGHT_OFFSET] << 8 | adu[ADU_DATA_REGION_OFFSET + DATA_REGION_CALIBRATION_WEIGHT_OFFSET + 1];
            log_debug("scale addr:%d calibration weight:%d...\r\n",scale_addr,calibration_weight);
            if (calibration_weight == 0) {
                rc = calibration_zero(&communication_task_contex,scale_addr,calibration_weight);
            } else {   
                rc = calibration_full(&communication_task_contex,scale_addr,calibration_weight);
            }
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_FAIL;
            }
            break;
        case CODE_QUERY_NET_WEIGHT:/*净重*/
            if (size != ADU_DATA_REGION_QUERY_NET_WEIGHT_SIZE) {
                log_error("query net weight data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_NET_WEIGHT_SIZE);
                return -1;
            }
            scale_addr = adu[ADU_DATA_REGION_OFFSET + DATA_REGION_SCALE_ADDR_OFFSET];
            log_debug("scale addr:%d query net weight...\r\n",scale_addr);
            
            rc = query_net_weight(&communication_task_contex,scale_addr,net_weight);
            if (rc <= 0) {
                log_error("query net weight internal err.\r\n");
                return -1;
            } 
            uint8_t index_end;
            if (scale_addr == 0) {
                index_end = ADU_SCALE_CNT_MAX;
            } else {
                index_end = 1;
            }

            for (uint8_t i = 0; i < index_end; i++) {
                if (i < rc) {
                    /*区别协议传感器故障值*/
                    if (net_weight[i] == -1) {
                        net_weight[i] = 0;
                    /*转换为协议传感器故障值*/
                    } else if (net_weight[i] == SCALE_TASK_NET_WEIGHT_ERR_VALUE) {
                        net_weight[i] = DATA_NET_WEIGHT_ERR_VALUE;
                    }
                    rsp[rsp_offset ++] = (net_weight[i] >> 8) & 0xFF;
                    rsp[rsp_offset ++] =  net_weight[i]  & 0xFF;
                } else {
                    rsp[rsp_offset ++] = 0;
                    rsp[rsp_offset ++] = 0;
                }
               
            } 
            break;
        case CODE_QUERY_SCALE_CNT:/*电子秤数量*/
            if (size != ADU_DATA_REGION_QUERY_SCALE_CNT_SIZE) {
                log_error("query scale data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_SCALE_CNT_SIZE);
                return -1;
            }
            log_debug("query scale cnt...\r\n");
            scale_cnt = query_scale_cnt(&communication_task_contex);
            rsp[rsp_offset ++] = scale_cnt;
            break;
        case CODE_QUERY_DOOR_STATUS:/*门状态*/
            if (size != ADU_DATA_REGION_QUERY_DOOR_STATUS_SIZE) {
                log_error("query door status data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_DOOR_STATUS_SIZE);
                return -1;
            }
            log_debug("query door status...\r\n");
            rc = query_door_status(&communication_task_contex,&status);
            if (rc != 0) {
                log_error("query door status internal err.\r\n");
                return -1;
            }
            if (status == LOCK_TASK_STATUS_DOOR_OPEN) {
                rsp[rsp_offset ++] = DATA_STATUS_DOOR_OPEN;
            } else {
                rsp[rsp_offset ++] = DATA_STATUS_DOOR_CLOSE; 
            }
            break;
        case CODE_LOCK_LOCK:/*关锁*/
            if (size != ADU_DATA_REGION_LOCK_LOCK_SIZE) {
                log_error("lock lock data size:%d != %d err.\r\n",size,ADU_DATA_REGION_LOCK_LOCK_SIZE);
                return -1;
            }
            log_debug("lock lock...\r\n");
            rc = lock_lock(&communication_task_contex);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_LOCK_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_LOCK_FAIL;
            }
            break;
        case CODE_UNLOCK_LOCK:/*开锁*/
            if (size != ADU_DATA_REGION_UNLOCK_LOCK_SIZE) {
                log_error("unlock lock data size:%d != %d err.\r\n",size,ADU_DATA_REGION_UNLOCK_LOCK_SIZE);
                return -1;
            }
            log_debug("unlock lock...\r\n");
            rc = unlock_lock(&communication_task_contex);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_UNLOCK_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_UNLOCK_FAIL;
            }
            break;      
        case CODE_QUERY_LOCK_STATUS:/*锁状态*/
            if (size != ADU_DATA_REGION_QUERY_LOCK_STATUS_SIZE) {
                log_error("query lock status data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_LOCK_STATUS_SIZE);
                return -1;
            }
            log_debug("query lock status...\r\n");
            rc = query_lock_status(&communication_task_contex,&status);
            if (rc != 0) {
                log_error("query lock status internal err.\r\n");
                return -1;
            }
            if (status == LOCK_TASK_STATUS_LOCK_LOCKED) {
                rsp[rsp_offset ++] = DATA_STATUS_LOCK_LOCKED;
            } else {
                rsp[rsp_offset ++] = DATA_STATUS_LOCK_UNLOCKED; 
            }
            break;
        case CODE_QUERY_TEMPERATURE:/*查询温度*/
            if (size != ADU_DATA_REGION_QUERY_TEMPERATURE_SIZE) {
                log_error("query temperature data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_TEMPERATURE_SIZE);
                return -1;
            }
            log_debug("query temperature...\r\n");
            rc = query_temperature(&communication_task_contex,&temperature);
            if (rc != 0) {
                log_error("query door status internal err.\r\n");
                return -1;
            }
            rsp[rsp_offset ++] = temperature;
            break; 
        case CODE_SET_TEMPERATURE:/*设置温度区间*/
            if (size != ADU_DATA_REGION_SET_TEMPERATURE_SIZE) {
                log_error("set temperature data size:%d != %d err.\r\n",size,ADU_DATA_REGION_SET_TEMPERATURE_SIZE);
                return -1;
            }
            temperature_setting = adu[ADU_DATA_REGION_OFFSET + DATA_REGION_TEMPERATURE_OFFSET];
            log_debug("set temperature :%d...\r\n",temperature_setting);
            rc = set_temperature_level(&communication_task_contex,temperature_setting);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_SET_TEMPERATURE_SUCCESS;     
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_SET_TEMPERATURE_FAIL; 
            }                
            break;
        case CODE_QUERY_MANUFACTURER:/*查询厂商ID*/
            if (size != ADU_DATA_REGION_QUERY_MANUFACTURER_SIZE) {
                log_error("query manafacture data size:%d != %d err.\r\n",size,ADU_DATA_REGION_QUERY_MANUFACTURER_SIZE);
                return -1;
            }
            log_debug("query manufacture...\r\n");
            query_manufacturer(&communication_task_contex,&manufacturer_id);
            rsp[rsp_offset ++] = (manufacturer_id >> 8) & 0xFF;      
            rsp[rsp_offset ++] = manufacturer_id & 0xFF;      
            break;  
        default:
            log_error("unknow code:%d err.\r\n",code);
    }
   
    /*添加CRC16*/
    rsp_size = adu_add_crc16(rsp,rsp_offset);
    return rsp_size;
}


/*
* @brief 通过串口回应处理结果
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param size 结果大小
* @param timeout 发送超时时间
* @return -1 失败 
* @return  0 成功 
* @note
*/
static int send_adu(int handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    uint8_t write_size;

    write_size = serial_write(handle,(char *)adu,size);
    for (int i = 0; i < write_size; i++){
        log_array("[%2X]\r\n",adu[i]);
    }
    if (size != write_size){
        log_error("communication err in  serial write. expect:%d write:%d.\r\n",size,write_size); 
        return -1;      
     }
  
    return serial_complete(handle,timeout);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int get_serial_port_by_addr(uint8_t addr)
{
    int port = -1;
    switch (addr) {
    case 1:
        port = 1;
        break;
    case 2:
        port = 3;
        break;
    case 3:
        port = 5;
        break;
    case 4:
        port = 7;
        break;
    case 5:
        port = 2;
        break;
    case 6:
        port = 4;
        break;
    case 7:
        port = 6;
        break;
    case 8:
        port = 8;
        break;
    default :
        log_error("addr:%d is invalid.\r\n",addr);
        break;
    }

    return port;
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int get_serial_handle_by_port(communication_task_contex_t *contex,uint8_t port)
{
    if (!contex->initialized) {
        return -1;
    }
    for (uint8_t i = 0;i <contex->cnt;i ++) {
        if (contex->scale_task_contex[i].port == port) {
            return contex->scale_task_contex[i].handle;
        }
    }
    return -1;
}

/*控制器任务通信中断处理*/
void FLEXCOMM0_IRQHandler()
{
    nxp_serial_uart_hal_isr(communication_serial_handle);

}

/*电子秤任务通信中断处理*/
void FLEXCOMM1_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,1);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM2_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,2);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM3_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,3);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM4_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,4);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM5_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,5);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM6_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,6);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM7_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,7);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM8_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&communication_task_contex,8);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}


/*
* @brief 通信任务上下文配置初始化
* @param contex 任务参数指针
* @param host_msg_id 主任务消息队列句柄
* @return 无
* @note
*/
static void communication_task_contex_init(communication_task_contex_t *contex)
{
    int rc;
    /*电子秤地址配置信息*/
    scale_addr_configration_t scale_addr;
    rc = communication_read_scale_addr_configration(&scale_addr);
    log_assert(rc == 0);

    contex->cnt = scale_addr.cnt;
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        contex->scale_task_contex[i].internal_addr = scale_addr.value[i];
        contex->scale_task_contex[i].phy_addr = COMMUNICATION_TASK_SCALE_DEFAULT_ADDR;
        contex->scale_task_contex[i].port = get_serial_port_by_addr(contex->scale_task_contex[i].internal_addr);
        contex->scale_task_contex[i].baud_rates = SCALE_TASK_SERIAL_BAUDRATES;
        contex->scale_task_contex[i].data_bits = SCALE_TASK_SERIAL_DATABITS;
        contex->scale_task_contex[i].stop_bits = SCALE_TASK_SERIAL_STOPBITS;
        contex->scale_task_contex[i].flag = 1 << i;

        rc = serial_create(&contex->scale_task_contex[i].handle,SCALE_TASK_RX_BUFFER_SIZE,SCALE_TASK_RX_BUFFER_SIZE);
        log_assert(rc == 0);
        rc = serial_register_hal_driver(contex->scale_task_contex[i].handle,&nxp_serial_uart_hal_driver);
        log_assert(rc == 0);
 
        rc = serial_open(contex->scale_task_contex[i].handle,
                         contex->scale_task_contex[i].port,
                         contex->scale_task_contex[i].baud_rates,
                         contex->scale_task_contex[i].data_bits,
                         contex->scale_task_contex[i].stop_bits);
        log_assert(rc == 0);
        /*清空接收缓存*/
        serial_flush(contex->scale_task_contex[i].handle);

        /*创建电子秤消息队列*/
        osMessageQDef(scale_task_msg_queue,1,uint32_t);
        contex->scale_task_contex[i].msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_queue),0);
        log_assert(contex->scale_task_contex[i].msg_q_id);
        /*创建电子秤任务*/
        osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 256);
        contex->scale_task_contex[i].task_hdl = osThreadCreate(osThread(scale_task),&contex->scale_task_contex[i]);
        log_assert(contex->scale_task_contex[i].task_hdl);
    }  
    /*开锁回应消息队列*/
    osMessageQDef(unlock_lock_rsp_msg_q,1,uint32_t);
    contex->unlock_lock_rsp_msg_q_id = osMessageCreate(osMessageQ(unlock_lock_rsp_msg_q),0);
    log_assert(contex->unlock_lock_rsp_msg_q_id);
    /*关锁回应消息队列*/
    osMessageQDef(lock_lock_rsp_msg_q,1,uint32_t);
    contex->lock_lock_rsp_msg_q_id = osMessageCreate(osMessageQ(lock_lock_rsp_msg_q),0);
    log_assert(contex->lock_lock_rsp_msg_q_id);
    /*锁状态回应消息队列*/
    osMessageQDef(query_lock_status_rsp_msg_q,1,uint32_t);
    contex->query_lock_status_rsp_msg_q_id = osMessageCreate(osMessageQ(query_lock_status_rsp_msg_q),0);
    log_assert(contex->query_lock_status_rsp_msg_q_id);
    /*门状态回应消息队列*/
    osMessageQDef(query_door_status_rsp_msg_q,1,uint32_t);
    contex->query_door_status_rsp_msg_q_id = osMessageCreate(osMessageQ(query_door_status_rsp_msg_q),0);
    log_assert(contex->query_door_status_rsp_msg_q_id);
    /*温度回应消息队列*/
    osMessageQDef(query_temperature_rsp_msg_q,1,uint32_t);
    contex->query_temperature_rsp_msg_q_id = osMessageCreate(osMessageQ(query_temperature_rsp_msg_q),0);
    log_assert(contex->query_temperature_rsp_msg_q_id);
    /*设置温度等级消息队列*/
    osMessageQDef(set_temperature_level_rsp_msg_q,1,uint32_t);
    contex->set_temperature_level_rsp_msg_q_id = osMessageCreate(osMessageQ(set_temperature_level_rsp_msg_q),0);
    log_assert(contex->set_temperature_level_rsp_msg_q_id);

    /*净重回应消息队列*/
    osMessageQDef(net_weight_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->net_weight_rsp_msg_q_id = osMessageCreate(osMessageQ(net_weight_rsp_msg_q),0);
    log_assert(contex->net_weight_rsp_msg_q_id);
    /*去皮回应消息队列*/
    osMessageQDef(remove_tare_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->remove_tare_rsp_msg_q_id = osMessageCreate(osMessageQ(remove_tare_rsp_msg_q),0);
    log_assert(contex->remove_tare_rsp_msg_q_id);
    /*0点校准回应消息队列*/
    osMessageQDef(calibration_zero_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->calibration_zero_rsp_msg_q_id = osMessageCreate(osMessageQ(calibration_zero_rsp_msg_q),0);
    log_assert(contex->calibration_zero_rsp_msg_q_id);
    /*增益校准回应消息队列*/
    osMessageQDef(calibration_full_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->calibration_full_rsp_msg_q_id = osMessageCreate(osMessageQ(calibration_full_rsp_msg_q),0);
    log_assert(contex->calibration_full_rsp_msg_q_id);
    /*厂商ID*/
    contex->manufacturer_id = DATA_MANUFACTURER_CHANGHONG_ID;

    contex->initialized = true;
}
       
/*
* @brief 与主机通信任务
* @param argument 任务参数
* @return 无
* @note
*/       
void communication_task(void const * argument)
{
    int rc; 
    uint8_t adu_recv[ADU_SIZE_MAX];
    uint8_t adu_send[ADU_SIZE_MAX];
 
    rc = serial_create(&communication_serial_handle,COMMUNICATION_TASK_RX_BUFFER_SIZE,COMMUNICATION_TASK_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(communication_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(communication_serial_handle,
                    COMMUNICATION_TASK_SERIAL_PORT,
                    COMMUNICATION_TASK_SERIAL_BAUDRATES,
                    COMMUNICATION_TASK_SERIAL_DATABITS,
                    COMMUNICATION_TASK_SERIAL_STOPBITS);
    log_assert(rc == 0); 

    communication_task_contex_init(&communication_task_contex);
    log_debug("communication task contex init ok.\r\n");

    /*清空接收缓存*/
    serial_flush(communication_serial_handle);
    while (1) {

        /*接收主机发送的adu*/
        rc = receive_adu(communication_serial_handle,(uint8_t *)adu_recv,ADU_SIZE_MAX,ADU_WAIT_TIMEOUT);
        if (rc < 0) {
            /*清空接收缓存*/
            serial_flush(communication_serial_handle);
            continue;
        }
        /*解析处理pdu*/
        rc = parse_adu(adu_recv,rc,adu_send);
        if (rc < 0) {
            continue;
        }
        /*回应主机处理结果*/
        rc = send_adu(communication_serial_handle,adu_send,rc,ADU_SEND_TIMEOUT);
        if (rc < 0) {
            continue;
        }

    }
}
