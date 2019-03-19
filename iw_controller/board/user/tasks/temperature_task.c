#include "cmsis_os.h"
#include "tasks_init.h"
#include "stdio.h"
#include "stdbool.h"
#include "adc_task.h"
#include "compressor_task.h"
#include "temperature_task.h"
#include "log.h"

/*任务句柄*/
osThreadId   temperature_task_handle;
/*消息句柄*/
osMessageQId temperature_task_msg_q_id;


/*温度传感器型号：LAT5061G3839G B值：3839K*/
static int16_t const t_r_map[][2]={
  {-12,12224},{-11,11577},{-10,10968},{-9,10394},{-8,9854},{-7,9344},{-6,8864},{-5,8410},{-4,7983},{-3 ,7579},
  {-2 ,7199} ,{-1,6839}  ,{0,6499}   ,{1,6178 } ,{2,5875 },{3,5588 },{4,5317} ,{5,5060} ,{6,4817} ,{7,4587}  ,
  {8,4370}   ,{9,4164}   ,{10,3969}  ,{11,3784} ,{12,3608},{13,3442},{14,3284},{15,3135},{16,2993},{17,2858} ,
  {18,2730}  ,{19,2609}  ,{20,2494}  ,{21,2384} ,{22,2280},{23,2181},{24,2087},{25,1997},{26,1912},{27,1831} ,
  {28,1754}  ,{29,1680}  ,{30,1610}  ,{31,1543} ,{32,1480},{33,1419},{34,1361},{35,1306},{36,1254},{37,1204} ,
  {38,1156}  ,{39,1110}  ,{40,1067}  ,{41,1025} ,{42,985} ,{43,947} ,{44,911} ,{45,876} ,{46,843} ,{47,811}  ,
  {48,781}   ,{49,752}   ,{50,724}   ,{51,697}  ,{52,672} ,{53,647} ,{54,624} ,{55,602} ,{56,580} ,{57,559}
};

#define  TR_MAP_IDX_MIN        0
#define  TR_MAP_IDX_MAX        68

typedef struct
{
    float value_float;
    int16_t value_int;
    int8_t dir;
    bool err;
    bool change;
}temperature_t;

/*温度对象实体*/
static temperature_t   temperature;


 /*
 * @brief 获取温度传感器电阻值
 * @param adc adc数值
 * @return 电阻值
 * @note
 */             
static uint32_t get_r(const uint16_t adc)
{
    float t_sensor_r;
    t_sensor_r = (TEMPERATURE_SENSOR_SUPPLY_VOLTAGE*TEMPERATURE_SENSOR_ADC_VALUE_MAX*TEMPERATURE_SENSOR_BYPASS_RES_VALUE)/(adc*TEMPERATURE_SENSOR_REFERENCE_VOLTAGE)-TEMPERATURE_SENSOR_BYPASS_RES_VALUE;
    return (uint32_t)t_sensor_r;
}

 /*
 * @brief 获取浮点温度值
 * @param r 温度传感器电阻值
 * @param idx 阻温映射表下标号
 * @return 浮点温度值
 * @note
 */ 
static float calcaulate_float_t(uint32_t r,uint8_t idx)
{
    uint32_t r1,r2;

    float t;

    r1 = t_r_map[idx][1];
    r2 = t_r_map[idx + 1][1];

    t = t_r_map[idx][0] + (r1 - r) * 1.0 /(r1 - r2) + TEMPERATURE_COMPENSATION_VALUE;
  
    log_debug("temperature: %.2f C.\r\n",t);
   
    return t;  
}

/*
* @brief 获取四舍五入整数温度值
* @param t_float 浮点温度值
* @return 整形四舍五入温度值
* @note
*/
static int16_t calculate_approximate_t(float t_float)
{
    int16_t t = (int16_t)t_float;

    t_float -= t;

    if(t_float >= 0.5 ) {
        t += 1; 
    } else if (t_float <= -0.5){
        t -= 1;
    }
  
    return t;  
}
/*
* @brief 检查adc
* @param adc adc数值
* @return 整形四舍五入温度值
* @note
*/
static bool is_adc_valid(uint16_t adc)
{
    uint32_t r;

    if (adc >= ADC_ERR_MAX ||
        adc <= ADC_ERR_MIN ){
        log_error("传感器短路或者开路错误.\r\n");
        return false;
    }
    r = get_r(adc);
    if (r <= t_r_map[TR_MAP_IDX_MAX][1]){
        log_error("NTC 小于最高温度阻值！r=%d\r\n",r); 
        return false;
    }else if (r >= t_r_map[TR_MAP_IDX_MIN][1]){
        log_error("NTC 大于最低温度阻值！r=%d\r\n",r); 
        return false;
    }
    return true;
}

/*
* @brief 获取实际浮点
* @param adc adc数值
* @return 整形四舍五入温度值
* @note
*/
float get_float_temperature_by_adc(uint16_t adc)
{
    uint32_t r; 
    uint8_t mid=0;
    int low = TR_MAP_IDX_MIN;  
    int high =TR_MAP_IDX_MAX; 
 
    while (low <= high) {  
        mid = (low + high) / 2;  
        if (r > t_r_map[mid][1]) {
            if (r <= t_r_map[mid-1][1]){
                /*返回指定精度的温度*/
                return calcaulate_float_t(r, mid - 1);
            } else {
                high = mid - 1;  
            }
        } else {
            if (r > t_r_map[mid+1][1]) {
                /*返回带有温度补偿值的温度*/
                return calcaulate_float_t(r, mid);
            } else {
                low = mid + 1;   
            }
        }  
    }

    return 0;
}


/*
* @brief 温度传感器任务
* @param argument 任务参数
* @return 
* @note
*/
void temperature_task(void const *argument)
{

    osEvent  os_event;
    osStatus status;
    temperature_task_message_t adc_completed_msg;
    compressor_task_message_t update_msg;
    uint16_t bypass_r_adc;
    int16_t t_int;
    float t_float;

    temperature.value_int = 127;
    temperature.value_float = 127.0;

    while (1) {
        os_event = osMessageGet(temperature_task_msg_q_id,TEMPERATURE_TASK_MSG_WAIT_TIMEOUT);
        if (os_event.status == osEventMessage){
            adc_completed_msg = *(temperature_task_message_t*)&os_event.value.v;
 
            /*温度ADC转换完成消息处理*/
            if (adc_completed_msg.request.type == TEMPERATURE_TASK_MSG_TYPE_ADC_COMPLETED){
                bypass_r_adc = adc_completed_msg.request.adc;
                if (is_adc_valid(bypass_r_adc) == false) {
                    if (temperature.err == false) {    
                        temperature.err = true;
                        temperature.change = true;
                    }
                } else {
                    t_float = get_float_temperature_by_adc(bypass_r_adc);
                    t_int = calculate_approximate_t(t_float);
                    /*判断是否在报警范围*/ 
                    if ( t_int > TEMPERATURE_ALARM_VALUE_MAX || t_int < TEMPERATURE_ALARM_VALUE_MIN){
                        if (temperature.err == false) {    
                            temperature.err = true;
                            temperature.change = true;
                        }
                    } else {  
                        temperature.err = false;
                        if (t_int > temperature.value_int){
                            temperature.dir += 1;    
                        }else if(t_int < temperature.value_int){
                            temperature.dir -= 1;      
                        }
                        /*当满足条件时 接受数据变化*/
                        if (temperature.dir >= TEMPERATURE_TASK_TEMPERATURE_CHANGE_CNT ||
                            temperature.dir <= -TEMPERATURE_TASK_TEMPERATURE_CHANGE_CNT){
                            temperature.value_int = t_int;
                            temperature.value_float = t_float;
                            temperature.dir = 0;
                            temperature.change = true;
                        }
                    }
                }
    
                if (temperature.change == true) {
         
                    if (temperature.err == true){
                        /*压缩机温度错误消息*/
                        update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_ERR;
                        log_error("temperature err.\r\n");
                    }else{
                        /*压缩机温度更新消息*/
                        update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_UPDATE;
                        update_msg.request.temperature_int = temperature.value_int;
                        update_msg.request.temperature_float = temperature.value_float;
                        log_info("teperature changed dir:%d value:%d C.\r\n",temperature.dir,temperature.value_int);
                    }
                    temperature.change = false;           
                    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&update_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
                    if (status !=osOK) {
                        log_error("put compressor t msg error:%d\r\n",status); 
                    } 
                }
            }
        }
    }
}