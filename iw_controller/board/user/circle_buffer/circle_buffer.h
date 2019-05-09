#ifndef  __CIRCLE_BUFFER_H__
#define  __CIRCLE_BUFFER_H__
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
#include "assert.h"

#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif


#ifdef __cplusplus
    extern "C" {
#endif


typedef struct
{
    uint8_t    *buffer;
    uint32_t   read;
    uint32_t   write;
    uint32_t   mask;
    uint32_t   size;
}circle_buffer_t;


/*
* @brief 循环缓存刷新
* @param cb 循环缓存指针
* @param 
* @return  刷新的长度
* @note 
*/
int circle_buffer_flush(circle_buffer_t *cb);

/*
* @brief 循环缓存容量
* @param cb 循环缓存指针
* @return 循环缓存容量
* @note
*/
int circle_buffer_size(circle_buffer_t *cb);
/*
* @brief 循环缓存空闲容量
* @param cb 循环缓存指针
* @return 循环缓存空闲容量
* @note
*/
int circle_buffer_free_size(circle_buffer_t *cb);
/*
* @brief 循环缓存已使用容量
* @param cb 循环缓存指针
* @return 循环缓存已使用容量
* @note
*/
int circle_buffer_used_size(circle_buffer_t *cb);

/*
* @brief 循环缓存是否已满
* @param cb 循环缓存指针
* @return true 已满
* @return false 未满
* @note
*/
bool circle_buffer_is_full(circle_buffer_t *cb);

/*
* @brief 循环缓存是否已空
* @param cb 循环缓存指针
* @return true 已空
* @return false 未空
* @note
*/
bool circle_buffer_is_empty(circle_buffer_t *cb);

/*
* @brief  读取循环缓存中的数据
* @param  cb   循环缓存指针
* @param  dst  目的地址
* @param  size 期望读取的数量
* @return 实际读取的数量
* @note
*/
int circle_buffer_read(circle_buffer_t *cb,char *dst,int size);

/*
* @brief 循环缓存写入数据
* @param cb 循环缓存指针
* @param src 数据源地址
* @param size 期望写入的数量
* @return 实际写入的数量
* @note
*/
int circle_buffer_write(circle_buffer_t *cb,const char *src,int size);


#ifdef __cplusplus
    }
#endif

#endif


