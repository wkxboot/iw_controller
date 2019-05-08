/*****************************************************************************
*  循环缓存库函数                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     circle_buffer.c                                                   
*  @brief    循环缓存库函数                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/8                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "circle_buffer.h"

/*
* @brief 循环缓存刷新
* @param cb 循环缓存指针
* @param 
* @return  刷新的长度
* @note 
*/
int circle_buffer_flush(circle_buffer_t *cb)
{
    int size;
    assert(cb);

    size = cb->write - cb->read;
    cb->read = cb->write;   

    return size;
}

/*
* @brief 循环缓存容量
* @param cb 循环缓存指针
* @return 循环缓存容量
* @note
*/
int circle_buffer_size(circle_buffer_t *cb)
{
    assert(cb);

    return cb->size;
}

/*
* @brief 循环缓存空闲容量
* @param cb 循环缓存指针
* @return 循环缓存空闲容量
* @note
*/
int circle_buffer_free_size(circle_buffer_t *cb)
{ 
    if (cb == NULL) {
        return -1;
    }
    return cb->size - (cb->write - cb->read);
}

/*
* @brief 循环缓存已使用容量
* @param cb 循环缓存指针
* @return 循环缓存已使用容量
* @note
*/
int circle_buffer_used_size(circle_buffer_t *cb)
{ 
    if (cb == NULL) {
        return -1;
    }
    return  cb->write - cb->read;
}

/*
* @brief 循环缓存是否已满
* @param cb 循环缓存指针
* @return true 已满
* @return false 未满
* @note
*/
bool circle_buffer_is_full(circle_buffer_t *cb)
{  
    if (cb == NULL) {
        return -1;
    }
    return cb->write - cb->read == cb->size ? true : false;
}

/*
* @brief 循环缓存是否已空
* @param cb 循环缓存指针
* @return true 已空
* @return false 未空
* @note
*/
bool circle_buffer_is_empty(circle_buffer_t *cb)
{
    return cb->read == cb->write ? true : false;
}

/*
* @brief  读取循环缓存中的数据
* @param  cb   循环缓存指针
* @param  dst  目的地址
* @param  size 期望读取的数量
* @return 实际读取的数量
* @note
*/
int circle_buffer_read(circle_buffer_t *cb,char *dst,int size)
{
    int read_cnt = 0;

    while (cb->read < cb->write && read_cnt < size) {
        dst[read_cnt] = cb->buffer[cb->read & cb->mask];
        cb->read++;
        read_cnt++;
    }

    return read_cnt;
}


/*
* @brief 循环缓存写入数据
* @param cb 循环缓存指针
* @param src 数据源地址
* @param size 期望写入的数量
* @return 实际写入的数量
* @note
*/
int circle_buffer_write(circle_buffer_t *cb,const char *src,int size)
{
    int write_cnt = 0;

    while (circle_buffer_free_size(cb) > 0 && write_cnt < size) {
        cb->buffer[cb->write & cb->mask] = src[write_cnt];
        cb->write++;
        write_cnt++;  
    }

    return write_cnt;
}


