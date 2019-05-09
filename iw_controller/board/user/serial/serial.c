/*****************************************************************************
*  串口库函数                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     serial.c                                                   
*  @brief    串口库函数                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.2                                                  
*  @date     2019/1/8                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "serial.h"


/*
* @brief  从串口非阻塞的读取指定数量的数据
* @param handle 串口句柄
* @param dst 数据目的地址
* @param size 期望读取的数量
* @return < 0 读取错误
* @return >= 0 实际读取的数量
* @note 可重入
*/
int serial_read(serial_handle_t *handle,char *dst,int size)
{
    int read;
  
    if (handle->init == false || size < 0){
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    read = circle_buffer_read(&handle->recv,dst,size); 
    if (read > 0 && handle->recv_full == true) {
        handle->recv_full = false;
        handle->driver->enable_rxne_it(handle->port);  
    }
    SERIAL_EXIT_CRITICAL();

    return read;
}

/*
* @brief 串口输出缓存可用空间
* @param handle 串口句柄
* @param 
* @param 
* @return < 0 错误
* @return >= 0 可写入的数量
* @note 可重入
*/
int serial_writeable(serial_handle_t *handle)
{
    int free_size;

    if (handle->init == false){
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    free_size = circle_buffer_free_size(&handle->send);
    SERIAL_EXIT_CRITICAL();

    return free_size;
}

/*
* @brief  从串口非阻塞的写入指定数量的数据
* @param handle 串口句柄
* @param src 数据源地址
* @param size 期望写入的数量
* @return < 0 写入错误
* @return >= 0 实际写入的数量
* @note 可重入
*/
int serial_write(serial_handle_t *handle,const char *src,int size)
{
    int write;

    if (handle->init == false || size < 0){
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    write = circle_buffer_write(&handle->send,src,size);

    if (write > 0 && handle->send_empty == true){
        handle->send_empty = false;
        handle->driver->enable_txe_it(handle->port);
    }
    SERIAL_EXIT_CRITICAL();

    return write;
}
/*
* @brief  串口刷新
* @param handle 串口句柄
* @return < 0 失败
* @return >=0 刷新的接收缓存数量
* @note 
*/
int serial_flush(serial_handle_t *handle)
{
    int size;

    if (handle->registered == false ) {
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    handle->send_empty = true;
    handle->recv_full = false;
    handle->driver->disable_txe_it(handle->port);
    handle->driver->enable_rxne_it(handle->port);
    circle_buffer_flush(&handle->send);
    size = circle_buffer_flush(&handle->recv);
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  打开串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_open(serial_handle_t *handle,uint8_t port,uint32_t baud_rates,uint8_t data_bits,uint8_t stop_bits)
{
    int rc;

    if (handle->registered == false){
        return -1;
    }
 
    rc = handle->driver->init(port,baud_rates,data_bits,stop_bits);
    if (rc != 0){
        return -1;
    }
    SERIAL_ENTER_CRITICAL();
    handle->init = true;
    handle->port = port;
    handle->baud_rates = baud_rates;
    handle->data_bits = data_bits;
    handle->stop_bits = stop_bits;
    handle->driver->enable_rxne_it(handle->port);
    SERIAL_EXIT_CRITICAL();

    return 0;
}

/*
* @brief  关闭串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_close(serial_handle_t *handle)
{
    int rc = 0;

    if (handle->registered == false){
        return -1;
    }

    rc = handle->driver->deinit(handle->port);
    if (rc != 0){
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    handle->init = false;
    handle->send_empty = true;
    handle->recv_full = false;
    handle->driver->disable_rxne_it(handle->port);
    handle->driver->disable_txe_it(handle->port);
    SERIAL_EXIT_CRITICAL();
 
    return 0;
}

/*
* @brief  串口注册硬件驱动
* @param handle 串口句柄
* @param driver 硬件驱动指针
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_register_hal_driver(serial_handle_t *handle,serial_hal_driver_t *driver)
{
    assert(driver);
    assert(driver->init);
    assert(driver->deinit);
    assert(driver->enable_txe_it);
    assert(driver->disable_txe_it);
    assert(driver->enable_rxne_it);
    assert(driver->disable_rxne_it);

    handle->driver = driver;
    handle->registered = true;

    return 0;
}



/*
* @brief  串口中断发送routine
* @param handle 串口句柄
* @param byte_send 从发送循环缓存中取出的将要发送的一个字节
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_get_byte_to_send(serial_handle_t *handle,char *byte_send)
{
    int size;

    if (handle->init == false){
        return -1;
    } 
    SERIAL_ENTER_CRITICAL();
    size = circle_buffer_read(&handle->send,byte_send,1);
    /*发送缓存中已经没有待发送的数据，关闭发送中断*/
    if (size == 0) {
        handle->send_empty = true; 
        handle->driver->disable_txe_it(handle->port);
    }
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  串口中断接收routine
* @param handle 串口句柄
* @param byte_send 把从串口读取的一个字节放入接收循环缓存
* @return = 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_put_byte_from_recv(serial_handle_t *handle,char recv_byte)
{
    int size;

    if (handle->init == false){
        return -1;
    } 
    SERIAL_ENTER_CRITICAL();
    size = circle_buffer_write(&handle->recv,&recv_byte,1);
    /*接收缓存中已经没有空间，关闭接收中断*/
    if (size == 0) {
        handle->recv_full = true;
        handle->driver->disable_rxne_it(handle->port);
    }
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  串口创建
* @param handle 串口句柄
* @param rx_size 接收循环缓存容量
* @param tx_size 发送循环缓存容量
* @return = 0 成功
* @return < 0 失败
* @note     rx_size和tx_size必须是2的x次方
*/
int serial_create(serial_handle_t *handle,uint8_t *rx_buffer,uint32_t rx_size,uint8_t *tx_buffer,uint32_t tx_size)
{ 

    if (!IS_POWER_OF_TWO(rx_size)) {
        return -1;
    }

    if (!IS_POWER_OF_TWO(tx_size)) {
        return -1;
    }

    handle->recv.buffer = rx_buffer;
    handle->send.buffer = tx_buffer;
 
    handle->recv.size = rx_size;
    handle->recv.mask = rx_size - 1;
    handle->send.size = tx_size;
    handle->send.mask = tx_size - 1;
    handle->recv.read = 0;
    handle->recv.write = 0;
 
    handle->send.read = 0;
    handle->send.write = 0;

    handle->driver = NULL;
    handle->registered = false;

    return 0;	
}


#if SERIAL_IN_FREERTOS  > 0
#include "cmsis_os.h"
/*
* @brief  串口等待数据
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待超时
* @return > 0 等待的数据量
* @note 
*/
int serial_select(serial_handle_t *handle,uint32_t timeout)
{
    int size = 0;
    utils_timer_t timer;

    if (handle->init == false) {
        return -1;
    } 

    utils_timer_init(&timer,timeout,false);

    while (utils_timer_value(&timer) > 0 && size == 0) {
        size = circle_buffer_used_size(&handle->recv);
        if (size == 0) {
            osDelay(1);
        }
    } 
        
    return size;
}


/*
* @brief  串口等待数据发送完毕
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待发送超时
* @return > 0 实际发送的数据量
* @note 
*/
int serial_complete(serial_handle_t *handle,uint32_t timeout)
{
    utils_timer_t timer; 

    if (handle->init == false){
        return -1;
    } 
    utils_timer_init(&timer,timeout,false);

    while (utils_timer_value(&timer) > 0 && handle->send_empty == false) {
        osDelay(1);
    }

    return circle_buffer_used_size(&handle->send);
}

#endif

