#include "device_env.h"
#if  DEVICE_ENV_USE_EEPROM > 0
#include "eeprom_if.h"
#else
#include "flash_if.h"
#endif
#include "crc16.h"
#include "log.h"

/*****************************************************************************
*  设备环境                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     device_env.c                                                   
*  @brief    设备环境                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/5/5                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#define  DEVICE_ENV_DATA_SIZE_LIMIT   (DEVICE_ENV_SIZE_LIMIT - 2)


typedef struct
{
    uint16_t crc;
    uint8_t  data_region[DEVICE_ENV_DATA_SIZE_LIMIT];
}device_env_t;

/*内部静态环境变量*/
static device_env_t device_env;

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static char device_env_get_char(int index) 
{
    return *((char *)(&device_env.data_region[0] + index));
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static char * device_env_get_addr(int index)
{
    return (char *)(&device_env.data_region[0] + index);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int device_env_match(char *name,int index2) 
{
    while (*name == device_env_get_char(index2 ++)) {
        if (*name ++ == '=') {
            return (index2);
        }
    }
    if (*name == '\0' && device_env_get_char(index2 - 1) == '=') {
        return (index2);
    }

    return (-1);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static void device_env_crc_update() 
{
    device_env.crc = calculate_crc16(device_env.data_region,DEVICE_ENV_DATA_SIZE_LIMIT);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int device_env_crc_check() 
{
    return device_env.crc == calculate_crc16(device_env.data_region,DEVICE_ENV_DATA_SIZE_LIMIT) ? 1 : 0;
}

/*
* @brief 环境变量值保存到非易存储体
* @param name 无
* @return 0：成功 -1：失败
* @note
*/
static int device_env_do_save(void) 
{
    int rc;
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
    rc = flash_if_erase(DEVICE_ENV_BASE_ADDR,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do save err.\r\n");
        return -1;
    }
    rc = flash_if_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif

    if (rc != 0) {
        log_error("do save err.\r\n");
        return -1;
    }

#if DEVICE_ENV_USE_BACKUP > 0  /*环境变量备份*/
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
    rc = flash_if_erase(DEVICE_ENV_BACKUP_BASE_ADDR,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do backup save err.\r\n");
        return -1;
    }
    rc = flash_if_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif
    if (rc != 0) {
        log_error("do backup save err.\r\n");
        return -1;
    }

#endif

    log_debug("do save ok.\r\n");
    return 0;
}

/*
* @brief 环境变量初始化
* @param 无
* @return 0：成功 -1：失败
* @note
*/
int device_env_init(void) 
{
    int rc;

    /*读取设备环境变量*/
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_init();
    if (rc != 0) {
        log_error("eeprom init err.\r\n");
        return -1;
    }
    rc = eeprom_if_read(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
    rc = flash_if_read(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif
    if (rc != 0) {
        log_error("read env err.code:%d.\r\n",rc);
        return -1;
    }
    /*对比校验值*/
    rc = device_env_crc_check();
    if (!rc) {
        log_warning("env crc bad. ");

#if DEVICE_ENV_USE_BACKUP > 0 
        log_debug("read backup env...\r\n");
#if DEVICE_ENV_USE_EEPROM > 0
        rc = eeprom_if_read(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
        rc = flash_if_read(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif
        if (rc != 0) {
            log_error("read backup env err.code:%d.\r\n",rc);
            return -1;
        }
        /*如果备份环境变量是有效的，就使用备份*/
        rc = device_env_crc_check();
        if (rc) {
            log_warning("backup env crc ok. ");
            return 0;
        }
        log_warning("backup crc bad. ");
#endif
        /*如果环境变量无效 初始化为0*/
        memset(&device_env,0x00,sizeof(device_env_t));
    } else {
        log_warning("env crc ok. ");
    }

    return 0;
}

/*
* @brief 环境变量冲刷
* @param 无
* @return 0：成功 -1：失败
* @note
*/
int device_env_flush(void) 
{
    int rc;

    memset(&device_env,0x00,sizeof(device_env_t));

#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
    rc = flash_if_erase(DEVICE_ENV_BASE_ADDR,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do flush err.\r\n");
        return -1;
    }
    rc = flash_if_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif

    if (rc != 0) {
        log_error("do flush err.\r\n");
        return -1;
    }

#if DEVICE_ENV_USE_BACKUP > 0  /*环境变量备份*/
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#else
    rc = flash_if_erase(DEVICE_ENV_BACKUP_BASE_ADDR,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do backup flush err.\r\n");
        return -1;
    }
    rc = flash_if_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
#endif
    if (rc != 0) {
        log_error("do backup flush err.\r\n");
        return -1;
    }

#endif

    log_debug("do flush ok.\r\n");

    return 0;
}

/*
* @brief 获取对应名称的环境变量值
* @param name 环境变量名
* @return 环境变量值 or null
* @note
*/
char *device_env_get(char *name) 
{
    int i, next;
    for (i=0; device_env_get_char(i) != '\0'; i = next + 1) {
        int val;
        for (next = i; device_env_get_char(next) != '\0'; ++ next) {
            if (next >= DEVICE_ENV_SIZE_LIMIT) {
                return (NULL);
            }
        }
        if ((val = device_env_match((char *)name, i)) < 0) {
            continue;
        }

        return ((char *)device_env_get_addr(val));
    }

    return (NULL);
}

/*
* @brief 设置环境变量值
* @param name 环境变量名
* @param value 环境变量值
* @return 0：成功 -1：失败
* @note
*/
int device_env_set(char *name,char *value) 
{
    int  len, oldval;

    char *env, *next = NULL;

    char *env_data = device_env_get_addr(0);

    if (strchr(name, '=')) {
        log_error("## Error: illegal character '=' in variable name \"%s\"\n", name);
        return -1;
    }

    /*
     * search if variable with this name already exists
     */
    oldval = -1;
    for ( env = env_data; *env; env = next + 1) {
        for (next = env; *next; ++ next);
        if ((oldval = device_env_match((char *)name, env - env_data)) >= 0)
            break;
    }

    /*
     * Delete any existing definition
     */
    if (oldval >= 0) {      
        if (*++next == '\0') {
            if (env > env_data) {
                env--;
            } else {
                *env = '\0';
            }
        } else {
            while(1) {
                *env = *next++;
                if ((*env == '\0') && (*next == '\0')) {
                    break;
                }
                ++env;
            }
        }
        *++env = '\0';
    }
    /* Delete only ? */
    if (value == NULL) {
        device_env_crc_update();
        return device_env_do_save();
    }

    /*
     * Append new definition at the end
     */
    for (env = env_data; *env || *(env + 1); ++ env);

    if (env > env_data) {
        ++ env;
    }
    /*
     * Overflow when:
     * "name" + "=" + "val" +"\0\0"  > ENV_SIZE - (env-env_data)
     */
    len = strlen(name) + 2;
    /* add '=' for first arg, ' ' for all others */
    len += strlen(value) + 1;

    if (len > (&env_data[DEVICE_ENV_DATA_SIZE_LIMIT] - env)) {
        log_error("## Error: environment overflow, \"%s\" deleted\n", name);
        return 1;
    }
    while ((*env = *name++) != '\0') {
        env++;
    }


    char *val = value;

    *env = '=';
    while ((*++env = *val++) != '\0');
   

    /* end is marked with double '\0' */
    *++env = '\0';

    /* Update CRC */
    device_env_crc_update();
    return device_env_do_save();
}
