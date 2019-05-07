#ifndef  __DEVICE_ENV_H__
#define  __DEVICE_ENV_H__

#ifdef  __cplusplus
extern "C" {
#endif

/********************    配置设备环境开始    **************************************/
#define  DEVICE_ENV_USE_BACKUP                 1            /*是否使用环境变量备份*/

#define  BOOTLOADER_BASE_ADDR                  0x00000000UL /*bootloader基地址*/
#define  BOOTLOADER_SIZE_LIMIT                 0xA000       /*bootloader最大容量*/

#define  APPLICATION_SIZE_LIMIT                0x19000      /*application最大容量*/

#define  APPLICATION_BASE_ADDR                 0x0000A000UL /*application基地址*/

#define  APPLICATION_BACKUP_BASE_ADDR          0x00023000UL /*application备份基地址*/

#define  APPLICATION_UPDATE_BASE_ADDR          0x0003C000UL /*application更新基地址*/

#define  DEVICE_MIN_ERASE_SIZE                 256          /*最小擦除单元大小 bytes*/
#define  DEVICE_ADDR_MAP_LIMIT                 0x00080000   /*设备最大地址映射*/    
    
#define  DEVICE_ENV_BASE_ADDR                  0x00055000   /*环境变量基地址*/   

#define  DEVICE_ENV_BACKUP_BASE_ADDR           0x00055200   /*环境变量备份基地址*/      
     
#define  DEVICE_ENV_SIZE_LIMIT                 (DEVICE_MIN_ERASE_SIZE * 2)   /*环境变量大小*/ 


/********************    配置设备环境结束    **************************************/
/*配置校验
*/
#if  (BOOTLOADER_BASE_ADDR + BOOTLOADER_SIZE_LIMIT) > APPLICATION_BASE_ADDR
#error "application base addr too small."
#endif

#if  (APPLICATION_BASE_ADDR + APPLICATION_SIZE_LIMIT) > APPLICATION_BACKUP_BASE_ADDR
#error "application backup base addr too small."
#endif

#if  (APPLICATION_BACKUP_BASE_ADDR + APPLICATION_SIZE_LIMIT) > APPLICATION_UPDATE_BASE_ADDR
#error "application update base addr too small."
#endif

#if  (APPLICATION_UPDATE_BASE_ADDR + APPLICATION_SIZE_LIMIT) > DEVICE_ENV_BASE_ADDR
#error "device env base addr too small."
#endif

#if  DEVICE_ENV_USE_BACKUP > 0
#if  (DEVICE_ENV_BASE_ADDR + DEVICE_ENV_SIZE_LIMIT) > DEVICE_ENV_BACKUP_BASE_ADDR
#error "device env backup base addr too small."
#endif

#if  (DEVICE_ENV_BACKUP_BASE_ADDR + DEVICE_ENV_SIZE_LIMIT) > DEVICE_ADDR_MAP_LIMIT
#error "device env backup limit addr large than device addr map."
#endif

#else
#if  (DEVICE_ENV_BASE_ADDR + DEVICE_ENV_SIZE_LIMIT) > DEVICE_ADDR_MAP_LIMIT
#error "device env base limit addr large than device addr map."."
#endif

#endif

#define  ENV_BOOTLOADER_FLAG_NAME                  "bootloader"
#define  ENV_BOOTLOADER_UPDATE_SIZE_NAME           "update_size"
#define  ENV_BOOTLOADER_APPLICATION_SIZE_NAME      "app_size"
#define  ENV_BOOTLOADER_BACKUP_SIZE_NAME           "backup_size"
#define  ENV_BOOTLOADER_UPDATE_MD5_NAME            "update_md5"
#define  ENV_BOOTLOADER_APPLICATION_MD5_NAME       "app_md5"
#define  ENV_BOOTLOADER_BACKUP_MD5_NAME            "backup_md5"

/*bootloader状态标志*/
#define  ENV_BOOTLOADER_INIT                       "INIT"    /*初始模式*/
#define  ENV_BOOTLOADER_NORMAL                     "NORMAL"  /*正常启动模式*/
#define  ENV_BOOTLOADER_NEW                        "NEW"     /*进入更新模式*/
#define  ENV_BOOTLOADER_UPDATE                     "UPDATE"  /*已经备份完毕，正在拷贝更新的数据模式*/
#define  ENV_BOOTLOADER_COMPLETE                   "COMPLETE"/*更新的数据已复制完毕模式*/
#define  ENV_BOOTLOADER_OK                         "OK"      /*升级成功模式*/



/*
* @brief 环境变量初始化
* @param 无
* @return 0：成功 -1：失败
* @note
*/
int device_env_init(void);

/*
* @brief 获取对应名称的环境变量值
* @param name 环境变量名
* @return 环境变量值 or null
* @note
*/
char *device_env_get(char *name);


/*
* @brief 设置环境变量值
* @param name 环境变量名
* @param value 环境变量值
* @return 0：成功 -1：失败
* @note
*/
int device_env_set(char *name,char *value);


#ifdef  __cplusplus
    }
#endif

#endif  