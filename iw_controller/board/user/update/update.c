#include "device_env.h"
#include "flash_if.h"
#include "update.h"
#include "log.h"

/*内部编程地址*/
static uint32_t update_addr = APPLICATION_UPDATE_BASE_ADDR;

/*
* @brief 更新程序复位
* @param 无
* @param
* @return 无
* @note
*/
void bootloader_update_init(void)
{


}

/*
* @brief 更新程序复位
* @param 无
* @param
* @return 无
* @note
*/
void bootloader_update_reset(void)
{
    update_addr = APPLICATION_UPDATE_BASE_ADDR;
}

/*
* @brief 下载更新程序到更新区域
* @param src 数据源地址
* @param size 数量
* @return 0：成功 -1：失败
* @note
*/
int bootloader_copy_new_image_to_update_region(uint8_t *src,uint32_t size)
{
    int rc;


    if ((size & (DEVICE_MIN_ERASE_SIZE - 1)) != 0 || size == 0) {
        log_error("err not align with erase size:%d.\r\n",DEVICE_MIN_ERASE_SIZE);
        goto err_handle;
    }
    /*先擦除*/  
    rc = flash_if_erase(update_addr,size);
    if (rc != 0) {
        goto err_handle;
    }
    /*然后编程写入*/
    rc = flash_if_write(update_addr,src,size);
    if (rc != 0) {
        goto err_handle;
    }
    /*更新地址*/
    update_addr += size;
    log_debug("copy %d bytes new image to update region ok \r\n",size);
    return 0;

err_handle:
    log_error("copy new image err.\r\n");
    bootloader_update_reset();

    return -1;
}

