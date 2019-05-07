#ifndef  __UPDATE_H__
#define  __UPDATE_H__

#ifdef  __cplusplus
extern "C" {
#endif



/*
* @brief 更新程序复位
* @param 无
* @param
* @return 无
* @note
*/
void bootloader_update_init(void);

/*
* @brief 更新程序复位
* @param 无
* @param
* @return 无
* @note
*/
void bootloader_update_reset(void);

/*
* @brief 下载更新程序到更新区域
* @param src 数据源地址
* @param size 数量
* @return 0：成功 -1：失败
* @note
*/
int bootloader_copy_new_image_to_update_region(uint8_t *src,uint32_t size);







#ifdef  __cplusplus
    }
#endif

#endif