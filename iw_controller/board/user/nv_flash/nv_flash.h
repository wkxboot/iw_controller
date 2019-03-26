#ifndef  __NV_FLASH_H__
#define  __NV_FLASH_H__

/*
* @brief nv_flash_region_int nv flash数据区域初始化
* @param 无
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_region_int(void);
/*
* @brief nv_flash_save_user_data 数据保存,保证断电可靠
* @param offset 保存的偏移地址
* @param src 数据保存的地址
* @param cnt 数据量
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_save_user_data(uint32_t offset,uint8_t *src,uint32_t cnt);
/*
* @brief nv flash 数据读取
* @param offset 读取偏移地址
* @param dst 数据保存的地址
* @param cnt 读取的数据量
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_read_user_data(uint32_t offset,uint8_t *dst,uint32_t cnt);















#endif