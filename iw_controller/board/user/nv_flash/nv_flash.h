#ifndef  __NV_FLASH_H__
#define  __NV_FLASH_H__

#ifdef __cplusplus
    extern "C" {
#endif


#define  NV_FLASH_PRIORITY_BITS                   3
#define  NV_FLASH_PRIORITY_HIGH                   2  

#define  NV_FLASH_MAX_INTERRUPT_PRIORITY          (NV_FLASH_PRIORITY_HIGH << (8 - NV_FLASH_PRIORITY_BITS))

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


#ifdef __ICCARM__

#if (defined (__ARM6M__) && (__CORE__ == __ARM6M__))             
#define NV_FLASH_ENTER_CRITICAL()                              \
{                                                              \
    unsigned int pri_mask;                                     \
    pri_mask = __get_PRIMASK();                                \
    __set_PRIMASK(1);
    
#define NV_FLASH_EXIT_CRITICAL()                               \
    __set_PRIMASK(pri_mask);                                   \
}
#elif ((defined (__ARM7EM__) && (__CORE__ == __ARM7EM__)) || (defined (__ARM7M__) && (__CORE__ == __ARM7M__)))

#define NV_FLASH_ENTER_CRITICAL()                              \
{                                                              \
   unsigned int base_pri;                                      \
   base_pri = __get_BASEPRI();                                 \
   __set_BASEPRI(NV_FLASH_MAX_INTERRUPT_PRIORITY);   

#define NV_FLASH_EXIT_CRITICAL()                               \
   __set_BASEPRI(base_pri);                                    \
}
  #endif
#endif


/*
*  serial critical configuration for KEIL
*/ 
#ifdef __CC_ARM
#if (defined __TARGET_ARCH_6S_M)
#define NV_FLASH_ENTER_CRITICAL()                                             
 {                                                              \
    unsigned int pri_mask;                                      \
    register unsigned char PRIMASK __asm( "primask");           \
    pri_mask = PRIMASK;                                         \
    PRIMASK = 1u;                                               \
    __schedule_barrier();

#define NV_FLASH_EXIT_CRITICAL()                                \
    PRIMASK = pri_mask;                                         \
    __schedule_barrier();                                       \
}
#elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))

#define NV_FLASH_ENTER_CRITICAL()                               \
 {                                                              \
     unsigned int base_pri;                                     \
     register unsigned char BASEPRI __asm( "basepri");          \
      base_pri = BASEPRI;                                       \
      BASEPRI = NV_FLASH_MAX_INTERRUPT_PRIORITY;           \
      __schedule_barrier();

#define NV_FLASH_EXIT_CRITICAL()                                \
     BASEPRI = base_pri;                                        \
     __schedule_barrier();                                      \
}
#endif
#endif  

#ifdef __cplusplus
    }
#endif













#endif