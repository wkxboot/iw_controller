#include "fsl_flashiap.h"
#include "nv_flash.h"
#include "log.h"

#define  NV_FLASH_SECTOR_CNT                                16   
#define  NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET             15  
#define  NV_FLASH_USER_DATA_SECTOR_BANK1_CNT                1
#define  NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET             14   
#define  NV_FLASH_USER_DATA_SECTOR_BANK2_CNT                1

#define  NV_FLASH_USER_DATA_PAGE_CNT_PER_SECTOR             (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES)  
#define  NV_FLASH_USER_DATA_SIZE                            248


typedef enum {
    NV_FLASH_REGION_BANK1,
    NV_FLASH_REGION_BANK2
}nv_flash_region_t;

typedef enum
{
    REGION_STATUS_VALID = 0x19890414
}nv_flash_status_t;



typedef struct
{
    nv_flash_status_t status;
    uint32_t id;
    uint8_t user[NV_FLASH_USER_DATA_SIZE];
}nv_flash_block_t;

/*当前数据块*/
static nv_flash_block_t current;
/*当前数据块指针*/
static nv_flash_block_t *start_addr_bank1 = (nv_flash_block_t *)(NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET * FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);
static nv_flash_block_t *end_addr_bank1 = (nv_flash_block_t *)((NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK1_CNT) * FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);
static nv_flash_block_t *start_addr_bank2 = (nv_flash_block_t *)(NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET * FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);
static nv_flash_block_t *end_addr_bank2 = (nv_flash_block_t *)((NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK2_CNT) * FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);



/*
* @brief nv_flash_search_free_block 查找可用block块
* @param region 存储区域类型
* @return block块地址
* @note
*/
static nv_flash_block_t *nv_flash_search_free_block(nv_flash_region_t region)
{
    nv_flash_block_t *addr,*start_addr,*end_addr;
    
    
    if (region == NV_FLASH_REGION_BANK1) {
        start_addr = start_addr_bank1;
        end_addr = end_addr_bank1;
    } else {
        start_addr = start_addr_bank2;
        end_addr = end_addr_bank2;
    }

    for ( addr = start_addr; addr + 1 <= end_addr; addr ++) {
        if (addr->status != REGION_STATUS_VALID) {
            return addr;
        }
    }

    return NULL;
}

/*
* @brief nv_flash_search_current_block 查找当前block块
* @param region 存储区域类型
* @return block块地址
* @note
*/
static nv_flash_block_t *nv_flash_search_current_block(nv_flash_region_t region)
{
    nv_flash_block_t *addr,*start_addr,*end_addr;
       
    if (region == NV_FLASH_REGION_BANK1) {
        start_addr = start_addr_bank1;
        end_addr = end_addr_bank1;
    } else {
        start_addr = start_addr_bank2;
        end_addr = end_addr_bank2;
    }
    addr = nv_flash_search_free_block(region);
    if (addr == NULL) {
        return end_addr - 1;
    } 

    return addr > start_addr ? addr - 1 : NULL;
}

/*
* @brief nv_flash_erase_region 擦除存储区域
* @param region 存储区域类型
* @return 0：成功 -1：失败
* @note
*/
static int nv_flash_erase_region(nv_flash_region_t region)
{
    status_t status;
    uint32_t sector_num,sector_cnt;

    if (region == NV_FLASH_REGION_BANK1) {
        sector_num = NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET;
        sector_cnt = NV_FLASH_USER_DATA_SECTOR_BANK1_CNT;
        log_debug("erase bank1...\r\n");
    } else {
        sector_num = NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET;
        sector_cnt = NV_FLASH_USER_DATA_SECTOR_BANK2_CNT;
        log_debug("erase bank2...\r\n");
    }
    /*保存块到bank*/
    status = FLASHIAP_PrepareSectorForWrite(sector_num,sector_num + sector_cnt - 1);
    if (status != kStatus_Success) {
        log_error("prepare bank status:%d err.\r\n",status);
        return -1;
    }

    status = FLASHIAP_EraseSector(sector_num,sector_num + sector_cnt - 1, SystemCoreClock);
    if (status != kStatus_Success) {
        log_error("erase fail.\r\n");
        return -1;
    }
    log_debug("erase success.\r\n");
    return 0;
}

   
/*
* @brief nv flash 数据读取
* @param offset 读取偏移地址
* @param dst 数据保存的地址
* @param cnt 读取的数据量
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_read_user_data(uint32_t offset,uint8_t *dst,uint32_t cnt)
{
    nv_flash_block_t *addr;
    uint32_t read_size = 0;
    
    /*只第一个bank查询*/
    addr = nv_flash_search_current_block(NV_FLASH_REGION_BANK1);
    if (addr == NULL) {
        return -1;
    }
    while (read_size < cnt && read_size + offset < NV_FLASH_USER_DATA_SIZE) {
        dst[read_size] = addr->user[offset + read_size];
        read_size ++;
    }

    return read_size;
}

/*
* @brief nv_flash_save_user_data 数据保存,保证断电可靠
* @param offset 保存的偏移地址
* @param src 数据保存的地址
* @param cnt 数据量
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_save_user_data(uint32_t offset,uint8_t *src,uint32_t cnt)
{
    int rc;
    uint32_t write_size = 0;
    status_t status;
    nv_flash_block_t *addr_current,*addr_free;
  
    /*查找bank1上的block块*/
    addr_free = nv_flash_search_free_block(NV_FLASH_REGION_BANK1);
    /*如果bank1已经存满,就擦除bank1,重新写入*/
    if (addr_free == NULL) {      
        rc = nv_flash_erase_region(NV_FLASH_REGION_BANK1); 
        if (rc != 0) {
            log_error("erase bank1 err.\r\n");
            return -1;
        }
        /*查找bank1空闲块*/
        addr_free = nv_flash_search_free_block(NV_FLASH_REGION_BANK1);   
        if (addr_free == NULL) {
            /*如果还是没有block bug*/
            log_error("nv flash BUG.\r\n");
            return -1;
        }
    }
        
    /*查找bank2上的存储块*/
    addr_current = nv_flash_search_current_block(NV_FLASH_REGION_BANK2);
    /*如果bank2不是空的*/
    if (addr_current) {
        /*获取当前数据*/
        current = *addr_current; 
        current.id ++;
    } else {
        current.id = 1;
    }
    /*更新数据*/
    while (write_size < cnt && write_size + offset < NV_FLASH_USER_DATA_SIZE) {
        current.user[offset + write_size] = src[write_size];
        write_size ++;
    }
    current.status = REGION_STATUS_VALID;
    /*保存块到bank1*/
    status = FLASHIAP_PrepareSectorForWrite(NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET, NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK1_CNT - 1);
    if (status != kStatus_Success) {
        log_error("prepare bank1 status:%d err.\r\n",status);
        return -1;
    }
    log_debug("start program bank1 addr:%d cnt:%d...\r\n",addr_free,sizeof(nv_flash_block_t));
    status = FLASHIAP_CopyRamToFlash((uint32_t)addr_free,(uint32_t*)&current,sizeof(nv_flash_block_t),SystemCoreClock);
    if (status != kStatus_Success) {
        log_error("program bank1 status:%d err.\r\n",status);
        return -1;
    }
    log_debug("program bank1 ok.\r\n");

    /*在bank2查找空闲块*/
    addr_free = nv_flash_search_free_block(NV_FLASH_REGION_BANK2);
    /*如果bank2满了就擦除,重新写入*/
    if (addr_free == NULL) {
        rc = nv_flash_erase_region(NV_FLASH_REGION_BANK2);
        if (rc != 0) {
            log_error("erase bank2 err.\r\n");
            return -1;
        }
        /*再次查找bank2空闲块*/
        addr_free = nv_flash_search_free_block(NV_FLASH_REGION_BANK2);   
        if (addr_free == NULL) {
            /*如果还是没有block bug*/
            log_error("nv flash BUG.\r\n");
            return -1;
        }  
    }
    /*保存块到bank2*/
    status = FLASHIAP_PrepareSectorForWrite(NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET, NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK2_CNT - 1);
    if (status != kStatus_Success) {
        log_error("prepare bank2 status:%d err.\r\n",status);
        return -1;
    }
    log_debug("start program bank2 addr:%d cnt:%d.\r\n",addr_free,sizeof(nv_flash_block_t));
    status = FLASHIAP_CopyRamToFlash((uint32_t)addr_free,(uint32_t*)&current,sizeof(nv_flash_block_t),SystemCoreClock);
    if (status != kStatus_Success) {
        log_error("program bank2 status:%d err.\r\n",status);
        return -1;
    }
    log_debug("program bank2 ok.\r\n");

    return 0;
}

/*
* @brief nv_flash_region_int nv flash数据区域初始化
* @param 无
* @return 0 成功 -1 失败
* @note
*/
int nv_flash_region_int(void)
{
    int rc;
    status_t status;
    nv_flash_block_t *sync_addr,*addr_bank1,*addr_bank2;
    nv_flash_region_t sync_region;

    /*在bank1查找存储块*/
    addr_bank1 = nv_flash_search_current_block(NV_FLASH_REGION_BANK1);
    /*在bank2查找存储块*/
    addr_bank2 = nv_flash_search_current_block(NV_FLASH_REGION_BANK2);

    /*如果bank1和bank2都不存在block，跳过*/
    if (addr_bank1 == NULL && addr_bank2 == NULL) {
        log_debug("nv flash check ok.no data.\r\n");
        return 0;
    } 
    /*如果都存在，就比较id*/
    if (addr_bank1 && addr_bank2) {
        if (addr_bank1->id == addr_bank2->id) {
            log_debug("nv flash check ok.normal.\r\n");
            return 0;
        } 
        if (addr_bank1->id > addr_bank2->id) {
            current = *addr_bank1;
            sync_region = NV_FLASH_REGION_BANK2;
        } else {
            current = *addr_bank2; 
            sync_region = NV_FLASH_REGION_BANK1;
        }
    } else if (addr_bank1) {/*如果只有bank1存在就同步bank2*/
        current = *addr_bank1; 
        sync_region = NV_FLASH_REGION_BANK2;
    } else {/*如果只有bank2存在就同步bank1*/
        current = *addr_bank2; 
        sync_region = NV_FLASH_REGION_BANK1;
    }
    log_debug("start sync block...\r\n");
    /*查找bank1空闲块*/
    sync_addr = nv_flash_search_free_block(sync_region);   
    if (sync_addr == NULL) {
        /*如果已经存满,就擦除,重新写入*/    
        rc = nv_flash_erase_region(sync_region); 
        if (rc != 0) {
            log_error("erase bank err.\r\n");
            return -1;
        }
        /*再次查找空闲块*/
        sync_addr = nv_flash_search_free_block(sync_region);   
        if (sync_addr == NULL) {
            /*如果还是没有block bug*/
            log_error("nv flash BUG.\r\n");
            return -1;
        }
    }
    /*保存块*/
    if (sync_region == NV_FLASH_REGION_BANK1) {
        status = FLASHIAP_PrepareSectorForWrite(NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET,NV_FLASH_USER_DATA_SECTOR_BANK1_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK1_CNT - 1);
    } else {
        status = FLASHIAP_PrepareSectorForWrite(NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET,NV_FLASH_USER_DATA_SECTOR_BANK2_OFFSET + NV_FLASH_USER_DATA_SECTOR_BANK2_CNT - 1);
    }
    if (status != kStatus_Success) {
        log_error("prepare bank status:%d err.\r\n",status);
        return -1;
    }
    log_debug("start program bank addr:%d cnt:%d.\r\n",sync_addr,sizeof(nv_flash_block_t));
    status = FLASHIAP_CopyRamToFlash((uint32_t)sync_addr,(uint32_t*)&current,sizeof(nv_flash_block_t),SystemCoreClock);
    if (status != kStatus_Success) {
        log_error("program bank status:%d err.\r\n",status);
        return -1;
    }
    log_debug("sync block ok.\r\n");
    
    return 0;
}