#ifndef  __TASKS_INIT_H__
#define  __TASKS_INIT_H__

#include "stdint.h"


#ifdef  __cplusplus
#define TASKS_BEGIN  extern "C" {
#define TASKS_END    }
#else
#define TASKS_BEGIN  
#define TASKS_END   
#endif

TASKS_BEGIN

extern EventGroupHandle_t tasks_sync_evt_group_hdl;
void tasks_init();


TASKS_END


#endif