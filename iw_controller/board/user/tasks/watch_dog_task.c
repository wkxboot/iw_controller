#include "stdbool.h"
#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "watch_dog_task.h"
#include "tasks_init.h"
#include "log.h"
#include "fsl_wwdt.h"
#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
#include "fsl_power.h"
#endif


/*******************************************************************************
 * Definitions
 ******************************************************************************/

osThreadId   watch_dog_task_hdl;


#define  WDT_ENABLE                  1

#if  WDT_ENABLE  > 0 

#define WDT_CLK_FREQ CLOCK_GetFreq(kCLOCK_WdtOsc)
/*
* @brief 看门狗中断
* @param 无
* @param
* @return 无
* @note
*/
void WDT_BOD_IRQHandler(void)
{
    uint32_t wdtStatus = WWDT_GetStatusFlags(WWDT);

    /* The chip will reset before this happens */
    if (wdtStatus & kWWDT_TimeoutFlag)
    {
        /* A watchdog feed didn't occur prior to window timeout */
        /* Stop WDT */
        WWDT_Disable(WWDT);
        WWDT_ClearStatusFlags(WWDT, kWWDT_TimeoutFlag);
        /* Needs restart */
        WWDT_Enable(WWDT);
    }

    /* Handle warning interrupt */
    if (wdtStatus & kWWDT_WarningFlag)
    {
        log_debug("feed dog.v:%d.\r\n",WWDT->TV);
        /* A watchdog feed didn't occur prior to warning timeout */
        WWDT_ClearStatusFlags(WWDT, kWWDT_WarningFlag);
       /* Feed only for the first 5 warnings then allow for a WDT reset to occur */
        WWDT_Refresh(WWDT);
    }
       

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */

#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

#endif


/*
* @brief 看门狗任务
* @param
* @param
* @return 
* @note
*/

void watch_dog_task(void const * argument)
{

#if  WDT_ENABLE  > 0 
    wwdt_config_t config;
    uint32_t wdtFreq;
    /* Enable clock of wwdt. */
    CLOCK_EnableClock(kCLOCK_Wwdt);

#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);
#endif

    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = WDT_CLK_FREQ / 4;
    NVIC_EnableIRQ(WDT_BOD_IRQn);
    NVIC_SetPriority(WDT_BOD_IRQn, 3);

    WWDT_GetDefaultConfig(&config);

    /* Check if reset is due to Watchdog */
    if (WWDT_GetStatusFlags(WWDT) & kWWDT_TimeoutFlag) {
        log_warning("Watchdog reset last.\r\n");
    } else {
        log_warning("reset normal last.\r\n");
    }

    /*
     * Set watchdog feed time constant to approximately 5s
     * Set watchdog warning time to 1023 ticks after feed time constant
     * Set watchdog window time to 4s
     */
    config.timeoutValue = wdtFreq * 5;
    config.warningValue = 1023;
    config.windowValue = wdtFreq * 4;
    /* Configure WWDT to reset on timeout */
    config.enableWatchdogReset = true;
    /* Setup watchdog clock frequency(Hz). */
    config.clockFreq_Hz = WDT_CLK_FREQ;

    WWDT_Init(WWDT, &config);
#endif

    while (1){
        osDelay(WATCH_DOG_TASK_INTERVAL); 
    }
}