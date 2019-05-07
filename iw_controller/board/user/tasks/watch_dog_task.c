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
#define WDT_IRQn WDT_BOD_IRQn
#define WDT_CLK_FREQ CLOCK_GetFreq(kCLOCK_WdtOsc)

osThreadId   watch_dog_task_hdl;

/*
* @brief 看门狗任务
* @param argument 任务参数
* @param
* @return 无
* @note
*/
void watch_dog_task(void const * argument)
{
    wwdt_config_t config;
    uint32_t wdtFreq;

#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);
#endif

    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = WDT_CLK_FREQ / 4;
    WWDT_GetDefaultConfig(&config);
    /* Check if reset is due to Watchdog */
    if (WWDT_GetStatusFlags(WWDT) & kWWDT_TimeoutFlag)
    {
        log_warning("device reset watch dog.\r\n");
    } else {
        log_warning("device reset normal.\r\n");
    }

    /*
     * Set watchdog feed time constant to approximately 2s
     * Set watchdog window time to 1s
     */
    config.timeoutValue = wdtFreq * 2;
    config.warningValue = 0;
    config.windowValue = wdtFreq * 1;
    /* Configure WWDT to reset on timeout */
    config.enableWatchdogReset = true;
    /* Setup watchdog clock frequency(Hz). */
    config.clockFreq_Hz = WDT_CLK_FREQ;

    //WWDT_Init(WWDT, &config);

    while (1)
    {

        osDelay(WATCH_DOG_TASK_INTERVAL);   
        /*
        if (WWDT->TV < WWDT->WINDOW) {
            WWDT_Refresh(WWDT);
            log_debug("feed dog.\r\n");
        }
        */
    }
}  
  