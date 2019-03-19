#include "cmsis_os.h"
#include "tasks_init.h"
#include "adc_task.h"
#include "temperature_task.h"
#include "board.h"
#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "log.h"


osThreadId   adc_task_hdl;

static volatile uint16_t adc_sample[2];
static volatile uint32_t adc_cusum[2];
static volatile uint16_t adc_average[2];

#define   TEMPERATURE_ADC           ADC0
#define   TEMPERATURE_ADC_IRQ_ID    ADC0_SEQA_IRQn
#define   TEMPERATURE_ADC_CHANNEL   0
#define   TEMPERATURE_ADC_CLK_SRC   kFRO_HF_to_ADC_CLK


static adc_result_info_t gAdcResultInfoStruct;
adc_result_info_t *volatile gAdcResultInfoPtr = &gAdcResultInfoStruct;

void  ADC0_SEQA_IRQHandler()
{
    if (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(TEMPERATURE_ADC)) {
        ADC_GetChannelConversionResult(TEMPERATURE_ADC, TEMPERATURE_ADC_CHANNEL, gAdcResultInfoPtr);
        adc_sample[ADC_TASK_TEMPERATURE_IDX] = gAdcResultInfoPtr->result;
        ADC_ClearStatusFlags(TEMPERATURE_ADC, kADC_ConvSeqAInterruptFlag);
        osSignalSet(adc_task_hdl,ADC_TASK_ADC_COMPLETED_SIGNAL);   
    }
 
}

static int adc_clk_pwr_config()
{
    /* SYSCON power. */
    POWER_DisablePD(kPDRUNCFG_PD_VDDA);    /* Power on VDDA. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);    /* Power on the ADC converter. */
    POWER_DisablePD(kPDRUNCFG_PD_VD2_ANA); /* Power on the analog power supply. */
    POWER_DisablePD(kPDRUNCFG_PD_VREFP);   /* Power on the reference voltage source. */

    /* Enable the clock. */
    CLOCK_AttachClk(TEMPERATURE_ADC_CLK_SRC);
    CLOCK_EnableClock(kCLOCK_Adc0); 
    return 0;
}

static int adc_converter_init()
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

    /* Configure the converter. */
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
    adcConfigStruct.clockDividerNumber = 1;                /* The divider for sync clock is 2. */
    adcConfigStruct.resolution = kADC_Resolution12bit;
    adcConfigStruct.enableBypassCalibration = false;
    adcConfigStruct.sampleTimeNumber = 0U;

    ADC_Init(TEMPERATURE_ADC, &adcConfigStruct);

    /* Enable channel TEMPERATURE_ADC_CHANNEL's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask =(1U << TEMPERATURE_ADC_CHANNEL); /* Includes channel TEMPERATURE_ADC_CHANNEL. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(TEMPERATURE_ADC, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(TEMPERATURE_ADC, true); /* Enable the conversion sequence A. */
    /* Clear the result register. */
    /*
    ADC_DoSoftwareTriggerConvSeqA(TEMPERATURE_ADC);
    while (!ADC_GetChannelConversionResult(TEMPERATURE_ADC, TEMPERATURE_ADC_CHANNEL, &gAdcResultInfoStruct))
    {
    }
    ADC_GetConvSeqAGlobalConversionResult(TEMPERATURE_ADC, &gAdcResultInfoStruct);
    */
    NVIC_SetPriority(TEMPERATURE_ADC_IRQ_ID, 3);
    /* Enable the interrupt. */
    /* Enable the interrupt the for sequence A done. */
    ADC_EnableInterrupts(TEMPERATURE_ADC, kADC_ConvSeqAInterruptEnable);
    EnableIRQ(TEMPERATURE_ADC_IRQ_ID);
  
    return 0;
}

static int adc_start()
{
    ADC_DoSoftwareTriggerConvSeqA(TEMPERATURE_ADC); 
    return 0;
}

static int adc_stop()
{
    return 0;
}


static void adc_calibration()
{
    if (ADC_DoSelfCalibration(TEMPERATURE_ADC) == false) {
        log_error("t adc calibrate err.\r\n");
    }
}

static void adc_reset()
{
    adc_clk_pwr_config();
    adc_calibration();
    adc_converter_init();
}


void adc_task(void const * argument)
{
    int rc;
    osEvent signals;
    osStatus status;
    temperature_task_message_t adc_completed_msg;
    uint8_t t_sample_cnt = 0;

    adc_stop();
    adc_reset();

    while (1) {
        osDelay(ADC_TASK_INTERVAL);
        rc = adc_start();
        if (rc != 0) {
            adc_reset();
            continue;
        } 
  
        signals = osSignalWait(ADC_TASK_ALL_SIGNALS,ADC_TASK_ADC_TIMEOUT);
        if (signals.status == osEventSignal ) {
            if (signals.value.signals & ADC_TASK_ADC_COMPLETED_SIGNAL) {
    
                /*temperature adc calculate*/    
                if (t_sample_cnt < ADC_TASK_ADC_SAMPLE_MAX){
                    adc_cusum[ADC_TASK_TEMPERATURE_IDX] += adc_sample[ADC_TASK_TEMPERATURE_IDX];   
                    t_sample_cnt ++;             
                } else {
                    adc_average[ADC_TASK_TEMPERATURE_IDX] = adc_cusum[ADC_TASK_TEMPERATURE_IDX] / t_sample_cnt;  
                    adc_cusum[ADC_TASK_TEMPERATURE_IDX] = 0;
                    t_sample_cnt = 0;
                    adc_completed_msg.request.type = TEMPERATURE_TASK_MSG_TYPE_ADC_COMPLETED;
                    adc_completed_msg.request.adc = adc_average[ADC_TASK_TEMPERATURE_IDX];  
                    status = osMessagePut(temperature_task_msg_q_id,(uint32_t)&adc_completed_msg,ADC_TASK_PUT_MSG_TIMEOUT);
                    if (status != osOK) {
                        log_error("put temperature msg error:%d\r\n",status);
                    }
                }
            }
    

        if (signals.value.signals & ADC_TASK_ADC_ERROR_SIGNAL) {
            log_error("adc error.reset.\r\n");
            adc_reset();
        }
    }
    }
}

