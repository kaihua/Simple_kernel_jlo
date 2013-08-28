/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved. */

#ifdef IMAGE_MODEM_PROC
    extern void adc_ipc_msg(const char *str, ...);
    extern void adc_ipc_init(void);
    #define ADC_MSG(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_HIGH(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_MED(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_LOW(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_HIGH(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_ERROR(...) adc_ipc_msg(__VA_ARGS__)
    #define MSG_FATAL(...) adc_ipc_msg(__VA_ARGS__)
#endif

#define ADC_LOG_BUFFER_SIZE 80
#define ADC_LOG_BUFFER_RECORDS 50

typedef struct  {
   unsigned int msg_index;
   char         msg[ADC_LOG_BUFFER_RECORDS][ADC_LOG_BUFFER_SIZE];
} adc_ipc_smem;

