/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved. */

/* FIH-SW3-KERNEL-VH-CHARGING-15*[ */
typedef enum
{
    CHG_IPC_PARAMETER_GAUGE_I,
    CHG_IPC_PARAMETER_GAUGE_CAPACITY,
    CHG_IPC_PARAMETER_GAUGE_TEMPERATURE,
    CHG_IPC_PARAMETER_GAUGE_VOLTAGE,
    CHG_IPC_PARAMETER_FLAG,
    CHG_IPC_PARAMETER_BATTERY_TYPE,
    CHG_IPC_PARAMETER_OTP_STATUS,
    CHG_IPC_NUM_PARAMETERS,
}CHG_IPC_PARAMATER_TYPE;

typedef enum
{
    CHG_IPC_OTP_STATUS_UNKNOWN,
    CHG_IPC_OTP_STATUS_NORMAL,
    CHG_IPC_OTP_STATUS_HIGH_TEMPERATURE,
    CHG_IPC_OTP_STATUS_CRITICAL_TEMPERATURE,
}CHG_IPC_OTP_STATUS_TYPE;

typedef enum
{
    CHG_IPC_BATTERY_TYPE_SONY,
    CHG_IPC_BATTERY_TYPE_ALIEN,  
}CHG_IPC_BATTERY_TYPE;

#ifdef IMAGE_MODEM_PROC
    extern void chg_ipc_msg(const char *str, ...);
    extern int chg_ipc_set_parameter(CHG_IPC_PARAMATER_TYPE id, int value);
    extern int chg_ipc_get_parameter(CHG_IPC_PARAMATER_TYPE id);
    extern void chg_ipc_init(void);
    #define CHG_MSG(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_HIGH(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_MED(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_LOW(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_HIGH(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_ERROR(...) chg_ipc_msg(__VA_ARGS__)
    #define MSG_FATAL(...) chg_ipc_msg(__VA_ARGS__)
    #define CHG_GET(X) chg_ipc_get_parameter(X)
    #define CHG_SET(X, Y) chg_ipc_set_parameter(X, Y)
#endif

#define CHG_LOG_BUFFER_SIZE 80
#define CHG_LOG_BUFFER_RECORDS 50

typedef struct  {
   unsigned int msg_index;
   char         msg[CHG_LOG_BUFFER_RECORDS][CHG_LOG_BUFFER_SIZE];
   int          parameters[CHG_IPC_NUM_PARAMETERS];
} chg_ipc_smem;
/* FIH-SW3-KERNEL-VH-CHARGING-15*] */

