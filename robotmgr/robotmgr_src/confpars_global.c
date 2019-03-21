/////////////////////////////////////////////////////////////////////////////
//
//  confpars_global.c: Global Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////

#include "robotmgr_main.h"

int g_nReqVersion           = -1;
int g_nTerminalInetPort     = -1;

char* g_pszWorkDir               = NULL;
char* g_pszJobDir                = NULL;
char* g_pszSensDir               = NULL;
char* g_pszWireCutJobFileName    = NULL;
char* g_pszNewCreatedJobFileName = NULL;
char* g_pszEcatConfigDir         = NULL;
char* g_pszConfigDir             = NULL;
char* g_pszLoadedJobFileName     = NULL;

char* g_pszTerminalGreeting = NULL;
char* g_pszLocale           = NULL;
char* g_pszPrompt           = NULL;

int g_nQNXTimerTick           = 0;   // [ns] unit, QNX Timer Tick
int g_nQNXTimerRes            = 0;   // [us] unit, QNX Timer Resolution
int g_nIoTime                 = 0;   // [ms] unit, Io Sample Time
int g_nTrajUpdateTime         = 0;   // [ms] unit, Trajectory Gneration Time
int g_nServoInterpolationTime = 0;   // [ms] unit, Servo Interpolation Time

int g_nServoOnBrakeDelayTime = 0;    // [ms] unit, defailt = 70 ms
int g_nServoOffBrakeDelayTime = 0;   // [ms] unit, defailt = 150 ms

int g_nEstopGasOffDelayTime        = 0;  // [ms] unit, defailt = 3000 ms
int g_nEstopTouchReadyOffDelayTime = 0;  // [ms] unit, defailt = 500 ms

int g_rgnEcatMasterResetHours[MAX_MASTER_RESET_CNT_DAY] = {0};
int g_nEcatMasterResetHoursCount = 0;
int g_nEcatMasterResetMin = -1;

int g_nSlaveCount           = 0;     // Slave Count
int g_nWriteOffsetSize      = 0;     // BeckHoff output size
int g_nReadOffsetSize       = 0;     // BeckHoff input size

int g_nErrHistorySaveMaxCnt = 0;     // Error History Save Max Count

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigGlobal()
//

int SYSC_LoadConfigGlobal(const char* pszKey, const char* pszValue)
{
    int nType, nVal;
    double rgdb[32];
    int rgn[32], nCount, i;

    if (stricmp(pszValue, "disable") == 0)
    {
        nType = CONFIG_NUM_INT;
        nVal = -1;
    }
    else
    {
        nType = PARAM_ConvNum(pszValue, &nVal, NULL);
    }

    ///////////////////////////////////////////////////////
    // VERSION = 12.34
    if (stricmp(pszKey, SYSCONF_KEY_VERSION) == 0)
    {
        if (g_nReqVersion != -1)
        {
            VERBOSE_ERROR("Req Version is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nVal = ConvVersion(pszValue);

        if (nVal == -1)
        {
            VERBOSE_ERROR("Invalid version : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nReqVersion = nVal;
    }
    ///////////////////////////////////////////////////////
    // terminal inet port : TERMINAL_INET_PORT = 2005
    else if (stricmp(pszKey, SYSCONF_KEY_TERMINAL_INET_PORT) == 0)
    {
        if (g_nTerminalInetPort != -1)
        {
            VERBOSE_ERROR("Inet Terminal Port is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszValue, "disable") == 0)
        {
            nVal = -2;
        }
        else
        {
            nType = PARAM_ConvNum(pszValue, &nVal, NULL);

            if (nType != CONFIG_NUM_INT || nVal < 0)
            {
                VERBOSE_ERROR("Invalid Inet Terminal Port : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
                return RESULT_ERROR;
            }
        }

        g_nTerminalInetPort = nVal;
    }
    ///////////////////////////////////////////////////////
    // WORK_DIR = /dsme
    else if (stricmp(pszKey, SYSCONF_KEY_WORK_DIR) == 0)
    {
        if (g_pszWorkDir != NULL)
        {
            VERBOSE_ERROR("Work Directory is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszWorkDir = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszWorkDir == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Work Dir\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }
#if defined(__QNXNTO__)
        CRT_strcpy(g_pszWorkDir, strlen(pszValue)+1, pszValue);
#else
        CRT_strcpy(g_pszWorkDir, strlen(pszValue)+1, DEF_WIN_WORK_DIR_NAME);
#endif
    }
    ///////////////////////////////////////////////////////
    // JOB_DIR = /dsme/job
    else if (stricmp(pszKey, SYSCONF_KEY_JOB_DIR) == 0)
    {
        if (g_pszJobDir != NULL)
        {
            VERBOSE_ERROR("Job Directory is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszJobDir = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszJobDir == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Job Dir\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }
#if defined(__QNXNTO__)
        CRT_strcpy(g_pszJobDir, strlen(pszValue)+1, pszValue);
#else
        CRT_strcpy(g_pszJobDir, strlen(pszValue)+1, DEF_WIN_JOB_DIR_NAME);
#endif
    }
    ///////////////////////////////////////////////////////
    // SENS_DIR = /dsme/sdata
    else if (stricmp(pszKey, SYSCONF_KEY_SENS_DIR) == 0)
    {
        if (g_pszSensDir != NULL)
        {
            VERBOSE_ERROR("Sensor Directory is already specified : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszSensDir = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszSensDir == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Sensor Dir\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }
#if defined(__QNXNTO__)
        CRT_strcpy(g_pszSensDir, strlen(pszValue)+1, pszValue);
#else
        CRT_strcpy(g_pszSensDir, strlen(pszValue)+1, DEF_WIN_SENS_DIR_NAME);
#endif
    }
    ///////////////////////////////////////////////////////
    // WIRECUT_JOBNAME = wirecut
    else if (stricmp(pszKey, SYSCONF_KEY_WIRECUT_JOB_NAME) == 0)
    {
        if (g_pszWireCutJobFileName != NULL)
        {
            VERBOSE_ERROR("WireCut Job Name is already specified : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszWireCutJobFileName = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszWireCutJobFileName == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for WireCut Job Name\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszWireCutJobFileName, strlen(pszValue)+1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // NEW_JOBNAME = blank.pgm
    else if (stricmp(pszKey, SYSCONF_KEY_NEW_CREATED_JOB_NAME) == 0)
    {
        if (g_pszNewCreatedJobFileName != NULL)
        {
            VERBOSE_ERROR("Newly Created Name is already specified : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszNewCreatedJobFileName = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszNewCreatedJobFileName == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Newly Created Job Name\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszNewCreatedJobFileName, strlen(pszValue)+1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // TERMINAL_GREETING = Hello
    else if (stricmp(pszKey, SYSCONF_KEY_TERMINAL_GREETING) == 0)
    {
        if (g_pszTerminalGreeting != NULL)
        {
            VERBOSE_ERROR("Terminal Greeting is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszTerminalGreeting = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszTerminalGreeting == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Terminal Greeting\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszTerminalGreeting, strlen(pszValue)+1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // LOCALE =             # keep current locale
    // LOCALE = default     # default locale
    else if (stricmp(pszKey, SYSCONF_KEY_LOCALE) == 0)
    {
        if (g_pszLocale != NULL)
        {
            VERBOSE_ERROR("Locale is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        if (pszValue == NULL || *pszValue == 0)
        {
            g_pszLocale = NULL;
        }
        else
        {
            if (strcmp(pszValue, "default") == 0)
                pszValue = "";

            g_pszLocale = (char*) DEBUG_MALLOC(strlen(pszValue) + 1);

            if (g_pszLocale == NULL)
            {
                VERBOSE_ERROR("Memory allocation failed for Locale\n");
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
                return RESULT_ERROR;
            }

            CRT_strcpy(g_pszLocale, strlen(pszValue) + 1, pszValue);
        }
    }
    ///////////////////////////////////////////////////////
    // PROMPT = $P$G
    else if (stricmp(pszKey, SYSCONF_KEY_PROMPT) == 0)
    {
        if (g_pszPrompt != NULL)
        {
            VERBOSE_ERROR("Prompt is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszPrompt = (char*) DEBUG_MALLOC(strlen(pszValue) + 1);

        if (g_pszPrompt == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Prompt\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszPrompt, strlen(pszValue) + 1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // ECAT_CONF_DIR = /root/ethercat/master_config_io_6.xml
    else if (stricmp(pszKey, SYSCONF_KEY_ECAT_CONFIG_DIR) == 0)
    {
        if (g_pszEcatConfigDir != NULL)
        {
            VERBOSE_ERROR("EtherCAT Directory is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_pszEcatConfigDir = (char *)DEBUG_MALLOC(strlen(pszValue)+1);

        if (g_pszEcatConfigDir == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for EtherCAT Dir\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszEcatConfigDir, strlen(pszValue)+1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // SLAVE_CNT = 17
    else if (stricmp(pszKey, SYSCONF_KEY_SLAVE_COUNT) == 0)
    {
        if (g_nSlaveCount != 0)
        {
            VERBOSE_ERROR("Slave Count is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 1 ~ 100 [EA]
        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 100)
        {
            VERBOSE_ERROR("Invalid Slave Count : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nSlaveCount = nVal;
    }
    ///////////////////////////////////////////////////////
    // ERRHIST_SAVE_CNT = 100
    else if (stricmp(pszKey, SYSCONF_KEY_ERRHITORY_SAVE_COUNT) == 0)
    {
        if (g_nErrHistorySaveMaxCnt != 0)
        {
            VERBOSE_ERROR("Error History Save Max Count is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > MAX_ERROR_STACK_SIZE)
        {
            VERBOSE_ERROR("Invalid Error History Save Max Count : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nErrHistorySaveMaxCnt = nVal;
    }
    ///////////////////////////////////////////////////////
    // WRITE_OFFSET_SIZE = 0x28
    else if (stricmp(pszKey, SYSCONF_KEY_WRITE_OFFSET_SIZE) == 0)
    {
        if (g_nWriteOffsetSize != 0)
        {
            VERBOSE_ERROR("Write Offset Size is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 255 (0x00~0xff)
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 255)
        {
            VERBOSE_ERROR("Invalid Write Offset Size : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nWriteOffsetSize = nVal;
    }
    ///////////////////////////////////////////////////////
    // READ_OFFSET_SIZE = 0x38
    else if (stricmp(pszKey, SYSCONF_KEY_READ_OFFSET_SIZE) == 0)
    {
        if (g_nReadOffsetSize != 0)
        {
            VERBOSE_ERROR("Read Offset Size is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 255 (0x00~0xff)
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 255)
        {
            VERBOSE_ERROR("Invalid Read Offset Size : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nReadOffsetSize = nVal;
    }
    ///////////////////////////////////////////////////////
    // TIMER_TICK = 1
    else if (stricmp(pszKey, SYSCONF_KEY_QNXTIMER_TICK) == 0)
    {
        if (g_nQNXTimerTick != 0)
        {
            VERBOSE_ERROR("QNX Timer Tick is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 1 ~ 1000 [ns]
        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 1000000000)
        {
            VERBOSE_ERROR("Invalid QNX Timer Tick : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nQNXTimerTick = nVal;
    }
    ///////////////////////////////////////////////////////
    // TIMER_RES = 1
    else if (stricmp(pszKey, SYSCONF_KEY_QNXTIMER_RES) == 0)
    {
        if (g_nQNXTimerRes != 0)
        {
            VERBOSE_ERROR("QNX Timer Resolution is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 1 ~ 1000 [us]
        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid QNX Timer Resolution : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nQNXTimerRes = nVal;
    }
    ///////////////////////////////////////////////////////
    // IO_TIME = 5
    else if (stricmp(pszKey, SYSCONF_KEY_IO_TIME) == 0)
    {
        if (g_nIoTime != 0)
        {
            VERBOSE_ERROR("IO Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 1 ~ 1000 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid I/O Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nIoTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // TRAJ_TIME = 1
    else if (stricmp(pszKey, SYSCONF_KEY_TRAJ_UPDATE_TIME) == 0)
    {
        if (g_nTrajUpdateTime != 0)
        {
            VERBOSE_ERROR("Trajectory Update Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 1 ~ 1,000 [ms]
        if (nType != CONFIG_NUM_INT || nVal <= 0 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid Trajectory Update Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nTrajUpdateTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // TRAJ_BUFFER = 2
    else if (stricmp(pszKey, SYSCONF_KEY_SRV_INTPOLATION_TIME) == 0)
    {
        if (g_nServoInterpolationTime != 0)
        {
            VERBOSE_ERROR("Servo Interpolation Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 250 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > MAX_INTERPOLATION_TIME)
        {
            VERBOSE_ERROR("Servo Interpolation Time Count : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nServoInterpolationTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // SERVOON_DELAY  = 70
    else if (stricmp(pszKey, SYSCONF_KEY_SERVOON_BRAKE_DELAY)  == 0)
    {
        if (g_nServoOnBrakeDelayTime != 0)
        {
            VERBOSE_ERROR("Servo On Brake Delay Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 1000 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid Servo On Brake Delay Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nServoOnBrakeDelayTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // SERVOOFF_DELAY = 150
    else if (stricmp(pszKey, SYSCONF_KEY_SERVOOFF_BRAKE_DELAY)  == 0)
    {
        if (g_nServoOffBrakeDelayTime != 0)
        {
            VERBOSE_ERROR("Servo Off Brake Delay Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 1000 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid Servo Off Brake Delay Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nServoOffBrakeDelayTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // ESTOP_GASOFF_DELAY = 3000
    else if (stricmp(pszKey, SYSCONF_KEY_ESTOP_GASOFF_DELAY)  == 0)
    {
        if (g_nEstopGasOffDelayTime != 0)
        {
            VERBOSE_ERROR("Estop Gas Off Delay Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 10000 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 10000)
        {
            VERBOSE_ERROR("Invalid Estop Gas Off Delay Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nEstopGasOffDelayTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // ESTOP_TOUCHREADYOFF_DELAY = 500
    else if (stricmp(pszKey, SYSCONF_KEY_ESTOP_TOUCHREADYOFF_DELAY)  == 0)
    {
        if (g_nEstopTouchReadyOffDelayTime != 0)
        {
            VERBOSE_ERROR("Estop TouchReady Off Delay Time is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 10000 [ms]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 10000)
        {
            VERBOSE_ERROR("Invalid Estop TouchReady Off Delay Time : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nEstopTouchReadyOffDelayTime = nVal;
    }
    ///////////////////////////////////////////////////////
    // MASTER_RESET_HOURS = 0, 1, 2
    else if (stricmp(pszKey, SYSCONF_KEY_MASTER_RESET_HOURS) == 0)
    {
        nCount = PARAM_ConvArrayNumInt(pszValue, rgn, MAX_MASTER_RESET_CNT_DAY);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid master restart hour was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }
        
        if (nCount > MAX_MASTER_RESET_CNT_DAY)
        {
            VERBOSE_ERROR("Too many master restart hour were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        for (i = 0; i < nCount; i++)
        {
            if (rgn[i] < 0 || rgn[i] > 23)
            {
                VERBOSE_ERROR("Invalid master restart hour was specified : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
                return RESULT_ERROR;
            }

            g_rgnEcatMasterResetHours[i] = rgn[i];
        }

        g_nEcatMasterResetHoursCount = nCount;
    }
    ///////////////////////////////////////////////////////
    // MASTER_RESET_MIN = 20
    else if (stricmp(pszKey, SYSCONF_KEY_MASTER_RESET_MIN)  == 0)
    {
        if (g_nEcatMasterResetMin >= 0 && g_nEcatMasterResetMin <= 59)
        {
            VERBOSE_ERROR("master restart min is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        nType = PARAM_ConvNum(pszValue, &nVal, NULL);

        // 0 ~ 59 [min]
        if (nType != CONFIG_NUM_INT || nVal < 0 || nVal > 59)
        {
            VERBOSE_ERROR("Invalid master restart min : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        g_nEcatMasterResetMin = nVal;
    }
    ///////////////////////////////////////////////////////
    // AIN_MAX_VOLT = 10.0
    // ADC_MAX_BIT = 32767
    else if (stricmp(pszKey, SYSCONF_KEY_ANALOG_IN_MAX_VOLT) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_AD_CONVERT_MAX_BIT) == 0)
    {
        // 1 means no of tuning input parameters
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 1);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Analog Input Parameter was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many Analog Input Parameter were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
            return RESULT_ERROR;
        }
        
        if (stricmp(pszKey, SYSCONF_KEY_ANALOG_IN_MAX_VOLT) == 0)
        {
            g_dbAinMaxVolt = rgdb[0];
        }
        if (stricmp(pszKey, SYSCONF_KEY_AD_CONVERT_MAX_BIT) == 0)
        {
            g_dbADCMaxBit = rgdb[0];
        }
    }
    ///////////////////////////////////////////////////////
    // WELDMAP = xxxx
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_WELDMAP) == 0)
    {
//        SVCWD_SetWeldMapFileName(pszValue);
    }
    ///////////////////////////////////////////////////////
    // ????????????
    else
    {
        VERBOSE_ERROR("unknown global configuration specified : %s\n", pszKey);
        SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_GLOBAL_PARAM);
        return RESULT_ERROR;
    }

    return 0;
}
