#define _USE_MATH_DEFINES

#include "servocon_main.h"
#include <math.h>
#if defined (__QNXNTO__)
#include "ecattypes.h"
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Variables
//

int g_hShm_sc           = DANDY_INVALID_SHMEM_HANDLE;
int g_hShm_SysStatus_rm = DANDY_INVALID_SHMEM_HANDLE;
int g_hShm_SysConfig_rm = DANDY_INVALID_SHMEM_HANDLE;
int g_hShm_SysParam_rm  = DANDY_INVALID_SHMEM_HANDLE;
int g_hShm_Status_te    = DANDY_INVALID_SHMEM_HANDLE;
int g_hShm_Task_te      = DANDY_INVALID_SHMEM_HANDLE;

// shared memory
SHM_SC_SYSTEM*              g_pShmem_sc = NULL;
SHM_RM_SYSSTATUS*           g_pShmem_SysStatus_rm = NULL;
SHM_RM_SYSCONFIG*           g_pShmem_SysConfig_rm = NULL;
SHM_RM_SYSPARAM*            g_pShmem_SysParam_rm = NULL;
SHM_TE_STATUS*              g_pShmem_Status_te = NULL;
volatile shm_task_servo_t*  g_pShmem_Task_te = NULL;

// file config

// control config
int g_nQNXTimerTick;
int g_nQNXTimerRes;
int g_nTrajUpdateTime;
int g_nIoScanTime;
int g_nServoOnBrakeDelayTime;
int g_nServoOffBrakeDelayTime;
int g_nEstopGasOffDelayTime        = 0;  // [ms] unit, defailt = 3000 ms
int g_nEstopTouchReadyOffDelayTime = 0;  // [ms] unit, defailt = 500 ms
double  g_dbAinMaxVolt;
double  g_dbADCMaxBit;
char*   g_pszSensDir = NULL;

double g_rgdbHwLimit[ROB_AXIS_COUNT][2];    // HW limit position
int g_rgfHwLimitUsed[ROB_AXIS_COUNT][2];    // HW limit used?

int g_rgnEncRes[MAX_MOTOR_COUNT];
double g_rgdbMotorEStopDec[MAX_MOTOR_COUNT];
int g_rgnEcnoderHomeVal[MAX_MOTOR_COUNT];
double g_rgdbGearRatio[MAX_MOTOR_COUNT];
int g_rgnAxisDirection[MAX_MOTOR_COUNT];

double g_dbRobotDecel_Estop;
double g_dbRobotDecel_NormalStop;

int g_nWeldTuneInParamApplyIndex  = 0;
int g_nWeldTuneOutParamApplyIndex = 0;

DIN_PORTNO       g_Din_portno;
DOUT_PORTNO      g_Dout_portno;
AIN_PORTNO       g_Ain_portno;
AOUT_PORTNO      g_Aout_portno;

//CONFIG_ECAT         g_nECATParam;

////////////////////////////////////////////////////////////////////////////////
//
// Functions
//

int SHM_RM_SysStatusOpenShmem(void);        // open RM_SysStatusSHM
int SHM_RM_SysConfigOpenShmem(void);        // open RM_SysConfigSHM
int SHM_RM_SysParamOpenShmem(void);         // open RM_SysConfigSHM
int SHM_RM_SysStatusDestroyShmem(void);     // destroy RM_SysStatusSHM
int SHM_RM_SysConfigDestroyShmem(void);     // open RM_SysConfigSHM
int SHM_RM_SysParamDestroyShmem(void);      // open RM_SysConfigSHM
int SHM_Status_TE_OpenShmem(void);          // open TE_SHM 
int SHM_Status_TE_DestroyShmem(void);       // destroy TE_SHM
int SHM_Task_TE_OpenShmem(void);            // open TE_SHM 
int SHM_Task_TE_DestroyShmem(void);         // destroy TE_S
int SHM_SC_CreateShmem(void);               // create SC_SHM
void SHM_SC_DestroyShmem(void);             // destroy SC_SHM

int SHM_LoadSysConfigParam(int nRobotIndex);

//////////////////////////////////////////////////////////////////////////
//
// SHM_SC_CreateShmem()
//
int SHM_SC_CreateShmem(void)
{
    // create SC_SHM
    if (g_pShmem_sc != NULL)
    {
        VERBOSE_WARNING("SC shared memory already created.\n");
        return 0;
    }

    g_hShm_sc = SHM_Create(SC_SHM_NAME, sizeof(SHM_SC_SYSTEM));
    
    if (g_hShm_sc == DANDY_INVALID_SHMEM_HANDLE)
    {  
    	g_fOpenShmSc = FALSE;   // flag of SC_SHM creation
        VERBOSE_ERROR("Cannot create the SC shared memory.\n");
        return -1;
    }

    // map the shared memory handle
    //g_pShmem_sc = (volatile SHM_SC_SYSTEM*) SHM_Map(g_hShm_sc, sizeof(SHM_SC_SYSTEM));
    g_pShmem_sc = (SHM_SC_SYSTEM*) SHM_Map(g_hShm_sc, sizeof(SHM_SC_SYSTEM));

    if (g_pShmem_sc == NULL)
    {
        g_fOpenShmSc = FALSE;   // flag of SC_SHM creation
        VERBOSE_ERROR("Cannot create the SC shared memory.\n");
        return -1;
    }

    memset((void*) g_pShmem_sc, 0, sizeof(SHM_SC_SYSTEM));  // memory init.
    g_pShmem_sc->nsize = sizeof(SHM_SC_SYSTEM);             // memory size

    g_fOpenShmSc = TRUE;   // flag of SC_SHM creation

    VERBOSE_VERBOSE("Created SC shared memory : size<%d bytes>\n", 
                    g_pShmem_sc->nsize);

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysStatusOpenShmem()
//
int SHM_RM_SysStatusOpenShmem(void)
{
    // RM shared memory open
    if (g_pShmem_SysStatus_rm != NULL)
    {
        VERBOSE_WARNING("RM SysStatus shared memory already opened.\n");
        return 0;
    }

    g_hShm_SysStatus_rm = SHM_Open(SHM_RM_SYSSTATUS_NAME);

    if (g_hShm_SysStatus_rm == -1)
    {
        VERBOSE_ERROR("Failed to open RM SysStatus shared memory.!!!\n");
        return -1;
    }
    else
    {
        VERBOSE_VERBOSE("Opened the RM SysStatus shared memory. Name(%s)\n", 
                        SHM_RM_SYSSTATUS_NAME);
    }

    // map
    g_pShmem_SysStatus_rm = (SHM_RM_SYSSTATUS*) SHM_Map(g_hShm_SysStatus_rm, 
                                                        sizeof(SHM_RM_SYSSTATUS));

    if (g_pShmem_SysStatus_rm == NULL)
    {
        VERBOSE_ERROR("Cannot map the RM SysStatus shared memory.!!!\n");
        return -1;
    }
    else
    {
        if (sizeof(SHM_RM_SYSSTATUS) == g_pShmem_SysStatus_rm->nSize)
        {
            g_fOpenSysStatusShmRm = TRUE;  // flag of open RM_SHM
            
            VERBOSE_VERBOSE("Mapped the RM SysStatus shared memory. Size(%dbytes), "
                            "Chk(%dbytes)\n", 
                            sizeof(SHM_RM_SYSSTATUS),
                            g_pShmem_SysStatus_rm->nSize);
        }
        else
        {
            g_fOpenSysStatusShmRm = FALSE;  // flag of open RM_SHM
            VERBOSE_ERROR("Mismatched the RM SysStatus shared memory. Size(%dbytes), "
                          "Chk(%dbytes)\n",
                          sizeof(SHM_RM_SYSSTATUS),
                          g_pShmem_SysStatus_rm->nSize);

            return -1;
        }
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysConfigOpenShmem()
//
int SHM_RM_SysConfigOpenShmem(void)
{
    // RM shared memory open
    if (g_pShmem_SysConfig_rm != NULL)
    {
        VERBOSE_WARNING("RM SysConfig shared memory already opened.\n");
        return 0;
    }

    g_hShm_SysConfig_rm = SHM_Open(SHM_RM_SYSCONFIG_NAME);

    if (g_hShm_SysConfig_rm == -1)
    {
        VERBOSE_ERROR("Failed to open RM SysConfig shared memory.!!!\n");
        return -1;
    }
    else
    {
        VERBOSE_VERBOSE("Opened the RM SysConfig shared memory. Name(%s)\n", 
                        SHM_RM_SYSCONFIG_NAME);
    }

    // map
    g_pShmem_SysConfig_rm = (SHM_RM_SYSCONFIG*) SHM_Map(g_hShm_SysConfig_rm, 
                                                        sizeof(SHM_RM_SYSCONFIG));

    if (g_pShmem_SysConfig_rm == NULL)
    {
        VERBOSE_ERROR("Cannot map the RM SysConfig shared memory.!!!\n");
        return -1;
    }
    else
    {
        if (sizeof(SHM_RM_SYSCONFIG) == g_pShmem_SysConfig_rm->dwLength)
        {
            g_fOpenSysConfigShmRm = TRUE;  // flag of open RM_SHM

            VERBOSE_VERBOSE("Mapped the RM SysConfig shared memory. Size(%d bytes), "
                            "Chk(%ld bytes)\n",
                            sizeof(SHM_RM_SYSCONFIG),
                            g_pShmem_SysConfig_rm->dwLength);
        }
        else
        {
            g_fOpenSysConfigShmRm = FALSE;  // flag of open RM_SHM

            VERBOSE_ERROR("Mismatched the RM SysConfig shared memory. Size(%d bytes), "
                          "Chk(%ld bytes)\n",
                          sizeof(SHM_RM_SYSCONFIG),
                          g_pShmem_SysConfig_rm->dwLength);

            return -1;
        }
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysParamOpenShmem()
//
int SHM_RM_SysParamOpenShmem(void)
{
    // RM shared memory open
    if (g_pShmem_SysParam_rm != NULL)
    {
        VERBOSE_WARNING("RM SysParam shared memory already opened.\n");
        return 0;
    }

    g_hShm_SysParam_rm = SHM_Open(SHM_RM_SYSPARAM_NAME);

    if (g_hShm_SysParam_rm == -1)
    {
        VERBOSE_ERROR("Failed to open RM SysParam shared memory.!!!\n");
        return -1;
    }
    else
    {
        VERBOSE_VERBOSE("Opened the RM SysParam shared memory. Name(%s)\n", 
                        SHM_RM_SYSPARAM_NAME);
    }

    // map
    g_pShmem_SysParam_rm = (SHM_RM_SYSPARAM*) SHM_Map(g_hShm_SysParam_rm, 
                                                      sizeof(SHM_RM_SYSPARAM));

    if (g_pShmem_SysParam_rm == NULL)
    {
        VERBOSE_ERROR("Cannot map the RM SysParam shared memory.!!!\n");
        return -1;
    }
    else
    {
        if (sizeof(SHM_RM_SYSPARAM) == g_pShmem_SysParam_rm->nSize)
        {
            g_fOpenSysParamShmRm = TRUE;   // flag of open RM_SHM

            VERBOSE_VERBOSE("Mapped the RM SysParam shared memory. Size(%d bytes), "
                            "Chk(%d bytes)\n",
                            sizeof(SHM_RM_SYSPARAM),
                            g_pShmem_SysParam_rm->nSize);
        }
        else
        {
            g_fOpenSysParamShmRm = FALSE;  // flag of open RM_SHM

            VERBOSE_ERROR("Mismatched the RM SysParam shared memory. Size(%d bytes), "
                          "Chk(%d bytes)\n",
                          sizeof(SHM_RM_SYSPARAM),
                          g_pShmem_SysParam_rm->nSize);

            return -1;
        }
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// PARAM_LoadDefaultParam()
//
void PARAM_LoadDefaultParam(void)
{
    int iAxis, iMotor;

    /* Global */
    g_pszEcatConfigDir     = DEF_ECAT_CONFIG_FILE_NAME;  // ECAT Config Dir
    g_nAxisCount           = DEF_AXIS_COUNT;             // Axis Count
    g_nSlaveCount          = DEF_SLAVE_COUNT;            // Slave Count
    g_nWriteInitOffsetSize = DEF_WRITE_INIT_OFFSET_SIZE; // BeckHoff output size
    g_nReadInitOffsetSize  = DEF_READ_INIT_OFFSET_SIZE;  // BeckHoff input size

    g_nQNXTimerTick        = DEF_TIMER_TICK;
    g_nQNXTimerRes         = DEF_TIMER_RES;
    g_nTrajUpdateTime      = DEF_TRAJ_UPDATE_TIME;
    g_nIoScanTime          = DEF_IO_SCAN_TIME;

    g_nServoOnBrakeDelayTime  = DEF_SERVOON_DELAY;
    g_nServoOffBrakeDelayTime = DEF_SERVOOFF_DELAY;

    g_nEstopGasOffDelayTime        = DEF_ESTOP_GASOFF_DELAY;
    g_nEstopTouchReadyOffDelayTime = DEF_ESTOP_TOUCHREADY_DELAY;

    g_dbAinMaxVolt         = DEF_AIN_MAX_VOLT;
    g_dbADCMaxBit          = DEF_ADC_MAX_BIT;

#if defined (__QNXNTO__)
    g_pszSensDir           = DEF_SENS_DIR_QNX;
#else
    g_pszSensDir           = DEF_SENS_DIR_WIN;
#endif
    
    /* Robot */
    g_dbRobotDecel_Estop        = DEF_ROBOT_ESTOP_DEC_TIME;
    g_dbRobotDecel_NormalStop   = DEF_ROBOT_NORMAL_STOP_DEC_TIME;

    /* welder */
    g_nWelderType             = WELDER_TYPE_HYOSUNG_UR;
    if(g_nWelderType == WELDER_TYPE_HYOSUNG_UR)
    {
        g_nMaxWeldCurrent = HYOSUNG_WELD_MAX_CURR_VAL;
        g_nMaxWeldVoltage = HYOSUNG_WELD_MAX_VOLT_VAL;
        g_nMaxInchSpeed   = HYOSUNG_WELD_MAX_INCH_VAL;
    }

    g_nWeldDinSlaveNo           = DEF_DIN_SLAVENO;

    g_Din_portno.nArcOn         = DEF_DIN_PORT_ARCON;
    g_Din_portno.nNoGas         = DEF_DIN_PORT_NOGAS;
    g_Din_portno.nNoWire        = DEF_DIN_PORT_NOWIRE;
    g_Din_portno.nWeldPowerFail = DEF_DIN_PORT_PWRFAIL;
    g_Din_portno.nTouchProcess  = DEF_DIN_PORT_TOUCHPROC;
    g_Din_portno.nTouchSignal   = DEF_DIN_PORT_TOUCHSIG;

    g_nWeldDoutSlaveNo          = DEF_DOUT_SLAVENO;

    g_Dout_portno.nArcOn        = DEF_DOUT_PORT_ARCON;
    g_Dout_portno.nGasOn        = DEF_DOUT_PORT_GASON;
    g_Dout_portno.nInchPos      = DEF_DOUT_PORT_INCHPOS;
    g_Dout_portno.nInchNeg      = DEF_DOUT_PORT_INCHNEG;
    g_Dout_portno.nTouchStart   = DEF_DOUT_PORT_TOUCHSTART;
    g_Dout_portno.nTouchReady   = DEF_DOUT_PORT_TOUCHREADY;
    g_Dout_portno.nWireCut      = DEF_DOUT_PORT_WIRECUT;

    g_Ain_portno.nVoltageInPortNo   = DEF_AIN_PORT_VOLT;
    g_Ain_portno.nCurrentInPortNo   = DEF_AIN_PORT_CURR;

    g_Aout_portno.nVoltageOutPortNo = DEF_AOUT_PORT_VOLT;
    g_Aout_portno.nCurrentOutPortNo = DEF_AOUT_PORT_CURR;

    /* weld tune param */
    g_nWeldTuneInParamApplyIndex        = 0;
    g_nWeldTuneOutParamApplyIndex       = 0;

    g_WeldTuneParam.input.dbVolt_a      = DEF_TUNE_IN_VOLT_A;
    g_WeldTuneParam.input.dbVolt_b      = DEF_TUNE_IN_VOLT_B;
    g_WeldTuneParam.input.dbVoltScale   = DEF_TUNE_IN_VOLT_SCALE;
    g_WeldTuneParam.input.dbVoltOffset  = DEF_TUNE_IN_VOLT_OFFSET;
    g_WeldTuneParam.input.dbCurr_a      = DEF_TUNE_IN_CURR_A;
    g_WeldTuneParam.input.dbCurr_b      = DEF_TUNE_IN_CURR_B;
    g_WeldTuneParam.input.dbCurrScale   = DEF_TUNE_IN_CURR_SCALE;
    g_WeldTuneParam.input.dbCurrOffset  = DEF_TUNE_IN_CURR_OFFSET;

    g_WeldTuneParam.output.dbVolt_a     = DEF_TUNE_OUT_VOLT_A;
    g_WeldTuneParam.output.dbVolt_b     = DEF_TUNE_OUT_VOLT_B;
    g_WeldTuneParam.output.dbVolt_c     = DEF_TUNE_OUT_VOLT_C;
    g_WeldTuneParam.output.dbVoltScale  = DEF_TUNE_OUT_VOLT_SCALE;
    g_WeldTuneParam.output.dbVoltOffset = DEF_TUNE_OUT_VOLT_OFFSET;
    g_WeldTuneParam.output.dbCurr_a     = DEF_TUNE_OUT_CURR_A;
    g_WeldTuneParam.output.dbCurr_b     = DEF_TUNE_OUT_CURR_B;
    g_WeldTuneParam.output.dbCurr_c     = DEF_TUNE_OUT_CURR_C;
    g_WeldTuneParam.output.dbCurrScale  = DEF_TUNE_OUT_CURR_SCALE;
    g_WeldTuneParam.output.dbCurrOffset = DEF_TUNE_OUT_CURR_OFFSET;
    

    /* Axis & Motor */
    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        g_rgfHwLimitUsed[iAxis][LIMIT_NEG]  = 0;
        g_rgfHwLimitUsed[iAxis][LIMIT_POS]  = 0;
        g_rgdbHwLimit[iAxis][LIMIT_NEG]     = INT_MIN;
        g_rgdbHwLimit[iAxis][LIMIT_POS]     = INT_MAX;
    }

    for(iMotor = 0; iMotor < ROB_AXIS_COUNT; iMotor++)
    {
        g_rgnEncRes[iMotor]         = DEF_ENCODER_RESOLUTION;
        g_rgdbMotorEStopDec[iMotor] = DEF_MOTOR_ESTOP_DEC_TIME;
        g_rgnEcnoderHomeVal[iMotor] = DEF_ENCODER_HOME_VAL;
        g_rgdbGearRatio[iMotor]     = DEF_GEAR_REDUCTION_RATIO;
        g_rgnAxisDirection[iMotor]  = DEF_MOTOR_DIRECTION;
    }

    g_nWriteEcatDataSize      = DEF_WRITE_ECAT_SRVDATA_SIZE;
    g_nWriteOffsetControlWord = DEF_WRITE_OFFSET_CONTROLWORD;
    g_nWirteOffsetPosition    = DEF_WRITE_OFFSET_POSITION;
    g_nWirteOffsetPhysicalOutput = DEF_WRITE_OFFSET_OUTPUT;

    g_nReadEcatDataSize       = DEF_READ_ECAT_SRVDATA_SIZE;
    g_nReadOffsetStatus       = DEF_READ_OFFSET_STATUS;
    g_nReadOffsetPosition     = DEF_READ_OFFSET_POSITION;
    g_nReadOffsetError        = DEF_READ_OFFSET_ERROR;
}


////////////////////////////////////////////////////////////////////////////////
//
// PARAM_LoadParamFromSHM()
//
void PARAM_LoadParamFromSHM(int nRobotIndex)
{
    int iAxis, iMotor;

    /* Global */
    g_pszEcatConfigDir     = g_pShmem_SysConfig_rm->ecat.szPathName;
    g_nAxisCount           = g_pShmem_SysConfig_rm->robot[nRobotIndex].nAxesCount;
    g_nSlaveCount          = g_pShmem_SysConfig_rm->ecat.nSlaveCount;
    g_nWriteInitOffsetSize = g_pShmem_SysConfig_rm->ecat.nWriteOffsetSize;
    g_nReadInitOffsetSize  = g_pShmem_SysConfig_rm->ecat.nReadOffsetSize;

    g_nQNXTimerTick        = g_pShmem_SysConfig_rm->ctrl.nQNXTimerTick;
    g_nQNXTimerRes         = g_pShmem_SysConfig_rm->ctrl.nQNXTimerRes;
    g_nTrajUpdateTime      = g_pShmem_SysConfig_rm->ctrl.nTrajUpdateTime;
    g_nIoScanTime          = g_pShmem_SysConfig_rm->ctrl.nIoScanTime;

    g_nServoOnBrakeDelayTime  = g_pShmem_SysConfig_rm->ctrl.nServoOnBrakeDelayTime;
    g_nServoOffBrakeDelayTime = g_pShmem_SysConfig_rm->ctrl.nServoOffBrakeDelayTime;

    g_nEstopGasOffDelayTime        = g_pShmem_SysParam_rm->nEstopGasOffDelayTime;
    g_nEstopTouchReadyOffDelayTime = g_pShmem_SysParam_rm->nEstopTouchReadyOffDelayTime;

    g_dbAinMaxVolt         = g_pShmem_SysParam_rm->dbAinMaxVolt;
    g_dbADCMaxBit          = g_pShmem_SysParam_rm->dbADCMaxBit;

    memcpy(g_pszSensDir, g_pShmem_SysParam_rm->szSensDir, PATH_NAME_BUFFER_SIZE);

    /* Robot */
    g_dbRobotDecel_Estop      = g_pShmem_SysConfig_rm->robot[0].dbDecel_Estop;
    g_dbRobotDecel_NormalStop = g_pShmem_SysConfig_rm->robot[0].dbDecel;

    /* welder */
    g_nWelderType             = g_pShmem_SysConfig_rm->welder[0].nType;
    if(g_nWelderType == WELDER_TYPE_HYOSUNG_UR)
    {
        g_nMaxWeldCurrent = HYOSUNG_WELD_MAX_CURR_VAL;
        g_nMaxWeldVoltage = HYOSUNG_WELD_MAX_VOLT_VAL;
        g_nMaxInchSpeed   = HYOSUNG_WELD_MAX_INCH_VAL;
    }

    g_nWeldDinSlaveNo           = g_pShmem_SysConfig_rm->welder[0].nDinSlaveNo;

    g_Din_portno.nArcOn         = g_pShmem_SysConfig_rm->welder[0].din_portno.nArcOn;
    g_Din_portno.nNoGas         = g_pShmem_SysConfig_rm->welder[0].din_portno.nNoGas;
    g_Din_portno.nNoWire        = g_pShmem_SysConfig_rm->welder[0].din_portno.nNoWire;
    g_Din_portno.nWeldPowerFail = g_pShmem_SysConfig_rm->welder[0].din_portno.nWeldPowerFail;
    g_Din_portno.nTouchProcess  = g_pShmem_SysConfig_rm->welder[0].din_portno.nTouchProcess;
    g_Din_portno.nTouchSignal   = g_pShmem_SysConfig_rm->welder[0].din_portno.nTouchSignal;

    g_nWeldDoutSlaveNo          = g_pShmem_SysConfig_rm->welder[0].nDoutSlaveNo;

    g_Dout_portno.nArcOn        = g_pShmem_SysConfig_rm->welder[0].dout_portno.nArcOn;
    g_Dout_portno.nGasOn        = g_pShmem_SysConfig_rm->welder[0].dout_portno.nGasOn;
    g_Dout_portno.nInchPos      = g_pShmem_SysConfig_rm->welder[0].dout_portno.nInchPos;
    g_Dout_portno.nInchNeg      = g_pShmem_SysConfig_rm->welder[0].dout_portno.nInchNeg;
    g_Dout_portno.nTouchStart   = g_pShmem_SysConfig_rm->welder[0].dout_portno.nTouchStart;
    g_Dout_portno.nTouchReady   = g_pShmem_SysConfig_rm->welder[0].dout_portno.nTouchReady;
    g_Dout_portno.nWireCut      = g_pShmem_SysConfig_rm->welder[0].dout_portno.nWireCut;

    g_Ain_portno.nVoltageInPortNo   = g_pShmem_SysConfig_rm->welder[0].nVoltageInPortNo;
    g_Ain_portno.nCurrentInPortNo   = g_pShmem_SysConfig_rm->welder[0].nCurrentInPortNo;

    g_Aout_portno.nVoltageOutPortNo = g_pShmem_SysConfig_rm->welder[0].nVoltageOutPortNo;
    g_Aout_portno.nCurrentOutPortNo = g_pShmem_SysConfig_rm->welder[0].nCurrentOutPortNo;

    /* weld tune param */
    g_nWeldTuneInParamApplyIndex        = g_pShmem_SysParam_rm->nWeldTuneInParamApplyIndex;
    g_nWeldTuneOutParamApplyIndex       = g_pShmem_SysParam_rm->nWeldTuneOutParamApplyIndex;

    g_WeldTuneParam.input.dbVolt_a      = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbVolt_a;
    g_WeldTuneParam.input.dbVolt_b      = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbVolt_b;
    g_WeldTuneParam.input.dbVoltScale   = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbVoltScale;
    g_WeldTuneParam.input.dbVoltOffset  = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbVoltOffset;
    g_WeldTuneParam.input.dbCurr_a      = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurr_a;
    g_WeldTuneParam.input.dbCurr_b      = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurr_b;
    g_WeldTuneParam.input.dbCurrScale   = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurrScale;
    g_WeldTuneParam.input.dbCurrOffset  = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurrOffset;

    g_WeldTuneParam.output.dbVolt_a     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbVolt_a;
    g_WeldTuneParam.output.dbVolt_b     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbVolt_b;
    g_WeldTuneParam.output.dbVolt_c     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbVolt_c;
    g_WeldTuneParam.output.dbVoltScale  = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbVoltScale;
    g_WeldTuneParam.output.dbVoltOffset = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbVoltOffset;
    g_WeldTuneParam.output.dbCurr_a     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbCurr_a;
    g_WeldTuneParam.output.dbCurr_b     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbCurr_b;
    g_WeldTuneParam.output.dbCurr_c     = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbCurr_c;
    g_WeldTuneParam.output.dbCurrScale  = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbCurrScale;
    g_WeldTuneParam.output.dbCurrOffset = g_pShmem_SysParam_rm->weld_tune[g_nWeldTuneOutParamApplyIndex].output.dbCurrOffset;

    /* Axis & Motor */
    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        g_rgfHwLimitUsed[iAxis][LIMIT_NEG]  = g_pShmem_SysConfig_rm->robot[0].axis[iAxis].fHwLim_min;
        g_rgfHwLimitUsed[iAxis][LIMIT_POS]  = g_pShmem_SysConfig_rm->robot[0].axis[iAxis].fHwLim_max;
        g_rgdbHwLimit[iAxis][LIMIT_NEG]     = g_pShmem_SysConfig_rm->robot[0].axis[iAxis].pos_hwlim_min;
        g_rgdbHwLimit[iAxis][LIMIT_POS]     = g_pShmem_SysConfig_rm->robot[0].axis[iAxis].pos_hwlim_max;
    }

    for(iMotor = 0; iMotor < ROB_AXIS_COUNT; iMotor++)
    {
        //g_rgnEncRes[iMotor]         = g_pShmem_SysConfig_rm->robot[0].axis[iMotor].motor[0]->nEncRes;
        g_rgnEncRes[iMotor]         = g_pShmem_SysConfig_rm->motor[iMotor].nEncRes;
        g_rgdbMotorEStopDec[iMotor] = g_pShmem_SysConfig_rm->motor[iMotor].dec_estop;
        g_rgnEcnoderHomeVal[iMotor] = g_pShmem_SysConfig_rm->robot[0].axis[iMotor].ori[0];
        g_rgdbGearRatio[iMotor]     = g_pShmem_SysConfig_rm->robot[0].axis[iMotor].red[0];
        g_rgnAxisDirection[iMotor]  = g_pShmem_SysConfig_rm->robot[0].axis[iMotor].dir[0];
    }

    g_nWriteEcatDataSize      = DEF_WRITE_ECAT_SRVDATA_SIZE;
    g_nWriteOffsetControlWord = g_nWriteInitOffsetSize;
    g_nWirteOffsetPosition    = g_nWriteInitOffsetSize + WRITE_CONTROLWORD_SIZE;
    g_nWirteOffsetPhysicalOutput  = g_nWriteInitOffsetSize + WRITE_CONTROLWORD_SIZE +
                                    WRITE_POSITION_SIZE;

    g_nReadEcatDataSize       = DEF_READ_ECAT_SRVDATA_SIZE;
    g_nReadOffsetStatus       = g_nReadInitOffsetSize;
    g_nReadOffsetPosition     = g_nReadInitOffsetSize + READ_STATUS_SIZE;
    g_nReadOffsetError        = g_nReadInitOffsetSize + READ_STATUS_SIZE +
                                READ_POSITION_SIZE;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_LoadSysConfigParam()
//
int SHM_LoadSysConfigParam(int nRobotIndex)
{
    g_pszSensDir = (char *)DEBUG_MALLOC(PATH_NAME_BUFFER_SIZE);

    if(g_pShmem_SysConfig_rm == NULL || g_pShmem_SysParam_rm == NULL)
    {
        if(g_pShmem_SysConfig_rm == NULL)
        {
            VERBOSE_ERROR("SysConfig Shared Memory Load Fail!\n");
        }
        if(g_pShmem_SysParam_rm == NULL)
        {
            VERBOSE_ERROR("SysParam Shared Memory Load Fail!\n");
        }
        VERBOSE_WARNING("Set Parameters to Default Value!\n");
        PARAM_LoadDefaultParam();
    }
    else
    {
        VERBOSE_MESSAGE("SysConfig & SysParam Shared Memory Load Done!\n");
        PARAM_LoadParamFromSHM(nRobotIndex);
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_ShowSystemParam()
//
int SHM_ShowSystemParam(void)
{
    int iAxis;

    VERBOSE_NOTIFY("\v\t<EtherCAT Parameter>\n"
                  "Slave Count : %d cnt\t"
                  "Axis Count : %d cnt\n"
                  "Write Offset Size: 0x%x \t"
                  "Read Offset Size: 0x%x \n"
                  "ECAT Conf File  : %s\n", 
                  g_nSlaveCount,
                  g_nAxisCount,
                  g_nWriteInitOffsetSize,
                  g_nReadInitOffsetSize,
                  g_pszEcatConfigDir);
    VERBOSE_NOTIFY("\v\n");

    VERBOSE_NOTIFY("\v\t<Control Parameter>\n"
                   "QNX Timer Tick  : %d ms,\t\tQNX Timer Res: %d us\n"
                   "Traj Update Time: %d ms,\t\tIO Scan Time : %d ms\n",
                   g_nQNXTimerTick, g_nQNXTimerRes,
                   g_nTrajUpdateTime, g_nIoScanTime);
    VERBOSE_NOTIFY("\vAnalog In Max Volt  : %.1lf V,\tAnalog Convert Max Bit: %.0lf bit\n",
                   g_dbAinMaxVolt, g_dbADCMaxBit);
    VERBOSE_NOTIFY("\vSensor Dir: %s\n", g_pszSensDir);
    VERBOSE_NOTIFY("\v\n");

    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        VERBOSE_NOTIFY("\v\t<Motor %d Parameter>\n"
                       "HW Limit-> +: %d(Pos: %.1lf), -: %d(neg: %.1lf)\n"
                       "Encoder Res: %d,\t\tEncoder Home Value: %d\n"
                       "Gear Reduction: %.2lf,\t\tMotor Direction: %d\n",
                       iAxis,
                       g_rgfHwLimitUsed[iAxis][LIMIT_POS], g_rgdbHwLimit[iAxis][LIMIT_POS] * (180/M_PI),
                       g_rgfHwLimitUsed[iAxis][LIMIT_NEG], g_rgdbHwLimit[iAxis][LIMIT_NEG] * (180/M_PI),
                       g_rgnEncRes[iAxis], g_rgnEcnoderHomeVal[iAxis],
                       g_rgdbGearRatio[iAxis], g_rgnAxisDirection[iAxis]);
    }
    VERBOSE_NOTIFY("\v\n");

    VERBOSE_NOTIFY("\v\t<Weld Input Tune Parameter(Idx: %d)>\n"
                   "Volt_a: %.10lf,\t\tVolt_b: %.10lf\n"
                   "Volt_Scale: %lf,\t\tVolt_Offset: %.10lf\n"
                   "Curr_a: %.10lf,\t\tCurr_b: %.10lf\n"
                   "Curr_Scale: %.10lf,\t\tCurr_Offset: %.10lf\n",
                   g_nWeldTuneInParamApplyIndex,
                   g_WeldTuneParam.input.dbVolt_a,
                   g_WeldTuneParam.input.dbVolt_b,
                   g_WeldTuneParam.input.dbVoltScale,
                   g_WeldTuneParam.input.dbVoltOffset,
                   g_WeldTuneParam.input.dbCurr_a,
                   g_WeldTuneParam.input.dbCurr_b,
                   g_WeldTuneParam.input.dbCurrScale,
                   g_WeldTuneParam.input.dbCurrOffset);
    
    VERBOSE_NOTIFY("\v\t<Weld Input Tune Parameter(Idx: %d)>\n"
                   "Volt_a: %.10lf,\t\tVolt_b: %.10lf\tVolt_c: %.10lf\n"
                   "Volt_Scale: %.10lf,\tVolt_Offset: %.10lf\n",
                   g_nWeldTuneOutParamApplyIndex,
                   g_WeldTuneParam.output.dbVolt_a,
                   g_WeldTuneParam.output.dbVolt_b,
                   g_WeldTuneParam.output.dbVolt_c,
                   g_WeldTuneParam.output.dbVoltScale,
                   g_WeldTuneParam.output.dbVoltOffset);

    VERBOSE_NOTIFY("\vCurr_a: %.10lf,\t\tCurr_b: %lf\tCurr_c: %.10lf\n"
                   "Curr_Scale: %.10lf,\tCurr_Offset: %.10lf\n",
                   g_WeldTuneParam.output.dbCurr_a,
                   g_WeldTuneParam.output.dbCurr_b,
                   g_WeldTuneParam.output.dbCurr_c,
                   g_WeldTuneParam.output.dbCurrScale,
                   g_WeldTuneParam.output.dbCurrOffset);
    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// SHM_Status_TE_OpenShmem()
//
int SHM_Status_TE_OpenShmem(void)
{
    // TE shared memory open
    if (g_pShmem_Status_te != NULL)
    {
        VERBOSE_WARNING("TE Status shared memory already opened.\n");
        return 0;
    }

    g_hShm_Status_te = SHM_Open(SHMNAME_TE_TEST);

    if (g_hShm_Status_te == -1)
    {
        VERBOSE_ERROR("Failed to open TE Status shared memory.!!!\n");
        return -1;
    }
    else
    {
        VERBOSE_VERBOSE("Opened the TE Status shared memory. Name(%s)\n", 
                        SHMNAME_TE_TEST);
    }

    // map
    g_pShmem_Status_te = (SHM_TE_STATUS*) SHM_Map(g_hShm_Status_te, sizeof(SHM_TE_STATUS));

    if (g_pShmem_Status_te == NULL)
    {
        VERBOSE_ERROR("Cannot map the TE Status shared memory.!!!\n");
        return -1;
    }
    else
    {
        if (sizeof(SHM_TE_STATUS) == g_pShmem_Status_te->size)
        {
            g_fOpenShmTeStatus = TRUE;  // flag of open TE_SHM
            
            VERBOSE_VERBOSE("Mapped the TE Status shared memory. Size(%dbytes), "
                            "Chk(%dbytes)\n", 
                            sizeof(SHM_TE_STATUS),
                            g_pShmem_Status_te->size);
        }
        else
        {
            g_fOpenShmTeStatus = FALSE;  // flag of open TE_SHM
            VERBOSE_ERROR("Mismatched the TE Status shared memory. Size(%dbytes), "
                          "Chk(%dbytes)\n",
                          sizeof(SHM_TE_STATUS),
                          g_pShmem_Status_te->size);

            return -1;
        }
    }    

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_Task_TE_OpenShmem()
//
int SHM_Task_TE_OpenShmem(void)
{
    // TE shared memory open
    if (g_pShmem_Task_te != NULL)
    {
        VERBOSE_WARNING("TE ArcSensor Task shared memory already opened.\n");
        return 0;
    }

    g_hShm_Task_te = SHM_Open(SHMNAME_SENSOR_SC);

    if (g_hShm_Task_te == -1)
    {
        VERBOSE_ERROR("Failed to open TE ArcSensor Task shared memory.!!!\n");
        return -1;
    }
    else
    {
        VERBOSE_VERBOSE("Opened the TE ArcSensor Task shared memory. Name(%s)\n", 
                        SHMNAME_SENSOR_SC);
    }

    // map
    g_pShmem_Task_te = (shm_task_servo_t*) SHM_Map(g_hShm_Task_te, sizeof(shm_task_servo_t));

    if (g_pShmem_Task_te == NULL)
    {
        VERBOSE_ERROR("Cannot map the TE ArcSensor Task shared memory.!!!\n");
        return -1;
    }
    else
    {
        if (sizeof(shm_task_servo_t) == g_pShmem_Task_te->size)
        {
            g_fOpenShmTeTask = TRUE;  // flag of open TE_SHM
            
            VERBOSE_VERBOSE("Mapped the TE ArcSensor Task shared memory. Size(%dbytes), "
                            "Chk(%dbytes)\n", 
                            sizeof(shm_task_servo_t),
                            g_pShmem_Task_te->size);
        }
        else
        {
            g_fOpenShmTeTask = FALSE;  // flag of open TE_SHM
            VERBOSE_ERROR("Mismatched the TE ArcSensor Task shared memory. Size(%dbytes), "
                          "Chk(%dbytes)\n",
                          sizeof(shm_task_servo_t),
                          g_pShmem_Task_te->size);

            return -1;
        }
    }    

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysStatusDestroyShmem()
//
int SHM_RM_SysStatusDestroyShmem(void)
{
    // unmap RM shared memory
    if (g_pShmem_SysStatus_rm != NULL)
    {
        SHM_Unmap((void*) g_pShmem_SysStatus_rm, sizeof(SHM_RM_SYSSTATUS));
        g_pShmem_SysStatus_rm = NULL;

        VERBOSE_VERBOSE("Unmapped the RM SysStatus shared memory. Size(%dbytes)\n", 
                        sizeof(SHM_RM_SYSSTATUS));
    }

    // close RM shared memory
    if (g_hShm_SysStatus_rm != DANDY_INVALID_SHMEM_HANDLE)
    {
        SHM_Close(g_hShm_SysStatus_rm);
        g_hShm_SysStatus_rm = DANDY_INVALID_SHMEM_HANDLE;
        g_fOpenSysStatusShmRm = FALSE;  // flag of open RM_SHM

        VERBOSE_VERBOSE("Destroyed the RM SysStatus shared memory. Name(%s)\n", 
                        SHM_RM_SYSSTATUS_NAME);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysConfigDestroyShmem()
//
int SHM_RM_SysConfigDestroyShmem(void)
{
    // unmap RM shared memory
    if (g_pShmem_SysConfig_rm != NULL)
    {
        SHM_Unmap((void*) g_pShmem_SysConfig_rm, sizeof(SHM_RM_SYSCONFIG));
        g_pShmem_SysConfig_rm = NULL;

        VERBOSE_VERBOSE("Unmapped the RM SysConfig shared memory. Size(%dbytes)\n", 
                        sizeof(SHM_RM_SYSCONFIG));
    }

    // close RM shared memory
    if (g_hShm_SysConfig_rm != DANDY_INVALID_SHMEM_HANDLE)
    {
        SHM_Close(g_hShm_SysConfig_rm);
        g_hShm_SysConfig_rm = DANDY_INVALID_SHMEM_HANDLE;
        g_fOpenSysConfigShmRm = FALSE;  // flag of open RM_SHM

        VERBOSE_VERBOSE("Destroyed the RM SysConfig shared memory. Name(%s)\n", 
                        SHM_RM_SYSCONFIG_NAME);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_RM_SysParamDestroyShmem()
//
int SHM_RM_SysParamDestroyShmem(void)
{
    // unmap RM shared memory
    if (g_pShmem_SysParam_rm != NULL)
    {
        SHM_Unmap((void*) g_pShmem_SysParam_rm, sizeof(SHM_RM_SYSPARAM));
        g_pShmem_SysParam_rm = NULL;

        VERBOSE_VERBOSE("Unmapped the RM SysParam shared memory. Size(%dbytes)\n", 
                        sizeof(SHM_RM_SYSPARAM));
    }

    // close RM shared memory
    if (g_hShm_SysParam_rm != DANDY_INVALID_SHMEM_HANDLE)
    {
        SHM_Close(g_hShm_SysParam_rm);
        g_hShm_SysParam_rm = DANDY_INVALID_SHMEM_HANDLE;
        g_fOpenSysParamShmRm = FALSE;  // flag of open RM_SHM

        VERBOSE_VERBOSE("Destroyed the RM SysParam shared memory. Name(%s)\n", 
                        SHM_RM_SYSPARAM_NAME);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_Status_TE_DestroyShmem()
//
int SHM_Status_TE_DestroyShmem(void)
{
    // unmap TE shared memory
    if (g_pShmem_Status_te != NULL)
    {
        SHM_Unmap((void*) g_pShmem_Status_te, sizeof(SHM_TE_STATUS));
        g_pShmem_Status_te = NULL;

        VERBOSE_VERBOSE("Unmapped the TE Status shared memory. Size(%dbytes)\n", 
                        sizeof(SHM_TE_STATUS));
    }

    // close TE shared memory
    if (g_hShm_Status_te != DANDY_INVALID_SHMEM_HANDLE)
    {
        SHM_Close(g_hShm_Status_te);
        //SHM_Destroy(g_hShm_rm, RMGR_SYSSTATUS_RM_SHM_NAME);
        g_hShm_Status_te = DANDY_INVALID_SHMEM_HANDLE;
        g_fOpenShmTeStatus = FALSE;  // flag of open TE_SHM

        VERBOSE_VERBOSE("Destroyed the TE Status shared memory. Name(%s)\n", 
                        SHMNAME_TE_TEST);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_Task_TE_DestroyShmem()
//
int SHM_Task_TE_DestroyShmem(void)
{
    // unmap TE shared memory
    if (g_pShmem_Task_te != NULL)
    {
        SHM_Unmap((void*) g_pShmem_Task_te, sizeof(shm_task_servo_t));
        g_pShmem_Task_te = NULL;

        VERBOSE_VERBOSE("Unmapped the TE ArcSensor Task shared memory. Size(%dbytes)\n", 
                        sizeof(shm_task_servo_t));
    }

    // close TE shared memory
    if (g_hShm_Task_te != DANDY_INVALID_SHMEM_HANDLE)
    {
        SHM_Close(g_hShm_Task_te);
        //SHM_Destroy(g_hShm_rm, RMGR_SYSSTATUS_RM_SHM_NAME);
        g_hShm_Task_te = DANDY_INVALID_SHMEM_HANDLE;
        g_fOpenShmTeTask = FALSE;  // flag of open TE_SHM

        VERBOSE_VERBOSE("Destroyed the TE ArcSensor Task shared memory. Name(%s)\n", 
                        SHMNAME_SENSOR_SC);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SHM_SC_DestroyShmem()
//
void SHM_SC_DestroyShmem(void)
{
    if (g_pShmem_sc != NULL)
    {
        SHM_Unmap((void*) g_pShmem_sc, sizeof(SHM_SC_SYSTEM));
        g_pShmem_sc = NULL;

        VERBOSE_VERBOSE("Unmapped the SC shared memory. Size(%dbytes)\n", 
                        sizeof(SHM_SC_SYSTEM));
    }

    if (g_hShm_sc != -1)
    {
        SHM_Destroy(g_hShm_sc, SC_SHM_NAME);
        g_hShm_sc = -1;

        VERBOSE_VERBOSE("Destroyed the SC shared memory. Name(%s)\n", 
                        SC_SHM_NAME);
    }

    g_fOpenShmSc = FALSE;   // flag of SC_SHM creation
    
}
