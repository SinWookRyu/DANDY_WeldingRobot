#ifndef SERVOCON_H__
#define SERVOCON_H__

#include <stdio.h>
#include <stdlib.h>

#include "dandy_platform.h"
#include "dandy_echo.h"
#include "dandy_msgpass.h"
#include "dandy_thread.h"
#include "dandy_shmem.h"
#include "dandy_timecon.h"
#include "dandy_debug.h"
#include "error_def.h"
#include "sys_conf.h"
#include "ecat_def.h"

#include "CRT.h"
#include "ipc_robotmgr.h"
#include "ipc_taskexec.h"
#include "ipc_servocon.h"

#include "ecattypes.h"
#if defined (__QNXNTO__)
#include "libmkpaiodev.h"
#include "ecatmkpa.h"
//#include "mkpaauxiliary.h"
//#include "config.h"
#endif

#define RESULT_OK                       0
#define RESULT_ERROR                    -1

#define RELEASE                         0
#define LOCK                            1

#define OFF                             0
#define ON                              1

#define EACH                            0
#define ALL                             1
#define ALL_AXES                        100

#define LOW                             0
#define HIGH                            1

#define LIMIT_NEG                       0
#define LIMIT_POS                       1

#define ESTOP_ACT                       0
#define ESTOP_DEACT                     1

#define STOP                            1
#define RUN                             2
#define PAUSE                           3

#define SMOOTH_MODE                     0
#define URGENT_MODE                     1

#define ESTOP_MODE                      0
#define STOP_MODE                       1
#define SRVOFF_MODE                     2

#define ROBOT_0_INDEX                   0
#define ROBOT_1_INDEX                   1
#define ROBOT_2_INDEX                   2
#define ROBOT_3_INDEX                   3

#define AXIS_0_INDEX                    0
#define AXIS_1_INDEX                    1
#define AXIS_2_INDEX                    2
#define AXIS_3_INDEX                    3
#define AXIS_4_INDEX                    4
#define AXIS_5_INDEX                    5

#define LEN_SINT_BYTE                   1
#define LEN_INT_BYTE                    2
#define LEN_DINT_BYTE                   4
#define LEN_USINT_BYTE                  1
#define LEN_UINT_BYTE                   2
#define LEN_UDINT_BYTE                  4

#define TARGET_POS_IDX                  0
#define ACTUAL_POS_IDX                  1

#define OPT_NULL                        0
#define OPT_QUITE                       1

#define OPT_READ                        0
#define OPT_WRITE                       1

#if 0
#define SRVSTATE_CODE_SERVO_ON_R            (0xe7ff & 0x1637)  //5687
#define SRVSTATE_CODE_SERVO_ON_NR           (0xe7ff & 0x1237)  //4663
#define SRVSTATE_CODE_SERVO_OFF             (0xe7ff & 0x0670)  //1648
#define SRVSTATE_CODE_SERVO_OFF_READY_SWON  (0xe7ff & 0x0631)  //1585
#define SRVSTATE_CODE_SERVO_OFF_SWON        (0xe7ff & 0x0633)  //1587
#define SRVSTATE_CODE_SERVO_OFF_SHUT        (0xe7ff & 0x0650)  //1616
#define SRVSTATE_CODE_CTRWORD_PROCEEDING    (0xe7ff & 0x0237)  //567
#define SRVSTATE_CODE_SWITCHON_DISABLED     (0xe7ff & 0x40)
#endif
#if 1
#define SRVSTATE_CODE_SERVO_ON_R            0x1637  //5687
#define SRVSTATE_CODE_SERVO_ON_NR           0x1237  //4663
#define SRVSTATE_CODE_SERVO_ON_NR1          0x1648
#define SRVSTATE_CODE_SERVO_ON_NR2          0x1587

#define SRVSTATE_CODE_SERVO_OFF_NR          0x0270
#define SRVSTATE_CODE_SERVO_OFF_NR1         0x1637
#define SRVSTATE_CODE_SERVO_OFF_NR2         0x1237
#define SRVSTATE_CODE_SERVO_QUICKSTOP       0x0237
#define SRVSTATE_CODE_SERVO_STOP            0x0217
#define SRVSTATE_CODE_SERVO_OFF             0x0670  //1648
#define SRVSTATE_CODE_SERVO_OFF_READY_SWON  0x0631  //1585
#define SRVSTATE_CODE_SERVO_OFF_SWON        0x0633  //1587
#define SRVSTATE_CODE_SERVO_OFF_SHUT        0x0650  //1616
#define SRVSTATE_CODE_CTRWORD_PROCEEDING    0x0237  //567
#define SRVSTATE_CODE_SWITCHON_DISABLED     0x40
#endif
#define OBJECT_ENCRESET_CHAR_LEN        16
#define OBJECT_STORE_CHAR_LEN           4

#define EXIT_COUNT_LIMIT                20

#define COE_OPERATION_TIMEOUT	    5000
#define COE_OPERATION_TIMEOUT_MS    1000

///////////////////////////////////////////////
//
// system mode of operation
//
#define PROFILED_POS_MODE                1
#define INTERPOLATED_POS_MODE            7
#define CYCLIC_SYNC_POS_MODE             8

///////////////////////////////////////////////
//
// system parameter default value
//

#define DEF_AXIS_COUNT                  6
#define DEF_SLAVE_COUNT                 17

#define DEF_TIMER_TICK                  1
#define DEF_TIMER_RES                   100
#define DEF_TRAJ_UPDATE_TIME            1
#define DEF_IO_SCAN_TIME                5

#define DEF_SERVOON_DELAY               70
#define DEF_SERVOOFF_DELAY              150

#define DEF_ESTOP_GASOFF_DELAY          3000    // [ms]
#define DEF_ESTOP_TOUCHREADY_DELAY      500     // [ms]

#define DEF_ENCODER_RESOLUTION          1048576
#define DEF_ENCODER_HOME_VAL            0
#define DEF_GEAR_REDUCTION_RATIO        1
#define DEF_MOTOR_DIRECTION             1

#define DEF_MOTOR_ESTOP_DEC_TIME        200
#define DEF_ROBOT_ESTOP_DEC_TIME        200
#define DEF_ROBOT_NORMAL_STOP_DEC_TIME  300

//#define GAS_OFF_DELAY                   3000

///////////////////////////////////////////////
//
// welder parameter default value
//

#define HYOSUNG_WELD_MAX_CURR_VAL      500      // 50 ~ 500 [A]
#define HYOSUNG_WELD_MAX_VOLT_VAL      45       // 14 ~ 45 [V] // 42?
#define HYOSUNG_WELD_MAX_INCH_VAL      1000     // 10 ~ 1000 [mm/s]

#define DEF_DIN_SLAVENO                0

#define DEF_DIN_PORT_ARCON             16
#define DEF_DIN_PORT_NOGAS             17
#define DEF_DIN_PORT_NOWIRE            18
#define DEF_DIN_PORT_PWRFAIL           19
#define DEF_DIN_PORT_TOUCHPROC         20
#define DEF_DIN_PORT_TOUCHSIG          21

#define DEF_DOUT_SLAVENO               0

#define DEF_DOUT_PORT_ARCON            0
#define DEF_DOUT_PORT_GASON            1
#define DEF_DOUT_PORT_INCHPOS          2
#define DEF_DOUT_PORT_INCHNEG          3
#define DEF_DOUT_PORT_TOUCHSTART       4
#define DEF_DOUT_PORT_TOUCHREADY       5
#define DEF_DOUT_PORT_WIRECUT          6

#define DEF_AIN_PORT_VOLT              0
#define DEF_AIN_PORT_CURR              1

#define DEF_AOUT_PORT_VOLT              0
#define DEF_AOUT_PORT_CURR              1

#define ANALOG_RAW_VAL_RES              32767
#define ANALOG_RAW_VAL_RES_DANDY_I      4095
#define ANALOG_RES_RATIO_DANDY_OLDVER   (ANALOG_RAW_VAL_RES_DANDY_I / ANALOG_RAW_VAL_RES)
#define ANALOG_MAX_ABS_VOLTAGE          10
#define ANALOG_MAX_CMD_VOLTAGE          5
#define ADC_DATA_AVR_CNT                200

#define DEF_AIN_MAX_VOLT                10
#define DEF_ADC_MAX_BIT                 32767

#define DEF_SENS_DIR_QNX                "/works/sdata/"
#define DEF_SENS_DIR_WIN                "./sdata/"

#if 1
// DANDY-I A/D Resolution 4095, DANDY-I A/D Resolution 32767(10V) -> multiplied by 8(0.00244140625)
//0.0024410 = 9.9954/4096, 10/32767 = 0.0003052, 5/32767 = 0.00015259
#define DEF_TUNE_IN_VOLT_A             4.5  //org: 15.3171997   //Lab: 4.5
#define DEF_TUNE_IN_VOLT_B             0.0  //org: -22.1378002  //Lab: 0.0
#define DEF_TUNE_IN_VOLT_SCALE         0.0003052    //0.0024410 = 9.9954/4096 // 10/32767 = 0.0003052
#define DEF_TUNE_IN_VOLT_OFFSET        0.0
#define DEF_TUNE_IN_CURR_A             42.0 //org: 70.0927963   //Lab: 42.0
#define DEF_TUNE_IN_CURR_B             0.0  //org: 100.0        //Lab: 0.0
#define DEF_TUNE_IN_CURR_SCALE         0.0003052    //0.0024410
#define DEF_TUNE_IN_CURR_OFFSET        0.0

#define DEF_TUNE_OUT_VOLT_A            0.2422890
#define DEF_TUNE_OUT_VOLT_B            2.9601600
#define DEF_TUNE_OUT_VOLT_C            0.85
#define DEF_TUNE_OUT_VOLT_SCALE        0.0003052//0.00015259   //0.0003052    //0.0024410
#define DEF_TUNE_OUT_VOLT_OFFSET       0.0
#define DEF_TUNE_OUT_CURR_A            -2.5738800
#define DEF_TUNE_OUT_CURR_B            63.5228996
#define DEF_TUNE_OUT_CURR_C            10.0
#define DEF_TUNE_OUT_CURR_SCALE        0.0003052//0.00015259   //0.0003052    //0.0024410
#define DEF_TUNE_OUT_CURR_OFFSET       0.0
#endif
#if 0
#define DEF_TUNE_IN_VOLT_A             15.3171997
#define DEF_TUNE_IN_VOLT_B             -22.1378002  //0
#define DEF_TUNE_IN_VOLT_SCALE         0.0003052    //0.0024410 // = 10/32767
#define DEF_TUNE_IN_VOLT_OFFSET        0.0
#define DEF_TUNE_IN_CURR_A             10.0
#define DEF_TUNE_IN_CURR_B             10.0         //0
#define DEF_TUNE_IN_CURR_SCALE         0.0003052    //0.0024410
#define DEF_TUNE_IN_CURR_OFFSET        0.0

#define DEF_TUNE_OUT_VOLT_A            0.2422890
#define DEF_TUNE_OUT_VOLT_B            2.9601600
#define DEF_TUNE_OUT_VOLT_C            0.85
#define DEF_TUNE_OUT_VOLT_SCALE        0.0003052    //0.0024410
#define DEF_TUNE_OUT_VOLT_OFFSET       0.0
#define DEF_TUNE_OUT_CURR_A            -2.5738800
#define DEF_TUNE_OUT_CURR_B            63.5228996
#define DEF_TUNE_OUT_CURR_C            10.0
#define DEF_TUNE_OUT_CURR_SCALE        0.0003052    //0.0024410
#define DEF_TUNE_OUT_CURR_OFFSET       0.0
#endif


////////////////////////////////////////////////////////////////////////////////
// ARGUMENT_OPTION
//
typedef struct
{
    BOOL        bVerbose;
    BOOL        bQuit;
    BOOL        bNoVGA;
    BOOL        bSingleExec;
    BOOL        bHardEstop;
    BOOL        bEcatStatDisplay;
    int         nVGADispLine;
    BOOL        bNoTPEstop;
    BOOL        bNoCartEstop;
    BOOL        bNoShockSensor;
    const char* pszConfigName;
} ARGUMENT_OPTION;

////////////////////////////////////////////////////////////////////////////////
// Variables
//

extern  ARGUMENT_OPTION     g_Arg;

/////////////////////////////
// system config parameter

// control config
extern int    g_nQNXTimerTick;
extern int    g_nQNXTimerRes;
extern int    g_nTrajUpdateTime;
extern int    g_nIoScanTime;

extern int    g_nServoOnBrakeDelayTime;
extern int    g_nServoOffBrakeDelayTime;

extern int    g_nEstopGasOffDelayTime;         // [ms] unit, defailt = 3000 ms
extern int    g_nEstopTouchReadyOffDelayTime;  // [ms] unit, defailt = 500 ms

// robot config
extern char*  g_pszEcatConfigDir;      // EtherCAT Config Directory
extern int    g_nAxisCount;            // Axis Count
extern int    g_nSlaveCount;           // Slave Count

extern double g_rgdbHwLimit[ROB_AXIS_COUNT][2];       // HW limit pos
extern int    g_rgfHwLimitUsed[ROB_AXIS_COUNT][2];    // HW limit used?

extern int    g_rgnEncRes[MAX_MOTOR_COUNT];
extern double g_rgdbMotorEStopDec[MAX_MOTOR_COUNT];
extern int    g_rgnEcnoderHomeVal[MAX_MOTOR_COUNT];
extern double g_rgdbGearRatio[MAX_MOTOR_COUNT];
extern int    g_rgnAxisDirection[MAX_MOTOR_COUNT];

extern double g_dbRobotDecel_Estop;
extern double g_dbRobotDecel_NormalStop;

// welder config
int     g_nWelderType;
int     g_nWeldDinSlaveNo;
int     g_nWeldDoutSlaveNo;
double  g_nMaxWeldCurrent;
double  g_nMaxWeldVoltage;
double  g_nMaxInchSpeed;

#pragma pack(push, 1)
typedef struct t_din_portno
{
    int     nArcOn;         // arc on/off state
    int     nNoGas;         // gas shortage state
    int     nNoWire;        // wire shortage state
    int     nWeldPowerFail; // welder power fail state
    int     nTouchProcess;  // touch process activated state
    int     nTouchSignal;   // touch signal state
} DIN_PORTNO; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_dout_portno
{
    int     nArcOn;         // arc on/off
    int     nGasOn;         // gas on/off
    int     nInchPos;       // (+) inching
    int     nInchNeg;       // (-) inching
    int     nTouchStart;    // touch start
    int     nTouchReady;    // touch ready (MC state control)
    int     nWireCut;       // wire cut on/off
} DOUT_PORTNO; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_ain_portno
{
    int     nCurrentInPortNo;       // welding current A/D channel (-1=invalid)
    int     nVoltageInPortNo;       // welding voltage A/D channel (-1=invalid)
} AIN_PORTNO; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_aout_portno
{
    int     nCurrentOutPortNo;      // welding current D/A channel (-1=invalid)
    int     nVoltageOutPortNo;      // welding voltage D/A channel (-1=invalid)
} AOUT_PORTNO; 
#pragma pack(pop)

DIN_PORTNO       g_Din_portno;
DOUT_PORTNO      g_Dout_portno;
AIN_PORTNO       g_Ain_portno;
AOUT_PORTNO      g_Aout_portno;

// weld tune parameter
WELD_PARAM_TUNE     g_WeldTuneParam;
extern int g_nWeldTuneInParamApplyIndex;
extern int g_nWeldTuneOutParamApplyIndex;

// ethercat config
extern int g_nSlaveCntNetwork;

extern int g_nWriteInitOffsetSize;    // BeckHoff output size
extern int g_nWriteEcatDataSize;
extern int g_nWriteOffsetControlWord;
extern int g_nWirteOffsetPosition;
extern int g_nWirteOffsetPhysicalOutput;

extern int g_nReadInitOffsetSize;     // BeckHoff input size
extern int g_nReadEcatDataSize;
extern int g_nReadOffsetStatus;
extern int g_nReadOffsetPosition;
extern int g_nReadOffsetError;

extern double g_dbTrg_Pos[ROB_AXIS_COUNT]; // target position of motor(rad)
extern int g_nTrg_Pulse[ROB_AXIS_COUNT];   // target pulse of motor
extern int g_nPhysicalOut[ROB_AXIS_COUNT]; // physical output value
extern int g_nCtrlWord[ROB_AXIS_COUNT];    // control word for PDO writing

extern double g_dbAct_Pos[ROB_AXIS_COUNT]; // actual position of motor(rad)
extern int g_nAct_Pulse[ROB_AXIS_COUNT];   // actual pulse of motor
extern unsigned short g_nReadStatusValue[ROB_AXIS_COUNT];// status word of PDO
extern int g_nErrCodeServo[ROB_AXIS_COUNT];         // error code of motor

extern ECAT_HANDLE g_hMaster;
extern BOOL g_fConfigLoad;

// global variables
extern int  g_nTime_Limit;
extern char g_chPrintLev;
extern BOOL g_fSetPosFuncActive;
extern int g_nEmergencyCodeServo[ROB_AXIS_COUNT];   // emergency code of motor
extern int g_nErrAxis;                                      // Emergency Msg Received Axis
extern ECAT_WORD g_wEmergencyErrStateCode[ROB_AXIS_COUNT];  // Emergency Msg State Code: 0xff00
extern int g_nErrCodeEcat;                          // error code of EtherCAT
extern ECAT_CHAR g_szEcatErrorDescription[ERROR_MESSAGE_BUFFER_SIZE];   // ethercat error description
extern ECAT_DWORD g_dwWrongWC;                      // Wrong working counter
extern ECAT_WORD  g_wFramesPerSecond;               // Frame Count per sec.
extern int g_nErrorRegister[ROB_AXIS_COUNT];        // check error register of motor
extern double g_dbActTorque[ROB_AXIS_COUNT];        // check actual torque of motor, Unit: %
extern int g_nServoState[ROB_AXIS_COUNT];
extern int g_nServoOnCmdState[ROB_AXIS_COUNT];
extern int g_fHWLimitMonAct;
extern int g_fHWLimitOnState;
extern int g_fIoTestModeActive;
extern int g_fAbsEncResetEventActive;               // Encoder Reset Event Flag
extern int g_fMasterInitEventActive;                // Master Init Event Flag
extern int g_fAxisDebugMsg;                         // Axis Debug Message Flag
extern int g_fDoutsSetZero;                         // All Douts Set Zero Act Flag
extern int g_fABSEncResetRequestflag[ROB_AXIS_COUNT];
extern int g_fABSEncResetDoneflag[ROB_AXIS_COUNT];
extern int g_fPosFileWrite;
extern int g_fSetEcatParam;
extern int g_nModeOperation;
extern int g_nTimerNanoRes;
extern int g_fShockSensorRelease;
extern double  g_dbAinMaxVolt;
extern double  g_dbADCMaxBit;
extern char*   g_pszSensDir;
extern int g_fArcOnSigHigh;
extern int g_fArcOnVirtualProc;

extern int g_nAnalogIOUnitTypeBySVC;
extern double g_rgdbWeldVoltInAvrVal[ADC_DATA_AVR_CNT];
extern double g_rgdbWeldCurrInAvrVal[ADC_DATA_AVR_CNT];

extern int g_nArcSensNodeIdx;

extern double g_dbWeldVoltInVal[ADC_DATA_AVR_CNT];
extern double g_dbWeldCurrInVal[ADC_DATA_AVR_CNT];

// I/O
extern unsigned short g_DinPortVal[ROBOT_DI_SLAVE_COUNT][SLAVE_DI_PORT_COUNT];
extern unsigned short g_DoutPortVal[ROBOT_DO_SLAVE_COUNT][SLAVE_DO_PORT_COUNT];
extern ECAT_REAL32    g_AinPortVal[ROBOT_AI_SLAVE_COUNT][SLAVE_AI_PORT_COUNT];  //-10~10[V]
extern ECAT_REAL32    g_AoutPortVal[ROBOT_AO_SLAVE_COUNT][SLAVE_AO_PORT_COUNT]; //-10~10[V]

extern ECAT_INT16     g_nAoutWriteBuff[ROBOT_AO_SLAVE_COUNT][SLAVE_AO_PORT_COUNT];  //16bit
extern ECAT_INT16     g_nAinReadBuff[ROBOT_AI_SLAVE_COUNT][SLAVE_AI_PORT_COUNT];   //16bit

#pragma pack(push, 1)
typedef struct t_value_out
{
    BYTE        nPort;
    int         nVal;
    //char        nVal;
} VALUE_OUT; 
#pragma pack(pop)

// shared memory
extern  int g_hShm_sc;
extern  int g_hShm_SysStatus_rm;
extern  int g_hShm_SysConfig_rm;
extern  int g_hShm_SysParam_rm;
extern  int g_hShm_Status_te;
extern  int g_hShm_Task_te;

extern  SHM_SC_SYSTEM*              g_pShmem_sc;
extern  SHM_RM_SYSSTATUS*           g_pShmem_SysStatus_rm; 
extern  SHM_RM_SYSCONFIG*           g_pShmem_SysConfig_rm; 
extern  SHM_RM_SYSPARAM*            g_pShmem_SysParam_rm; 
extern  SHM_TE_STATUS*              g_pShmem_Status_te;
extern  volatile shm_task_servo_t*  g_pShmem_Task_te;

extern  ECAT_STATISTICS           g_EcatStatistics;

extern  BOOL g_fOpenSysStatusShmRm;  // flag of open RM_SHM
extern  BOOL g_fOpenSysConfigShmRm;  // flag of open RM_SHM
extern  BOOL g_fOpenSysParamShmRm;   // flag of open RM_SHM
extern  BOOL g_fOpenShmSc;           // flag of open SC_SHM
extern  BOOL g_fOpenShmTeStatus;     // flag of open TE_SHM
extern  BOOL g_fOpenShmTeTask;       // flag of open TE_SHM

extern  BOOL g_fLoopGo;
extern  BOOL g_fExit;

// message passing
extern  int g_coidSC;
extern  int g_chidSC;
extern  int g_chidSCTime;
extern  int g_coidSCTime;
extern  int g_rcvidSC;
extern  int g_coidRM;
extern  int g_coidTETime;
extern  int g_coidTE;
extern  int g_hTimer;

// thread state
extern  BOOL g_fThreadRuntimeExitState;
extern  BOOL g_fThreadEcatTraceExitState;
extern  BOOL g_fScanButtonThreadExitState;
extern  BOOL g_fEStopStateThreadExitState;
extern  BOOL g_fDisplayThreadExitState;
extern  BOOL g_fServiceProcThreadExitState;
extern  BOOL g_fScanSDOInputThreadExitState;
extern  BOOL g_fMappingInputThreadExitState;

extern  BOOL g_fScanButtonThreadRunState;
extern  BOOL g_fEStopStateThreadRunState;
extern  BOOL g_fScanSDOInputThreadRunState;
extern  BOOL g_fMappingInputThreadRunState;

extern  SC_MSG         SC_msg;           // msg packet
extern  SC_REPLY       SC_reply;         // reply packet

// time
#pragma pack(push, 1)
typedef struct  t_current_time
{
    int     nYear;
    int     nMonth;
    int     nDay;
    int     nHour;
    int     nMinute;
    int     nSec;
} CURRENT_TIME;
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////
// Functions
//

// thread handle
extern  THREAD_HANDLE hRuntimeThread;         // thread to receive timer pulse
extern  THREAD_HANDLE hEcatTraceThread;       // thread to EtherCAT trace  
extern  THREAD_HANDLE hCheckTimeLimitThread;  // thread of receive time checking from RM
extern  THREAD_HANDLE hScanButtonStateThread; // thread to check button state
extern  THREAD_HANDLE hScanServoStateThread;  // thread to monitoring E-stop state
extern  THREAD_HANDLE hVGADisplayThread;      // thread to VGA Display
extern  THREAD_HANDLE hServiceProcThread;     // thread to service receive & process
extern  THREAD_HANDLE hScanInputStateThread;  // thread to check input state
extern  THREAD_HANDLE hMappingInputThread;    // thread to mapping input state

// thread routine
extern  THREAD_ENTRY_TYPE RuntimeThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE EcatTraceThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE CheckTimeLimitThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE ScanButtonStateThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE ScanEStopStateThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE DSP_VGADisplyThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE SVC_ServiceProcThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE ScanSDOInputStateThreadRoutine(void* pParam);
extern  THREAD_ENTRY_TYPE MappingInputThreadRoutine(void* pParam);

// service routine
extern  int  MSG_SC_ConnectChannel(const char* pName, int nOpt);
extern  int  MSG_SC_CreateChannel(const char* pName);
extern  int  MSG_NamedDestroyChannel(const char* pName, int chid);
extern  int  MSG_NamedDetachConnection(const char* pName, int coid);
extern  int  SERV_ClearServoAlarm(void);
extern  int  SERV_RestartMaster(void);
extern  int  SERV_EStop(int nValue);
extern  int  SERV_ServoOnCmd(int nValue);
extern  int  SERV_BrakeReleaseCmd(int nAxis, int nValue);
extern  int  SERV_EcatDigitalOut(int nSlave, int nIndex, BOOL bValue);
extern  int  SERV_GetNetworkState(void);
extern  int  SERV_SetControlMode(int nMode, int nOpt);
extern  int  SERV_ReadControlMode(void);
extern  int  SERV_GetPosition(void);
extern  int  SERV_GetServoAlarmCode(void);
extern  int  SERV_GetServoState(void);
extern  int  SERV_SetVersion(SC_MSG* pMsg);
extern  int  SERV_SetPosition(int nValue);
extern  int  SERV_PosFileWrite(void);
extern  int  SVC_PositionDataFileOpen(void);
extern  int  SVC_SavePositionDataToFile(void);
extern  int  SVC_PositionDataFileClose(void);
extern  int  SERV_DoService(const SC_MSG* pMsg, SC_MSG* pReply);  // service function
extern  void FUNC_ConvertPosToPulse(int nAxis, int nOpt);
extern  void FUNC_ConvertPulseToPos(int nAxis, int nOpt);
extern  void DSP_SetVerbose(void);

extern  int  SERV_ArcOn_out(int nValue);
extern  int  SERV_GasOn_out(int nValue);
extern  int  SERV_InchingPos_out(int nValue);
extern  int  SERV_InchingNeg_out(int nValue);
extern  int  SERV_TouchStart_out(int nValue);
extern  int  SERV_TouchReady_out(int nValue);

extern  int  SERV_LampControllerReady_out(int nValue);
extern  int  SERV_LampUnderOperating_out(int nValue);
extern  int  SERV_LampServoOn_out(int nValue);
extern  int  SERV_LampEtherCATRun_out(int nValue);
extern  int  SERV_LampError_out(int nValue);

extern  int  SERV_CartLampAlarm_out(int nValue);
extern  int  SERV_CartLampRun_out(int nValue);
extern  int  SERV_WireCut_out(int nValue);
extern  int  SERV_CartJobStartConfirm_out(int nValue);
extern  int  SERV_CartDoutSpare_out(int nValue);

extern  int  SERV_WeldVolt_out(double dbValue);
extern  int  SERV_WeldCurr_out(double dbValue);

extern  int  SERV_WriteDoutPort(int nRecvValue);
extern  int  SERV_WriteAoutPort(int nRecvValue, int nOpt);
extern  int  SERV_ChangeAnalogIOUnit(int nOpt);
extern  int  SERV_ShockSensorRelease(int nValue);

extern  int  SERV_SetInterpolationTimePeriod(int nTimePeriod, int nOpt);
extern  int  SERV_SetInterpolationTimeIndex(int nTimeIndex, int nOpt);
extern  int  SERV_SetSyncErrorCountLimit(int nCountLimit);

extern  int  SERV_Scan_Welder_IO(int nOpt);
extern  int  SERV_Scan_Servo_IO(int nOpt);
extern  int  SERV_Scan_System_State(int nOpt);

extern  void SERV_DoutsSetZero(int nOpt);

extern  int  FUNC_SyncActualPosToTargetPos(void);

extern  void FUNC_MapContInputToWeldInput(void);
extern  void FUNC_MapWeldOutputCmdToContOutput(int nSlave, int nPort);

// main procesure
extern  int  MAIN_Exit(void);
extern  int  MAIN_Init_External(void);
extern  BOOL ParseArgument(int nArgc, char* rgpszArgv[]);

// EtherCAT Internal Lib
extern  ECAT_CHAR* ECATLIB_GetErrorDescription(ECAT_RESULT nErrorCode);
extern  void  ECATLIB_SleepMS(ECAT_TIME_MS timeMS);
extern  void  ECATLIB_wchar2char(ECAT_CHAR* psz, const ECAT_WCHAR* pwsz);

extern  int  ECATLIB_WriteIntegerSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, ECAT_DWORD dwWriteData);
extern  int  ECATLIB_WriteSignedIntegerSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, int nWriteData, ECAT_DWORD dwLength);
extern  int  ECATLIB_WriteStringSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, ECAT_BYTE *pszWriteData, ECAT_DWORD dwLength);
extern  ECAT_DWORD ECATLIB_ReadSlaveCoEObjectInteger(int nSlave, int nReadIdx, int nSubIdx, ECAT_DWORD dwReadData);
extern  int        ECATLIB_ReadSlaveCoEObjectInteger2(int nSlave, int nReadIdx, int nSubIdx, int nReadData);
extern  ECAT_BYTE* ECATLIB_ReadSlaveCoEObjectString(int nSlave, int nReadIdx, int nSubIdx, ECAT_BYTE *pszReadData, ECAT_DWORD dwLength);
extern  int  ECATLIB_ReadSlaveCoEObjectSignedInteger(int nSlave, int nReadIdx, int nSubIdx, int nReadData);

// EtherCAT Network Function
extern  ECAT_CHAR* ECATNET_GetEcatModeString(ECAT_BYTE byState);

extern  int  ECATNET_InitializeMaster(void);
extern  int  ECATNET_ReleaseMaster(void);
extern  int  ECATNET_GetSlaveCountFromNetwork(void);
extern  int  ECATNET_GetSlaveCountFromFile(void);
extern  int  ECATNET_GetSlaveState(void);

// EtherCAT Service Function
extern  int  ECATSERV_ServoOn(int nAxis, int fAllAxis);
extern  int  ECATSERV_ServoOff(int nAxis, int fAllAxis);
extern  int  ECATSERV_QuickStop(void);
extern  int  ECATSERV_AlarmClear(void);
extern  int  ECATSERV_ReadPosition(void);
extern  int  ECATSERV_ReadStatus(int nAxis, int fAllAxis);
extern  int  ECATSERV_GetServoAlarmCode(void);
extern  int  ECATSERV_WriteControlWord(int nAxis, ECAT_WORD wValue, int fAllAxis);
extern  int  ECATSERV_WriteTargetPosition(int nAxis, int fAllAxis);
extern  int  ECATSERV_WriteDigitalOut(int nSlave, int nPort, ECAT_BOOL bValue);
extern  int  ECATSERV_WriteAnalogOut(int nSlave, int nPort, ECAT_REAL32 rValue);
extern  int  ECATSERV_WritePhysicalOutput(int nAxis, ECAT_WORD wValue, int nAllFlag);
extern  int  ECATSERV_WriteHomeOffsetVal(int nAxis);
extern  ECAT_DWORD  ECATSERV_ServoUserParamReset(int nAxis);
extern  int  ECATSERV_ServoStateSetSwitchOnDisable(int nAxis);
extern  int  ECATSERV_ControlModeSet(int nAxis, int nMode, int nOpt);
extern  int  ECATSERV_ControlModeRead(int nAxis);
extern  int  ECATSERV_SetInterpolationTimePeriod(int nAxis, int nTimePeriod, int nOpt);
extern  int  ECATSERV_SetInterpolationTimeIndex(int nAxis,  int nTimeIndex, int nOpt);
extern  int  ECATSERV_SetSyncErrorCountLimit(int nAxis, int nCountLimit);
extern  int  ECATSERV_StoreParameters(int nAxis);
extern  int  ECATSERV_ABSEncoderReset(int nAxis);
extern  int  ECATSERV_StartReadInputWriteOutput(void);
extern  int  ECATSERV_GetInputShadow(void);
extern  int  ECATSERV_ReadTotalInput(void);
extern  int  ECATSERV_WriteTotalOutput(void);
extern  int  ECATSERV_DoneReadInputWriteOutput(void);

// shared memory
extern  int  SHM_RM_SysStatusOpenShmem(void);
extern  int  SHM_RM_SysConfigOpenShmem(void);
extern  int  SHM_RM_SysParamOpenShmem(void);
extern  int  SHM_RM_SysStatusDestroyShmem(void);
extern  int  SHM_RM_SysConfigDestroyShmem(void);
extern  int  SHM_RM_SysParamDestroyShmem(void);
extern  int  SHM_Status_TE_OpenShmem(void);
extern  int  SHM_Status_TE_DestroyShmem(void);
extern  int  SHM_Task_TE_OpenShmem(void);
extern  int  SHM_Task_TE_DestroyShmem(void);
extern  int  SHM_SC_CreateShmem(void);
extern  void SHM_SC_DestroyShmem(void);

extern  int   SHM_LoadSysConfigParam(int nRobotIndex);
extern  int   SHM_ShowSystemParam(void);
extern  void  PARAM_LoadDefaultParam(void);

extern  char* ERR_GetErrorDescription(int nErrCode);

#endif  // SERVOCON_H__
