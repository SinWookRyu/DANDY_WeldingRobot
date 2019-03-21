#ifndef __ROBOT_MGR_H__
#define __ROBOT_MGR_H__

/////////////////////////////////////////////////////////////////////////////
//
//  robotmgr_main.h: RM module main header
//                                            2013.04.11  Ryu SinWook

// Standard Lib. Header
#include <stdlib.h>
#include <string.h>         // for strcpy(), strlen(), memset
#include <stdio.h>
//#include <math.h>
#include <assert.h>         // for assert()
#include <limits.h>         // INI_MIN, INT_MAX used

// DANDY-2015 External Lib. Header
#include "dandy_thread.h"   // for service request thread
#include "dandy_echo.h"     // for VERBOSE
#include "dandy_msgpass.h"  // for message passing (service request)
#include "dandy_ansi.h"     // for key input value
#include "dandy_shmem.h"
#include "dandy_timecon.h"

#include "dandy_debug.h"    // for DEBUG_MALLOC()
#include "dandy_platform.h" // for DANDY_SLEEP()
#include "dandy_jobasm.h"   // for Job Compile

// Header for InterProcessCommunication
#include "ipc_robotmgr.h"
#include "ipc_taskexec.h"
#include "ipc_servocon.h"

// Internal Defined Header
#include "sys_conf.h"
#include "ascii_def.h"
#include "error_def.h"
#include "conf_mgr.h"
#include "CRT.h"

///////////////////////////////////////

#if defined(__linux__)
#define STR_IGCASE_CMP  strcasecmp
#elif defined(_MSC_VER)
#define STR_IGCASE_CMP  _stricmp
#else
#define STR_IGCASE_CMP  stricmp
#endif

#if defined(_WIN32)
#include <windows.h>
#include <conio.h>
#include <tlhelp32.h>
#include "resource.h"
#else
#include <unistd.h>     // delay(), STDIN_FILENO, ...
#include <termios.h>
#endif

#if defined(_WIN32)
int TERM_SetCurrentConsoleIcon(HICON hIconSmall, HICON hIconBig, HICON* phSmallOld, HICON* phBigOld);
#endif

///////////////////////////////////////
//
//  Value Definition
//

#define     VERBOSE_NAME                "RM_VERBOSE_"
#define     INIT_WAIT_TIME_LIMIT_SEC    5
#define     WAITTIME_CONNECTION_SEC     1
#define     CONNECT_RETRY_NO            2

///////////////////////////////////////
//
//  Service Related Symbol Definition
//

#define     RESULT_OK                   0
#define     RESULT_ERROR                -1

#define     RELEASE                     0
#define     LOCK                        1

#define     OFF                         0
#define     ON                          1

#define     STOP                        1
#define     RUN                         2
#define     PAUSE                       3

#define     BASEADDR                    0

#define     TE_CHANNEL_ID               1
#define     SC_CHANNEL_ID               2
#define     RM_CHANNEL_ID               3

#define     CHANNEL_NAME_LEN            18
#define     SYSMODE_NAME_LEN            10
#define     EXECSTATE_NAME_LEN          12
#define     WORKTYPE_NAME_LEN           11
#define     SERV_NAME_LEN               19
#define     ESTOP_NAME_LEN              12
#define     ERROR_MODE_NAME_LEN         8
#define     SIBLINGSTATE_NAME_LEN       10
#define     INIT_STEP_LEN               5
#define     SYSTIME_DATA_LEN            20
#define     RMGR_ERROR_DATA_LEN         32
#define     RMGR_ESTOP_DATA_LEN         32

#define     CONFIG_SECTION_GLOBAL       0
#define     CONFIG_SECTION_ROBOT        1
#define     CONFIG_SECTION_AXIS         2
#define     CONFIG_SECTION_MOTOR        3
#define     CONFIG_SECTION_WELDER       4
#define     CONFIG_SECTION_PARAM        5
#define     CONFIG_SECTION_STATIS       6
#define     CONFIG_SECTION_SENSOR       7

// Initailize Step Define
#define     INIT_STEP0_LOAD_PARAM       0
#define     INIT_STEP1_CREATE_SHM       1
#define     INIT_STEP2_CONNECT_CH       2
#define     INIT_STEP3_INIT_SERV        3
#define     INIT_STEP4_OPEN_SHM         4

#define     ROBOT_0_INDEX               0
#define     ROBOT_1_INDEX               1
#define     ROBOT_2_INDEX               2
#define     ROBOT_3_INDEX               3

#define     SVC_OPT_REBOOT              0
#define     SVC_OPT_SHUTDOWN            1

#define     END_OF_JOBRUNIDX            -1
#define     INIT_JOBRUNIDX              0

#define     TUNE_PARAM_VOLT             0
#define     TUNE_PARAM_CURR             1

#define 	VOLT_OUT				    0	
#define 	CURRENT_OUT				    1

#define     WELD_COEFF_COUNT            3

///////////////////////////////////////
//
//  System Config Related Symbol Definition
//

#define ROBSTRUC_NONE            -1
#define ROBSTRUC_VOID            -2   // no axis robot
#define ROBSTRUC_CARTESIAN       -3   // 
#define ROBSTRUC_VERTART_5B      -4   // Vertical Articulated Robotic (5 bar linkage)
#define ROBSTRUC_VERTART_4B      -5   // Vertical Articulated Robotic (4 bar linkage)
#define ROBSTRUC_BLAST           -6   // DSME blasting working robot

#define MAX_INTERPOLATION_TIME   250

///////////////////////////////////////
//
//  Job Related System Config Data Definition
//

////////////////////////////////////////////////////////////////////////////////
// Job program information

#define JOB_NAME_LENGTH          32   // Job name length (available JOB_NAME_LENGTH-1)
#define JOB_COMMENT_LENGTH       128  // job file comment (available JOB_COMMENT_LENGTH-1)

#define COMPILE_TYPE_UNKNOWN     0
#define COMPILE_TYPE_ASSEMBLE    1
#define COMPILE_TYPE_DISASSEM    2

#define COMPILE_FILE_COUNT       10
#define SYSTEM_CMD_LEN           32

#if 0
#pragma pack(push, 1)
typedef struct
{
    INT     nCmdCount;
    INT     nPosCount;
    INT     nWeavCount;
    INT     nSWFCount;
    INT     nMWFCount;
    INT     nEWFCount;

    CHAR    szName[JOB_NAME_LENGTH];        // job name (no extension name)
    CHAR    __reserved1[32];
    CHAR    szComment[JOB_COMMENT_LENGTH];  // first line comment of the file
} JOB_MODULE_INFO;
#pragma pack(pop)
#endif

////////////////////////////////////////////////////////////////////////////////
// Job Commnad

#define MIN_CMD_COUNT           10      // min command buffer count
#define MAX_CMD_COUNT           32767   // max command buffer count, system can support
#define MAX_EMB_POS_COUNT       32766   // max embedded position count, system can support
#define MAX_GLB_POS_COUNT       32766   // max global position count, system can support
#define MAX_VAR_COUNT           32766   // default I/R vairable count if not specified
#define MAX_WEAVE_COUNT         128     // max weaving type
#define MAX_WELD_COND_COUNT     128     // max welding condition
//#define MAX_HOME_COUNT          16      // Max home orientation (global home, not depend on job prog)
#define MAX_T_VAR_COUNT         MAX_EMB_POS_COUNT
#define MAX_P_VAR_COUNT         MAX_GLB_POS_COUNT
#define MAX_ARG_COUNT           32

#define DEF_CMD_COUNT           8192    // default command count if not specified
#define DEF_EMB_POS_COUNT       256     // default embedded position count
#define DEF_GLB_POS_COUNT       1024    // default global position count
#define DEF_VAR_COUNT           1024    // default I/R vairable count if not specified
#define DEF_WEAVE_COUNT         32      // default weaving condition
#define DEF_WELD_COND_COUNT     128     // default welding condition (dandy use 20)
                                        // (use larger number than 20 for dandy compatible)

///////////////////////////////////////
//
//  ARGUMENT_OPTION
//

#pragma pack(push, 1)
typedef struct
{
    const char* pszConfigName;      // -cf argument

    BOOL        bManualConfig;      // -cf argument
    BOOL        bVerbose;           // -verbose argument
    BOOL        bQuiet;             // -quiet argument
    BOOL        bVGA;               // -nv argument
    BOOL        bHelp;              // -help argument
    BOOL        bManualInit;        // -mi argument
    BOOL        fKeyIn;             // -key argument
    unsigned    nWaitInitTime;      // -wi argument
    unsigned    nWaitConnTime;      // -wc argument
    BOOL        fShutdown;          // -sd argument
    BOOL        fDebug;             // -d argument
    BOOL        fClean;             // -c argument
    BOOL        fCfIgnore;          // -cfi argument
    BOOL        fRobotConfig;       // -r argument
    BOOL        fSensorConfig;      // -sen argument
    BOOL        fTrajTime;          // -tj argument
    BOOL        fIoTime;            // -io argument
    BOOL        fWelder;            // -w argument
    BOOL        fWeldMap;           // -wm argument
    BOOL        fNoAutoInit;        // -nai argument
    BOOL        bLegacySyntax;      // -l argument
    BOOL        bAssemble;          // -a argument
    BOOL        bDisassem;          // -d argument
    unsigned    nCompileType;       // COMPILE_TYPE_xxxx
    BOOL        bOutFile;           // -o argument
    BOOL        bMapFile;           // -m argument
    BOOL        bNoDeadMan;         // -nd argument
    BOOL        bSingleRun;         // -s argument

    // robot
    int         nRobotCount;
    const char* rgpszRobotTypeName[MAX_ROBOT_COUNT];
    int         rgnStartAxis[MAX_ROBOT_COUNT];

    // update time[ms]
    int         nTrajTime;          // -tj argument(trajectory)
    int         nIoTime;            // -io argument(I/O)
    
    // sysio style
    int         nSysIoStyle;

    // welding
    int         nWelderCount;
    int         rgnWelderRobot[MAX_WELDER_COUNT];
    const char* rgpszWelderTypeName[MAX_WELDER_COUNT];
    int         rgnWelderBoard[MAX_WELDER_COUNT];

    const char* pszWeldMapFile;
    
    // job
    const char* pszOutFileName;
    const char* pszMapFileName;

    int         nFileCount;
    const char* rgpszFiles[COMPILE_FILE_COUNT];
} ARGUMENT_OPTION;
#pragma pack(pop)


///////////////////////////////////////
//
//  function proto-types
//

BOOL ParseArgument(int nArgc, char* rgpszArgv[], ARGUMENT_OPTION* pArg);
extern  int SVC_ExecService(int fKeyIn);
extern  int SVC_InitRMService(void);
extern  int SVC_InitTEService(void);
extern  int SVC_InitSCService(void);
extern  int SVC_ExitTEService(void);
extern  int SVC_ExitSCService(void);
extern  int SVC_Write_ErrorHistory(int nState, int nErrorCode);
extern  int SVC_DefineErrorState(int nState, int nErrorCode);
extern  int SVC_RetryInitProcessRM(void);

extern  int MSG_ConnectChannelServer(int nCoid);
extern  int MSG_CloseSiblingConnection(int nCoid, int nCoidAlive);
extern  int MSG_CloseRMInternalConnection(void);
extern  int MSG_CloseRMChannel(void);
extern  int MSG_CreateRMChannel(void);
extern  int MSG_ReplyRMVerToOtherProc(void);
extern  int MSG_CheckOtherProcVersion(void);

extern  int SHM_OpenSharedMemory(void);
extern  int SHM_CreateSharedMemory(void);
extern  int SHM_DestroyShmem(void);
extern  int SYSC_LoadParamToTE_RestartShmem(void);
extern  int SYSC_LoadParamToTE_TaskShmem(void);
extern  int FUNC_SaveRestartParamToFile(void);
extern  int FUNC_ReadRestartParamFromFile(void);
extern  int FUNC_SaveConstVarToFile(int nOpt);
extern  int FUNC_ReadConstVarFromFile(int nOpt);

extern  int SYSMON_ParceErrCodeToErrContent(int s_nErrCode);
extern  int SYSMON_ParceEstopCode(int nEstopCode);
extern  int SYSMON_CheckJobExecLineIndex(void);
extern  int SYSC_LoadSystemConfigParameter(void);

extern  int MAIN_Initialize(void);

extern  int DSP_InitDisplay(void);
extern  int DSP_FinalizeDisplay(void);
extern  void DSP_DispDivider(void);

extern  void Fn_GetSystemTime(void);

extern  BOOL JOB_LoadJobToShmem(const char* pszModuleName, const DANDY_JOB_MEM* pJobMem);
extern  int  JOB_DumpJobShmem(int nOpt);
extern  int  JOB_DoJobAssemble(void);
extern  int  JOB_DoJobDisassemble(void);
extern  void JOB_GenerateMapFile(JOBASM_ASSEMBLER hAssembler);
extern  int  JOB_SaveJobToFile(int nOpt, const char* pszRcvSaveFileName);

///////////////////////////////////////
//
//  global variables
//

//return value & ID
extern  int g_coidTE;
extern  int g_coidSC;
extern  int g_coidRM;
extern  int g_coidTEalive;
extern  int g_coidSCalive;
extern  int g_chidRM;

extern  int g_retTEmsg;
extern  int g_retSCmsg;
extern  int g_retOpenShmemTE;
extern  int g_retOpenShmemSC;
extern  int g_retInitRM;

extern  char g_szSystemMode[SYSMODE_NAME_LEN];
extern  char g_szExecStat[EXECSTATE_NAME_LEN];
extern  char g_szWorkType[WORKTYPE_NAME_LEN];
extern  char g_szErrContent[ERROR_NAME_LEN];
extern  char g_szErrModeContent[ERROR_MODE_NAME_LEN];
extern  char g_szEstopContent[ESTOP_NAME_LEN];
extern  char g_szServContent[SERV_NAME_LEN];
extern  char g_szRMState[SIBLINGSTATE_NAME_LEN];
extern  char g_szTEState[SIBLINGSTATE_NAME_LEN];
extern  char g_szSCState[SIBLINGSTATE_NAME_LEN];

//flag
extern  int g_fSVCExecThRun;
extern  int g_fSysModeStatusThRun;
extern  int g_fSysErrorStatusThRun;
extern  int g_fSysIOStatusThRun;
extern  int g_fSysAliveThRun;
extern  int g_fSysStatThRun;
extern  int g_fVGA_DSPThRun;
extern  int g_fExtraRconSVCThRun;

extern  int g_fSVCExecThExit;
extern  int g_fSysModeStatusThExit;
extern  int g_fSysErrorStatusThExit;
extern  int g_fSysIOStatusThExit;
extern  int g_fSysStatCheckThExit;
extern  int g_fSysAliveThExit;
extern  int g_fVGA_DSPThExit;
extern  int g_fExtraRconSVCThExit;

extern  int g_fServiceStatus;           // check service proc routine active state
extern  int g_fInitRet[INIT_STEP_LEN];  // check init succeeded state by init step
extern  int g_fConsoleExitAct;          // check console exit action done
extern  int g_fArgAseembleDone;         // check prog argument job compile done
extern  int g_fAseembleDone;            // check for assembling done or not
extern  int g_fJobExecRun;              // check job exec state(event triggered)
extern  int g_fJobLoadDoneCheck;        // check job load done
extern  int g_fAssemError;              // check for assembling fail or not
extern  int g_fJobExecStepMode;         // check job exec state under step mode or not
extern  int g_nStepModeLineIndex;       // check step mode line index
extern  int g_fCallModeJobLoad;         // check call mode job load req. or external req.
extern  int g_fSavedJobLoad;            // check file saved job load done at init state
extern  int g_fJobExecStepFinalMode;
extern  int g_fJobStopRequest;
extern  int g_fRestartStepMode;

extern  int g_fErrorActiveState;        // flag for error active state
extern  int g_fErrorReset;              // flag for error reset state
extern  int g_fLoadStatFile;            // check statistics file load state

extern  int g_fHomeExecSVCAct;
extern  int g_fWireCutSVCAct;
extern  int g_fStepExecSVCAct;
extern  int g_fSkipBvarModifySVCAct;
extern  int g_fGapSkipBvarModifySVCAct;

extern  int g_fLeftVertSkip;     // Left  Vertical Weld Skip Set flag (1: skip, 0: not skip)
extern  int g_fRightVertSkip;    // Right Vertical Weld Skip Set flag (1: skip, 0: not skip)
extern  int g_fLeftCollarSkip;   // Left  Collar   Weld Skip Set flag (1: skip, 0: not skip)
extern  int g_fRightCollarSkip;  // Right Collar   Weld Skip Set flag (1: skip, 0: not skip)
extern  int g_fHorizontalSkip;   // Horizontal     Weld Skip Set flag (1: skip, 0: not skip)

extern  int g_nLeftVertGapSkipCond;     // Left  Vertical Weld Gap/Skip Set Condition
extern  int g_nRightVertGapSkipCond;    // Right Vertical Weld Gap/Skip Set Condition
extern  int g_nLeftCollarGapSkipCond;   // Left  Collar   Weld Gap/Skip Set Condition
extern  int g_nRightCollarGapSkipCond;  // Right Collar   Weld Gap/Skip Set Condition
extern  int g_nHorizontalGapSkipCond;   // Horizontal     Weld Gap/Skip Set Condition
extern  int g_nLeftBracketGapSkipCond;  // Left  Bracket  Weld Gap/Skip Set Condition
extern  int g_nRightBracketGapSkipCond; // Right Bracket  Weld Gap/Skip Set Condition
extern  int g_fErrorStop;

//handle
/* Thread Handle */
extern  THREAD_HANDLE       hSysCheck_Thread;
extern  THREAD_HANDLE       hSysError_Thread;
extern  THREAD_HANDLE       hSysAlive_Thread;
extern  THREAD_HANDLE       hVGA_Display_Thread;
extern  THREAD_HANDLE       hSVCCheck_Thread;
extern  THREAD_HANDLE       hSysIOCheck_Thread;
extern  THREAD_HANDLE       hExtraRconSVC_Thread;

/* SHM Handle */
extern  int g_hShmemTEStatus;
extern  int g_hShmemTE_Restart;
extern  int g_hShmemTE_Task_t;
extern  int g_hShmemSC;
extern  int g_hShm_SysStatus;
extern  int g_hShm_SysConfig;
extern  int g_hShm_SysParam;
extern  int g_hShmemJobRM;

//shared memory
extern  SHM_TE_STATUS*      g_pShmemTEStatus;
extern  SHM_RESTART*        g_pShmemTE_Restart;
extern  shm_mmi_task_t*     g_pShmemTE_Task_t;
extern  SHM_SC_SYSTEM*      g_pShmemSC;
extern  SHM_RM_SYSSTATUS*   g_pShm_SysStatus;
extern  SHM_RM_SYSCONFIG*   g_pShm_SysConfig;
extern  SHM_RM_SYSPARAM*    g_pShm_SysParam;
extern  SHM_DANDY_JOB*      g_pShmemJobRM;

//structure
extern  ARGUMENT_OPTION         g_Arg;
extern  ERROR_CODE_STACK        g_ErrCodeStack;
extern  RMGR_JOBCALL_STACK_DATA g_rgCmdCallStack[MAX_CALL_COUNT];
extern  RMGR_JOBRETURN_DATA     g_ReturnTarget[MAX_CALL_COUNT];

//packet
extern  RMGR_PACKET         RM_packet;
extern  RMGR_REPLY_PACKET   RM_reply_packet;
extern  MSG_INFO            info_msg;
extern  TE_MSG              TE_msg;
extern  SC_MSG              SC_msg;
extern  TE_REPLY            TE_reply;
extern  SC_REPLY            SC_reply;
extern  RMGR_JOB_LOAD_DATA  Job_msg_data;

//variable
extern  int     g_nWaitInitSec;
extern  int     g_nWaitConnSec;
extern  int     g_nSystemMode;
extern  int     g_nAssembleOpt;
extern  char*   g_rgpszJobTargetFiles[COMPILE_FILE_COUNT][PATH_NAME_BUFFER_SIZE];
extern  char    g_rgszLoadedJobModName[DANDY_JOB_MODULE_NAME_SIZE];
extern  int     g_nJobTargetFileCount;
extern  int     g_nConfigLoadOption;
extern  int     g_nCmdCallTop;
extern  int     g_nJobExecLineIdx;
extern  int     g_nCmdLoadCount;
extern  unsigned g_nLoadSizeRestartSHM;
extern  int     g_nHomeExecIndex;

extern  int     g_fBinSndHeaderDefine;
extern  int     g_fBinRcvHeaderDefine;
extern  unsigned long g_dwTotalBinLoadSize;
extern  unsigned long g_dwTotalBinRcvSize;
extern  unsigned long g_dwTotalBinCompLoadSize;
extern  unsigned long g_dwTotalBinCompRcvSize;
extern  int           g_nSendBodyDivCount;
extern  int           g_nRcvBodyDivCount;

extern  int     g_dwCmdRcvCount;
extern  int     g_dwTVaRcvCount;
extern  int     g_dwWvfRcvCount;
extern  int     g_dwSwfRcvCount;
extern  int     g_dwMwfRcvCount;
extern  int     g_dwEwfRcvCount;

extern  int     g_nEcatAutoResetCnt;

extern  char    g_szErrSysTime[SYSTIME_DATA_LEN];

extern  BYTE    g_fSkip;
extern  int     g_nSkipBvarIdx; 

extern  int     g_nTERunNextIdx;
extern  int     g_nJobLoadCnt;

typedef enum _RobotRunRequestTime
{
    ROBOT_RUN_REQ_TIME_AUTORUN = 0,
    ROBOT_RUN_REQ_TIME_DRYRUN,
    ROBOT_RUN_REQ_TIME_RESTART,

    ROBOT_RUN_REQ_COUNT
} RobotRunRequestTime;

#define         THRES_RUN_DUPLICATE_REQ_SEC   2

extern  unsigned long g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_COUNT];
extern  unsigned long g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_COUNT];
extern  unsigned long g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_COUNT];

extern  int g_nOldRunReqLineIdx[ROBOT_RUN_REQ_COUNT];

////////////////////////////////////////////////////////////////////

#if 0
extern  int     g_nGapSkipCond;
extern  int     g_nGapSkipBvarIdx; 
#endif

#pragma pack(push, 1)
typedef struct t_weld_coeff
{
	double 	dbA;
	double 	dbB;
	double 	dbC;
} WELD_COEFF;
#pragma pack(pop)

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

extern  CURRENT_TIME    curr_time;


//thread
extern  THREAD_ENTRY_TYPE DSP_VGADisplyThread(void* pParam);
extern  THREAD_ENTRY_TYPE SYSMON_SysAliveCheckThread(void* pParam);
extern  THREAD_ENTRY_TYPE SYSMON_SysStatusCheckThread(void* pParam);
extern  THREAD_ENTRY_TYPE SYSMON_SysErrorCheckThread(void* pParam);
extern  THREAD_ENTRY_TYPE SYSMON_SysIOCheckThread(void* pParam);
extern  THREAD_ENTRY_TYPE SYSMON_SysStatisticsCheckThread(void* pParam);
extern  THREAD_ENTRY_TYPE SVC_ExtraRobotControlSVCThread(void* pParam);

#endif  // end of __ROBOT_MGR_H__
