/////////////////////////////////////////////////////////////////////////////
//
//  robot_mgr.c: RobotManager Main
//                                            2013.04.11  Ryu SinWook


///////////////////////////////////////

#include "robotmgr_main.h"
#include "ipc_jobshm.h"
#include "service.h"
#include "statistics.h"

///////////////////////////////////////
#define VERBOSE_NAME    "RM_VERBOSE_"

///////////////////////////////////////
//Global_variables

int g_fSysModeStatusThRun   = RUN;
int g_fSysErrorStatusThRun  = RUN;
int g_fSysIOStatusThRun     = RUN;
int g_fSysAliveThRun        = RUN;
int g_fSysStatThRun         = RUN;
int g_fVGA_DSPThRun         = RUN;
int g_fExtraRconSVCThRun    = RUN;

int g_fSVCExecThExit        = FALSE;
int g_fSysModeStatusThExit  = FALSE;
int g_fSysErrorStatusThExit = FALSE;
int g_fSysIOStatusThExit    = FALSE;
int g_fSysStatCheckThExit   = FALSE;
int g_fSysAliveThExit       = FALSE;
int g_fVGA_DSPThExit        = FALSE;
int g_fSavedJobLoad         = FALSE;
int g_fExtraRconSVCThExit   = FALSE;

int g_fConsoleExitAct = FALSE;
int g_fInitRet[INIT_STEP_LEN] = {-1, -1, -1, -1, -1};
char g_szSystemMode[SYSMODE_NAME_LEN];
char g_szExecStat[EXECSTATE_NAME_LEN];
char g_szWorkType[WORKTYPE_NAME_LEN];
char g_szRMState[SIBLINGSTATE_NAME_LEN];
char g_szTEState[SIBLINGSTATE_NAME_LEN];
char g_szSCState[SIBLINGSTATE_NAME_LEN];

int g_retInitRM = FALSE;
int g_nSystemMode = MODE_STAT_ENTRY;
int g_fArgAseembleDone = FALSE;
int g_fAseembleDone = FALSE;

int g_fLoadStatFile = 0;

RMGR_PACKET                     RM_packet;
RMGR_REPLY_PACKET               RM_reply_packet;
MSG_INFO                        info_msg;
TE_MSG                          TE_msg;
SC_MSG                          SC_msg;
TE_REPLY                        TE_reply;
SC_REPLY                        SC_reply;
RMGR_JOB_LOAD_DATA              Job_msg_data;

ERROR_CODE_STACK                g_ErrCodeStack;

THREAD_HANDLE                   hSysCheck_Thread;
THREAD_HANDLE                   hSysError_Thread;
THREAD_HANDLE                   hSysIOCheck_Thread;
THREAD_HANDLE                   hSysAlive_Thread;
THREAD_HANDLE                   hSysStatCheck_Thread;
THREAD_HANDLE                   hVGA_Display_Thread;
THREAD_HANDLE                   hExtraRconSVC_Thread;

static BOOL _loc_ConsoleExitAct(int nCtrl);
static void _loc_SetWorkDirectory(void);

///////////////////////////////////////
// Functions

int             MSG_ConnectChannelServer(int nCoid);
int             MSG_CloseSiblingConnection(int nCoid, int nCoidAlive);
int             MSG_CloseRMInternalConnection(void);
int             MSG_CreateRMChannel(void);
int             SVC_InitRMService(void);
int             SHM_OpenSharedMemory(void);
int             SYSC_LoadSystemConfigParameter(void);
int             SHM_CreateSharedMemory(void);
int             SVC_InitTEService(void);
int             SVC_InitSCService(void);
int             SVC_ExitTEService(void);
int             SVC_ExitSCService(void);
int             SVC_ExecService(int fKeyIn);
int             DSP_FinalizeDisplay(void);
int             DSP_InitDisplay(void);
static  int     _loc_MAIN_Initialize(void);
static  int     _loc_MAIN_Finalize(void);
void            DSP_DispDivider(void);

static  int     _loc_PROC_WaitforOtherProcInitStatus(void);
static  void    _loc_ChangeConsoleDecoration(void);
static  void    _loc_CleanDirtyFiles(void);
static  int     _loc_PROC_WaitforOtherExitStatus(void);
static  void    _loc_RestoreConsoleDecoration(void);
static  int     _loc_CreateBlankJobFile(void);


/////////////////////////////////////////////////////////////////////////////
//
//  main()
//

int main(int nArgc, char* rgpszArgv[])
{
    ///////////////////////////////////
    //
    //  Parse Program Option
    //

    memset(&g_Arg, 0, sizeof(g_Arg));

    if (ParseArgument(nArgc, rgpszArgv, &g_Arg) == FALSE)
    {
        if(g_Arg.bHelp == FALSE)
        {
            VERBOSE_ERROR("Argument Parsing Fail!\n");
        }
        return EXIT_FAILURE;
    }
    
    ///////////////////////////////////
    //
    //  Initialize system
    //
    
    g_retInitRM = _loc_MAIN_Initialize();
    

    if(g_Arg.bManualInit == FALSE &&
       g_fConsoleExitAct == FALSE &&
       g_Arg.bSingleRun  == FALSE)
    {
        _loc_PROC_WaitforOtherProcInitStatus();
    }

    ///////////////////////////////////
    //
    //  Execute service
    //  - Keyin & External Input Available
    //  - Keyin Enable/Disable optional by agument(/k)

    if(SVC_ExecService(g_Arg.fKeyIn) == RESULT_ERROR)
    {
        VERBOSE_ERROR("Execute Service Step Fail\n");
        _loc_MAIN_Finalize();
        return EXIT_FAILURE;
    }

    ///////////////////////////////////
    //
    // In Case of Console Exit Act,
    // Skip Normal Finalize Process

    if(g_fConsoleExitAct == TRUE && g_Arg.fKeyIn == FALSE)
        goto CONSOLE_EXIT_ACT;
    else if(g_fConsoleExitAct == TRUE && g_Arg.fKeyIn == TRUE)
    {
         DANDY_SLEEP(500);
         goto CONSOLE_EXIT_ACT;
    }

    DANDY_SLEEP(1);
    
    ///////////////////////////////////
    //
    //  Finalize system
    //

    if(_loc_MAIN_Finalize() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Main Finalize Step Fail!\n");
        return EXIT_FAILURE;
    }

    ///////////////////////////////////
    //
    // Program Argument: Shutdown Option is TRUE
    //

    if (g_Arg.fShutdown == TRUE)
    {
        SVC_ShutdownSystem(SVC_OPT_SHUTDOWN);
    }

CONSOLE_EXIT_ACT:
    
    ///////////////////////////////////
    //
    // Wait for Console Act Done
    //

    while(g_fConsoleExitAct == TRUE)
    {
        DANDY_SLEEP(100);
    }

#if defined(_WIN32)
    system("pause");
#endif

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_MAIN_Initialize()
//

static int _loc_MAIN_Initialize(void)
{
    //static int iCnt, iIdx;
    static int s_ConnectRet[4];
    
    ///////////////////////////////////
    //
    // Change Console Decoration
    //

    _loc_ChangeConsoleDecoration();

    ///////////////////////////////////
    //
    // For DEBUG_MALLOC
    //

    DEBUG_InstallTrap();

    ///////////////////////////////////
    //
    // Set System mode ENTRY
    //

    g_nSystemMode = MODE_STAT_ENTRY;
    CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "ENTRY");

    ///////////////////////////////////
    //
    // Set Console Handle
    //

    DEBUG_SetConsoleHandler((DEBUG_CONSOLE_HANDLER) _loc_ConsoleExitAct);

    ///////////////////////////////////
    //
    // Set Non-Canonical Form
    //

	CIO_SetModeCanonical(FALSE); 

#if defined(_MSC_VER)
    ///////////////////////////////////
    //
    // Set Process Priority
    //

    THREAD_SetPriorityClass(ABOVE_NORMAL_PRIORITY_CLASS);
#endif

    ///////////////////////////////////
    //
    // Program Argument: Clean Dirty File
    //

    if(g_Arg.fClean == TRUE)
    {
        _loc_CleanDirtyFiles();
    }

    ///////////////////////////////////
    //
    // Configure display service
    //

    DSP_InitDisplay();

    ///////////////////////////////////
    //
    // Set No Service Request State to Display
    //

    CRT_strcpy(g_szServContent, SERV_NAME_LEN, "NO SERVICE REQUEST");

    ///////////////////////////////////
    //
    // Launch VGA Display Thread
    //

    hVGA_Display_Thread = THREAD_Create(DSP_VGADisplyThread,
                                        NULL,
                                        0,                 // auto stack size
                                        THREAD_PRIO_BELOW_NORMAL,// priority(0: normal)
                                        THREAD_DETACHED,   // detach thread
                                        THREAD_POLICY_RR); // round-robin
    
    if (hVGA_Display_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the VGA display thread...\n");

        exit(1);
    }
    
    g_fVGA_DSPThRun = RUN;

    ///////////////////////////////////
    //
    // Create RM Channel
    //

    VERBOSE_MESSAGE("--- Create Channel & Config Param Load ---\n\n");

    if(MSG_CreateRMChannel() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Create RM Channel Fail!\n");
        SVC_DefineErrorState(ON, SYS_ERR_INIT_RM);    //Not Created SHM

        return RESULT_ERROR;
    }
    
    ///////////////////////////////////
    //
    // Load system parameters
    //

    g_fInitRet[INIT_STEP0_LOAD_PARAM] = SYSC_LoadSystemConfigParameter();
    
    ///////////////////////////////////
    //
    // Create shared memory
    //

    g_fInitRet[INIT_STEP1_CREATE_SHM] = SHM_CreateSharedMemory();
    g_pShm_SysStatus->nSystemMode = MODE_STAT_ENTRY;
    g_pShm_SysStatus->nExecStat = EXEC_STAT_IDLE;
    g_pShm_SysStatus->nWorkType = WORK_TYPE_NONE;

    ///////////////////////////////////
    //
    //  set exec(work) directory & create blank job
    //
    if(g_fInitRet[INIT_STEP0_LOAD_PARAM] == RESULT_OK &&
       g_fInitRet[INIT_STEP1_CREATE_SHM] == RESULT_OK)
    {
        _loc_SetWorkDirectory();
        _loc_CreateBlankJobFile();
    }

    ///////////////////////////////////
    //
    // Connect to other proc channel
    //
    if(g_Arg.bSingleRun  == FALSE)
    {
        DANDY_SLEEP(100);
        if(g_fInitRet[INIT_STEP1_CREATE_SHM] == RESULT_OK)
        {
            VERBOSE_MESSAGE("------------- Channel Connecting ------------\n\n");
            s_ConnectRet[TE_CHANNEL_ID] = MSG_ConnectChannelServer(TE_CHANNEL_ID);
            s_ConnectRet[SC_CHANNEL_ID] = MSG_ConnectChannelServer(SC_CHANNEL_ID);
            s_ConnectRet[RM_CHANNEL_ID] = MSG_ConnectChannelServer(RM_CHANNEL_ID);
            
            if(s_ConnectRet[TE_CHANNEL_ID] == RESULT_OK &&
               s_ConnectRet[SC_CHANNEL_ID] == RESULT_OK &&
               s_ConnectRet[RM_CHANNEL_ID] == RESULT_OK)
            {
                g_fInitRet[INIT_STEP2_CONNECT_CH] = RESULT_OK;
            }
            else
            {
                g_fInitRet[INIT_STEP2_CONNECT_CH] = RESULT_ERROR;
            }
        }
    }
    else
    {
        VERBOSE_MESSAGE("------------- Channel Connecting ------------\n\n");
        s_ConnectRet[RM_CHANNEL_ID] = MSG_ConnectChannelServer(RM_CHANNEL_ID);

        if(s_ConnectRet[RM_CHANNEL_ID] == RESULT_OK)
        {
            g_fInitRet[INIT_STEP2_CONNECT_CH] = RESULT_OK;
        }
        else
        {
            g_fInitRet[INIT_STEP2_CONNECT_CH] = RESULT_ERROR;
        }
    }

    ///////////////////////////////////
    //
    // Set system mode: INIT state
    //

    DANDY_SLEEP(100);
    g_pShm_SysStatus->nSystemMode = MODE_STAT_INIT;
    g_nSystemMode = MODE_STAT_INIT;

    g_pShm_SysStatus->nEstopCode = SYS_ESTOP_NONE;
    if(g_pShm_SysStatus->fErrorState == OFF)
    {
        g_pShm_SysStatus->nErrCode = SYS_ERR_OK;
    }

    g_pShm_SysStatus->fEstopDone   = OFF;

    ///////////////////////////////////
    //
    // Init RM Services
    //

    g_fInitRet[INIT_STEP3_INIT_SERV] = SVC_InitRMService();
    
    ///////////////////////////////////
    //
    // Open shared memory
    //

    if(g_Arg.bSingleRun  == FALSE)
    {
        g_fInitRet[INIT_STEP4_OPEN_SHM]  = SHM_OpenSharedMemory();
    }
    else
    {
        g_fInitRet[INIT_STEP4_OPEN_SHM] = RESULT_OK;
    }

    ///////////////////////////////////
    //
    // Request TE, SC Initialize (in case of shm & channel valid state)
    //

    if(g_Arg.bSingleRun  == FALSE)
    {
        if(g_fInitRet[INIT_STEP1_CREATE_SHM] != RESULT_ERROR &&
           g_fInitRet[INIT_STEP2_CONNECT_CH] != RESULT_ERROR &&
           g_Arg.bManualInit == FALSE)
        {
            SVC_InitTEService();
            SVC_InitSCService();
        }
    }

    ///////////////////////////////////
    //
    // Launch System Alive Check Thread
    //

    if(g_Arg.bSingleRun  == FALSE)
    {
        hSysAlive_Thread = THREAD_Create(SYSMON_SysAliveCheckThread,
                                         NULL,
                                         0,                        // auto stack size
                                         THREAD_PRIO_BELOW_NORMAL, // priority
                                         THREAD_DETACHED,          // detach thread
                                         THREAD_POLICY_RR);        // round-robin

        if (hSysAlive_Thread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create the system alive check thread...\n");

            exit(1);
        }
    }

    g_fSysAliveThRun = RUN;

    ///////////////////////////////////
    //
    // Launch System Statistics Check Thread
    //

    hSysStatCheck_Thread = THREAD_Create(SYSMON_SysStatisticsCheckThread,
                                         NULL,
                                         0,                        // auto stack size
                                         THREAD_PRIO_BELOW_NORMAL, // priority
                                         THREAD_DETACHED,          // detach thread
                                         THREAD_POLICY_RR);        // round-robin

    if (hSysStatCheck_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the system statistics check thread...\n");

        exit(1);
    }

    g_fSysStatThRun = RUN;

    ///////////////////////////////////
    //
    // If One of Initialize Step is Fail, Define Error State
    //
#if 1
    if(g_fLoadStatFile != RESULT_OK)
    {
        SVC_DefineErrorState(ON, SYS_ERR_OPEN_STATISTICS_PARAM);
        VERBOSE_ERROR("Statistics Data File Open Fail!(%d)\n", g_fLoadStatFile);
    }
#endif
    if(g_fInitRet[INIT_STEP0_LOAD_PARAM] == RESULT_ERROR ||
       g_fInitRet[INIT_STEP1_CREATE_SHM] == RESULT_ERROR ||
       g_fInitRet[INIT_STEP2_CONNECT_CH] == RESULT_ERROR ||
       g_fInitRet[INIT_STEP3_INIT_SERV]  == RESULT_ERROR ||
       g_fInitRet[INIT_STEP4_OPEN_SHM]   == RESULT_ERROR)
    {
        SVC_DefineErrorState(ON, SYS_ERR_INIT_RM);
        if(g_fInitRet[INIT_STEP0_LOAD_PARAM] == RESULT_ERROR)
        {
            SVC_DefineErrorState(ON, SYS_ERR_INIT_CONF_LOAD);
        }

        if(g_Arg.fDebug == TRUE)
            VERBOSE_WARNING("[DEBUG] ParamLoad: %d, SharedMemoryCreate: %d,"
                   "ConnectChannel: %d,\n RMServInit: %d, OpenSharedMem: %d\n",
                   g_fInitRet[INIT_STEP0_LOAD_PARAM],
                   g_fInitRet[INIT_STEP1_CREATE_SHM],
                   g_fInitRet[INIT_STEP2_CONNECT_CH],
                   g_fInitRet[INIT_STEP3_INIT_SERV],
                   g_fInitRet[INIT_STEP4_OPEN_SHM]);
    }

    return (g_fInitRet[INIT_STEP0_LOAD_PARAM] == RESULT_ERROR ||
            g_fInitRet[INIT_STEP1_CREATE_SHM] == RESULT_ERROR ||
            g_fInitRet[INIT_STEP2_CONNECT_CH] == RESULT_ERROR ||
            g_fInitRet[INIT_STEP3_INIT_SERV]  == RESULT_ERROR ||
            g_fInitRet[INIT_STEP4_OPEN_SHM]   == RESULT_ERROR) ?
                RESULT_ERROR : RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_ConsoleExitAct()
//

static BOOL _loc_ConsoleExitAct(int nCtrl)
{
    DANDY_UNUSED_PARAM(nCtrl);
    
    VERBOSE_NOTIFY("Received Console Exit Event!!\n");
    VERBOSE_NOTIFY("Now Finalizing System....\n");
    
    ///////////////////////////////////
    //
    // Stop Alive Check Thread Routine
    //

    g_fSysAliveThRun = STOP;

    ///////////////////////////////////
    //
    // Console Exit Act Start
    //

    g_fConsoleExitAct = TRUE;
    g_fServiceStatus = STOP;

    ///////////////////////////////////
    //
    // Exit Command to Sibling Process
    //
    if(g_Arg.bSingleRun == FALSE)
    {
        SVC_ExitTEService();
        SVC_ExitSCService();
    }

    DANDY_SLEEP(500);
    
    ///////////////////////////////////
    //
    // Stop Thread Routine
    //   - System Mode State, I/O Status Check, Service Exec, VGA Display

    g_fSysModeStatusThRun  = STOP;
    g_fSysErrorStatusThRun = STOP;
    g_fSysIOStatusThRun    = STOP;
    g_fSVCExecThRun        = STOP;
    g_fSysStatThRun        = STOP;
    g_fVGA_DSPThRun        = STOP;
    g_fExtraRconSVCThRun   = STOP;

    MSG_SendPulse(g_coidRM, RMGR_SERV_SYSEXIT, 0);

	VERBOSE_VERBOSE("RM Service Quit\n"); 

    ///////////////////////////////////
    //
    // Normal Finalize Process
    //

    if(_loc_MAIN_Finalize() == -1)
    {
        VERBOSE_ERROR("Main Finalize Step Fail!\n");
        return EXIT_FAILURE;
    }
    else
    {
        VERBOSE_NOTIFY("Console Exit Act Done! Goodbye~~\n");
    }

    ///////////////////////////////////
    //
    // Console Exit Act Done
    //

    g_fConsoleExitAct = FALSE;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_MAIN_Finalize()
//

static int _loc_MAIN_Finalize(void)
{
    static int nRet, iCnt;

    ///////////////////////////////////
    //
    // Stop Alive Check Thread Routine
    //

    g_fSysAliveThRun = STOP;

    ///////////////////////////////////
    //
    // Save Current Loaded Job File
    //
    if(g_pszLoadedJobFileName != NULL)
    {
        //JOB_SaveJobToFile(JOBASM_AF_DANDY1996, g_pszLoadedJobFileName);
        JOB_SaveJobToFile(0, g_pszLoadedJobFileName);

        SVC_ConstVarFileHandle(OPTION_WRITE);
    }

    ///////////////////////////////////
    //
    // Save Statistics Data to File
    //

    Fn_WriteStatisticsData(OPT_NULL);
    nRet = SVC_SaveStatisticsDataToFile(ROBOT_0_INDEX, OPT_NULL);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Statistics Data Write Done!\n");
    }

    ///////////////////////////////////
    //
    // Wait for Sibling Process Exit
    //

    if(g_hShm_SysStatus != 0 && g_hShm_SysStatus != -1)
    {
        // Set system mode: TERMINATE state
        g_pShm_SysStatus->nSystemMode = MODE_STAT_TERMINATE;
        g_nSystemMode = MODE_STAT_TERMINATE;

        // Wait for TE, SC Exit status
        if(g_Arg.bSingleRun  == FALSE)
        {
            if((g_pShm_SysStatus->fExitProcTE == FALSE ||
               g_pShm_SysStatus->fExitProcSC == FALSE) &&
               g_fConsoleExitAct != TRUE)
            {
                _loc_PROC_WaitforOtherExitStatus();
            }
        }
        
        VERBOSE_MESSAGE("Ready to Exit service!!\n");
    }
   
    ///////////////////////////////////
    //
    // Disassemble job data
    //

    if(g_Arg.bAssemble == TRUE || g_fAseembleDone == TRUE)
    {
        nRet = JOB_DoJobDisassemble();
    }

    DANDY_SLEEP(100);

    ///////////////////////////////////
    //
    // Stop Thread Routine
    //   -System Mode State, I/O Status Check, Service Exec

    g_fSysModeStatusThRun  = STOP;
    g_fSysErrorStatusThRun = STOP;
    g_fSysIOStatusThRun    = STOP;
    g_fSVCExecThRun        = STOP;
    g_fSysStatThRun        = STOP;
    g_fExtraRconSVCThRun   = STOP;

    DANDY_SLEEP(100);
    
    ///////////////////////////////////
    //
    // Close Internal Connection
    //

    MSG_CloseRMInternalConnection();

    ///////////////////////////////////
    //
    // Close RM Channel
    //

    nRet = MSG_CloseRMChannel();
    
    ///////////////////////////////////
    //
    // Before shared memory destroy step, check error state
    //

    if(nRet == -1)
    {
        SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_RM);
    }

    DANDY_SLEEP(600);   //wait for thread exit
    
    ///////////////////////////////////
    //
    // Init Memory for Configuration
    //

    SYSC_ClearConfig();
    
    ///////////////////////////////////
    //
    // Stop Display Thread Routine
    //

    g_fVGA_DSPThRun =  STOP;


    ///////////////////////////////////
    //
    // Destroy the Shared Memory
    //

    nRet = SHM_DestroyShmem();

    VERBOSE_MESSAGE("---- All Services are Finalized Done!! ----\n");
    DSP_DispDivider();
    VERBOSE_VERBOSE("Wait for thread exit state...\n");
    
    DANDY_SLEEP(100);

    ///////////////////////////////////
    //
    // Finalize Verbose & VGA
    //

    DSP_FinalizeDisplay();

    ///////////////////////////////////
    //
    // Reset Input mode to Canonical Mode
    //

    CIO_SetModeCanonical(TRUE); 

    DANDY_SLEEP(100);

    if(g_Arg.bSingleRun  == TRUE)
    {
        g_fSVCExecThExit  = TRUE;
        g_fSysAliveThExit = TRUE;
    }

    ///////////////////////////////////
    //
    // Wait for thread routine exit state
    //

    while(g_fSVCExecThExit        == FALSE ||
          g_fSysErrorStatusThExit == FALSE ||
          g_fSysModeStatusThExit  == FALSE ||
          g_fSysIOStatusThExit    == FALSE ||
          g_fSysAliveThExit       == FALSE ||
          g_fSysStatCheckThExit   == FALSE ||
          g_fVGA_DSPThExit        == FALSE ||
          g_fExtraRconSVCThExit   == FALSE)
    {
        iCnt++;
        if(g_Arg.fDebug == TRUE)
        {
            printf("[fExit] SVCExecTh: %d, SysModeTh: %d, SysIOTh: %d, SysErr: %d"
                   "SysAliveTh: %d, StatCheckTh: %d, VGA_DSPTh: %d, ExtraSVC: %d\n",
                   g_fSVCExecThExit,
                   g_fSysModeStatusThExit,
                   g_fSysIOStatusThExit,
                   g_fSysErrorStatusThExit,
                   g_fSysAliveThExit,
                   g_fSysStatCheckThExit,
                   g_fVGA_DSPThExit,
                   g_fExtraRconSVCThExit);
        }
        else
        {
            ;
        }
        DANDY_SLEEP(100);

        if(iCnt >= 100 ||
          (g_fSVCExecThExit        == TRUE &&
           g_fSysModeStatusThExit  == TRUE &&
           g_fSysIOStatusThExit    == TRUE &&
           g_fSysErrorStatusThExit == TRUE &&
           g_fSysAliveThExit       == TRUE &&
           g_fSysStatCheckThExit   == TRUE &&
           g_fVGA_DSPThExit        == TRUE &&
           g_fExtraRconSVCThExit   == TRUE))
            break;
    }
    
    ///////////////////////////////////
    //
    // Thread Kill
    //

    THREAD_Kill(hSysCheck_Thread);
    THREAD_Kill(hSysError_Thread);
    THREAD_Kill(hSysStatCheck_Thread);
    THREAD_Kill(hSysIOCheck_Thread);
    THREAD_Kill(hSysAlive_Thread);
    THREAD_Kill(hSVCCheck_Thread);
    THREAD_Kill(hVGA_Display_Thread);
    THREAD_Kill(hExtraRconSVC_Thread);

    ///////////////////////////////////
    //
    // For Check Memory Leak
    //

    DEBUG_DumpLeakMemory( );

    ///////////////////////////////////
    //
    // Restore Console Decoration
    //

    _loc_RestoreConsoleDecoration();

    return (nRet == RESULT_ERROR) ? RESULT_ERROR : RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SVC_ExecJobArgument()
//

static int _loc_SVC_ExecJobArgument(void)
{
    int nResult = 0;
    
    if(g_Arg.bAssemble == TRUE)
    {
        // assemble multiple .pgm files as a .job file
        nResult = JOB_DoJobAssemble();

        if(nResult == RESULT_OK)
            g_fArgAseembleDone = TRUE;
    }

    if(g_Arg.bOutFile == TRUE)
    {
        // dump the job of the shared memory for the sibling (TE)
        nResult = JOB_DumpJobShmem(0);  // Option 0: All, Option 1: cmd
    }

    if(g_Arg.bDisassem == TRUE)
    {
        nResult = JOB_DoJobDisassemble();
    }

    return nResult;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SVC_InitRMService()
//

int SVC_InitRMService(void)
{
    int nResult;
    
    VERBOSE_MESSAGE("-----------Initialize RM Services ----------\n\n");

    ///////////////////////////////////
    //
    // Launch System Monitoring Thread
    //

    // Launch System Status Check Thread
    hSysCheck_Thread = THREAD_Create(SYSMON_SysStatusCheckThread,
                                     NULL,
                                     0,                 // auto stack size
                                     //THREAD_PRIO_ABOVE_NORMAL,// priority
                                     //THREAD_PRIO_REALTIME,// priority
                                     140,
                                     THREAD_DETACHED,   // detach thread
                                     THREAD_POLICY_RR); // round-robin
    
    if (hSysCheck_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the system mode status check thread...\n");

        exit(1);
    }

    // Launch System Error Check Thread
    hSysError_Thread = THREAD_Create(SYSMON_SysErrorCheckThread,
                                     NULL,
                                     0,                 // auto stack size
                                     //THREAD_PRIO_ABOVE_NORMAL,// priority
                                     //THREAD_PRIO_REALTIME,// priority
                                     140,
                                     THREAD_DETACHED,   // detach thread
                                     THREAD_POLICY_RR); // round-robin
    
    if (hSysError_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the system error status check thread...\n");

        exit(1);
    }

    // Launch System I/O Status Check Thread
    hSysIOCheck_Thread = THREAD_Create(SYSMON_SysIOCheckThread,
                                       NULL,
                                       0,                 // auto stack size
                                       THREAD_PRIO_ABOVE_NORMAL,// priority
                                       THREAD_DETACHED,   // detach thread
                                       THREAD_POLICY_RR); // round-robin

    if (hSysIOCheck_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the system I/O status check thread...\n");

        exit(1);
    }

    // Launch Extra Robot Control Service Thread
    hExtraRconSVC_Thread = THREAD_Create(SVC_ExtraRobotControlSVCThread,
                                         NULL,
                                         0,                 // auto stack size
                                         THREAD_PRIO_NORMAL,// priority
                                         THREAD_DETACHED,   // detach thread
                                         THREAD_POLICY_RR); // round-robin

    if (hExtraRconSVC_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create the extra robot control thread...\n");

        exit(1);
    }

    g_fSysModeStatusThRun  = RUN;
    g_fSysErrorStatusThRun = RUN;
    g_fSysIOStatusThRun    = RUN;
    g_fSVCExecThRun        = RUN;
    g_fExtraRconSVCThRun   = RUN;

    nResult = _loc_SVC_ExecJobArgument();
    
    if(g_pszLoadedJobFileName != NULL)
    {
        g_fSavedJobLoad = TRUE;

        if(stricmp(g_pszLoadedJobFileName, g_pszNewCreatedJobFileName) != 0)
        {
            SVC_LoadJobData(g_pszLoadedJobFileName, JOBASM_AF_DANDY1996);
        }
        
        g_pShm_SysStatus->nExecStat = EXEC_STAT_PREPARATION;

        DANDY_SLEEP(100);
        SVC_ConstVarFileHandle(OPTION_READ);

        g_fSavedJobLoad = FALSE;
    }
    
    return nResult;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadSystemConfigParameter()
//

int SYSC_LoadSystemConfigParameter(void)
{
    static int nResult;

    char szConfigFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szUserParamFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szStatisticsFileName[PATH_NAME_BUFFER_SIZE] = "";

    ///////////////////////////////////
    //
    // Set Config File Path & Name
    //

#if defined (__QNXNTO__)
    memcpy(szConfigFileName,     DEF_QNX_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
    memcpy(szUserParamFileName,  DEF_QNX_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
    memcpy(szStatisticsFileName, DEF_QNX_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
#else
    memcpy(szConfigFileName,     DEF_WIN_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
    memcpy(szUserParamFileName,  DEF_WIN_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
    memcpy(szStatisticsFileName, DEF_WIN_WORK_DIR_NAME, PATH_NAME_BUFFER_SIZE);
#endif
    CRT_strcat(szConfigFileName,     PATH_NAME_BUFFER_SIZE, DEF_SYSCONFIG_FILENAME);
    CRT_strcat(szUserParamFileName,  PATH_NAME_BUFFER_SIZE, DEF_USERPARAM_FILENAME);
    CRT_strcat(szStatisticsFileName, PATH_NAME_BUFFER_SIZE, DEF_STATISTIC_FILENAME);

    ///////////////////////////////////
    //
    // Load Configurations
    //

    nResult = 0;

    // If Prog. Argument Ignore is TRUE, Load Default Parameter
    if (g_Arg.fCfIgnore == TRUE)
    {
        SYSC_LoadConfigDefault();
    }
    else
    {
        // Load Config Parameter by Config File
        if(g_Arg.bManualConfig == TRUE)
        {
            nResult = SYSC_LoadConfig(g_Arg.pszConfigName,    OPTION_SYSCONFIG_LOAD);
            nResult = SYSC_LoadConfig(DEF_USERPARAM_FILENAME, OPTION_USERPARAM_LOAD);
            nResult = SYSC_LoadConfig(DEF_STATISTIC_FILENAME, OPTION_STATISTIC_LOAD);
        }
        else
        {
            nResult = SYSC_LoadConfig(szConfigFileName, OPTION_SYSCONFIG_LOAD);            
            if(g_fConfigLoadCheck != RESULT_OK)
            {
                goto CONF_LOAD_FAIL;
            }

            nResult = SYSC_LoadConfig(szUserParamFileName, OPTION_USERPARAM_LOAD);
            if(g_fConfigLoadCheck != RESULT_OK)
            {
                goto CONF_LOAD_FAIL;
            }

            SYSC_LoadConfig(szStatisticsFileName, OPTION_STATISTIC_LOAD);
        }

CONF_LOAD_FAIL:
        // If Result is Fail, Load Default Parameter
        if (nResult != RESULT_OK || g_fConfigLoadCheck != RESULT_OK)
        {
            VERBOSE_ERROR("Fail to load the config, "
                          "load default configurations\n");

            SYSC_LoadConfigDefault();

            if(g_fConfigLoadCheck != RESULT_OK)
            {
                return RESULT_ERROR;
            }
        }
    }

    // If Prog. Argument Robot Config is TRUE, Load Specific Robot Parameter
    if (g_Arg.fRobotConfig == TRUE)
    {
        nResult = SYSC_ResetRobotInfo(g_Arg.nRobotCount,
                                      g_Arg.rgpszRobotTypeName,
                                      g_Arg.rgnStartAxis);

        // If Result is Fail, Load Default Parameter
        if (nResult == -1)
        {
            VERBOSE_ERROR("Fail to reset robot config, "
                          "load default configurations\n");

            SYSC_LoadConfigDefault();
        }
    }

    // Set Trajectory Scan Time by Prog. Argument
    if (g_Arg.fTrajTime == TRUE)
    {
        SYSC_SetTrajUpdateTime(g_Arg.nTrajTime);
    }

    // Set I/O Scan Time by Prog. Argument
    if (g_Arg.fIoTime == TRUE)
    {
        SYSC_SetIoUpdateTime(g_Arg.nIoTime);
    }

    // If Prog. Argument Welder Config is TRUE, Load Specific Welder Parameter
    if (g_Arg.fWelder == TRUE)
    {
        nResult = SYSC_ResetWelderInfo(g_Arg.nWelderCount,
                                       g_Arg.rgnWelderRobot,
                                       g_Arg.rgpszWelderTypeName,
                                       g_Arg.rgnWelderBoard);
        
        // If Result is Fail, Load Default Parameter
        if (nResult == -1)
        {
            VERBOSE_ERROR("Fail to reset welder config, "
                          "load default welder configurations\n");

            SYSC_LoadConfigDefault();
        }
    }

    // If Prog. Argument WeldMap Config is TRUE, Load Specific WeldMap Parameter
    if (g_Arg.fWeldMap == TRUE)
    {
        //SVCWD_SetWeldMapFileName(options.pszWeldMapFile);
    }

    // If Prog. Argument Sensor Config is TRUE, Load Specific Sensor Parameter
    if (g_Arg.fSensorConfig == TRUE)
    {
       /* SYSC_ResetSensorInfo(g_Arg.nSensorCount, g_Arg.rgnSensorRobot,
                             g_Arg.rgpszSensorTypeName,
                             g_Arg.rgnSensorSampTime,
                             g_Arg.rgpszSensorAddr);*/
    }

    //SYSC_SetSysIoStyle(g_Arg.nSysIoStyle);

    // Set Exec Locale
    SYSC_ExecLocale();

    return nResult;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_PROC_WaitforOtherProcInitStatus()
//      - Wait for Other Proc Init Status

static int _loc_PROC_WaitforOtherProcInitStatus(void)
{
    int nForPrintTE;
    int nForPrintSC;
    int nTimeLimit;
    static unsigned s_nWaitExitmsec;

    nTimeLimit = 0;
    nForPrintTE = 0;    //Prevent Continuous Print
    nForPrintSC = 0;    //Prevent Continuous Print

    ///////////////////////////////////
    //
    // Convert sec Unit to msec (Prog. Argument Unit: sec)
    //

#if defined(__QNXNTO__)
    s_nWaitExitmsec = g_nWaitInitSec * 60;
#endif
#if defined(_WIN32)
    s_nWaitExitmsec = g_nWaitInitSec * 80;
#endif

    VERBOSE_MESSAGE("Wait for TE & SC Processes are Init Status..\n");
    VERBOSE_MESSAGE("For %d sec...\n", g_nWaitInitSec);
    DSP_DispDivider();

    ///////////////////////////////////
    //
    // Check SHM Init Flag: 
    // If Both Sibling Processes are Not Activated, Skip Wait Process

    while((g_pShm_SysStatus->fInitProcTE == FALSE ||
           g_pShm_SysStatus->fInitProcSC == FALSE) &&
           nTimeLimit <= (int) s_nWaitExitmsec &&
           g_fConsoleExitAct != TRUE)
    {
        DANDY_SLEEP(10);

        // Check TE Init State
        if(g_pShm_SysStatus->fInitProcTE == TRUE && nForPrintTE ==0)
        {
            VERBOSE_VERBOSE("%s Ready to start service!(Init flag: %d)\n\n",
                            TE_PROCESS_NAME, g_pShm_SysStatus->fInitProcTE);
            nForPrintTE++;
        }
        // Check SC Init State
        if(g_pShm_SysStatus->fInitProcSC == TRUE && nForPrintSC == 0)
        {
            VERBOSE_VERBOSE("%s Ready to start service!(Init flag: %d)\n\n",
                            SC_PROCESS_NAME, g_pShm_SysStatus->fInitProcSC);
            nForPrintSC++;
        }

        nTimeLimit++;
        
        // If Time Limit is Exceeded
        if(nTimeLimit > (int) s_nWaitExitmsec)
        {
            if(g_pShm_SysStatus->fInitProcTE == FALSE)
            {
                SVC_DefineErrorState(ON, SYS_ERR_INIT_TE);
                VERBOSE_ERROR("TE Initialize Fail!\n");
            }
            if(g_pShm_SysStatus->fInitProcSC == FALSE)
            {
                SVC_DefineErrorState(ON, SYS_ERR_INIT_SC);
                VERBOSE_ERROR("SC Initialize Fail!\n");
            }
            VERBOSE_WARNING("Initialize Time Limit is Exceeded!\n");
            break;
        }

        if(g_fConsoleExitAct == TRUE)
        {
            return 0;
        }
    }

    // Check Elapsed Time
#if defined(__QNXNTO__)
    VERBOSE_MESSAGE("elapsed %3.1f sec...\n", (float) nTimeLimit/60);
#endif
#if defined(_WIN32)
    VERBOSE_MESSAGE("elapsed %3.1f sec...\n", (float) nTimeLimit/80);
#endif

    DSP_DispDivider();

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_PROC_WaitforOtherExitStatus()
//

static int _loc_PROC_WaitforOtherExitStatus(void)
{
    int nForPrintTE;
    int nForPrintSC;
    int nTimeLimit;
    static unsigned s_nWaitInitmsec;

    nTimeLimit = 0;
    nForPrintTE = 0;    //Prevent Continuous Print
    nForPrintSC = 0;    //Prevent Continuous Print

    ///////////////////////////////////
    //
    // Convert sec Unit to msec (Prog. Argument Unit: sec)
    //

#if defined(__QNXNTO__)
    s_nWaitInitmsec = g_nWaitInitSec * 60;
#endif
#if defined(_WIN32)
    s_nWaitInitmsec = g_nWaitInitSec * 80;
#endif

    ///////////////////////////////////
    //
    //  Wait for Other Process Exit Status
    //

    // Check SHM Open: 
    // If Both Sibling Processes are Not Activated, Skip Wait Process

    if(g_retOpenShmemTE != -1 || g_retOpenShmemSC != -1)
    {
        if(g_fConsoleExitAct != TRUE)
        {
            VERBOSE_MESSAGE("Wait for TE & SC Processes are Exit Status..\n");
            VERBOSE_MESSAGE("For %d sec...\n", g_nWaitInitSec);
            DSP_DispDivider();
        }

        // Check SHM Exit Flag: 
        while((g_pShm_SysStatus->fExitProcTE == FALSE ||
               g_pShm_SysStatus->fExitProcSC == FALSE) &&
               nTimeLimit <= (int) s_nWaitInitmsec &&
               g_fConsoleExitAct != TRUE)
        {
            DANDY_SLEEP(10);
            
            // Check TE Exit State
            if(g_pShm_SysStatus->fExitProcTE == TRUE && nForPrintTE == 0)
            {
                VERBOSE_VERBOSE("%s Ready to exit service! (Exit flag: %d)\n",
                                TE_PROCESS_NAME, g_pShm_SysStatus->fExitProcTE);
                nForPrintTE++;
            }
            // Check SC Exit State
            if(g_pShm_SysStatus->fExitProcSC == TRUE && nForPrintSC == 0)
            {
                VERBOSE_VERBOSE("%s Ready to exit service! (Exit flag: %d)\n",
                                SC_PROCESS_NAME, g_pShm_SysStatus->fExitProcSC);
                nForPrintSC++;
            }

            nTimeLimit++;

            // If Time Limit is Exceeded
            if(nTimeLimit > (int) s_nWaitInitmsec)
            {
                if(g_pShm_SysStatus->fExitProcTE == FALSE)
                {
                    SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_TE);
                    VERBOSE_ERROR("TE Finalize Fail!\n");
                }
                if(g_pShm_SysStatus->fExitProcSC == FALSE)
                {
                    SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_SC);
                    VERBOSE_ERROR("SC Finalize Fail!\n");
                }

                VERBOSE_WARNING("Finalize Time Limit is Exceeded!\n");
                break;
            }

            if(g_fConsoleExitAct == TRUE)
            {
                return 0;
            }
        }
    }
    
    // Check Elapsed Time
#if defined(__QNXNTO__)
    VERBOSE_MESSAGE("elapsed %3.1f sec...\n", (float) nTimeLimit/60);
#endif
#if defined(_WIN32)
    VERBOSE_MESSAGE("elapsed %3.1f sec...\n", (float) nTimeLimit/80);
#endif
    DSP_DispDivider();

    return 0;
}



/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_CreateBlankJobFile()
//

static int _loc_CreateBlankJobFile(void)
{
    FILE*   fp = 0;
    char szJobDir[PATH_NAME_BUFFER_SIZE] = "./job/";
    char szNewCreatedJobFileName[PATH_NAME_BUFFER_SIZE] = "";

    memcpy(szJobDir, g_pszJobDir, strlen(g_pszJobDir)+1);

    memcpy(szNewCreatedJobFileName, szJobDir, strlen(szJobDir)+1);
    CRT_strcat(szNewCreatedJobFileName, PATH_NAME_BUFFER_SIZE, g_pszNewCreatedJobFileName);
    remove(szNewCreatedJobFileName);

#if defined(__QNXNTO__)
    fp = fopen(szNewCreatedJobFileName, "wt");
#else
    errno = fopen_s(&fp, szNewCreatedJobFileName, "wt");
#endif
    if (fp == NULL)
    {
        VERBOSE_ERROR("Cannot open Newly Created file for write : '%s'\n",
                      szNewCreatedJobFileName);
    
        SVC_DefineErrorState(ON, SVC_ERR_BLANK_JOB_OPEN);
        return SVC_ERR_BLANK_JOB_OPEN;
    }

    fclose(fp);
    
    return RESULT_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SetWorkDirectory()
//

char szWorkDir[PATH_NAME_BUFFER_SIZE] = "./";
char szChangeWorkDirCmd[PATH_NAME_BUFFER_SIZE] = "cd";

static void _loc_SetWorkDirectory(void)
{
#if 0
    // Absolute path define
#if defined(_WIN32)
    GetCurrentDirectory(sizeof(szJobDir), szJobDir);
#else
    getcwd(szJobDir, sizeof(szJobDir));
#endif
#endif
    memcpy(szWorkDir, g_pszWorkDir, strlen(g_pszWorkDir)+1);

    // Relative path define
    CRT_strcat(szChangeWorkDirCmd, 4, " ");
    CRT_strcat(szChangeWorkDirCmd, PATH_NAME_BUFFER_SIZE, szWorkDir);
    
    VERBOSE_VERBOSE("Change Directory (cmd: %s)\n", szChangeWorkDirCmd);
    system(szChangeWorkDirCmd);

    system("pwd");
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_CleanDirtyFiles()
//

static void _loc_CleanDirtyFiles(void)
{
    // Clear ECHO/VERBOSE Files
    VERBOSE_CleanDirtyFile(VERBOSE_NAME);

#if defined(_WIN32)
    //...
#else
    // Clear Shared Memory
    unlink(SHM_RM_SYSSTATUS_NAME);
    unlink(SHM_RM_SYSCONFIG_NAME);
    unlink(SHM_DANDY_JOB_NAME);
#endif
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_ChangeConsoleDecoration()
//

#if defined(_WIN32)
static HICON s_hIconSmall, s_hIconBig;
#endif

static void _loc_ChangeConsoleDecoration(void)
{
#if defined(_WIN32)
    HICON hIconFrame;

    hIconFrame = LoadIcon(GetModuleHandle(NULL), MAKEINTRESOURCE(IDR_MAINFRAME));
    TERM_SetCurrentConsoleIcon(hIconFrame, hIconFrame, &s_hIconSmall, &s_hIconBig);
    DestroyIcon(hIconFrame);
#endif
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_RestoreConsoleDecoration()
//

static void _loc_RestoreConsoleDecoration(void)
{
#if defined(_WIN32)
    TERM_SetCurrentConsoleIcon(s_hIconSmall, s_hIconBig, NULL, NULL);
#endif
}
