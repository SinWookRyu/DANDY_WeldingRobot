#include "servocon_main.h"
#include "service.h"


////////////////////////////////////////////////////////////////////////////////
//
// global variable
//

BOOL g_fExit               = FALSE;  // flag of SC process exit
BOOL g_fInit_Internal      = FALSE;  // flag of SC(internal) initialization
BOOL g_fOpenSysStatusShmRm = FALSE;  // flag of open RM_SysStatusSHM
BOOL g_fOpenSysConfigShmRm = FALSE;  // flag of open RM_SysConfigSHM
BOOL g_fOpenSysParamShmRm  = FALSE;  // flag of open RM_SysParamSHM
BOOL g_fOpenShmSc          = FALSE;  // flag of open SC_SHM
BOOL g_fOpenShmTeStatus    = FALSE;  // flag of open TE_SHM
BOOL g_fOpenShmTeTask      = FALSE;  // flag of open TE_SHM
BOOL g_fLoopGo             = TRUE;   // flag of while loop
BOOL g_fEcat               = FALSE;  // flag of EtherCAT Master state
BOOL g_fConfigLoad         = FALSE;  // config file load state
BOOL g_fSetPosFuncActive   = FALSE;  // set position function active state
BOOL g_fMainExitDone       = FALSE;

int g_coidRM      = INVALID_COID;    // connection identifier(to RM)
int g_coidTETime  = INVALID_COID;    // connection identifier(to TE Time)
int g_coidTE      = INVALID_COID;    // connection identifier(to TE)
int g_coidSC      = INVALID_COID;    // connection identifier(to SC)
int g_chidSC      = INVALID_CHID;    // service channel identifier
int g_chidSCTime  = INVALID_CHID;    // timer channel identifier
int g_coidSCTime  = INVALID_COID;    // connection identifier(to SC Time)
int g_rcvidSC     = INVALID_RCVID;   // receive identifier
int g_rcvidDevice = INVALID_RCVID;   // receive identifier of deviece execution
int g_nTimer = -1;           // timer number
int g_nTime_Limit = 0;       // time limit for SC init.
char g_chPrintLev = 'v';     // print level ('v', 'm', 'w', 'e')
int g_hTimer;
int g_fSharedMemRMOpen = OFF;

// set variable for PDO write
int g_nCtrlWord[ROB_AXIS_COUNT];             // control word for PDO write
double g_dbTrg_Pos[ROB_AXIS_COUNT];          // target position of motor for PDO write
int g_nTrg_Pulse[ROB_AXIS_COUNT];            // target pulse of motor
int g_nPhysicalOut[ROB_AXIS_COUNT];          // physical output value

// get variable for PDO read
double g_dbAct_Pos[ROB_AXIS_COUNT];          // actual position of motor for PDO read
int g_nAct_Pulse[ROB_AXIS_COUNT];            // actual pulse of motor
int g_nErrCodeServo[ROB_AXIS_COUNT];         // error code of servo-pack for PDO read
extern unsigned short g_nReadStatusValue[ROB_AXIS_COUNT];// status word of PDO

// get variable for servo state
int g_nErrorRegister[ROB_AXIS_COUNT];        // check error register of motor
int g_nServoState[ROB_AXIS_COUNT];           // state of servo on/off 
int g_nServoOnCmdState[ROB_AXIS_COUNT];      // state of servo on/off 
double g_dbActTorque[ROB_AXIS_COUNT];        // check actual torque of motor, Unit: %

ECAT_HANDLE g_hMaster = ECAT_NULL; // EtherCAT master handle

////////////////////////////////////////////////////////////////////////////////
//
// function
//
static int _loc_MAIN_Init_Internal(void);          // SC internal Init. 
static int _loc_MAIN_local_Exit(void);             // local Exit Process
int MAIN_Init_External(void);                      // SC external Init.
int MAIN_Exit(void);                               // end of SC process
int _loc_ExitConsole(void);                        // console handler
static int _loc_DSP_InitDisplay(void);             // Display Init

////////////////////////////////////////////////////////////////////////////////
//
// main()
//
int main(int nArgc, char* rgpszArgv[])
{
    int nRet = -1;             // return value of function

    THREAD_SetPriorityClass(THREAD_CLASS_PRIO_REALTIME);
	THREAD_SetSchedule(0, THREAD_POLICY_FIFO, THREAD_PRIO_ABOVE_NORMAL);

    ////////////////////////////////////////////////////////////////////////////
    // parse program option
    if (ParseArgument(nArgc, rgpszArgv) == FALSE)
    {
        //VERBOSE_ERROR("Argument Parcing Fail!\n");
        return EXIT_SUCCESS;
    }

    ////////////////////////////////////////////////////////////////////////////
    // SC internal initialization
    _loc_MAIN_Init_Internal();    

    ////////////////////////////////////////////////////////////////////////////
    // main loop
    do
    {
        THREAD_Sleep(1);
    }while(g_fLoopGo == TRUE || g_fMainExitDone == FALSE);

    THREAD_Sleep(10);
    nRet = _loc_MAIN_local_Exit();

    return EXIT_SUCCESS;
}  // end main()


//////////////////////////////////////////////////////////////////////////
//
// _loc_MAIN_local_Exit()
//  - connection & display finalize
static int _loc_MAIN_local_Exit(void)  
{
    int i;

    // check thread exit state
    for (i = 0; i < EXIT_COUNT_LIMIT; i++)
    {
        if (g_fScanButtonThreadExitState   == TRUE &&
            g_fScanSDOInputThreadExitState == TRUE &&
            g_fServiceProcThreadExitState  == TRUE &&
            g_fMappingInputThreadExitState == TRUE)
        {
            break;
        }
        
        VERBOSE_VERBOSE("Wait for Exit Threads Step2..(Button:%d, SVC: %d, InScan: %d, InMap: %d (Cnt:%d))\n",
                        g_fScanButtonThreadExitState,
                        g_fServiceProcThreadExitState,
                        g_fScanSDOInputThreadExitState,
                        g_fMappingInputThreadExitState,
                        i);

        THREAD_Sleep(100);
    }

    if (i >= EXIT_COUNT_LIMIT)
    {
        if (hScanButtonStateThread != NULL)
        {
            THREAD_Kill(hScanButtonStateThread);
            hScanButtonStateThread = NULL;
        }

        if (hServiceProcThread != NULL)
        {
            THREAD_Kill(hServiceProcThread);
            hServiceProcThread = NULL;
        }

        if (hScanInputStateThread != NULL)
        {
            THREAD_Kill(hScanInputStateThread);
            hScanInputStateThread = NULL;
        }

        if (hMappingInputThread != NULL)
        {
            THREAD_Kill(hMappingInputThread);
            hMappingInputThread = NULL;
        }

        VERBOSE_WARNING("Forced termination of threads Step2.\n");        
    }

    ////////////////////////////////////////////////////////////////////////////
    // detach RM connection
    //
    if (g_coidSC != INVALID_COID)
    {
        MSG_NamedDetachConnection(SC_CHANNEL_NAME, g_coidSC);
        g_coidSC = INVALID_COID;
    }

    ////////////////////////////////////////////////////////////////////////////
    // detach RM connection
    //
    if (g_coidSCTime != INVALID_COID)
    {
        MSG_NamedDetachConnection(SC_TIME_CHANNEL, g_coidSCTime);
        g_coidSCTime = INVALID_COID;
    }

    ////////////////////////////////////////////////////////////////////////////
    // detach RM connection
    //
    if (g_coidRM != INVALID_COID)
    {
        MSG_NamedDetachConnection(SYS_RM_CHANNEL_NAME, g_coidRM);
        g_coidRM = INVALID_COID;
    }
#if 1
    ////////////////////////////////////////////////////////////////////////////
    // detach SC Time
    //
    if (g_chidSCTime != INVALID_CHID)
    {
        MSG_NamedDestroyChannel(SC_TIME_CHANNEL, g_chidSCTime);
        g_chidSCTime = INVALID_CHID;
    }
#endif
    ////////////////////////////////////////////////////////////////////////////
    // detach TE Runtime connection
    //
    if(g_coidTETime != INVALID_COID)
    {
        MSG_NamedDetachConnection(RUN_CHANNEL_NAME, g_coidTETime);
        g_coidTETime = INVALID_COID;
    }

    ////////////////////////////////////////////////////////////////////////////
    // detach TE main connection
    //
    if(g_coidTE != INVALID_COID)
    {
        MSG_NamedDetachConnection(TE_CHANNEL_NAME, g_coidTE);
        g_coidTE = INVALID_COID;
    }

    ////////////////////////////////////////////////////////////////////////////
    // destroy SC-Channel
    //
    if (g_chidSC != INVALID_CHID)
    {
        MSG_NamedDestroyChannel(SC_CHANNEL_NAME, g_chidSC);
        g_chidSC = INVALID_CHID;
    }

    VERBOSE_MESSAGE("SC process shut down...\n");
    THREAD_Sleep(50);  // wait for message complete

    ////////////////////////////////////////////////////////////////////////////
    // destroy display service
    // 
    VGA_UnmapVGAMemory();
    VERBOSE_WaitForComplete();
    VERBOSE_Destroy();

    ///////////////////////////////////
    // For Check Memory Leak
    DEBUG_DumpLeakMemory( );

    return RESULT_OK;
}


//////////////////////////////////////////////////////////////////////////
//
// _loc_DSP_InitDisplay()
//
static int _loc_DSP_InitDisplay(void)  
{
    const char* VERBOSE_NAME = "Servo_Program";
    const char* VERBOSE_PREFIX = "[SC] ";

    ////////////////////////////////////////////////////////////////////////////
    // set display
    //

    // Clear ECHO/VERBOSE Files
    VERBOSE_CleanDirtyFile(VERBOSE_NAME);

    // verbose create
    VERBOSE_Create(VERBOSE_NAME, VERBOSE_PREFIX);

    DSP_SetVerbose();  // set type of verbose

    // init VGA
    if(VGA_MapVGAMemory(0) == -1)
    {
        VERBOSE_ERROR("Map Error\n");
        return RESULT_ERROR;
    }

    // config default VGA display color
    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_BLACK));

    return RESULT_OK;
}

//////////////////////////////////////////////////////////////////////////
//
// _loc_IO_SetZeroOutput()
//
static int _loc_IO_SetZeroOutput(void)  
{
    int iSlave, iPort;

    //////////////////////////
    // Digital Output
    for(iSlave = 0; iSlave < ROBOT_DO_SLAVE_COUNT; iSlave++)
    {
        for(iPort = 0; iPort < SLAVE_DO_PORT_COUNT; iPort++)
        {
            g_DoutPortVal[iSlave][iPort] = 0;
        }
    }
    
    if(g_pShmem_sc != NULL)
    {
        for(iPort = 0; iPort < ROBOT_DO_PORT_COUNT; iPort++)
        {
            g_pShmem_sc->outputcmd.nDoutPortCmd[iPort] = 0;
        }
    }

    //////////////////////////
    // Analog Output
    for(iSlave = 0; iSlave < ROBOT_AO_SLAVE_COUNT; iSlave++)
    {
        for(iPort = 0; iPort < SLAVE_AO_PORT_COUNT; iPort++)
        {
            g_AoutPortVal[iSlave][iPort] = 0;
        }
    }
    
    if(g_pShmem_sc != NULL)
    {
        for(iPort = 0; iPort < ROBOT_AO_PORT_COUNT; iPort++)
        {
            g_pShmem_sc->outputcmd.dbAoutPortCmd[iPort] = 0;
        }
    }

    return RESULT_OK;
}

#if 1
//////////////////////////////////////////////////////////////////////////
//
// _loc_CheckInitEstopState()
//
static int _loc_CheckInitEstopState(void)  
{
    if(g_Arg.bNoTPEstop != TRUE && g_Arg.bNoCartEstop != TRUE)
    {
        if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
           g_pShmem_sc->inputstate.fTP_EstopInState         == ESTOP_ACT ||
           g_pShmem_sc->inputstate.fCart_EstopInState       == ESTOP_ACT)
        {
            SERV_EStop(ON);

            if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                    VERBOSE_MESSAGE("Estop Activated By ControBox(Init Mode)\n");
                }
            }
            if(g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_TP;
                    VERBOSE_MESSAGE("Estop Activated By TP(Init Mode)\n");
                }
            }
            if(g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CART;
                    VERBOSE_MESSAGE("Estop Activated By Cart(Init Mode)\n");
                }
            }
        }
    }
    else if(g_Arg.bNoTPEstop == TRUE && g_Arg.bNoCartEstop != TRUE)
    {
        if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
           g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
        {
            SERV_EStop(ON);

            if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                }
            }
            if(g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CART;
                }
            }
        }
    }
    else if(g_Arg.bNoTPEstop != TRUE && g_Arg.bNoCartEstop == TRUE)
    {
        if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
           g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
        {
            SERV_EStop(ON);

            if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                }
            }
            if(g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_TP;
                }
            }
        }
    }
    else if(g_Arg.bNoTPEstop == TRUE && g_Arg.bNoCartEstop == TRUE)
    {
        if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
        {
            SERV_EStop(ON);

            if(g_pShmem_SysStatus_rm != NULL)
            {
                g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
            }
        }
    }

    if(g_Arg.bNoShockSensor != TRUE && g_fShockSensorRelease == OFF)
    {
        if(g_pShmem_sc->inputstate.fShockSensorInState == ESTOP_ACT)
        {
            SERV_EStop(ON);

            if(g_pShmem_SysStatus_rm != NULL)
            {
                g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_SHOCKSENSOR;
                VERBOSE_MESSAGE("Estop Activated By ShockSensor(Init Mode)\n");
            }
        }
    }    

    return RESULT_OK;
}
#endif


//////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_StartThreadRoutine()
//

#define     THREAD_IDX_RUNTIME              0
#define     THREAD_IDX_ECAT_TRACE           1
#define     THREAD_IDX_CHECK_TIME           2
#define     THREAD_IDX_SCAN_BUTTON_STATE    3
#define     THREAD_IDX_SCAN_ESTOP_STATE     4
#define     THREAD_IDX_VGA_DISPLAY          5
#define     THREAD_IDX_SERVICE_PROC         6
#define     THREAD_IDX_SCAN_SDO_IN_STATE    7
#define     THREAD_IDX_MAPPING_INPUT_STATE  8

static void _loc_FUNC_StartThreadRoutine(int nThreadIndex)
{
    if(nThreadIndex == THREAD_IDX_RUNTIME)
    {
#if 1
        hRuntimeThread = THREAD_Create(RuntimeThreadRoutine,   // function
                                       NULL,                   // variable
                                       0,                      // stack size
                                       //THREAD_PRIO_REALTIME,   // priority
                                       //255,   // priority
                                       152,   // priority
                                       THREAD_DETACHED,        // option
                                       //THREAD_POLICY_RR);
                                       THREAD_POLICY_FIFO);    // policy

        if(hRuntimeThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Runtime thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_ECAT_TRACE)
    {
#if 1
        hEcatTraceThread = THREAD_Create(EcatTraceThreadRoutine, // function
                                         NULL,                   // variable
                                         0,                      // stack size
                                         //THREAD_PRIO_REALTIME,   // priority
                                         THREAD_PRIO_NORMAL,   // priority
                                         THREAD_DETACHED,        // option
                                         THREAD_POLICY_RR);    // policy

        if(hEcatTraceThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create EtherCAT Trace thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_CHECK_TIME)
    {
#if 1
        hCheckTimeLimitThread = THREAD_Create(CheckTimeLimitThreadRoutine,  // function
                                              NULL,                         // variable
                                              0,                            // stack size
                                              THREAD_PRIO_LOWEST,           // priority
                                              THREAD_DETACHED,              // option
                                              THREAD_POLICY_RR);            // policy

        if (hCheckTimeLimitThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create RM Connect Wait thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_SCAN_BUTTON_STATE)
    {
#if 1
        hScanButtonStateThread = THREAD_Create(ScanButtonStateThreadRoutine,
                                               NULL,
                                               0,
                                               100,
                                               //THREAD_PRIO_REALTIME,
                                               THREAD_DETACHED,
                                               THREAD_POLICY_FIFO);
        
        if (hScanButtonStateThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Button Scan thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_SCAN_ESTOP_STATE)
    {
#if 1
        hScanServoStateThread = THREAD_Create(ScanEStopStateThreadRoutine,
                                              NULL,
                                              0,
                                              120,
                                              //THREAD_PRIO_REALTIME, // have to calib
                                              THREAD_DETACHED,
                                              THREAD_POLICY_FIFO);
        
        if (hScanServoStateThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Servo Scan thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_VGA_DISPLAY)
    {
#if 1
        hVGADisplayThread = THREAD_Create(DSP_VGADisplyThreadRoutine,
                                          NULL,
                                          0,
                                          THREAD_PRIO_BELOW_NORMAL,
                                          THREAD_DETACHED,
                                          THREAD_POLICY_RR);

        if (hVGADisplayThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create VGA Display thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_SERVICE_PROC)
    {
#if 1
        hServiceProcThread = THREAD_Create(SVC_ServiceProcThreadRoutine,
                                           NULL,
                                           0,
                                           THREAD_PRIO_HIGHEST,
                                           THREAD_DETACHED,
                                           //THREAD_POLICY_FIFO);
                                           THREAD_POLICY_RR);

        if (hServiceProcThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Service Process Routine thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_SCAN_SDO_IN_STATE)
    {
#if 1
        hScanInputStateThread = THREAD_Create(ScanSDOInputStateThreadRoutine,
                                              NULL,
                                              0,
                                              //THREAD_PRIO_REALTIME,
                                              THREAD_PRIO_HIGHEST, // have to calib
                                              THREAD_DETACHED,
                                              THREAD_POLICY_FIFO);
        
        if (hScanInputStateThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Input Scan thread...\n");

            exit(1);
        }
#endif
    }
    else if(nThreadIndex == THREAD_IDX_MAPPING_INPUT_STATE)
    {
#if 1
        hMappingInputThread = THREAD_Create(MappingInputThreadRoutine,
                                            NULL,
                                            0,
                                            120,
                                            //THREAD_PRIO_REALTIME,
                                            THREAD_DETACHED,
                                            THREAD_POLICY_FIFO);
        
        if (hMappingInputThread == INVALID_THREAD)
        {
            VERBOSE_ERROR("cannot create Input Mapping thread...\n");

            exit(1);
        }
#endif
    }
}

//////////////////////////////////////////////////////////////////////////
//
// _loc_MAIN_Init_Internal()
//
#define QNX_TIMER_TICK_MS       1
#define QNX_TIMER_RESOLUTION    100
//#define LIBTIMER_USE            1

static int _loc_MAIN_Init_Internal(void)  
{
    int nRet;
    //int nCnt = 0;

#if defined(__QNXNTO__)
#if defined(LIBTIMER_USE)

#else
    struct itimerspec itime;
    struct sigevent se;
    timer_t timerid;
#endif
#endif
    
    ////////////////////////////////////////////////////////////////////////////
    // set console handler
    DEBUG_SetConsoleHandler((DEBUG_CONSOLE_HANDLER) _loc_ExitConsole);

    ///////////////////////////////////
    // For DEBUG_MALLOC
    DEBUG_InstallTrap();

    if(g_fInit_Internal == TRUE)
    {
        VERBOSE_WARNING("SC already initialized.!!!\n");
        return 0;
    }

    // (1) Display Init (Verbose & VGA)
    nRet = _loc_DSP_InitDisplay();

    // (2) create SC SHM(shared memory)
    if(g_fOpenShmSc == FALSE)
    {
        SHM_SC_CreateShmem();
    }

    // (3) create SC-channel & connect
        // create SC Channel
    if(g_chidSC == INVALID_CHID)
    {
        g_chidSC = MSG_SC_CreateChannel(SC_CHANNEL_NAME);
    }

        // connect SC Channel
    if(g_coidSC == INVALID_COID)
    {
        g_coidSC = MSG_SC_ConnectChannel(SC_CHANNEL_NAME, OPT_NULL);
    }
    
    // (4) create SC-RunTime-channel
    if(g_chidSCTime == INVALID_CHID)
    {
        g_chidSCTime = MSG_SC_CreateChannel(SC_TIME_CHANNEL);
    }

    // connect SC Channel
    if(g_coidSCTime == INVALID_COID)
    {
        g_coidSCTime = MSG_SC_ConnectChannel(SC_TIME_CHANNEL, OPT_NULL);
    }
#if 0
    THREAD_Sleep(5);

    while(g_coidRM == INVALID_COID)
    {
        THREAD_Sleep(100);

        if(g_coidRM == INVALID_COID && g_Arg.bSingleExec != TRUE)
        {
            g_coidRM = MSG_SC_ConnectChannel(SYS_RM_CHANNEL_NAME, OPT_QUITE);
        }

        if(nCnt >= 50 || g_coidRM != INVALID_COID)
        {
            if(nCnt >= 50)
            {
                VERBOSE_WARNING("RM channel Connect time limit exceeded!\n");
            }
            nCnt = 0;
            break;
        }
        nCnt++;
    }
    nCnt = 0;

    if(g_coidRM != INVALID_COID)
    {
        VERBOSE_MESSAGE("RM channel Connect done!\n");
    }
#endif
#if 1
    // Wait for RM connect to protect init error
    THREAD_Sleep(400);
#endif
    // (5) open RM_SHM
        // System Status SHM
    if(g_fOpenSysStatusShmRm == FALSE && g_Arg.bSingleExec != TRUE)
    {
        SHM_RM_SysStatusOpenShmem();
    }
        // System Config SHM
    if(g_fOpenSysConfigShmRm == FALSE && g_Arg.bSingleExec != TRUE)
    {
        SHM_RM_SysConfigOpenShmem();
    }
        // System Param SHM
    if(g_fOpenSysParamShmRm == FALSE && g_Arg.bSingleExec != TRUE)
    {
        SHM_RM_SysParamOpenShmem();
    }

    // (6) system parameter load by sysconf RM SHM
    SHM_LoadSysConfigParam(ROBOT_0_INDEX);

    g_fSharedMemRMOpen = ON;
    
    // (7) create timer
        // Register Timer (QNX)
#if defined(__QNXNTO__)
#if defined(LIBTIMER_USE)
    g_hTimer = TIME_RegTimerPulse(g_coidSCTime, RUNSERV_TIMER, 0, g_nQNXTimerTick, 0);
#else
    itime.it_value.tv_sec  = 0;
    //itime.it_value.tv_nsec = 999847 * g_nQNXTimerTick; // 999847;
    itime.it_value.tv_nsec = g_nQNXTimerTick; // 1;
    itime.it_interval.tv_sec = 0;
    //itime.it_interval.tv_nsec = 999847 * g_nQNXTimerTick; // 999847;
    itime.it_interval.tv_nsec = g_nQNXTimerTick; // 1;

    if(g_coidSCTime != INVALID_COID)
    {
        SIGEV_PULSE_INIT(&se, g_coidSCTime, SIGEV_PULSE_PRIO_INHERIT, RUNSERV_TIMER, 0);
    }
    
    if(timer_create(CLOCK_MONOTONIC, &se, &timerid))    //CLOCK_MONOTONIC
    //if(timer_create(CLOCK_REALTIME, &se, &timerid))    //CLOCK_REALTIME
    {
        VERBOSE_ERROR("Fail to Register Timer!\n");
        g_hTimer = -1;
    }
    else
    {
        timer_settime(timerid, 0, &itime, NULL);
        g_hTimer = (int)timerid;
    }
#endif
#else
        // Register Timer (Windows)
    g_hTimer = TIME_RegTimerPulse(g_coidSCTime, RUNSERV_TIMER, 0, g_nQNXTimerTick, 0);
#endif

    VERBOSE_MESSAGE("Registered Timer Set to %d.\n", g_nQNXTimerTick);

#if defined(__QNXNTO__)
        // Timer Change Resolution(QNX)
    TIME_ChangeResolution(g_nQNXTimerRes);
#else
        // Timer Change Resolution(Windows)
    TIME_ChangeResolution(g_nQNXTimerRes * 100);
#endif

    // (8) ECAT Master Init. (arg option '-single' is valid, by default value)
    if(g_Arg.bSingleExec == TRUE)
    {
        // system parameter load by default value
        SHM_LoadSysConfigParam(ROBOT_0_INDEX);

        nRet = ECATNET_InitializeMaster();
    }

    // (9) create run-time thread (TE <-> SC)
    _loc_FUNC_StartThreadRoutine(THREAD_IDX_RUNTIME);
    

    // (10) create EtherCAT trace thread
#if defined (__QNXNTO__)
#if 1
    _loc_FUNC_StartThreadRoutine(THREAD_IDX_ECAT_TRACE);
#endif
#else
    g_fThreadEcatTraceExitState = TRUE;
#endif
    // (11) create wait RM connection thread
    if(g_nTime_Limit > 0)
    {
        // create thread that elapsed time of receive msg from RM
        _loc_FUNC_StartThreadRoutine(THREAD_IDX_CHECK_TIME);
    }

    if(g_Arg.bSingleExec == TRUE)
    {
        // (10) create thread that monitoring of hardware system
        _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_BUTTON_STATE);

        _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_ESTOP_STATE);

        _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_SDO_IN_STATE);

        _loc_FUNC_StartThreadRoutine(THREAD_IDX_MAPPING_INPUT_STATE);
    }

    // (12) create thread that display state(for VGA)
    _loc_FUNC_StartThreadRoutine(THREAD_IDX_VGA_DISPLAY);
    
    // (13) create thread that service receive & process
    _loc_FUNC_StartThreadRoutine(THREAD_IDX_SERVICE_PROC);

    // (14) initialize parameter variables
    //PARAM_LoadDefaultParam();
    
    if ((g_Arg.bSingleExec == TRUE && g_pShmem_sc->sysstate.fEcatInitState == FALSE) ||
         g_fOpenShmSc == FALSE || g_chidSC == INVALID_CHID ||
         g_chidSCTime == INVALID_CHID || hRuntimeThread == NULL)
    {
        VERBOSE_ERROR("Failed to initialize the SC(internal) process.\n");
        return RESULT_ERROR;
    }
    else
    {
        if(g_Arg.bSingleExec == TRUE)
        {
            FUNC_SyncActualPosToTargetPos();
#if 0
            if (g_pShmem_sc != NULL)
            {
                for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
                {
                    // write actual pos. to SC_SHM
                    FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
                    g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
                    g_pShmem_sc->outputcmd.dbTrgPos[iAxis] = g_dbAct_Pos[iAxis];
                    g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];
                }
            }
#endif
        }

        g_fInit_Internal = TRUE;
        VERBOSE_MESSAGE("Completed to initialize the SC(internal) process.\n");

        return RESULT_OK;
    }
}


//////////////////////////////////////////////////////////////////////////
//
// MAIN_Init_External()
//
int MAIN_Init_External(void)  // coidRM, coidTE, coidSC, nTimer
{
    int nRet_rm   = 0;
    int nRet_te   = 0;
    int nRet_ecat = 0;
    //int iAxis;

    // (1) connect channel
        // connect RM channel
    if(g_coidRM != INVALID_COID)
    {
        MSG_NamedDetachConnection(SYS_RM_CHANNEL_NAME, g_coidRM);
        g_coidRM = INVALID_COID;
    }
    
    if(g_coidRM == INVALID_COID && g_Arg.bSingleExec != TRUE)
    {
        g_coidRM = MSG_SC_ConnectChannel(SYS_RM_CHANNEL_NAME, OPT_NULL);
    }

        // connect TE main Channel
    if(g_coidTE != INVALID_COID)
    {
        MSG_NamedDetachConnection(TE_CHANNEL_NAME, g_coidTE);
        g_coidTE = INVALID_COID;
    }
    
    if(g_coidTE == INVALID_COID && g_Arg.bSingleExec != TRUE)
    {
        g_coidTE = MSG_SC_ConnectChannel(TE_CHANNEL_NAME, OPT_NULL);
    }

        // connect TE Runtime Channel
    if(g_coidTETime != INVALID_COID)
    {
        MSG_NamedDetachConnection(RUN_CHANNEL_NAME, g_coidTETime);
        g_coidTETime = INVALID_COID;
    }
    
    if(g_coidTETime == INVALID_COID && g_Arg.bSingleExec != TRUE)
    {
        g_coidTETime = MSG_SC_ConnectChannel(RUN_CHANNEL_NAME, OPT_NULL);
    }

    // (2) open RM_SHM
    if(g_fSharedMemRMOpen == OFF)
    {
            // System Status SHM
        if(g_fOpenSysStatusShmRm == FALSE && g_Arg.bSingleExec != TRUE)
        {
            nRet_rm = SHM_RM_SysStatusOpenShmem();
        }

            // System Config SHM
        if(g_fOpenSysConfigShmRm == FALSE && g_Arg.bSingleExec != TRUE)
        {
            nRet_rm = SHM_RM_SysConfigOpenShmem();
        }

            // System Param SHM
        if(g_fOpenSysParamShmRm == FALSE && g_Arg.bSingleExec != TRUE)
        {
            nRet_rm = SHM_RM_SysParamOpenShmem();
        }

        // (3) system parameter load by sysconf RM SHM
        SHM_LoadSysConfigParam(ROBOT_0_INDEX);
    }
    
    g_fSharedMemRMOpen = OFF;   // flag for service request

    // (4) open TE_SHM
    if(g_fOpenShmTeStatus == FALSE)
    {
        nRet_te = SHM_Status_TE_OpenShmem();
    }

    if(g_fOpenShmTeTask == FALSE)
    {
        nRet_te = SHM_Task_TE_OpenShmem();
    }
#if 0
    // Register Timer
    g_hTimer = TIME_RegTimerPulse(g_coidSCTime, RUNSERV_TIMER, 0, g_nQNXTimerTick, 0);
    VERBOSE_MESSAGE("Registered Timer Set to %d.\n", g_nQNXTimerTick);

    // Timer Change Resolution
    //TIME_ChangeResolution(10);
#endif
    // (5) Ecat init. (arg option '-single' is invalid, by sysconf RM SHM value)
    if(g_pShmem_sc->sysstate.fEcatInitState == FALSE)
    {
        nRet_ecat = ECATNET_InitializeMaster();
    }

    if(g_Arg.bSingleExec != TRUE)
    {
        // create thread that monitoring of hardware system
        if(g_fScanButtonThreadRunState != TRUE)
        {
            _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_BUTTON_STATE);
            //VERBOSE_MESSAGE("Create Thread Button Scan.\n");
        }

        if(g_fEStopStateThreadRunState != TRUE)
        {
            _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_ESTOP_STATE);
            //VERBOSE_MESSAGE("Create Thread E-stop.\n");
        }

        if(g_fScanSDOInputThreadRunState != TRUE)
        {
            _loc_FUNC_StartThreadRoutine(THREAD_IDX_SCAN_SDO_IN_STATE);
            //VERBOSE_MESSAGE("Create Thread Input Scan.\n");
        }
        
        if(g_fMappingInputThreadRunState != TRUE)
        {
            _loc_FUNC_StartThreadRoutine(THREAD_IDX_MAPPING_INPUT_STATE);
            //VERBOSE_MESSAGE("Create Thread Input Scan.\n");
        }
    }

    // read actual position
    if(g_pShmem_sc->sysstate.fEcatInitState == TRUE)
    {
#if defined(_MSC_VER)
        if(g_fInit_Internal == TRUE)
        {
            SYS_CurrPositionDataFileOpen(OPT_READ);
            SYS_CurrPositionDataFileLoad();
            SYS_CurrPositionDataFileClose();
            g_fInit_Internal = FALSE;
        }
#endif

        FUNC_SyncActualPosToTargetPos();
#if 0
        if (g_pShmem_sc != NULL)
        {
            for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                // write actual pos. to SC_SHM
                FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
                g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
                g_pShmem_sc->outputcmd.dbTrgPos[iAxis] = g_dbAct_Pos[iAxis];
                g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];
            }
        }
#endif

#if defined (__QNXNTO__)
        EcatIODevResetStatistics(g_hMaster);
#endif
    }
    
    g_fHWLimitMonAct = ON;
    //SERV_SetControlMode(PROFILED_POS_MODE); // Set Porfiled Position Mode for test
    //ECATSERV_WriteControlWord(ALL_AXES, 0x10, ALL);

    if(g_Arg.bSingleExec != TRUE &&
      (g_coidRM == INVALID_COID ||
       nRet_rm != RESULT_OK || nRet_te != RESULT_OK || nRet_ecat != RESULT_OK))
    {
        g_pShmem_sc->sysstate.fErrorState = TRUE;
        g_pShmem_sc->sysstate.nErrorCode  = SYS_ERR_INIT_SC;

        if (g_pShmem_SysStatus_rm != NULL)
        {            
            g_pShmem_SysStatus_rm->fExitProcSC = FALSE;  // write 'Exit' flag to RM_SHM
        	g_pShmem_SysStatus_rm->fInitProcSC = FALSE;  // write 'Init' flag to RM_SHM
        }

        VERBOSE_ERROR("Fail to initialize the SC(external) process!\n");

        return RESULT_ERROR;
    }
    else
    {
        if (g_pShmem_SysStatus_rm != NULL)
        {
            g_pShmem_SysStatus_rm->fExitProcSC = FALSE; // write 'Exit' flag to RM_SHM
            g_pShmem_SysStatus_rm->fInitProcSC = TRUE;  // write 'Init' flag to RM_SHM
        }

        VERBOSE_MESSAGE("Completed to initialize the SC(external) process.\n");
        VERBOSE_MESSAGE("Service ready for SC process.\n");

        _loc_IO_SetZeroOutput();    // Set Zero Digital, Analog Output

        if(g_pShmem_Task_te != NULL)
        {
            g_pShmem_Task_te->ADC_gathering_index = 0;
        }

#if defined (__QNXNTO__)
        _loc_CheckInitEstopState();
#endif
        return RESULT_OK;
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// MAIN_Exit()
//

int MAIN_Exit(void)  // coidSC, chidSCTime, chidSC
{
    int i;

    // (1) check servo on/off state
    if (g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
    {
        SERV_ServoOnCmd(OFF);  // servo off
    }

#if defined(_MSC_VER)
        SYS_CurrPositionDataFileOpen(OPT_WRITE);
        SYS_CurrPositionDataFileSave();
        SYS_CurrPositionDataFileClose();
#endif

    if (g_pszSensDir != NULL)
    {
        DEBUG_FREE(g_pszSensDir);
        g_pszSensDir = NULL;
    }

    _loc_IO_SetZeroOutput();    // Set Zero Digital, Analog Output

    // (2) release EtherCAT Master
    if (g_hMaster != NULL)
    {
    	ECATNET_ReleaseMaster();
    }

    THREAD_Sleep(100);

    g_fExit = TRUE;

    THREAD_Sleep(100);
#if 0
    // (3) destroy SC(time)-Channel
    if (g_chidSCTime != INVALID_CHID)
    {
        MSG_NamedDestroyChannel(SC_TIME_CHANNEL, g_chidSCTime);
        g_chidSCTime = INVALID_CHID;
    }
#endif
    // (4) destroy TE_SHM
    if (g_pShmem_Status_te != NULL)
    {       
        SHM_Status_TE_DestroyShmem();
    }

    if (g_pShmem_Task_te != NULL)
    {       
        SHM_Task_TE_DestroyShmem();
    }
    
    // (5) destroy RM_SHM
    if (g_pShmem_SysStatus_rm != NULL)
    {
        if (g_fOpenSysStatusShmRm == TRUE)
        {
            g_pShmem_SysStatus_rm->fInitProcSC = FALSE; // flag of init for RM SHM
            g_pShmem_SysStatus_rm->fExitProcSC = TRUE;  // flag of exit for RM SHM
        }
        
        SHM_RM_SysStatusDestroyShmem();
    }

    if (g_pShmem_SysConfig_rm != NULL)
    {
        SHM_RM_SysConfigDestroyShmem();
    }

    if (g_pShmem_SysParam_rm != NULL)
    {
        SHM_RM_SysParamDestroyShmem();
    }

    // check thread exit state
    for (i = 0; i < EXIT_COUNT_LIMIT; i++)
    {
        if (g_fThreadRuntimeExitState == TRUE &&
            g_fEStopStateThreadExitState == TRUE &&
            g_fDisplayThreadExitState == TRUE)
        {
            break;
        }
        
        VERBOSE_VERBOSE("Wait for Exit Threads Step1..(Run:%d,Est:%d,Disp:%d, %d)\n",
                        g_fThreadRuntimeExitState, g_fEStopStateThreadExitState,
                        g_fDisplayThreadExitState, i);
        THREAD_Sleep(100);
    }

    if (i >= EXIT_COUNT_LIMIT)
    {
        // kill run-time thread 
        if (hRuntimeThread != NULL)
        {
            THREAD_Kill(hRuntimeThread);
            hRuntimeThread = NULL;
        }

        // Display thread 
        if (hVGADisplayThread != NULL)
        {
            THREAD_Kill(hVGADisplayThread);
            hVGADisplayThread = NULL;
        }

        // Servo State thread 
        if (hScanServoStateThread != NULL)
        {
            THREAD_Kill(hScanServoStateThread);
            hScanServoStateThread = NULL;
        }

        VERBOSE_WARNING("Forced termination of threads Step1.\n");        
    }

    // (6) destroy SC_SHM
    if (g_pShmem_sc != NULL)
    {
        SHM_SC_DestroyShmem();
    }
#if 1
    // Unregister Timer
    TIME_UnregTimerPulse(g_hTimer);
    VERBOSE_MESSAGE("Unregistered Timer Done.\n");
#endif
    g_fMainExitDone = TRUE;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ExitConsole()
//
int _loc_ExitConsole(void)
{
    if (g_coidSC != INVALID_COID)
    {
        MSG_SendPulse(g_coidSC, SC_SERV_EXIT, 0);
    }

    return EXIT_SUCCESS;
}
