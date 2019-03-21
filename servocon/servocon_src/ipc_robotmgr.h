#ifndef __IPC_ROBOTMGR_H__
#define __IPC_ROBOTMGR_H__

/////////////////////////////////////////////////////////////////////////////
//
//  ipc_robotmgr.h: RM process header for IPC(inter process communication)
//                                            2013.04.11  Ryu SinWook

#include "sys_conf.h"
#include "dandy_job.h"
#include "ipc_jobshm.h"
 
///////////////////////////////////////
//
//  Version Information
//

#define SYS_RM_VERSION         "5.01a"
#define SYS_RM_BUILD           "2014.06.30"

///////////////////////////////////////
//
// RM Channel & SHM Name Define
//
#define SYS_RM_CHANNEL_NAME     "CHANNEL_RMGR"
#define SHM_RM_SYSSTATUS_NAME   "SHM_RM_SYSSTATUS"
#define SHM_RM_SYSCONFIG_NAME   "SHM_RM_SYSCONFIG"
#define SHM_RM_SYSPARAM_NAME    "SHM_RM_SYSPARAM"


/////////////////// Symbol Definition for Service, State //////////////////////

///////////////////////////////////////
//
// RM Service Symbol
//
//Code: Service Name                : Msg Format  : Value
//    :                             :             : 0       : 1       : 2        : 3      : 4       : 5       : 6
// 0  : RMGR_SERV_SYSEXIT           : pulse       : RM(ALL) : TE(only): SC(only)
// 1  : RMGR_SERV_SYSVERSION        : message     : None    :
// 2  : RMGR_SERV_SYSINIT           : pulse       : RM(ALL) : TE(only): SC(only)
// 3  : RMGR_SERV_SYSSTATE          : message     : None    :
// 4  : RMGR_SERV_SYSINFO           : message(Idx): Ctrl    : Ecat    : Robot    : Axis   : Motor   : Welder
// 5  : RMGR_SERV_TIMERTEST         : pulse       : None    :
// 6  : RMGR_SERV_CONFRELOAD        : pulse       : None    :
// 7  : RMGR_SERV_ERRHISTORY        : message     : Opt     :
// 8  : RMGR_SERV_GETSTATISTIC      : pulse       : Opt     :
// 10 : RSVC_SERV_JOBLOAD           : message     : Refer to Below Additional Comment
// 10 : RSVC_SERV_JOBLOAD           : pulse       : for Call/Return Service, Load Data Refer to SHM
// 11 : RSVC_SERV_GENMAPFILE        : message     : None    :
// 12 : RSVC_SERV_JOBSHMDUMP        : pulse       : all     : cmd     : Tvar    : Wvf      : Swf    : Mwf     : Ewf
// 13 : RSVC_SERV_JOBEXECAUTO       : pulse       : LineIdx : 
// 14 : RSVC_SERV_JOBEXECDRY        : pulse       : LineIdx : 
// 15 : RSVC_SERV_JOBEXESTOP        : pulse       : None    :
// 16 : RSVC_SERV_JOBLINE_DSP       : pulse       : LineIdx :
// 17 : RSVC_SERV_SYSMON_TO_TP      : message     : None    :->Use Reply Msg
// 18 : RSVC_SERV_JOBEXECSTEP       : pulse       : LineIdx :
// 19 : RSVC_SERV_JOBFILESAVE       : message     : Opt     :->Set Job Name Msg
// 20 : RCON_SERV_ERROR_RESET       : pulse       :         :
// 21 : RCON_SERV_HOME_MOVE         : pulse       : Idx     :
// 22 : RCON_WIRECUT_JOB_CALL       : pulse       : None    :
// 23 : RCON_VOLT_REALTIME_OFFSET   : message     : Opt(Inc/Dec) ->Use Reply Msg
// 24 : RCON_CURR_REALTIME_OFFSET   : message     : Opt(Inc/Dec) ->Use Reply Msg
// 30 : RSVC_SERV_HOME_EDIT         : message     : Idx     :
// 31 : RSVC_SERV_USERCRD_EDIT      : message     : Idx     :
// 32 : RSVC_SERV_PARAM_EDIT        : message     : TCP     : World   :WeldTuneIn:WeldTuneOut:Restart  (Opt)
// 33 : RSVC_SERV_SHOWHOME_TP       : message     : Idx     :->Use Reply Msg
// 34 : RSVC_SERV_SHOWUSRCRD_TP     : message     : Idx     :->Use Reply Msg
// 35 : RSVC_SERV_SHOWPARAM_TP      : message     : Opt     :->Use Reply Msg
// 36 : RSVC_SERV_SET_GAPCOND       : message     : None    :
// 37 : RSVC_JOB_CONSTVARINFO       : message     : VarType : Msg: VarIndex(Req) ->Use proper Reply Msg
// 38 : RSVC_JOB_WELDVARINFO        : message     : VarType : Msg: VarIndex(Req) ->Use proper Reply Msg
// 39 : RSVC_JOB_CONSTVAREDIT       : message     : VarType : Msg: VarIndex(Data)->Confirm proper Reply Msg
// 40 : RSVC_JOB_BIN_SNDHEADER      : message     : Opt     :
// 41 : RSVC_JOB_BIN_SNDBODY        : message     : Idx     :
// 42 : RSVC_JOB_BIN_RCVHEADER      : message     : Opt     :
// 43 : RSVC_JOB_BIN_RCVBODY        : message     : Idx     :
// 44 : RSVC_JOB_RESTART            : pulse       : None    :
// 45 : RSVC_JOB_COMP_BIN_SNDHEADER : message     : Opt     :
// 46 : RSVC_JOB_COMP_BIN_SNDBODY   : message     : Idx     :
// 47 : RSVC_JOB_COMP_BIN_RCVBODY   : message     : Idx     :
// 48 : RSVC_JOB_COMP_BIN_RCVHEADER : message     : Opt     :
// 49 : RSVC_SERV_WELDTUNE_IN_EDIT  : message     : Idx     :
// 50 : RSVC_SERV_WELDTUNE_OUT_EDIT : message     : Idx     :
// 51 : RSVC_SERV_SHOWWELDTUNE_IN   : message     : Idx     :
// 52 : RSVC_SERV_SHOWWELDTUNE_OUT  : message     : Idx     :
// 53 : RSVC_SRV_WELDTUNE_IN_APPLY  : pulse       : Idx     :
// 54 : RSVC_SRV_WELDTUNE_OUT_APPLY : pulse       : Idx     :
// 55 : RSVC_JOBFILE_ZIP_UNCOMPRESS : message     : None    : Msg: ZipFileName
// 56 : RSVC_SERV_SET_SKIPCOND      : message     : Show    : LeftVert : RightVert : LeftCollar : RightCollar: Horiz
// 57 : RSVC_CONSTVAR_FILE_HANDLE   : pulse       : Write   : Read
// 58 : RSVC_SERV_SET_GAPSKIPCOND   : message     : Show    : Set      :
// 59 : RSVC_SERV_REBOOT_SYSTEM     : pulse       : Reboot  : Shutdown
// 60 : RSVC_SERV_CONSTVAR_INIT     : pusle       : P       : B        : I         : R
// 90 : RSVC_JOB_CMDJUMP            : message     : None    :
// 91 : RSVC_JOB_CMDCALL            : message     : None    :
// 92 : RSVC_JOB_CMDRETURN          : pulse       : None    :
// 93 : RSVC_JOB_CMDCONTINUE        : pulse       : None    :
// 127: RMGR_SYS_ALIVE_CHECK        : pulse       : None    :
//
// Remark: Both msssage and pulse are compatible,
//         But above method recommended

    // system startup & status
#define     RMGR_SERV_SYSEXIT            0    // Service Finalize
#define     RMGR_SERV_SYSVERSION         1    // Check Software Version
#define     RMGR_SERV_SYSINIT            2    // Service Initialize
#define     RMGR_SERV_SYSSTATE           3    // System State Inform
#define     RMGR_SERV_SYSINFO            4    // System Information Display
                                              // Request -> Value: Info Target, Msg: Index
                                              // Result  -> Msg includes Requested info
#define     RMGR_SERV_TIMERTEST          5    // Timer Test
#define     RMGR_SERV_CONFRELOAD         6    // Configuration File Reload & Dump
#define     RMGR_SERV_ERRHISTORY         7    // Send Error History
                                              // Value: 0 -> Send
                                              // Value: 1 -> Clear
                                              // Msg: Start Stack Idx, Stack Cnt
                                              // Start Stack Idx: -1 (Recent)
#define     RMGR_SERV_GETSTATISTIC       8    // Get System Statistics data
                                              // Value: 0 -> Send
                                              // Value: 1 -> Clear
#define     RMGR_SYS_ALIVE_CHECK         127  // System Alive Check

    // job file handling
#define     RSVC_SERV_JOBLOAD            10   // Job file Compile & SHM Load
                                              // Value : Assemble Option
            // Symbols -> Refer to dandy_jobasm.h
            // JOBASM_AF_NONE          0x0000    JOBASM_AF_IGNORE_ERROR  0x0020
            // JOBASM_AF_ONEPASS       0x0001    JOBASM_AF_NO_COMMENT    0x0040
            // JOBASM_AF_CHECK_ONLY    0x0002    JOBASM_AF_NO_EMPTY_LINE 0x0080
            // JOBASM_AF_STRICT        0x0004    JOBASM_AF_NO_EMPTY_ADJ  0x0100
            // JOBASM_AF_NO_CINDEX     0x0010    
            // JOBASM_AF_FORMAT_MASK   0x00f00000  JOBASM_AF_DANDY1996     0x00100000
            // JOBASM_AF_CAS           0x00200000
#define     RSVC_SERV_GENMAPFILE         11   // Generate Map File
#define     RSVC_SERV_JOBSHMDUMP         12   // Dump Job shared memory
                                              // Value: Display Option
#define     RSVC_SERV_JOBEXECAUTO        13   // Request Job Execute All to TE
                                              // Value: Start Line Index for Execution
#define     RSVC_SERV_JOBEXECDRY         14   // Request Job Execute All to TE
                                              // Value: Start Line Index for Execution
#define     RSVC_SERV_JOBEXESTOP         15   // Request Job Execute Stop to TE
#define     RSVC_SERV_JOBLINE_DSP        16   // Job Prog Line Display to TP
                                              // Value: Line Index for Display
#define     RSVC_SERV_SYSMON_TO_TP       17   // System Monitoring Data Send to TP
#define     RSVC_SERV_JOBEXECSTEP        18   // Request Job Execute Step to TE
                                              // Value: Line Index for Execution
#define     RSVC_SERV_JOBFILESAVE        19   // Job file Save
                                              // Value: JOBDIS_DF_DANDY1996(0x00100000) or Not
                                              // Msg: Job File Name
    // device control
#define     RCON_SERV_ERROR_RESET        20
#define     RCON_SERV_HOME_MOVE          21   // Move to home
                                              // Value: Home Index
#define     RCON_WIRECUT_JOB_CALL        22   // Wire Cut Job Call
#define     RCON_VOLT_REALTIME_OFFSET    23   // Voltage realtime control
                                              // Value: 1-> increase, -1->decrease, 0->just data send
                                              // by 0.1V, Set by Config, To Confirm Use Reply Msg
#define     RCON_CURR_REALTIME_OFFSET    24   // Current realtime control
                                              // Value: 1-> increase, -1->decrease, 0->just data send
                                              // by 10A, Set by Config, To Confirm Use Reply Msg
    // set parameter
#define     RSVC_SERV_HOME_EDIT          30   // set/get home position
                                              // Value: Home Index
                                              // Msg: idx(int), joint value(double)
#define     RSVC_SERV_USERCRD_EDIT       31   // set/get user coord
                                              // Value: UserCoord Index
                                              // Msg: idx(int), euler value(double)
#define     RSVC_SERV_PARAM_EDIT         32   // calibrate TCP
                                              // Value: 0 -> TCP
                                              // Value: 1 -> world
                                              // Value: 2 -> weld tune input
                                              // Value: 3 -> weld tune output
                                              // Value: 4 -> restart param
                                              // Value: 5 -> arcsensor param
                                              // Value: 6 -> ? Not Yet defined
#define     RSVC_SERV_SHOWHOME_TP        33   // Show home position
                                              // Value: Home Index
#define     RSVC_SERV_SHOWUSRCRD_TP      34   // Show User Coordinate value
                                              // Value: UserCoord Index
#define     RSVC_SERV_SHOWPARAM_TP       35   // Show User Parameter to TP(Not included index)
                                              // Value: 0 -> TCP
                                              // Value: 1 -> world
                                              // Value: 2 -> weld tune input
                                              // Value: 3 -> weld tune output
                                              // Value: 4 -> restart param
                                              // Value: 5 -> arcsensor param
                                              // Value: 6 -> ? Not Yet defined
#define     RSVC_SERV_SET_GAPCOND        36   // Receive Data, Copy to SHM
#define     RSVC_JOB_CONSTVARINFO        37   // Value: Const Var Type (P, B, I, R, T), Msg: Request Idx
            // Symbols: DANDY_JOB_VAL_TYPE_P_VAR,   DANDY_JOB_VAL_TYPE_B_VAR,
            //          DANDY_JOB_VAL_TYPE_I_VAR,   DANDY_JOB_VAL_TYPE_R_VAR
            //          DANDY_JOB_VAL_TYPE_T_VAR
#define     RSVC_JOB_WELDVARINFO         38   // Value: Var Var Type (Wvf, Swf, Mwf, Ewf), Msg: Request Idx
            // Symbols: DANDY_JOB_VAL_TYPE_WVF_VAR, DANDY_JOB_VAL_TYPE_SWF_VAR
            //          DANDY_JOB_VAL_TYPE_MWF_VAR, DANDY_JOB_VAL_TYPE_EWF_VAR
#define     RSVC_JOB_CONSTVAREDIT        39   // Value: Const Var Type (P, B, I, R), Msg: Idx in Data (P, Const)
            // Symbols: DANDY_JOB_VAL_TYPE_P_VAR,   DANDY_JOB_VAL_TYPE_B_VAR,
            //          DANDY_JOB_VAL_TYPE_I_VAR,   DANDY_JOB_VAL_TYPE_R_VAR
#define     RSVC_JOB_BIN_SNDHEADER       40   // Send Job Binary Header
                                              // Value: 0 -> Need to Send Data (Raw: RSVC_JOB_BIN_SNDBODY)
                                              // Value: 1 -> Just Header Define
#define     RSVC_JOB_BIN_SNDBODY         41   // Send Job Binary Data (Raw Binary)
                                              // Value: Index of Body
#define     RSVC_JOB_BIN_RCVHEADER       42   // Receive Job Binary Header
                                              // Value: 0 -> Need to Send Data (Raw: RSVC_JOB_BIN_RCVBODY)
                                              // Value: 1 -> Just Header Define
#define     RSVC_JOB_BIN_RCVBODY         43   // Receive Job Binary Data (Raw Binary)
                                              // Value: Index of Body
#define     RSVC_JOB_RESTART             44   // Restart Execution

#define     RSVC_JOB_COMP_BIN_SNDHEADER  45   // Send Job Binary Header (Compressed Binary)
                                              // Value: 0 -> Need to Send Data (Compressed: RSVC_JOB_BIN_COMP_SNDBODY)
                                              // Value: 1 -> Just Header Define
#define     RSVC_JOB_COMP_BIN_SNDBODY    46   // Send Job Binary Data (Compressed Binary)
                                              // Value: Index of Body
#define     RSVC_JOB_COMP_BIN_RCVHEADER  47   // Receive Job Binary Header (Compressed Binary)
                                              // Value: 0 -> Need to Send Data (Compressed Binary: RSVC_JOB_BIN_COMP_RCVBODY)
                                              // Value: 1 -> Just Header Define
#define     RSVC_JOB_COMP_BIN_RCVBODY    48   // Receive Job Binary Data (Compressed Binary)
                                              // Value: Index of Body
#define     RSVC_SERV_WELDTUNE_IN_EDIT   49   // set weld tune input
                                              // Value: Index
#define     RSVC_SERV_WELDTUNE_OUT_EDIT  50   // set weld tune output
                                              // Value: Index
#define     RSVC_SERV_SHOWWELDTUNE_IN    51   // show weld tune input
                                              // Value: Index
#define     RSVC_SERV_SHOWWELDTUNE_OUT   52   // show weld tune output
                                              // Value: Index
#define     RSVC_SRV_WELDTUNE_IN_APPLY   53   // appply weld tune input
                                              // Value: Index
#define     RSVC_SRV_WELDTUNE_OUT_APPLY  54   // apply weld tune output
                                              // Value: Index
#define     RSVC_JOBFILE_ZIP_UNCOMPRESS  55   // Unzip JobFile to Job Directory
                                              // Msg: ZipFileName (extent 'zip' supported only)
#define     RSVC_SERV_SET_SKIPCOND       56   // Receive Data, Set B Var
                                              // Value: Option
                                              // SHOW_SKIP_COND, LEFT_VERT_SKIP, RIGHT_VERT_SKIP,
                                              // LEFT_COLLAR_SKIP, RIGHT_COLLAR_SKIP, HORIZONTAL_SKIP
#define     RSVC_CONSTVAR_FILE_HANDLE    57   // Constant Variables(P, B, I, R) File Handling
                                              // Value: 0 -> Write, 1 -> Read
#define     RSVC_SERV_SET_GAPSKIPCOND    58   // Receive Data, Set B Var
                                              // Gap, Skip Condition Use One Symbol (OLP Interacted)
                                              // Value: 0 -> Show, 1 -> Set
#define     RSVC_SERV_REBOOT_SYSTEM      59   // Reboot System
                                              // Value: 0 -> Reboot, 1 -> Shutdown
#define     RSVC_SERV_CONSTVAR_INIT      60   // Initialize Const Variables
                                              // Value: Const Var Type (0: P, 1: B, 2: I, 3: R)

    // commnad process
#define     RSVC_JOB_CMDJUMP             90   // Job Jump Cmd Process(TE use only)
                                              // Msg: Caller/Callee Name string, Addr int
                                              // Reply: if Success,Caller/Callee Name string, Addr int
#define     RSVC_JOB_CMDCALL             91   // Job Call Cmd Process(TE use only)
                                              // Msg: Caller/Callee Name string, Addr int
                                              // Reply: if Success,Caller/Callee Name string, Addr int
#define     RSVC_JOB_CMDRETURN           92   // Job Return Cmd Porcess(TE use only)
                                              // Reply: if Success,Return Target Name string, Addr int
#define     RSVC_JOB_CMDCONTINUE         93   // Release Pause Command

    // for developer service
#define     RSVC_DSP_STATISTICSDATA      94   // Display Statistics Data
                                              // Value: 0 -> display off, 1 >= display y position
#define     RSVC_SET_SERVOCONTROL_TIME   95   // Set Interpolation time & Trajectory Cal Time
                                              // By data in config file
 

///////////////////////////////////////
//
// job execution state
//
// IDLE: idle state (no job exec) (nWorkType = WORK_TYPE_NONE)
// PREPARATION: preparing the job execution (RM preparing the job)
// WARM_EXEC: warming up job execution (RM order job exec, TE preparing)
// WARM_EXCEPT: warming up exceptional execution 
//                              (RM order exceptional exec, TE preparing)
// EXECUTING: job program running (TE running) (nWorkType = EXEC_TYPE_JOB)
// EXCEPT:exceptional program running (nWorkType = WORK_TYPE_HOME, ...)
// TERMINATING: job execution terminating... (TE stopped, RM clearing)

#define     EXEC_STAT_IDLE               0
#define     EXEC_STAT_PREPARATION        1
#define     EXEC_STAT_WARM_EXEC          2
#define     EXEC_STAT_WARM_EXCEPT        3
#define     EXEC_STAT_EXECUTING          4
#define     EXEC_STAT_EXCEPT             5
#define     EXEC_STAT_TERMINATING        6

// service exceptional working motion
#define     WORK_TYPE_NONE               0
#define     WORK_TYPE_JOB                1   // job executing...
#define     WORK_TYPE_HOME               2   // HOME_MOVE
#define     WORK_TYPE_JOG                3
#define     WORK_TYPE_WIRECUT            4   // WIRE_CUT

// system mode
#define     MODE_STAT_ENTRY              1
#define     MODE_STAT_INIT               2
#define     MODE_STAT_MANUAL             3
#define     MODE_STAT_AUTORUN            4
#define     MODE_STAT_DRYRUN             5
#define     MODE_STAT_STEP               6
#define     MODE_STAT_ERROR              7
#define     MODE_STAT_ESTOP              8
#define     MODE_STAT_TERMINATE          9


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SET_GAPSKIPCOND
//
#define    SHOW_GAPSKIP_COND             0
#define    SET_GAPSKIP_COND              1

#define    GAPSKIP_COND_SKIP_ON          0
#define    GAPSKIP_COND_GAP_NONE         1
#define    GAPSKIP_COND_GAP_MIDDLE       2
#define    GAPSKIP_COND_GAP_BIG          3

///////////////////////////////////////
//
// Service Name: RSVC_SERV_SET_SKIPCOND
//
#define    SHOW_SKIP_COND                0
#define    LEFT_VERT_SKIP_ON             1
#define    RIGHT_VERT_SKIP_ON            2
#define    LEFT_COLLAR_SKIP_ON           3
#define    RIGHT_COLLAR_SKIP_ON          4
#define    HORIZONTAL_SKIP_ON            5
#define    LEFT_VERT_SKIP_OFF            11
#define    RIGHT_VERT_SKIP_OFF           12
#define    LEFT_COLLAR_SKIP_OFF          13
#define    RIGHT_COLLAR_SKIP_OFF         14
#define    HORIZONTAL_SKIP_OFF           15


//////////////////// Data Definition included in Packet ///////////////////////

///////////////////////////////////////
//
// Service Name: Test for Service
//
// MSG Data Format for test
//  - int tmp_0, tmp_1
#if 0
#pragma pack(push, 1)
typedef struct t_serv_test_msg_data
{
    int tmp_0; 
    int tmp_1; 
} SERV_TEST_MSG_DATA;
#pragma pack(pop)
#endif

/////////////////////////////////////////////////////////////////////
// Pre-defined Structure, not for direct use
// Used in SHM, Services
// Service Name: RSVC_SERV_SHOWPARAM_TP
//               RSVC_SERV_PARAM_EDIT
//
#pragma pack(push, 1)
typedef struct  t_weld_tune_input
{
    double  dbVolt_a;
    double  dbVolt_b;
    double  dbVoltScale;
    double  dbVoltOffset;

    double  dbCurr_a;
    double  dbCurr_b;
    double  dbCurrScale;
    double  dbCurrOffset;
} WELD_TUNE_INPUT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  t_weld_tune_output
{
    double  dbVolt_a;
    double  dbVolt_b;
    double  dbVolt_c;
    double  dbVoltScale;
    double  dbVoltOffset;

    double  dbCurr_a;
    double  dbCurr_b;
    double  dbCurr_c;
    double  dbCurrScale;
    double  dbCurrOffset;
    //double  dbGain;         //not used
} WELD_TUNE_OUTPUT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  t_weld_param_tune
{
    // weld analog input tuning
    WELD_TUNE_INPUT     input;
    
    // weld analog output tuning
    WELD_TUNE_OUTPUT    output;

} WELD_PARAM_TUNE;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct t_te_restart_param
{
    // restart control var's
    unsigned moving_type;   // RESTART_MOVE_TYPE_LIN & _JNT (refer to 'ipc_taskexec.h')
#if 0
	double   over_dist;
#endif 
    double   d_overlap_horz;        // [mm] Horizontal Overlap Distance
    double   d_overlap_vert;        // [mm] Vertical   Overlap Distance
	
	double   path_speed;    // [%]:for joint motion (0~100, TE internal 0~1) & [mm/s] for linear    
	double   hori_start_vol;
	double   hori_start_cur;
	double   hori_main_vol;
	double   hori_main_cur;
	double   vert_start_vol;
	double   vert_start_cur;
	double   vert_main_vol;
	double   vert_main_cur;
} TE_RESTART_PARAM;  
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  t_arcsensor_param
{
    unsigned        fSaveArcSensorData;
    unsigned        nStartSaveNodeNo;
    unsigned        nSaveNodeCount;
} ARCSENSOR_PARAM;
#pragma pack(pop)

#pragma pack(1)
typedef struct  t_format_time
{
    int nHour;          // hours since midnight - [0,23]
    int nMinute;        // minutes after the hour - [0,59]
    int nSecond;        // seconds after the minute - [0,59]
} FORMAT_TIME;
#pragma pack()

#pragma pack(1)
typedef struct  t_format_date
{
    int nYear;          // current year [1900 ~ 2006 ~ ]
    int nMonth;         // current month [1,12]
    int nDay;           // day of the month - [1,31]
} FORMAT_DATE;
#pragma pack()


/////////////////////////////////////////////////////////////////////


///////////////////////////////////////
//
// Service Name: RMGR_SERV_SYSVERSION
//
// Reply Data Format for version check
// - rm_vers: version information of RM process
// - rm_build: build number information of RM process

#define     RMGR_VERSION_DATA_LEN        16
#define     RMGR_BUILD_DATA_LEN          16

#pragma pack(push, 1)
typedef struct t_rmgr_ver_reply_data
{
    char rgchRM_vers[RMGR_VERSION_DATA_LEN]; 
    char rgchRM_build[RMGR_BUILD_DATA_LEN]; 
} RMGR_VER_REPLY_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RMGR_SERV_SYSSTATE
//
// Reply Data Format for system state check
// - ErrCode: error code (included in error_def.h)
// - EStopCode: estop code (included in error_def.h)
// - nExecStat: execution state (EXEC_STAT_IDLE..)
// - nWorkType: service for exceptional (WORK_TYPE_NONE..)


#pragma pack(push, 1)
typedef struct t_rmgr_state_reply_data
{
    BYTE   nErrCode;         // error code
    BYTE   nEstopCode;       // estop code
    BYTE   nExecStat;        // execution state
    BYTE   nWorkType;        // service for exceptional
    BYTE   nSystemMode;      // system mode
} RMGR_STATE_REPLY_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RMGR_SERV_ERRHISTORY
//
// Message Data Format for Error History
#define     MAX_ERROR_HISTORY_SND_SIZE      16
#define     SYSTIME_DATA_LEN                20

#define     ERROR_NAME_LEN                  21

// Request packet
#pragma pack(push, 1)
typedef struct t_rmgr_err_history_req_data
{
    int    nErrHistoryStartIdx;
    int    nErrHistoryReqCnt;
} RMGR_ERR_HISTORY_REQ_DATA;
#pragma pack(pop)

// Reply packet
#pragma pack(push, 1)
typedef struct t_rmgr_err_history_reply_data
{
    int    nErrCnt;
    char   szErrStackSysTime[MAX_ERROR_HISTORY_SND_SIZE][SYSTIME_DATA_LEN]; // error time stack
    int    nErrorCodeStack[MAX_ERROR_HISTORY_SND_SIZE];                     // error code stack
    short  fErrorActiveState[MAX_ERROR_HISTORY_SND_SIZE];                   // active flag stack
    char   szErrContent[MAX_ERROR_HISTORY_SND_SIZE][ERROR_NAME_LEN];        // error description
} RMGR_ERR_HISTORY_REPLY_DATA; 
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RMGR_SERV_GETSTATISTIC
//
// Message Data Format for Get System Statistics Data
// 

#pragma pack(push, 1)
typedef struct t_rmgr_statistics_data
{
    FORMAT_DATE     DateReset;    // statistics data reset date
    FORMAT_TIME     TimeReset;    // statistics data reset time
    
    FORMAT_TIME     TimePowerOn;  // power on time
    FORMAT_TIME     TimeServoOn;  // servo enable time
    FORMAT_TIME     TimeJobExec;  // job execution time
    FORMAT_TIME     TimeError;    // error or fault time
    FORMAT_TIME     TimeStop;     // robot stop time by error, fault, estop
    FORMAT_TIME     TimeMove;     // robot moving time
    FORMAT_TIME     TimeWeld;     // total welding time

    double          dbDistFilletWeld;  // fillet joint welding distance
    double          dbDistWeavWeld;    // weav welding distance
} RMGR_STATISTICS_DATA;
#pragma pack(pop)

///////////////////////////////////////
//
// Service Name: RMGR_SERV_SYSINFO
//
// Belongs to RMGR_INFO_REPLY_DATA, Robot Info.
#pragma pack(push, 1)
typedef struct t_rmgr_info_robot
{
    char        szRobotName[ROBOT_NAME_LEN];
    int         fUsed;                     
    int         nRobotType;         // ROBOT_TYPE_...
   
    // maximum speed
    double      dbMaxJointSpeed[ROB_AXIS_COUNT];    //[rad/ms]
    double      dbMaxLinearSpeed;   // [mm/ms]
    double      dbMaxOrientSpeed;   // [rad/ms]

    // Acceleration / Deceleration
    double      dbAccel;            // [ms]
    double      dbDecel;            // [ms]    
    double      dbJerk;             // [ms] Must less then Accel value(0:Trapezoidal)
    
    // Deceleration by Event
    double      dbDecel_Error;      // [ms]
    double      dbDecel_Estop;      // [ms]
    double      dbDecel_Touch;      // [ms]

	 // DH Params 
    DH_PARAMS   dh[ROB_AXIS_COUNT]; 
	int         nAxesCount;
} RMGR_INFO_ROBOT;
#pragma pack(pop)

///////////////////////////////////////
//
// Service Name: RMGR_SERV_SYSINFO
//
// Reply Data Format for system info check
// Value: 0 -> Global(Control), 1-> Global(Ecat),
//        2 -> Robot, 3 -> Axis, 4 -> Motor, 5 -> Welder
#pragma pack(push, 1)
typedef struct t_rmgr_info_reply_data
{
    int nReqSystemIndex;            // Request Index

    union info
    {
        CONFIG_CTRL         shm_ctrl;   // value: 0 (Idx: N/A)
        CONFIG_ECAT         shm_ecat;   // value: 1 (Idx: N/A)
        RMGR_INFO_ROBOT     shm_robot;  // value: 2 (Idx: 0(0~4 available))
        CONFIG_AXIS         shm_axis;   // value: 3 (Idx: 0~5(0~32 available))
        CONFIG_MOTOR        shm_motor;  // value: 4 (Idx: 0~5(0~32 available))
        CONFIG_WELDER       shm_welder; // value: 5 (Idx: 0(0~32 available))
    } INFO;
} RMGR_INFO_REPLY_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_JOBLOAD
//               RSVC_SERV_JOBFILESAVE
//
// Message Data Format for Job Load

#pragma pack(push, 1)
typedef struct t_rmgr_job_load_data
{
    // job file info(file name)
    char    szJobFileName[JOB_MODULE_NAME_SIZE];
} RMGR_JOB_LOAD_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOBFILE_ZIP_UNCOMPRESS
//
// Message Data Format for Unzip
#pragma pack(push, 1)
typedef struct t_rmgr_job_unzip_data
{
    // job file info(file name)
    char    szJobZipFileName[JOB_MODULE_NAME_SIZE];
} RMGR_JOB_UNZIP_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SYSMON_TO_TP
//
// System Monitoring Data (for External)

#pragma pack(push, 1)
typedef struct t_rmgr_system_mon_data
{
    int   nErrCode;                       // error code
    int   nErrOwner;                      // error owner
    int   fEstopState;                    // estop state
    int   nEstopCode;                     // estop code
    int   fEcatRunState;                  // EtherCAT Master init. state 

    /*  Refer to job execution state symbol */
    int   nExecStat;                      // execution state (EXEC_STAT_IDLE, EXEC_STAT_EXECUTING..)
    int   nWorkType;                      // working type (WORK_TYPE_NONE, WORK_TYPE_JOB..)
    int   nSystemMode;                    // system mode (MODE_STAT_INIT, MODE_STAT_AUTORUN..)

    int   nCmdLoadCount;                  // loaded job total index
    int   nJobRunIndex;                   // job run Index
    char  szJobFileName[JOB_MODULE_NAME_SIZE];
    int   fDeadManState;                  // deadnman switch state

    int   fServoOnOutState;               // servo on/off state
    int   nJobLoadCnt;                    // count of job load (if sucess, count add one)
} RMGR_SYSTEM_MON_DATA;
#pragma pack(pop)
 
 
///////////////////////////////////////
//
// Service Name: RSVC_SERV_JOBLINE_DSP
//
// Message Data Format for Job Display

#define     MAX_CMD_CHAR_SIZE             92

#pragma pack(push, 1)
typedef struct t_rmgr_job_dsp_data
{
    int     nCmdLoadCount;
    char    szCommand[MAX_CMD_CHAR_SIZE];
} RMGR_JOB_DSP_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SHOWHOME_TP
//               RSVC_SERV_HOME_EDIT
//

#pragma pack(push, 1)
typedef struct t_rmgr_home_data
{
    int     nHomeIndex;
    double  rgdbHomePosVal[ROB_AXIS_COUNT];
} RMGR_HOME_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SHOWUSRCRD_TP
//               RSVC_SERV_USERCRD_EDIT
//

#pragma pack(push, 1)
typedef struct t_rmgr_usercrd_data
{
    int             nUserCoordIndex;
    COORD_EULER     usercoord;
} RMGR_USERCRD_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RCON_VOLT_REALTIME_OFFSET
//

#pragma pack(push, 1)
typedef struct t_rmgr_volt_reatime_offset
{
    double dbVoltRealTimeCmdOffsetUnit;
    double dbVoltRealTimeCmdOffset;
} RMGR_VOLT_REALTIME_OFFSET;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RCON_CURR_REALTIME_OFFSET
//

#pragma pack(push, 1)
typedef struct t_rmgr_curr_reatime_offset
{
    double dbCurrRealTimeCmdOffsetUnit;
    double dbCurrRealTimeCmdOffset;
} RMGR_CURR_REALTIME_OFFSET;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SHOWPARAM_TP
//               RSVC_SERV_PARAM_EDIT
// Value: 0 -> TCP, Value: 1 -> world , Value: 2 -> weld tune input
// Value: 3 -> weld tune output, Value: 4 -> restart param
//
// Service Name: RSVC_SERV_WELDTUNE_IN_EDIT
//               RSVC_SERV_WELDTUNE_OUT_EDIT
//               RSVC_SERV_SHOWWELDTUNE_IN
//               RSVC_SERV_SHOWWELDTUNE_OUT
// Value: Wire Index

#pragma pack(push, 1)
typedef struct t_rmgr_user_param_data
{
    union param
    {
        COORD_EULER         TCP;
        COORD_EULER         world;
        WELD_TUNE_INPUT     input;      // weld analog input tuning
        WELD_TUNE_OUTPUT    output;     // weld analog output tuning
        TE_RESTART_PARAM    restart;    // TE Restart parameters
        ARCSENSOR_PARAM     arcsensor;  // ArcSensor parameters
        //
        // ..
        //
    } PARAM;
} RMGR_USER_PARAM_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOB_CMDJUMP
//               RSVC_JOB_CMDCALL

#pragma pack(push, 1)
typedef struct t_rmgr_jobcall_stack_data
{
    // caller job file info
    char    szCallerFileName[JOB_MODULE_NAME_SIZE];
    int     nCallerAddr;

    // caller job file info
    char    szCalleeFileName[JOB_MODULE_NAME_SIZE];
    int     nCalleeAddr;
} RMGR_JOBCALL_STACK_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOB_CMDRETURN
//

#pragma pack(push, 1)
typedef struct t_rmgr_jobreturn_data
{
    // caller job file info
    char    szReturnTargetFileName[JOB_MODULE_NAME_SIZE];
    int     nReturnTargetAddr;
} RMGR_JOBRETURN_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SET_GAPCOND
//

#pragma pack(push, 1)
typedef struct t_rmgr_gapcond_data
{
    int fLeftSkip;          // Left  Weld Skip Set flag (1: skip, 0: not skip)
    int fRightSkip;         // Right Weld Skip Set flag (1: skip, 0: not skip)

    int nHorzGapCond;       // Horizontal Gap Condition
    int nVertLeftGapCond;   // Vertical Left Gap Condition (GAP_COND_NONE, SMALL, BIG,..)
    int nVertRightGapCond;  // Vertical Right Gap Condition
} RMGR_GAPCOND_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SET_SKIPCOND
//

#pragma pack(push, 1)
typedef struct t_rmgr_weld_skip_data
{
    int fLeftVertSkip;     // Left  Vertical Weld Skip Set flag (1: skip, 0: not skip)
    int fRightVertSkip;    // Right Vertical Weld Skip Set flag (1: skip, 0: not skip)
    int fLeftCollarSkip;   // Left  Collar   Weld Skip Set flag (1: skip, 0: not skip)
    int fRightCollarSkip;  // Right Collar   Weld Skip Set flag (1: skip, 0: not skip)
    int fHorizontalSkip;   // Horizontal     Weld Skip Set flag (1: skip, 0: not skip)
} RMGR_WELD_SKIP_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_SERV_SET_GAPSKIPCOND
//
#pragma pack(push, 1)
typedef struct t_rmgr_weld_gapskip_data
{
    int nLeftVertGapSkipCond;     // Left  Vertical Weld Gap/Skip Set Cond (Symbol)
    int nRightVertGapSkipCond;    // Right Vertical Weld Gap/Skip Set Cond (Symbol)
    int nLeftCollarGapSkipCond;   // Left  Collar   Weld Gap/Skip Set Cond (Symbol)
    int nRightCollarGapSkipCond;  // Right Collar   Weld Gap/Skip Set Cond (Symbol)
    int nHorizontalGapSkipCond;   // Horizontal     Weld Gap/Skip Set Cond (Symbol)
    int nLeftBracketGapSkipCond;  // Left  Bracket  Weld Gap/Skip Set Cond (Symbol)
    int nRightBracketGapSkipCond; // Right Bracket  Weld Gap/Skip Set Cond (Symbol)
} RMGR_WELD_GAPSKIP_DATA;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOB_CONSTVARINFO
//               RSVC_JOB_WELDVARINFO

#pragma pack(push, 1)
typedef struct t_rmgr_job_var_mon_req
{
    int     nVarIndex;                  // Request Monitoring
} RMGR_JOB_VAR_MON_REQ;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOB_CONSTVARINFO
//               RSVC_JOB_CONSTVAREDIT
//

#pragma pack(push, 1)
typedef struct t_rmgr_job_const_var_val
{
    int     nVarIndex;                  // Request Editing
    union
    {
        BYTE    ByteVal;
        INT     IntVal;
        double  RealVal;
    } value;
} RMGR_JOB_CONST_VAR_VAL;
#pragma pack(pop)


///////////////////////////////////////
//
//  Service Name: RSVC_JOB_CONSTVARINFO
//                RSVC_JOB_CONSTVAREDIT
//
//  robot position
//  joint : all axis unit = degree
//  carte : 0~2=millimeter, 3~=degree

#pragma pack(push, 1)
typedef struct t_rmgr_job_pos_var_val
{
    int     nVarIndex;                  // Request Editing
	double  dbPosVal[ROB_AXIS_COUNT];   // Joint or World Cartesian
	int     nCoordIdx;                  // DANDY_JOB_POS_JOINT/CART
} RMGR_JOB_POS_VAL;
#pragma pack(pop)


///////////////////////////////////////
//
// Service Name: RSVC_JOB_WELDVARINFO
//               Refer to dandy_job.h

typedef DANDY_JOB_WEAV  RMGR_JOB_WEAV;  // rgdbWeavCond[DANDY_JOB_WVF_ELEMENT_COUNT]
typedef DANDY_JOB_SWF   RMGR_JOB_SWF;   // rgdbWeldCond[DANDY_JOB_SWF_ELEMENT_COUNT]
typedef DANDY_JOB_MWF   RMGR_JOB_MWF;   // rgdbWeldCond[DANDY_JOB_MWF_ELEMENT_COUNT]
typedef DANDY_JOB_EWF   RMGR_JOB_EWF;   // rgdbWeldCond[DANDY_JOB_EWF_ELEMENT_COUNT]


///////////////////////////////////////
//
// Service Name: RSVC_JOB_BIN_SNDHEADER
//               RSVC_JOB_BIN_RCVHEADER
//               RSVC_JOB_COMP_BIN_SNDHEADER
//               RSVC_JOB_COMP_BIN_RCVHEADER

#pragma pack(push, 1)
typedef struct t_rmgr_job_bin_header
{
    unsigned long   dwRawBinLoadSize;
    unsigned long   dwCompBinLoadSize;  //In raw data service, set zero
    int             nBodyDivIndex;

    unsigned long   dwCmdLoadCount;
    unsigned long   dwTVaLoadCount;
    unsigned long   dwWvfLoadCount;
    unsigned long   dwSwfLoadCount;
    unsigned long   dwMwfLoadCount;
    unsigned long   dwEwfLoadCount;

    int             nJobExecLineIdx;
} RMGR_JOB_BIN_HEADER;
#pragma pack(pop)

//////////////////// Packet Definition for MSG, Pulse /////////////////////////

///////////////////////////////////////
//
// Message Packet Fomat for common use
//     in case of Message(Reply) & Pulse
//
//  consists of 
// nCode: Service Code(Required)
// nValue: Service Option(Required)
// nDataSize: Size of data(Required)
// data: Service Contents(Optional)
#if 0
#define     RMGR_PACKET_LEN             108
#define     RMGR_PACKET_DATA_LEN        96
#define     RMGR_PACKET_HEAD_LEN        12

#define     RMGR_REPLY_PACKET_LEN       108
#define     RMGR_REPLY_PACKET_DATA_LEN  96
#define     RMGR_REPLY_PACKET_HEAD_LEN  12
#endif

#if 1
#define     RMGR_PACKET_LEN             1036
#define     RMGR_PACKET_DATA_LEN        1024
#define     RMGR_PACKET_HEAD_LEN        12

#define     RMGR_REPLY_PACKET_LEN       1036
#define     RMGR_REPLY_PACKET_DATA_LEN  1024
#define     RMGR_REPLY_PACKET_HEAD_LEN  12
#endif
#pragma pack(push, 1)
typedef struct t_rmgr_packet
{
    unsigned nCode; 
    unsigned nValue; 
    unsigned nDataSize;
    
    // message body
    union rm_msg_data
    {
        BYTE data[RMGR_PACKET_DATA_LEN]; 

    // For data of MSG 
        RMGR_VER_REPLY_DATA           reply_ver;
        RMGR_STATE_REPLY_DATA         reply_state;
        RMGR_INFO_REPLY_DATA          sys_info;
        RMGR_ERR_HISTORY_REPLY_DATA   errhistory_reply;
        RMGR_ERR_HISTORY_REQ_DATA     errhistory_req;
        RMGR_STATISTICS_DATA          statistics;
        
        RMGR_JOB_LOAD_DATA            job_load;
        RMGR_JOB_DSP_DATA             job_dsp;
        RMGR_SYSTEM_MON_DATA          sys_mon;
        RMGR_HOME_DATA                home_dat;
        RMGR_USERCRD_DATA             usrcrd_dat;
        RMGR_USER_PARAM_DATA          usrparam;
        RMGR_JOBCALL_STACK_DATA       call_cmd;
        RMGR_JOBRETURN_DATA           return_cmd;
        RMGR_GAPCOND_DATA             gap_cond;
        RMGR_WELD_SKIP_DATA           skip_cond;
        RMGR_WELD_GAPSKIP_DATA        gapskip_cond;
        RMGR_JOB_UNZIP_DATA           job_unzip;

        RMGR_JOB_VAR_MON_REQ          var_mon_req;
        RMGR_JOB_CONST_VAR_VAL        const_var_val;
        RMGR_JOB_POS_VAL              pvar_val;
        RMGR_JOB_POS_VAL              tvar_val;
        RMGR_JOB_WEAV                 wvf_val;
        RMGR_JOB_SWF                  swf_val;
        RMGR_JOB_MWF                  mwf_val;
        RMGR_JOB_EWF                  ewf_val;
        RMGR_JOB_BIN_HEADER           snd_header;
        RMGR_JOB_BIN_HEADER           rcv_header;

        RMGR_VOLT_REALTIME_OFFSET     volt_offset;
        RMGR_CURR_REALTIME_OFFSET     curr_offset;
    } Data;
} RMGR_PACKET, RMGR_REPLY_PACKET; 
#pragma pack(pop) 


///////////////////////// Shared Memory Definition ////////////////////////////

///////////////////////////////////////
//
//  Shared memory for System Status
//      -fInit+Pro+name: check for process init done state
//      -fExit+Pro+name: check for process Exit done state
//      -nExecStat: execution state 
//      -nWorkType: service for exceptional
//      -fErrorState: error state flag
//      -fEStopState: estop stae flag

#define     PATH_NAME_BUFFER_SIZE       256

#pragma pack(push, 1)
typedef struct t_shm_rm_sysstatus
{
    // check for the size of shared memory
    int     nSize; 

    // flag for initialize done state check
    BOOL    fInitProcTE;
    BOOL    fInitProcSC;
    
    // flag for finalize done state check
    BOOL    fExitProcTE;
    BOOL    fExitProcSC;

    // system runtime mode
    int     nExecStat;          // execution state (EXEC_STAT_IDLE..)
    int     nWorkType;          // service for exceptional (WORK_TYPE_NONE..)

    // flag for error & estop state
    BOOL    fErrorState;                  // error state flag
    BOOL    fEStopState;                  // estop stae flag

    // error & estop code
    int     nErrCode;                     // error code
    int     nEstopCode;                   // estop code

    // system mode
    int     nSystemMode;
    
    // job run Index
    int     nJobRunIndex;

    // job file info(file name)
    int     fJobLoadDone;              // success: 1, fail: -1
    char    szTargJobFileName[PATH_NAME_BUFFER_SIZE];
    char    szCurrJobFileName[PATH_NAME_BUFFER_SIZE];

    // Gap condition Setting Value
    int fLeftSkip;          // Left  Weld Skip Set flag (1: skip, 0: not skip)
    int fRightSkip;         // Right Weld Skip Set flag (1: skip, 0: not skip)

    int nVertLeftGapShift;   // Vertical Left Gap Condition (GAP_COND_NONE, SMALL, BIG,..)
    int nVertRightGapShift;  // Vertical Right Gap Condition
    int nHorzGapShift;       // Horizontal Gap Condition
    
    int fEstopDone;      // Check Estop Process Done!

} SHM_RM_SYSSTATUS;
#pragma pack(pop)


///////////////////////////////////////
//
//  Shared memory for System Parameters
//      -SHM for frequently modifiable parameter

#pragma pack(push, 1)
typedef struct t_shm_rm_sysparam
{
    // check for the size of shared memory
    int     nSize;
    
    WELD_PARAM_TUNE     weld_tune[MAX_WELDER_COUNT];
    double  dbAinMaxVolt;
    double  dbADCMaxBit;
    int     nWeldTuneInParamApplyIndex;
    int     nWeldTuneOutParamApplyIndex;
    char    szSensDir[PATH_NAME_BUFFER_SIZE];
    int     nEstopGasOffDelayTime;
    int     nEstopTouchReadyOffDelayTime;

    TE_RESTART_PARAM    TE_restart[MAX_ROBOT_COUNT];

    ARCSENSOR_PARAM     arcsensor[MAX_ROBOT_COUNT];
    
} SHM_RM_SYSPARAM;
#pragma pack(pop)

#endif  // end of __IPC_ROBOTMGR_H__
