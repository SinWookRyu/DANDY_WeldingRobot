#ifndef IPC_TASKEXEC_H_
#define IPC_TASKEXEC_H_

#include "dandy_platform.h"

#define TE_VERSION			"10.0b"
#define TE_BUILD			"2014.03.12"

#define SHMNAME_TE_TEST     "SHM_TE_TEST"
#define SHMNAME_RESTART     "SHM_RESTART"
#define SHMNAME_SENSOR_RM   "SHM_SENSOR_RM"
#define SHMNAME_SENSOR_SC   "SHM_SENSOR_SC"
#define TE_CHANNEL_NAME     "CHANNEL_TE"
#define RUN_CHANNEL_NAME    "CHANNEL_RUNTIME"
        
////////////////////////////////////////////////////////////////////////////////
// TE Services
// 
// 1 Service Consists of 
// - MSG            (or PULSE) 
// - REPLY          (Required if MSG)
// 
// 1 PULSE consists of 
// - Service Code
// - Service Value
// 
// 1 MSG (REPLY) consists of 
// - Service Code   (Required)
// - Service Value  (Required)
// - Size of Data   (Required)
// - Data of MSG    (Optional as Size of Data)
// 
//
// Service Code             	// Value Format | MSG Data Format   | REPLY Data Format | Remarks
// -----------------------------------------------------------------------------
typedef enum                    
{
    TESERV_UNDEFINED    = -1, 
    TESERV_EXIT     	= 0,    // -            | -                 | -                 |       
    TESERV_VERSION      = 1,    // -            | msg_vers          | rep_vers          | TE Service Help Print Also.
    TESERV_INIT         = 2,    // -            | -                 | -                 |
    TESERV_LIFE_CHK     = 3,    // -            | -                 | -                 | No Message Out	
    TESERV_STOP         = 4,    // Norm:0 Quick:1| -                | -                 |
    TESERV_JOG          = 5,    // VALUE_JOG    | -                 | -                 | MAKE_VALUE_JOG to make Value.
    TESERV_PROG_THRU,           // Index        | -                 | -                 | 
    TESERV_PROG_STEP,           // Index        | -                 | -                 | 
    TESERV_PROG_DRY,            // Index        | -                 | -                 | -
    TESERV_RESTART,             // -            | -                 | -                 |
#if 0
    TESERV_VGA,                 // ON:1, OFF:0  | -                 | -                 |
#endif 
    TESERV_DISP_TOGGLE,         // 0:Help Els:Obj| -                | -                 |
    TESERV_SAMPTIME,		    // [ms] (>0)    | -                 | -                 |
    TESERV_FILE,                // Open:1 Cls:0 | -                 | -                 |
    TESERV_TIMETEST, 	        // -            | -                 | -                 |

    TESERV_RESET_ERR,           // -            | -                 | -                 |
#if 0
    TESERV_TRAJ_INFO,
#else
    TESERV_INFO_PRINT,          // 0:help EL:Obj| -                 | -                 |
#endif 
    TESERV_JOG_MODE,            // 1:WV, ELS:Nor| -                 | -                 |

    TESERV_JNT_2_CART,          // Coord Index  | JOINT             | XYZRPY            | 
    TESERV_DISP_COORD,          // COORD_INDEX
    TESERV_TIMER_SKIP,          // -            | -                 | -                 | 
    TESERV_PRG_VAR_CLR,         // 1:Stack 2:Restart | -            | -                 | 
    TESERV_WELD_IO_MAP,         // -            | -                 | -                 | 
    TESERV_PRG_CTR_VAR,         // -            | -                 | -                 | 
    TESERV_RUN_PAUSE,           // 1:Pause 0:Reset  | -             | -                 | 
#if 0
    TESERV_RESTART_PRT,         // -            | -                 | -                 | 
#endif 
} TESERV_CODE; 

////////////////////////////////////////////////////////////////////////////////
// Value Formats for Services     

// To make the value for TESERV_JOG. 
#define MAKE_VALUE_JOG(rob_, coord_, axis_, time_, speed_) \
  ( ( ((rob_)   & 0xff) << 0 )  |    \
    ( ((coord_) & 0xff) << 8 )  |    \
    ( ((axis_)  & 0x0f) << 16 ) |    \
    ( ((time_)  & 0x0f) << 20 ) |    \
    ( ((speed_) & 0xff) << 24 ) )
    
// Jog Value Format (Interal Processes Use Only, Don't External Machine use)
/*
#pragma pack(push, 1)
typedef struct t_msgval_jog
{
    BYTE        nRobNum; 
    char        coord;      // Coord Index(COORDINDEX_xxx). ex)COORDINDEX_BASE
    BYTE        axis   : 4; // Axis Index(AXIS_xxx). ex)AXIS_LIN_X    
    BYTE        t_keep : 4; // Jog Keep Time = 2^t_keep [ms]
    char        nSpeed;     // [%] -100~100:-100~100%, <-100:(nSpeed+100)*0.1, >100:(nSpeed-100)*0.1
} VALUE_JOG; 
#pragma pack(pop)
*/

////////////////////////////////////////////////////////////////////////////////
// MSG Formats for Services 

#pragma pack(push, 1)
typedef struct te_msgdata_vers
{
    int test_0; 
    int test_1; 
} TE_MSGDATA_VERS; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct te_replydata_vers
{
    char vers[16]; 
    char build[16]; 
} TE_REPLYDATA_VERS; 
#pragma pack(pop)

// Sizes 
// TE_MSG size of each Message  : TE_MSG_HEAD_SIZE + sizeof(struct msg_xxx)
// TE_REPLY size of each Reply  : TE_REPLY_HEAD_SIZE + sizeof(struct reply_xxx)
#define TE_MSG_HEAD_SIZE    12  // (sizeof(unsigned) + sizeof(unsigned) + sizeof(unsigned))
#define TE_REPLY_HEAD_SIZE  TE_MSG_HEAD_SIZE
#define TE_MSG_DATA_MAX     96
#define TE_REPLY_DATA_MAX   TE_MSG_DATA_MAX

// Format
#pragma pack(push,1)
typedef struct t_te_msg
{
    // header 
    unsigned    code; 
    unsigned    value;
    unsigned    size;   // size of body

    // body
    union te_msg_data
    {
        BYTE              dummy[TE_MSG_DATA_MAX]; 

        TE_MSGDATA_VERS   msg_vers; 
        TE_REPLYDATA_VERS rep_vers;  
        double            msg_joint2cart[6]; 
        double            rep_joint2cart[6]; 
    } data; 
} TE_MSG, TE_REPLY; 
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////
// TE Runtime Thread Services
// - Runtime Thread Supports Pulse Only. No supports MSG
// - RUNSERV_TIMER must be sent by the timer responsible thread only.(Recommanded strongly)
// - RUNSERV_EXIT must be sent by TE Runtime Thread Termination responsible thread only. 
#define RUNSERV_EXIT        0
#define RUNSERV_TIMER       1

////////////////////////////////////////////////////////////////////////////////
// Shared Memory by Taskexec

// Runtime Execution Mode 
#define RUNMODE_NONE            0
#define RUNMODE_TIMETEST        1
#define RUNMODE_JOG				2
#define RUNMODE_PROG            3
#define RUNMODE_RESTART         4

// Program Mode 
#define PROG_RUNMODE_NONE   0
#define PROG_RUNMODE_THRU   1   // Program Runs Through
#define PROG_RUNMODE_STEP   2   // Program Runs Step by Step
#define PROG_RUNMODE_DRY    3   // Program Runs Through Dry

// Runtime Error
#define ERR_NONE                0x0000    
#define ERR_NULL_PTR            0x0001  // program level error
#define ERR_NO_JOBSHM           0x0002
#define ERR_CMDINIT_FAIL        0x0003
#define ERR_UNSUPPORT_CMD       0x0004
#define ERR_INVALID_ARGVAL_TYPE 0x0005  // Invalid Arg Value Type
#define ERR_UNSUPPORT_VAL       0x0006  // 
#define ERR_NO_JNT_ACCESS       0x0007  // Rob-Pos Config is Joint. But Another Access is tried
#define ERR_NO_CART_ACCESS      0x0008  // Rob-Pos Config is Cart.  But Another Access is tried
#define ERR_UNSUPPORT_DBL_OPER  0x0009  // Unsupport Double Operation(Mod, And, ..)
#define ERR_UNSUPPORT_OPER_CMD  0x0010  // Unsupport Operation Cmd
#define ERR_UNSUPPORT_BRANCH_CMD 0x0011 // Unsupport Branch Cmd 
#define ERR_UNDEF_STEP          0x0012  // Undefined Step tried. 
#define ERR_STOP_REQ            0x0101  // system level error
#define ERR_QUICK_STOP_REQ      0x0102
#define ERR_ESTOP_ON            0x0103
#define ERR_MOTOR_ERR           0x0104
#define ERR_SERVO_OFF           0x0105
#define ERR_NO_RESTART_DATA     0x0106  // no restart data
#define ERR_NO_LMOT_DATA_SAVED  0x0107  // no lmot data saved(possible to be stopped by inv err)
#define ERR_NO_WEAVE_DATA_SAVED 0x0108  // no weave data saved
#define ERR_TRAJ_ERR_NO_RESTART 0x0109  // Traj Error(including Inverse Error, Lim Stop) cannot be restarted. 
#define ERR_INVALID_AXIS        0x0201  // 
#define ERR_INVALID_COORD       0x0202
#define ERR_INVALID_DIR         0x0203
#define ERR_WATCHDOG_OVER       0x0204  // jog time watch-dog over
#define ERR_TIME_OVER           0x0205
#define ERR_STACK_NOMORE_PUSH   0x0206
#define ERR_STACK_NOMORE_POP    0x0207
#define ERR_JOBLOAD_FAIL        0x0208
#define ERR_LESS_ARG_COUNT      0x0209  // Less Argument Count 
#define ERR_TCH_NOT_READY       0x0211  // Touch Sensor Ready Fail 
#define ERR_TOUCH_SENSOR_RESET_FAIL 0x0212  // Touch Sensor Ready Fail 
#define ERR_TOUCH_FAIL          0x0213  // Touch Failed 
#define ERR_TOUCH_ALREADY       0x0214  // Touch Already 
#define ERR_WELDER_POWER_FAIL   0x0220  // 
#define ERR_ARCON_SIG_FAIL      0x0221
#define ERR_GASON_SIG_FAIL      0x0222
#define ERR_CWEAV_SHORT_HORZ    0x0230  // Too Short Cweave Horzontal Distance
#define ERR_TRAJ_START          0x0300  // trajectory

// Traj Error -> Global Error : ERR_TRAJ_START & (0xff & traj_error)
// Traj Error Check : ERR_TRAJ_START & error_code == ERR_TRAJ_START
// Traj Error Code Getting : error_code & 0xff
#define ERR_ARG_ELEM_MISMATCH   0x0401  // 0x0400 Job Program Error 
#define ERR_DIV_BY_0            0x0402  // Divide by 0
#define ERR_WEAV_START          0x0500  // weaving 
#define ERR_INVERSE_START       0x0600  // 0x06xx is Inverse Error Error 

// Profile Secttion
#if 0
#define PROF_NONE      (0) // Not motion 
#define PROF_ACC       (1) // Acceleration Section 
#define PROF_UNI       (2) // Uniform Vel Section
#define PROF_DEC       (3) // Deceleration Section
#define PROF_OVER      (4) // Section All Over 
#else
#define PROF_NONE       (0) // Not motion 
#define PROF_A1         (1) // Jerk1 of Acceleration Section 
#define PROF_AU         (2) // Uniform Acceleration Section 
#define PROF_A2         (3) // Jerk2 of Acceleration Section 
#define PROF_U          (4) // Uniform Velocity Section
#define PROF_D1         (5) // Jerk1 of Deceleration Section 
#define PROF_DU         (6) // Uniform Deceleration Section 
#define PROF_D2         (7) // Jerk2 of Deceleration Section 
#define PROF_OVER       (8) // Section All Over 
#endif

// Program Section (file level)
#define SECT_NONE       0   
#define SECT_RUN        1   // Runtime Thread
#define SECT_JOG        2   // Run Jog Service
#define SECT_PROG       3   // 
#define SECT_CMD_MOV    4
#define SECT_CMD_TOUCH  5
#define SECT_CMD_WELD   6
#define SECT_CMD_PORT   7
#define SECT_CMD_BRANCH 8 
#define SECT_CMD_CWEAV  9
#define SECT_RESTART    10

#pragma pack(push, 1)
typedef union
{
    unsigned dummy; 
    struct
    {
        unsigned short code;    // error code        
        unsigned char  sect;    // error occured section (not mode)
        unsigned char  proc;    // error occured process (internally not used.)
    };  
} TE_ERROR; 
#pragma pack(pop)

#pragma pack(push,1)
typedef struct t_shm_te_test
{
    int size;           // size of SHM_TE_TEST   
    int run_mode;       // Mode of Runtime Module. Refer to RUNMODE_XXX_.    
    int run_prog_mod;   // Mode of Program running. Refer to PROG_RUNMODE_XXX..
    int run_prog_idx;   // Program Running Index    
    int run_next_idx;   // Next Program Index to run. <0:End of Job. Useful at Normal Mode. 
    int run_cmd_code;   // running command Index
    int run_f_weld;     // Flag for Weld Mode of Program. Sets at ARCON. 
    int run_stop;       // In Stopping of Runtime Service
    TE_ERROR run_error; // Error Codes of Runtime Service. Refer to RUNERR_XXX.
    int prof_sect;      // Motion Profile Section            
    int coord_ref;      // Ref. Coord of 'xyzrpy_act'. 
    int f_restart_exist;// Indicator for Restart Data Exist
    double xyzrpy_act[6];   // Cart Info of Actual Joint Pos. 
    double dist_work;   // [mm] working distance. Valid when Mot is CART. 
    
} SHM_TE_STATUS; 
#pragma pack(pop)

// TE Program mode Info's to restart
// This shm must be refered at SHM_TE_STATUS::run_mode == RUNMODE_NONE. 

#define RESTART_MOVE_TYPE_JNT   0
#define RESTART_MOVE_TYPE_LIN   1

#pragma pack(push,1)
typedef struct t_shm_te_restart
{
    unsigned size_total;            // size of this-SHM + RESTART_DATA(for saving)

    // restart control var's
    unsigned moving_type;           // RESTART_MOVE_TYPE_LIN & _JNT
#if 0
	double   over_dist;
#endif 
    double   d_overlap_horz;        // [mm] Horizontal Overlap Distance
    double   d_overlap_vert;        // [mm] Vertical   Overlap Distance
	
	double   path_speed;            // [0<,<=1]:for joint mot & [mm/ms] for lin    
    double   hori_start_vol; 
	double   hori_start_cur; 
	double   hori_main_vol; 
	double   hori_main_cur; 
	double   vert_start_vol; 
	double   vert_start_cur; 
	double   vert_main_vol; 
	double   vert_main_cur; 

    // Another restart data buffer for saving is needed. 

} SHM_RESTART; 
#pragma pack(pop)

// !! WARNING !! These Types must be used for Arc-sensor. 
#ifndef byte
typedef unsigned char  byte;
#endif
#ifndef word
typedef unsigned short word;
#endif
#ifndef dword
typedef unsigned long  dword;
#endif
#ifndef uint
typedef unsigned int   uint;
#endif

#define NUM_OF_NODE		4000
#define DEF_NODE_ADD    200
#define RDATA_SIZE		1000
#define ARC_ARRAY_SIZE	4000

// Arc Sensor Data Used with RM
#pragma pack(push, 1)
typedef struct 
{		
    uint size;          // size of this shm
    
    // V_shunt -> I_weld. I_weld = V_shunt * c_a + c_b;	
    // V_shunt = ADC * AIN_MAX / ADC_MAX   
    struct 
    {   
	    double c_a;
	    double c_b;
        double ain_max; // [V] Max. Voltage of A-In Port for Current Measuring
        double adc_max; // Max ADC Value of A-In Port for Current Measuring
    } wm;   

    // arc-sensro save control var's
    struct
    {
	    uint status;    // save on/off flag
	    uint start;     // node index of arc sensor to start saving 
	    uint number;    // node count to save
    } sdata_save_cond;
} shm_mmi_task_t;
#pragma pack(pop)

// Arc Sensor Data Used with SC
#pragma pack(push, 1)
typedef struct
{
    uint size;                      // size of this shm

    // ADC Aquisition Var's
    word ADC_gathering_request;     // ADC_Data gathering status : ON/OFF
    word ADC_gathering_index;       // ADC_Data gathering index
	word ADC_Data[ARC_ARRAY_SIZE];  // arc sensor data. V=(ADC*9.9954)/4096
	
    // Arc-sensor data for SC to save 
    double Mean_Moving_Weight[NUM_OF_NODE]; // Measured Weight wrt Even 'yw'
    double Mean_Moving_Ampere[NUM_OF_NODE]; // Measured Current Value
    double Delta_T[NUM_OF_NODE];    // correction value wrt Even 'yw' dir. 
	double Delta_Z[NUM_OF_NODE];    // correction value wrt Current value. 
    uint arc_save_num;              // Data Saved Count 
    word Rdata[DEF_NODE_ADD][RDATA_SIZE];
    uint n_rdata[DEF_NODE_ADD];     // R-Data Count of Each Node
    
    // Welding & Weaving Condition Data to save
    double Iw;                      
    double Vw; 
    double pitch; 
    double width; 
    double speed; 
    double dwl_e;   // dwell time of even node
    double dwl_o;   // dwell time of odd node                

    // Saving Data of Correction of Weaving Module
    double d_xw[NUM_OF_NODE];       // Correction wrt xw dir at a node of main weaving
    double d_yw[NUM_OF_NODE];       // Correction wrt yw dir at a node of main weaving
    double d_zw[NUM_OF_NODE];       // Correction wrt zw dir at a node of main weaving
} shm_task_servo_t; 
#pragma pack(pop)

#endif

