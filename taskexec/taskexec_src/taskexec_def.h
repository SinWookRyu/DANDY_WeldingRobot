#ifndef TE_DEF_H_
#define TE_DEF_H_

#include "stdio.h"
#include "float.h"

#include "dandy_platform.h"
#include "dandy_echo.h"
#include "dandy_msgpass.h"
#include "dandy_shmem.h"
#include "dandy_thread.h"
#include "dandy_timecon.h"
#include "dandy_debug.h"
#include "dandy_job.h"

#include "argument.h"
#include "utility.h"
#include "ipc_robotmgr.h"
#include "ipc_taskexec.h"
#include "ipc_servocon.h"
#include "ipc_jobshm.h"
#include "dandy_job.h"
#include "CRT.h"
#include "math.h"
#include "sys_conf.h"
#include "init.h"
// #include "te_serv.h"
// #include "runtime.h"
#include "error.h"
#include "str_conv.h"
// #include "trajectory.h"
// #include "run_serv.h"
#include "weave.h"
#include "geometry.h"
#include "kinematics.h"
#include "sensor.h"
#include "call_stack.h"


#if defined (__QNXNTO__)
#if 0
#include "ecatmkpa.h"
#include "libmkpaiodev.h"
#include "mkpaauxiliary.h"
#include "startup.h"
#include "config.h"
#include "ecatpi.h"
#include "ecatsvr.h"
#include "ecatcmn.h"
#include "ecatsyncdata.h"
#include "ecattypes.h"
#endif
#include <inttypes.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#endif 
// 
UNT ERRCODE_TRAJ(unsigned err); 
UNT ERRCODE_WEAVE(unsigned err); 
UNT ERRCODE_INVERSE(unsigned err); 
VOD ERROR_SET(unsigned char sect, unsigned short code); 
TIME_PRECISE TimeGet(void); 

////////////////////////////////////////////////////////////////////////////////

// Default Values 

#define DEF_IPC_WAITTIME        0           // [ms] IPC Wait Time. '<=0':Inf. Wait
#define DEF_PRINT_OPTION        ARG_VERB

#define DEF_SAMP_TIME           2       // [ms]
#define DEF_JOG_KEEP            500     // [ms]
#define DEF_VEL_MAX_AXIS  		(0.001 * 2 * PI)	// [rad/ms]
#define DEF_VEL_MAX_LIN         1.   // [mm/ms]
#define DEF_VEL_MAX_ORI         DEF_VEL_MAX_AXIS
#define DEF_TIME_JERK           0 // [ms] time to acc
#define DEF_TIME_ACCEL			200 // [ms]
#define DEF_TIME_DECEL_STOP		200 // [ms]
#define DEF_TIME_DECEL_ESTOP	100 // [ms]
#define DEF_MOTOR_ORI			0	// Motor Origin Pulse
#define DEF_MOTOR_DIR			1	// Direction +(1), -(-1)
#define DEF_MOTOR_RED			1	// Reduction Ratio
#define DEF_MOTOR_PUL			1048576	// 1 turn Pulse
#define DEF_MOTOR_CNT           1   // Motor Count Per Axis 
#define DEF_AXIS_LIMIT_MIN		(-LARGE_NUM)
#define DEF_AXIS_LIMIT_MAX		( LARGE_NUM)
#define DEF_TCP_Z               128.0

// default Welder IO Values
#define DEF_DO_IDX_ARC 0 
#define DEF_DO_IDX_GAS 1
#define DEF_DO_IDX_MC 5
#define DEF_DO_IDX_TCH_PROC 4
#define DEF_DO_IDX_WIREFEED 2
#define DEF_DO_IDX_WIREBACK 3
#define DEF_DI_IDX_ARC 16
#define DEF_DI_IDX_NO_GAS 17
#define DEF_DI_IDX_TCH_PROC 20
#define DEF_DI_IDX_TCH_SIG  21
#define DEF_DI_IDX_POWER_FAIL 19
#define DEF_AO_IDX_VOLT 0
#define DEF_AO_IDX_CURR 1
#define DEF_AO_IDX_WIRE 1
#define DEF_CWEAVE_TOUCH_DIST 5     // [mm]
#define DEF_CWEAVE_MARGIN_DIST 12   // [mm] 
#define DEF_CWEAVE_WELDLEG_DIST 3   // [mm]

///// Redefinitions of Job Argument Definitions /////

#define ROBPOS_TYPE_JOINT   DANDY_JOB_POS_JOINT
#define ROBPOS_TYPE_CART    DANDY_JOB_POS_CART

// simple const(real, int, byte)
#define VALUE_TYPE_R_CONST      DANDY_JOB_VAL_TYPE_R_CONST          
#define VALUE_TYPE_I_CONST      DANDY_JOB_VAL_TYPE_I_CONST        
#define VALUE_TYPE_B_CONST      DANDY_JOB_VAL_TYPE_B_CONST    
// variable & ROBOT_POS
#define VALUE_TYPE_R_VAR        DANDY_JOB_VAL_TYPE_R_VAR         
#define VALUE_TYPE_I_VAR        DANDY_JOB_VAL_TYPE_I_VAR   
#define VALUE_TYPE_B_VAR        DANDY_JOB_VAL_TYPE_B_VAR  
// variable of ROBOT_POS 
#define VALUE_TYPE_T_VAR        DANDY_JOB_VAL_TYPE_T_VAR          
#define VALUE_TYPE_P_VAR        DANDY_JOB_VAL_TYPE_P_VAR         
// DIO / AIO value is reference of SHM_ROBOT_STATUS
#define VALUE_TYPE_DI_VAR       DANDY_JOB_VAL_TYPE_DIP_VAR          
#define VALUE_TYPE_DO_VAR       DANDY_JOB_VAL_TYPE_DOP_VAR           
#define VALUE_TYPE_AI_VAR       DANDY_JOB_VAL_TYPE_AIP_VAR           
#define VALUE_TYPE_AO_VAR       DANDY_JOB_VAL_TYPE_AOP_VAR     
// File Type Variable
#define VALUE_TYPE_WVF_VAR      DANDY_JOB_VAL_TYPE_WVF_VAR     
#define VALUE_TYPE_SWF_VAR      DANDY_JOB_VAL_TYPE_SWF_VAR     
#define VALUE_TYPE_MWF_VAR      DANDY_JOB_VAL_TYPE_MWF_VAR     
#define VALUE_TYPE_EWF_VAR      DANDY_JOB_VAL_TYPE_EWF_VAR     
// I type of File Variable
#define VALUE_TYPE_WVFI_VAR     DANDY_JOB_VAL_TYPE_WVFI_VAR     
#define VALUE_TYPE_SWFI_VAR     DANDY_JOB_VAL_TYPE_SWFI_VAR     
#define VALUE_TYPE_MWFI_VAR     DANDY_JOB_VAL_TYPE_MWFI_VAR     
#define VALUE_TYPE_EWFI_VAR     DANDY_JOB_VAL_TYPE_EWFI_VAR     
// I type Scalar Variable
#define VALUE_TYPE_RI_VAR       DANDY_JOB_VAL_TYPE_RI_VAR            
#define VALUE_TYPE_II_VAR       DANDY_JOB_VAL_TYPE_II_VAR    
#define VALUE_TYPE_BI_VAR       DANDY_JOB_VAL_TYPE_BI_VAR 
// I type POSE Variable
#define VALUE_TYPE_PI_VAR       DANDY_JOB_VAL_TYPE_PI_VAR          
#define VALUE_TYPE_TI_VAR       DANDY_JOB_VAL_TYPE_TI_VAR          
// I type of DIO / AIO 
#define VALUE_TYPE_DII_VAR      DANDY_JOB_VAL_TYPE_DIPI_VAR
#define VALUE_TYPE_DOI_VAR      DANDY_JOB_VAL_TYPE_DOPI_VAR          
#define VALUE_TYPE_AII_VAR      DANDY_JOB_VAL_TYPE_AIPI_VAR
#define VALUE_TYPE_AOI_VAR      DANDY_JOB_VAL_TYPE_AOPI_VAR

#define ARG_VALUE               DANDY_JOB_VAL 
#define ROBOT_POS               DANDY_JOB_POS

#define WELD_COND_START         DANDY_JOB_SWF
#define WELD_COND_MAIN          DANDY_JOB_MWF
#define WELD_COND_END           DANDY_JOB_EWF

#define WEAVE_PAR               DANDY_JOB_WEAV

#define DO_SET(i_, val_)      (g_do[(i_)] = (val_), g_do_exec[(i_)] = 1)
#define AO_SET(i_, val_)      (g_ao[(i_)] = (val_), g_ao_type[(i_)] = 0, g_ao_exec[(i_)] = 1)

// Welder IO Access Definitions
// The Exclusive Not Operation for Getting Logical IO Value. 
//   <LOGIC VAL>   Sig 0  | Sig 1
//   ON-Level 0 :   1(0)     0(1)   ... In () is Exclusive Operation Result. 
//   ON-Level 1 :   0(1)     1(0)

// welder do level 
#define WELD_SET_ARC(sig_)      (g_do[g_idx_do_arc]      = !(g_onlev_do_arc ^ (sig_)),      g_do_exec[g_idx_do_arc] = 1)
#define WELD_SET_GAS(sig_)      (g_do[g_idx_do_gas]      = !(g_onlev_do_gas ^ (sig_)),      g_do_exec[g_idx_do_gas] = 1)
#define WELD_SET_TCH_MC(sig_)   (g_do[g_idx_do_tch_mc]   = !(g_onlev_do_tch_mc   ^ (sig_)), g_do_exec[g_idx_do_tch_mc] = 1)
#define WELD_SET_TCH_PROC(sig_) (g_do[g_idx_do_tch_proc] = !(g_onlev_do_tch_proc ^ (sig_)), g_do_exec[g_idx_do_tch_proc] = 1)
#define WELD_SET_WIREFEED(sig_) (g_do[g_idx_do_wirefeed] = !(g_onlev_do_wirefeed ^ (sig_)), g_do_exec[g_idx_do_wirefeed] = 1)
#define WELD_SET_WIREBACK(sig_) (g_do[g_idx_do_wireback] = !(g_onlev_do_wireback ^ (sig_)), g_do_exec[g_idx_do_wireback] = 1)

// wleder di 
#define WELD_GET_ARC            (!(g_di[g_idx_di_arc]      ^ g_onlev_di_arc))
#define WELD_GET_TCH_PROC       (!(g_di[g_idx_di_tch_proc] ^ g_onlev_di_tch_proc))
#define WELD_GET_TCH_SIG        (!(g_di[g_idx_di_tch_sig]  ^ g_onlev_di_tch_sig))
#define WELD_GET_PWR_FAIL       (!(g_di[g_idx_di_pwr_fail] ^ g_onlev_di_pwr_fail))
#define WELD_GET_NO_GAS         (!(g_di[g_idx_di_no_gas]   ^ g_idx_di_no_gas))

// ao type : (0) raw, (1) volt/curr, (2) wire spd
#define WELD_SET_VOLT(sig_)  (g_ao[g_idx_ao_volt] = (sig_), g_ao_type[g_idx_ao_volt] = 1, g_ao_exec[g_idx_ao_volt] = 1)
#define WELD_SET_CURR(sig_)  (g_ao[g_idx_ao_curr] = (sig_), g_ao_type[g_idx_ao_curr] = 1, g_ao_exec[g_idx_ao_curr] = 1)
#define WELD_SET_WIRE(sig_)  (g_ao[g_idx_ao_wire] = (sig_), g_ao_type[g_idx_ao_wire] = 2, g_ao_exec[g_idx_ao_wire] = 1)

// Prog Condition    
#define PROGCOND_SKIP_LEFT  (g_pshm_rm_sys && g_pshm_rm_sys->fLeftSkip  && g_varB[g_i_bvar_left]  )
#define PROGCOND_SKIP_RIGHT (g_pshm_rm_sys && g_pshm_rm_sys->fRightSkip && g_varB[g_i_bvar_right] )
#define PROGCOND_GAP_HORZ   (g_pshm_sys_conf && g_pshm_sys_conf->robot->weld_func.fGapRefVarUsed && g_varB[g_i_bvar_gap_sel] == HORZ_GAP_COND)
#define PROGCOND_GAP_LEFT   (g_pshm_sys_conf && g_pshm_sys_conf->robot->weld_func.fGapRefVarUsed && g_varB[g_i_bvar_gap_sel] == VERT_LEFT_GAP_COND)
#define PROGCOND_GAP_RIGHT  (g_pshm_sys_conf && g_pshm_sys_conf->robot->weld_func.fGapRefVarUsed && g_varB[g_i_bvar_gap_sel] == VERT_RIGHT_GAP_COND)
#define WELDCOND_SHIFT_WV_EDIST 30

#define NEAR_POS                1   // Near Pos makes traj calc impossible. 

// IPC Name Space 
#define SHM_RM_SYSSTATUS        SHM_RM_SYSSTATUS
#define SHM_SC_MOTORCTRL        SHM_SC_SYSTEM

// Robot Configure Name Space 

// Program General Definition
#define TE_VERBOSE_NAME		    "TE_VERVOSE"
#define TE_VERBOSE_PREFIX	    "[TE] "

#define BASIC_SAMPLE            999847  // [ns]
#define WAITTIME_SLICE          100     // [ms]
#define WAITTIME_THR_READY      5000    // [ms]
#define WAITTIME_THR_EXIT       5000    // [ms]
#define WAITTIME_JOBLOAD        5000    // [ms]
#define WAITTIME_IO_CHANGE      500     // [ms]
#define MAX_SPEND_TIME          500000  // [ns] // Max Runtime Spend Time 
// 
#define DEF_TE_VGA_ROW          2              
#define VGA_SAMPLING            30

// Print off Option Bit 
// EX) if(g_print_off & BIT_XXX_OFF) { no print XXX}
#define BIT_VGA_OFF             0x01
#define BIT_TIMECHK_OFF         0x02
#define BIT_RUNVRB_OFF          0x04
#define BIT_RUNMSG_OFF          0x08
#define BIT_RUNWRN_OFF          0x10
#define BIT_RUNERR_OFF          0x20

// Certify Index of Axis or Robot
#define IS_ROB_NUM(num_)    (0 <= (num_) && (num_) < MAX_ROBOT_COUNT)
#define IS_AXIS_NUM(num_)   (0 <= (num_) && (num_) < MAX_AXIS_COUNT)

// Runtime Verbose Setting. Ex) [TE] R/ Runtime Thread Starts.
#define R_VERB_ERR(__format, ...)   (g_print_off & BIT_RUNERR_OFF)? 0 : VERBOSE_Format(VERBOSE_TYPE_ERROR,     "R/ "__format, ##__VA_ARGS__)
#define R_VERB_WRN(__format, ...)   (g_print_off & BIT_RUNWRN_OFF)? 0 : VERBOSE_Format(VERBOSE_TYPE_WARNING,   "R/ "__format, ##__VA_ARGS__)
#define R_VERB_MSG(__format, ...)   (g_print_off & BIT_RUNMSG_OFF)? 0 : VERBOSE_Format(VERBOSE_TYPE_MESSAGE,   "R/ "__format, ##__VA_ARGS__)
#define R_VERB_VRB(__format, ...)   (g_print_off & BIT_RUNVRB_OFF)? 0 : VERBOSE_Format(VERBOSE_TYPE_VERBOSE,   "R/ "__format, ##__VA_ARGS__)
#define R_VERB_TIMECHK(_form,...)   (g_print_off & BIT_TIMECHK_OFF)? 0 : VERBOSE_Format(VERBOSE_TYPE_VERBOSE,   "R/ "_form, ##__VA_ARGS__)  

#pragma pack(push, 1)
typedef struct
{
    double joint[MAX_AXIS_COUNT]; 
}HOME; 
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct t_restart_data
{    
    // flag for restartable / no-restartable
    // Belows are meaningful when this is ON. 
    unsigned f_restart;             

    // program general data
    double   jnt_f[6];              // final joint [rad]        
    unsigned mode_prog;             // program mode. 
    unsigned i_prog; 
    double   dist_work;             // working distance of last cmd [mm] 
    double   comp_result;           // COMP Cmd Result
        
    // final cweave data
    int      cweave_mode;         
    double   cweave_y_dir;     
    double   cweave_t_rest; 
    unsigned cweave_conf;    
        
    // final welding mode data
    unsigned f_welding;             // Weld mode of program is presently ON/OFF. 
    double   weld_spd;              // Weld Mode Motion Speed [mm/ms]   
    double   ao_val_curr;           // [A] final curr val
    double   ao_val_volt;           // [V] final volt val
    int      ao_type_curr;          // final curr outport mode 
    int      ao_type_volt;          // final volt outport mode

    // arc-sensro data
    int      f_sensor;              // sensor on/off flag. weld_idx::arc_sensor 
    double   ref_curr;              // sensor reference current. ref_current
    double   ref_cent;              // sensor reference weight factor (center)
    // no meaning. 'ref_current & weight' are reseted Main Weaving Node starts. 
    // Main Weaving Node always new starts after restart is operated.

    CALL_STACK stack; 
    TRAJ traj;
    WEAVE weave; 
    TRANS Tf_cwv; 
    TRANS Tv1_cwv; 
    TRANS Tv2_cwv;     
    DANDY_JOB_WVF wvf_cwv;          // weaving file for cweave 

    DANDY_JOB_SWF swf; 
    DANDY_JOB_EWF ewf; 
    DANDY_JOB_MWF mwf_pri;          // main welding condition primarilly used
    DANDY_JOB_MWF mwf_sub;          // main welding condition subordinately used
    DANDY_JOB_MWF mwf_pri_ed;       // pri-mwf when weaving end dist exists. 
    DANDY_JOB_MWF mwf_sub_ed;       // sub-mwf when weaving end dist exitts. 

} RESTART_DATA; 
////////////////////////////////////////////////////////////////////////////////
// Globals

#if defined(__cplusplus)
extern "C" {
#endif

// 

THREAD_ENTRY_TYPE VgaThread(void* pParam); 

////////////////////////////////////////////////////////////////////////////////
// IPC & Init Resources  

extern TIME_PRECISE CPS;            // cycles per second for qnx time consumtion

extern int g_argc; 
extern char** g_argv; 

// Channel 
#if 0
extern int g_ch_run_ext;            // Extra-Channel for Runtime Thread
#endif
extern int g_ch_run;                // Channel of Runtime Thread
extern int g_ch_te;                 // Channel of TE

// Connection
extern int g_co_run;                // Connection to Runtime Thread
#if 1
extern int g_co_rm;
#endif
extern int g_co_te;                 // Connection to TE Thread
extern int g_co_sc;                 // Connection to SC 

// SHM Pointer
extern volatile SHM_RM_SYSSTATUS*       g_pshm_rm_sys; 
extern volatile SHM_RM_SYSCONFIG* 	    g_pshm_sys_conf; 
extern volatile SHM_TE_STATUS*          g_pshm_te_test; 
extern volatile SHM_RESTART*            g_pshm_restart; 
extern volatile SHM_SC_MOTORCTRL*       g_pshm_sc_motor; 
extern volatile SHM_DANDY_JOB*          g_pshm_job; 

// SHM Handler
extern int g_hshm_rm_sys; 
extern int g_hshm_sys_conf; 
extern int g_hshm_te_test; 
extern int g_hshm_te_restart; 
extern int g_hshm_sc_motor; 
extern int g_hshm_job;
extern int g_hshm_sensor_rm; 
extern int g_hshm_sensor_sc; 

// Pointers to the Job Components
extern int g_size_job; 
extern DANDY_JOB_CMD*  g_rcmd;
extern DANDY_JOB_POS*  g_varT;
extern DANDY_JOB_POS*  g_varP;
extern BYTE*           g_varB; 
extern INT*            g_varI; 
extern DOUBLE*         g_varR; 
extern DANDY_JOB_WVF*  g_wvf;
extern DANDY_JOB_SWF*  g_swf; 
extern DANDY_JOB_MWF*  g_mwf;
extern DANDY_JOB_EWF*  g_ewf;

// Program Hanlder
extern int g_htmr_run; 
extern int g_htmr_exit; 
extern THREAD_HANDLE g_hthr_run;
extern THREAD_HANDLE g_hthr_vga; 

// Start & Exit Var's 

extern int g_f_runtime_ready;		// Indicator for Runtime Thread Ready
extern int g_f_runtime_exit;		// Indicator for Runtime Thread Exit Done
extern int g_f_taskexec_exit;       // flag for taskexec just exitting.
extern int g_f_ipc_init;            // Flag for IPC Resource Initialized
extern int g_f_vga_exit;	

////////////////////////////////////////////////////////////////////////////////
// Robot Configuration

// Dynamics for Linear Motion
extern double g_vel_lin_max; 		// [xx/ms]
extern double g_acc_lin;
#if 0
extern double g_dec_lin;
#endif
extern double g_dec_lin_stop;
extern double g_dec_lin_estop;
extern double g_jerk_lin;

// Dynamics for Orietation Motion
extern double g_vel_ori_max; 		// [xx/ms]
extern double g_acc_ori;
#if 0
extern double g_dec_ori;
#endif
extern double g_dec_ori_stop;
extern double g_dec_ori_estop;
extern double g_jerk_ori;

// Dynamics for Axis Motion
extern double g_vel_axis_max[MAX_AXIS_COUNT];	// axis vel max
extern double g_acc_axis[MAX_AXIS_COUNT];
#if 0
extern double g_dec_axis[MAX_AXIS_COUNT];
#endif
extern double g_dec_axis_stop[MAX_AXIS_COUNT];
extern double g_dec_axis_estop[MAX_AXIS_COUNT];
extern double g_dec_time_touch;                // [ms]

extern double g_jerk_axis[MAX_AXIS_COUNT];

// Limits for Axis 
extern double g_lim_min[MAX_AXIS_COUNT];
extern double g_lim_max[MAX_AXIS_COUNT];

// home
extern HOME g_home[MAX_HOME_COUNT]; 

#if 0
// Motors
extern int g_mot_ori[MAX_AXIS_COUNT];       // motor pulse 
extern int g_mot_dir[MAX_AXIS_COUNT];
extern unsigned g_mot_pul[MAX_AXIS_COUNT]; 
extern double g_mot_red[MAX_AXIS_COUNT];
#endif

// Coordination
// USER Coord is better to be defined online for re-definition in runtime
#if 0
extern TRANS g_bTt;     // TCP,     BASE->TCP
#endif
extern TRANS g_bTe;     // END,     BASE->END    g_bTe is updated in realtime. 
extern TRANS g_bTt;     // TCP,     BASE->TCP 
extern TRANS g_bTw;     // WORLD, BASE->WORLD
extern TRANS g_eTt;     // TCP,  END->TCP
extern TRANS g_tTs;     // SENSOR,  TCP->SENSOR

extern TRANS g_bTe_act; // END,     BASE->END    g_bTe is updated in realtime. 
extern TRANS g_bTt_act; // Actual BASE->TCP

// For Display Cart Coord
extern int g_coord_ref; // Ref. Coord. of Cart Pos 'g_xyzrpy' & 'g_xyzrpy_act'
extern XYZRPY g_xyzrpy; // g_bTt
extern XYZRPY g_xyzrpy_act; 

////////////////////////////////////////////////////////////////////////////////
// Realtime Robot Data 

extern unsigned g_t_samp; 			    // [ms] sampling time
extern TRAJ  g_traj; 
extern WEAVE g_weave; 

#if 0
// Profile Value
extern double g_profile[MAX_AXIS_COUNT];       
#else
extern double g_profile;       
#endif


// Kinematics 
extern void (*g_Forward)(TRANS *bTe,    //[out]trans of base->6
                  const double *pTh,    //[in] physical joint.
                  const DH *pDh);       //[in] dh-param

// Gets theta for trans. from base to 6th link, no regards to TCP.
// return suc(0), redendent(-1), over area(-2)
extern int (*g_Inverse)(double*  pTh,          // [out]physical joint angles.
                 const TRANS*  bTe,     // [in] trans from BASE to LINK6      
                 unsigned uConfig,      // [in] configure of robot pose
                 const DH *pDh,         // [in] dh-param
                 const double* th_prev, // [in] prev th
                 unsigned f_running);   // [in] running flag

// Gets config of Robot Pose
extern unsigned (*g_Config)(const double* pTh, //[in] joint
                                const DH *pDh);    //[in] dh-param

// Physical -> Logical Joint
extern void (*g_AxisLogical)(double *pLogical, const double *pPhysical); 

// DH Parameters
extern DH g_dh[MAX_AXIS_COUNT]; 

// Axis
extern double g_pos_act [MAX_AXIS_COUNT]; 	// actual axis pos (from Motor)
extern double g_pos_trg [MAX_AXIS_COUNT];  	// target axis pos (Calculated)
extern double g_pos_prev[MAX_AXIS_COUNT];  	// Prev. target axis pos
extern double g_vel_prev[MAX_AXIS_COUNT];  	// Prev. vel calculated with target
      
// IO Law
extern int g_di[ROBOT_DI_PORT_COUNT];      
extern DBL g_ai[ROBOT_AI_PORT_COUNT]; 
extern int g_do[ROBOT_DO_PORT_COUNT];
extern DBL g_ao[ROBOT_AO_PORT_COUNT]; 
extern int g_ao_type[ROBOT_AO_PORT_COUNT];     // AO type. 0:raw, 1:curr/volt, 2:wire spd
extern int g_do_exec[ROBOT_DO_PORT_COUNT]; 
extern DBL g_ao_exec[ROBOT_AO_PORT_COUNT]; 

extern int g_do_act[ROBOT_DO_PORT_COUNT];     // Actual DO. 
extern DBL g_ao_act[ROBOT_AO_PORT_COUNT];     // Actual AO. 

#if 0
extern int g_ao_type_prev[ROBOT_AO_PORT_COUNT];// prev AO Type 
#endif

// Welder IO for Vitual Process. 

// do index
extern int g_idx_do_arc;      
extern int g_idx_do_gas ;     
extern int g_idx_do_tch_mc;       
extern int g_idx_do_tch_proc;      
extern int g_idx_do_wirefeed; 
extern int g_idx_do_wireback; 

// do ON level
extern int g_onlev_do_arc;      
extern int g_onlev_do_gas;      
extern int g_onlev_do_tch_mc;       
extern int g_onlev_do_tch_proc;      
extern int g_onlev_do_wirefeed; 
extern int g_onlev_do_wireback; 

// di index
extern int g_idx_di_arc;      
extern int g_idx_di_tch_proc;    
extern int g_idx_di_tch_sig; 
extern int g_idx_di_pwr_fail; 
extern int g_idx_di_no_gas; 

// di level
extern int g_onlev_di_arc; 
extern int g_onlev_di_tch_proc; 
extern int g_onlev_di_tch_sig; 
extern int g_onlev_di_pwr_fail; 
extern int g_onlev_di_no_gas; 

// ao index
extern int g_idx_ao_volt; 
extern int g_idx_ao_curr; 
extern int g_idx_ao_wire; 

#if 0
// motor 
extern int g_mot_trg[MAX_MOTOR_COUNT];    		// target motor pos
extern int g_mot_act[MAX_MOTOR_COUNT]; 			// actual motor pos
#endif

// Cweave System Param's
extern double g_cweave_touch_dist; 
extern double g_cweave_horz_margin; 
extern double g_cweave_weldleg_dist;  

// Cweave Program Control Var
extern int   cwv_mode;          // actual running mode of cweave
extern TRANS cwv_Tf; 
extern TRANS cwv_Tv1; 
extern TRANS cwv_Tv2;           // Cweave Used Points. Modified from Tf_in & Tv_in.    
extern UNT   cwv_conf;          // robot pose configuration
extern DBL   cwv_y_dir;         // Cweave Dir of Y wrt BASE. To what dir the horz weaving will go. 
extern DBL   cwv_t_rest;        // Cweave Rest Time Btw Weavings 
extern DANDY_JOB_WEAV cwv_wvf; 

////////////////////////////////////////////////////////////////////////////////
// Runtime Service Variables 

// Actual Var's

extern TE_ERROR g_error; 

extern UNT      g_n_samp_over;   // Over Sampling Count
extern int      g_run_tick;
extern double   g_run_time; 

// Runtimer Value for VGA 
extern volatile TIME_PRECISE g_time_diff;
extern volatile TIME_PRECISE g_time_spend;       // runtime thread spend time
extern volatile TIME_PRECISE g_time_spend_max;
extern volatile int          g_n_spend_over;

#if 1
extern TIME_PRECISE g_time_err;  
extern TIME_PRECISE g_time_err_max; 
#endif

////////////////////////////////////////////////////////////////////////////////
// sensor data 
UNT g_n_adc_data; 

////////////////////////////////////////////////////////////////////////////////
// TASKEXEC Program Control Var's 

#if 0
extern BYTE g_f_vga_on;
#endif
extern DWORD g_ipc_wait; 			// [ms] IPC Wait Time. '<=0':Inf. Wait
extern int g_n_print_opt;		    // Print Level Option
extern BYTE g_f_no_rm;				//
extern BYTE g_f_no_sc;
extern BYTE g_f_loop_back;          // Act Motor Data is from Target Motor Data.
extern BYTE g_f_file;               // Flag for file writing
extern BYTE g_f_auto_err_reset;     // Resets err automatically when new starts. 
#if 0
extern BYTE g_mode_jog; 
#endif
extern int  g_i_vga;                // vga start line index#if 1
extern UNT g_f_runtime_pause;
extern BYTE g_print_off;            // Print off Option.   
// 0x01 : Runtime Print 
// 0x02 : Timetest Print

////////////////////////////////////////////////////////////////////////////////
// Runtime Var's

extern int g_mode;          // Indicator for Actual Mode
extern BYTE g_motor_err;
extern BYTE g_estop;
extern BYTE g_servo;

////////////////////////////////////////////////////////////////////////////////
// Runtime Program Mode Control Var's 

extern UNT g_f_skip;                // skip condition    
extern UNT g_n_shift;               // Welding Condition Shift Value

extern STR g_CmdStr;                // Cmd String 
extern UNT g_i_prog_act;            // Actual Running CMD ID
extern int g_i_branch;              // 
extern int g_i_prog_next;           // Next Program Index to run. This is useful at Normal Mode.
extern DBL g_comp_result; 
extern LMOT g_prog_lmot;            // Linear Motion info. 

extern int g_last_mode_cweave; // last cweave mode
extern int g_last_mode_weav;   // last weave mode
extern int g_last_mode_traj;   // last traj type

extern int g_f_weld;                // Weld mode of program is presently ON/OFF. 
extern DBL g_weld_spd;              // Weld Mode Motion Speed [mm/ms]

extern double ref_current; 
extern double ref_weight; 

// 
extern UNT g_i_bvar_left;           // Bvar Index Indicating Left Welding Cmd
extern UNT g_i_bvar_right;          // Bvar Index Indicating Right Welding Cmd
extern UNT g_i_bvar_gap_sel;        // Bvar Index Which Gap Condtion is applied.

// weld condition vars
extern WELD_COND_START g_weld_start; // start welding condition [mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_major; // major welding condition [mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_weave; // start-weaving welding condition[mm, ms, A, V]
#if 0   // Definition of EDIST Welding Condition is corrected. 
extern WELD_COND_MAIN  g_weld_major_edist; // major welding condition when edist of weaving is exist[mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_weave_edist; // start-weaving welding condition[mm, ms, A, V]
#endif 
extern WELD_COND_MAIN  g_weld_major_edist; // EDIST welding condition [mm, ms, A, V]
extern WELD_COND_END   g_weld_final; // final welding condition [mm, ms, A, V]

extern double g_dist_work; 

// restart
extern RESTART_DATA g_restart; 

#if defined(__cplusplus)
}
#endif

#pragma pack(pop)
#endif
