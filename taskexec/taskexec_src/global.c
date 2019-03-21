////////////////////////////////////////////////////////////////////////////////
// GLOBAL.C is the gloval variables file for TASKEXEC
// 2013-05-09 mrch0

#include "taskexec_def.h"

// Program Arguments
int g_argc; 
char** g_argv; 

////////////////////////////////////////////////////////////////////////////////
// IPC & Init Resources  

TIME_PRECISE CPS; 

#if 1
int g_co_rm = INVALID_COID;
#endif
int g_co_te = INVALID_COID; // Connection to TE Thread
int g_co_sc = INVALID_COID; // Connection to SC 
int g_ch_te = INVALID_CHID; // Channel of TE Thread

#if 0
int g_ch_run_ext = INVALID_CHID; 
#endif
int g_ch_run = INVALID_CHID;// Channel of Runtime Thread
int g_co_run = INVALID_COID;// Connection to Runtime Thread
int g_htmr_run = -1; 
int g_htmr_exit = -1; 

int g_hshm_rm_sys = -1; 
int g_hshm_sys_conf = -1; 
int g_hshm_te_test = -1;
int g_hshm_te_restart = -1; 
int g_hshm_sc_motor = -1; 
int g_hshm_job = -1; 
int g_hshm_sensor_rm = -1; 
int g_hshm_sensor_sc = -1; 

volatile SHM_RM_SYSSTATUS*  g_pshm_rm_sys   = NULL; 
volatile SHM_RM_SYSCONFIG* 	g_pshm_sys_conf = NULL; 
volatile SHM_TE_STATUS*     g_pshm_te_test  = NULL; 
volatile SHM_RESTART*       g_pshm_restart  = NULL; 
volatile SHM_SC_MOTORCTRL*  g_pshm_sc_motor = NULL; 
volatile SHM_DANDY_JOB*     g_pshm_job      = NULL; 

// Pointers to the Job Components
int g_size_job = 0; 
DANDY_JOB_CMD*  g_rcmd = NULL;
DANDY_JOB_POS*  g_varT = NULL;
DANDY_JOB_POS*  g_varP = NULL;
BYTE*           g_varB = NULL; 
INT*            g_varI = NULL; 
DOUBLE*         g_varR = NULL; 
DANDY_JOB_WVF*  g_wvf  = NULL;
DANDY_JOB_SWF*  g_swf  = NULL; 
DANDY_JOB_MWF*  g_mwf  = NULL;
DANDY_JOB_EWF*  g_ewf  = NULL;

THREAD_HANDLE g_hthr_run = INVALID_THREAD; 
THREAD_HANDLE g_hthr_vga = INVALID_THREAD; 

int g_f_runtime_ready   = FALSE; 
int g_f_runtime_exit    = FALSE; 
int g_f_taskexec_exit	= FALSE;        // flag for taskexec just exitting. 
int g_f_ipc_init        = FALSE;    
int g_f_vga_exit        = FALSE; 

////////////////////////////////////////////////////////////////////////////////
// Robot Configuration

// Dynamics for Linear Motion
double g_vel_lin_max; 				    // [xx/ms]
double g_acc_lin;
#if 0
double g_dec_lin;
#endif
double g_dec_lin_stop;
double g_dec_lin_estop;
double g_jerk_lin; 

// Dynamics for Orietation Motion
double g_vel_ori_max; 				    // [xx/ms]
double g_acc_ori;
#if 0
double g_dec_ori;
#endif
double g_dec_ori_stop;
double g_dec_ori_estop;
double g_jerk_ori; 

// Dynamics for Axis Motion
double g_vel_axis_max[MAX_AXIS_COUNT];	// axis vel max
double g_acc_axis[MAX_AXIS_COUNT];
#if 0
double g_dec_axis[MAX_AXIS_COUNT];
#endif
double g_dec_axis_stop[MAX_AXIS_COUNT];
double g_dec_axis_estop[MAX_AXIS_COUNT];
double g_dec_time_touch;                // [ms]
double g_jerk_axis[MAX_AXIS_COUNT]; 

// Limits for Axis 
double g_lim_min[MAX_AXIS_COUNT];
double g_lim_max[MAX_AXIS_COUNT];

// homes
HOME g_home[MAX_HOME_COUNT]; 

// D/A
int g_di[ROBOT_DI_PORT_COUNT];           
DBL g_ai[ROBOT_AI_PORT_COUNT]; 
int g_do[ROBOT_DO_PORT_COUNT];         // DO only by TE. This may be diff with act DO. Result of DO is checked by DI. 
DBL g_ao[ROBOT_AO_PORT_COUNT];         // AO only by TE. This may be diff with act AO. Result of AO is checked by AI. 
int g_ao_type[ROBOT_AO_PORT_COUNT];    // AO type. 0:raw, 1:curr/volt, 2:wire spd
int g_do_exec[ROBOT_DO_PORT_COUNT];    // DO execution flag
DBL g_ao_exec[ROBOT_AO_PORT_COUNT];    // AO execution flag

int g_do_act[ROBOT_DO_PORT_COUNT];     // Actual DO. 
DBL g_ao_act[ROBOT_AO_PORT_COUNT];     // Actual AO. 

#if 0
int g_ao_type_prev[ROBOT_AO_PORT_COUNT];// prev. AO type
#endif 

// Welder IO's

// do Index
int g_idx_do_arc;      
int g_idx_do_gas ;     
int g_idx_do_tch_mc;       
int g_idx_do_tch_proc;      
int g_idx_do_wirefeed; 
int g_idx_do_wireback; 

// do ON level 
int g_onlev_do_arc;      
int g_onlev_do_gas;      
int g_onlev_do_tch_mc;       
int g_onlev_do_tch_proc;      
int g_onlev_do_wirefeed; 
int g_onlev_do_wireback; 

// di index
int g_idx_di_arc;      
int g_idx_di_tch_proc;    
int g_idx_di_tch_sig; 
int g_idx_di_pwr_fail; 
int g_idx_di_no_gas; 

// di ON level
int g_onlev_di_arc;      
int g_onlev_di_tch_proc;    
int g_onlev_di_tch_sig; 
int g_onlev_di_pwr_fail; 
int g_onlev_di_no_gas; 

// ao index
int g_idx_ao_volt; 
int g_idx_ao_curr; 
int g_idx_ao_wire; 

// System Params for Cweave
double g_cweave_touch_dist;   
double g_cweave_horz_margin;  
double g_cweave_weldleg_dist; 

// Cweave Control Var
int   cwv_mode;             // actual running mode of cweave
TRANS cwv_Tf; 
TRANS cwv_Tv1;
TRANS cwv_Tv2;              // Cweave Used Points. Modified from Tf_in & Tv_in.    
UNT   cwv_conf;             // robot pose configuration
DBL   cwv_y_dir;            // Cweave Dir of Y. To what dir the horz weaving will go. 
DBL   cwv_t_rest;           // Cweave Rest Time Btw Weavings 
DANDY_JOB_WEAV cwv_wvf; 

// Coordination
// USER Coord is better to be defined online for re-definition in runtime
TRANS g_bTe;    // END,     BASE->END    g_bTe is updated in realtime. 
TRANS g_bTt;    // TCP,     BASE->TCP 
TRANS g_bTw;    // WORLD,   BASE->WORLD
TRANS g_eTt;    // TCP,     END->TCP
TRANS g_tTs;    // SENSOR,  TCP->SENSOR

// For Display 
TRANS g_bTe_act; // Actual BASE->END
TRANS g_bTt_act; // Actual BASE->TCP
int g_coord_ref = COORDINDEX_WORLD;// Ref. Coord. of Cart Pos 'g_xyzrpy' & 'g_xyzrpy_act'
XYZRPY g_xyzrpy;        // For VGA Display Only
XYZRPY g_xyzrpy_act;    // For VGA Display Only 

// Coordination with Actual Joint Pos
// TRANS g_cTt_act;// Actual cTt. not Target. Ref. Coord is by SHM_TE_TEST::coord_ref.

////////////////////////////////////////////////////////////////////////////////
// Realtime Robot Data

unsigned g_t_samp = DEF_SAMP_TIME;	    // [ms] sampling time

TRAJ    g_traj;                         // Trajectory Calculation
WEAVE   g_weave; 

// Profile Values
double g_profile;    

// Kinematics 
void (*g_Forward)(TRANS *bTe,    //[out]trans of base->6
                  const double *pTh,    //[in] physical joint.
                  const DH *pDh);       //[in] dh-param

// Gets theta for trans. from base to 6th link, no regards to TCP.
// return suc(0), redendent(-1), over area(-2)
int (*g_Inverse)(double*  pTh,          // [out]physical joint angles.
                 const TRANS*  bTe,     // [in] trans from BASE to LINK6      
                 unsigned uConfig,      // [in] configure of robot pose
                 const DH *pDh,         // [in] dh-param
                 const double* th_prev, // [in] prev th
                 unsigned f_running);   // [in] running flag

// Gets config of joint of DR6
unsigned (*g_Config)(const double* pTh, //[in] joint
                         const DH *pDh);    //[in] dh-param


// Physical -> Logical Joint
void (*g_AxisLogical)(double *pLogical, const double *pPhysical); 

// DH Parameters
DH g_dh[MAX_AXIS_COUNT]; 

// Axis Values
double g_pos_act [MAX_AXIS_COUNT]; 	 	// actual axis pos (from Motor)
double g_pos_trg [MAX_AXIS_COUNT];  	// target axis pos (Calculated)
double g_pos_prev[MAX_AXIS_COUNT];  	// Prev. target axis pos
double g_vel_prev[MAX_AXIS_COUNT];  	// Prev Vel calculated with Target

////////////////////////////////////////////////////////////////////////////////
// Runtime Service Variables 

TE_ERROR g_error; 
int     g_run_tick;
double  g_run_time; 

volatile TIME_PRECISE g_time_diff;
volatile TIME_PRECISE g_time_spend;      // runtime thread spend time
volatile TIME_PRECISE g_time_spend_max; 
volatile int          g_n_spend_over; 

#if 1
TIME_PRECISE g_time_err;  
TIME_PRECISE g_time_err_max = 0; 
#endif

////////////////////////////////////////////////////////////////////////////////
// sensor data 
UNT g_n_adc_data; 

////////////////////////////////////////////////////////////////////////////////
// Program Control Var's 

#if 0
BYTE g_f_vga_on = TRUE;
#endif
BYTE g_f_no_rm 	    = FALSE;
BYTE g_f_no_sc 	    = FALSE;
BYTE g_f_loop_back  = FALSE;    // Act Motor Data Direct from Target Motor Data 
BYTE g_f_auto_err_reset = TRUE; // Error Resets Automatically When New Start. 
BYTE g_f_file       = FALSE;            // File Write Option 
DWORD g_ipc_wait 	= DEF_IPC_WAITTIME; // [ms] IPC Wait Time. '<=0':Inf. Wait
int  g_n_print_opt  = ARG_VERB;			// Print Level Option
int  g_i_vga        = DEF_TE_VGA_ROW;   // vga start line index
UNT g_f_runtime_pause = OFF;
BYTE g_print_off    = (BIT_RUNMSG_OFF | BIT_RUNVRB_OFF | BIT_TIMECHK_OFF); // Print off Option.

// job program Control Var's 
UNT g_f_skip;       // skip condition    
UNT g_n_shift;      // Welding Condition Shift Value

DBL g_comp_result = 0.; 
UNT g_i_prog_act;   // Actual Running CMD ID
int g_i_branch;     // cmd_branch to go 
int g_i_prog_next;  // Next Program Index to run. This is useful at Normal Mode.
STR g_CmdStr = "NONE";       // Cmd String 

int g_last_mode_cweave; // cweave mode of last program-run
int g_last_mode_weav;   // weave mode of last program-run
int g_last_mode_traj;   // traj type of last program-run

int g_f_weld;       // Presently Weld Mode of Program is On/Off. Sets at ARCON. 
DBL g_weld_spd;    // Weld Mode Motion Speed [mm/ms]. Sets at ARCON. 

// 
UNT g_i_bvar_left;           // Bvar Index Indicating Left Welding Cmd
UNT g_i_bvar_right;          // Bvar Index Indicating Right Welding Cmd
UNT g_i_bvar_gap_sel;        // Bvar Index Which Gap Condtion is applied. 

// weld condition
WELD_COND_START g_weld_start; // start welding condition [mm, ms, A, V]
WELD_COND_MAIN  g_weld_major; // major welding condition [mm, ms, A, V]
WELD_COND_MAIN  g_weld_weave; // start-weaving welding condition[mm, ms, A, V]
#if 0   // Definition of EDIST Welding Condition is corrected. 
WELD_COND_MAIN  g_weld_major_edist; // major welding condition when edist of weaving is exist[mm, ms, A, V]
WELD_COND_MAIN  g_weld_weave_edist; // start-weaving welding conditionn when edist of weaving is exist[mm, ms, A, V]
#endif 
WELD_COND_MAIN  g_weld_major_edist; // EDIST welding condition [mm, ms, A, V]
WELD_COND_END   g_weld_final; // final welding condition [mm, ms, A, V]

// 
double g_dist_work;    // actual working-dist from start( to the xw dir)

// restart 
RESTART_DATA g_restart; 

////////////////////////////////////////////////////////////////////////////////

