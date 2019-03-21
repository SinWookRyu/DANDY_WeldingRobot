// RUN_PROG.C is Runtime Service Program for Program Mode. 
// 2013.09.13 mrch0
// Prog_xxx Represents RUN_PROG.

#include "dandy_job.h"
#include "taskexec_def.h"
#include "cmd.h"
#include "arg.h"
#include "call_stack.h"

extern void RestartDataSet(UNT f_set, UNT mode_prog);  

static int s_f_stop  = 0;       // Flag for Stop Request
static int s_mod_stop= 0;       // Stop Mode. 0:normal, 1:estop
static int s_mod_run = 0;       // Running Mode. PROG_RUNMODE_XXX
static int s_cmd_act = 0;       // Actual Running CMD

// f_fail_exit : OFF When Prog exits normally. ON When Prog exits with Fail. 
static void OnExit(UNT f_fail_exit); 
static VOD OnStepExit(void); 
// Increments Cmd Index 's_cid_act'
// Returns Normal(0) or End of CMD_END(Else)
static int CmdIndexIncrement(void); 
// Returns Next Cmd Index to run. 
// Normal(0) or End of CMD_END(<0)
static int NextCmdIndex(void); 

////////////////////////////////////////////////////////////////////////////////

// Inits PROG Runtime Service
// Returns Suc(0), Fail(Else).
// - index : Index of job prog to start
// - mode  : PROG Running Mode. PROG_RUNMODE_THRU, PROG_RUNMODE_STEP, ..
// 
// !! CAUTIONS !! 
// 1) Program Mode Parameters must not be initialized at 'RunServProg_Init' but 
//    at each CMD. The Params may be used for Restart and At Restart, the params 
//    can be set externally. 
// 2) After call this func, 'g_pshm_restart' is reset. You maynot get proper 
//    value of the shm.
int RunServProg_Init(const RUNPARAM_PROG* prog)
{   
    int ret; 
    UNT f_dry; 
    
    ASSERT_RETURN(prog, -1); 

    if(!g_pshm_job)
    {   
#if RESTART_
        OnExit(); 
#endif
        return ERR_NO_JOBSHM; 
    }
    
    // new Cmd 
    f_dry = (prog->mode_run == PROG_RUNMODE_DRY || 
             prog->mode_run == PROG_RUNMODE_STEP)? ON : OFF; 
    ret = Cmd_Init(prog->i_start, g_pos_act, f_dry); 
    if(ret != 0)
    {   
#if RESTART_
        OnExit(); 
#endif
        return ret;                    
    }

    // Sets Control Var's    
    s_f_stop  = 0;       
    s_mod_stop= 0; 
    s_mod_run = prog->mode_run;    
    s_cmd_act = g_rcmd[prog->i_start].nCode;
    g_i_prog_act = prog->i_start;  

    // 
    g_last_mode_cweave = CWEAVE_MODE_NONE; 
    g_last_mode_weav = WV_MODE_NONE; 
    g_last_mode_traj = TRAJ_TYPE_NORMAL; 

    // Inits Restart Data
    RestartDataSet(OFF, 0);     
    return 0;
}

// Resets Program Mode with 'prog'
void RunServProg_Reset(const RUNPARAM_PROG* prog)
{
    ASSERT_RETURN(prog, ); 

    // Sets Control Var's    
    s_f_stop  = 0;       
    s_mod_stop= 0; 
    s_mod_run = prog->mode_run;    
    s_cmd_act = g_rcmd[prog->i_start].nCode;
    g_i_prog_act = prog->i_start;  

    // Inits Restart Data
    RestartDataSet(OFF, 0);         
}

void RunServProg_Stop(int f_quick)
{   
    s_f_stop   = 1; 
    s_mod_stop = (f_quick)? 1:0;   // stop mode 0:normal, 1:quick
}

// Updates PROG Runtime Service. 
// Returns PROG is running(0), End of Running(Else)
int RunServProg_Update(void)
{
    int ret; 
    int f_dry; 

    // Check Stop Request
    if(g_error.code)
    {
        Cmd_Stop(g_i_prog_act, 1);         
    }
    // On welding & no arc on signal or welder power fail
    // The check is excluded in case of ARCOFF. 
    if(g_f_weld==ON && s_cmd_act != DANDY_JOB_CODE_ARCOFF)
    {
        if(WELD_GET_ARC==OFF)
        {
            ERROR_SET(SECT_PROG, ERR_ARCON_SIG_FAIL); 
            Cmd_Stop(g_i_prog_act, 1);                 
        }    
        if(WELD_GET_PWR_FAIL==ON)
        {
            ERROR_SET(SECT_PROG, ERR_WELDER_POWER_FAIL); 
            Cmd_Stop(g_i_prog_act, 1);                 
        }
    }
    // stop condition
    if(s_f_stop)
    {
        Cmd_Stop(g_i_prog_act, s_mod_stop);                 
    }

    // Check Sub Module is Running
    if(!Cmd_Update(g_i_prog_act))
    {
        g_last_mode_cweave = cwv_mode; 
        g_last_mode_weav   = g_weave.mode; 
        g_last_mode_traj   = g_traj.type; 
        return 0; 
    }

    // End of Sub Module, Update for Next Step /////////////////////////////////

    // End 1 Cmd with Stop Condition.
    if(g_error.code || s_f_stop)
    {
        OnExit(ON);     
        return -1; 
    }

    // !! IMPORTANT !! 
    // Next Program Index must be located Error Stop Check & Before Done 1 CMD. 
    // Next Program Index to run. It is just for indicating on VGA. 
    g_i_prog_next = NextCmdIndex(); 

    // Step Run Mode, 1 Cmd Run & Exit.
    if(s_mod_run == PROG_RUNMODE_STEP)
    {
        OnStepExit(); 
        return -1; 
    }

    // program index setting
    if(CmdIndexIncrement())         
    {        
        OnExit(OFF);
        return -1;
    }
    
    // update new cmd             
    f_dry = (s_mod_run == PROG_RUNMODE_DRY)? ON : OFF; 
    ret = Cmd_Init(g_i_prog_act, g_pos_trg, f_dry); 
    if(ret)
    {
        ERROR_SET(SECT_PROG, ret); 
        OnExit(ON);
        return -1;                    
    }

    s_cmd_act = g_rcmd[g_i_prog_act].nCode;
    return 0;  
}

// Returns Running Program Index
int RunServProg_IndexGet(void)
{
    return g_i_prog_act;
}

// Returns Running Program Mode
int RunServProg_ModeGet(void)
{
    return s_mod_run; 
}

// Returns Next Running Program Index
// Next Index is useful at Normal Mode. 
int RunServProg_NextGet(void)
{
    return g_i_prog_next;
}

////////////////////////////////////////////////////////////////////////////////

// f_fail_exit : OFF When Prog exits normally. ON When Prog exits with Fail. 
static void OnExit(UNT f_fail_exit)
{
#if RESTART_
    // servo off
    if(g_co_sc != INVALID_COID)
    {
        MSG_SendPulse(g_co_sc, SC_SERV_SERVO, OFF); 
    }
#endif
        
    RestartDataSet(f_fail_exit, s_mod_run); 

    // welder io off    
    WELD_SET_ARC(OFF); 
    // Gas, Volt, Curr is set 0 by SC. 
#if 0 
    WELD_SET_TCH_MC(ON);     
    WELD_SET_GAS(OFF); 
    WELD_SET_VOLT(0.); 
    WELD_SET_CURR(0.); 
#endif 
    
    // Weld Mode Flag Clear
    g_f_weld = OFF;
    // Arc sensor flag clear 
    weld_idx.arc_sensor = OFF; 
    
    s_mod_run = PROG_RUNMODE_NONE; 
}

// On the contrary to 'OnExit', this doesn't resets program mode parameters. 
// Like as 'g_f_weld', 'g_comp_result', and so on. 
static void OnStepExit(void)
{   
    s_mod_run = PROG_RUNMODE_NONE; 
}

#if 0   // 2014-06-16
// Increments Cmd Index 's_cid_act'
// Returns Normal(0) or End of CMD_END(Else)
static int CmdIndexIncrement(void)
{   
    // program index is updated by CMD
    // program ctrl cmd is processed here
    // 's_cmd_act' must be used instead of 'g_rcmd[g_i_prog_act].nCode'.
    // for the current job program can be reloaded by CALL or RET. 

    switch(s_cmd_act)
    {
    case DANDY_JOB_CODE_JUMP:
    case DANDY_JOB_CODE_CALL:        
    case DANDY_JOB_CODE_RET:
        g_i_prog_act = g_i_branch; 
        break; 
    default:
        g_i_prog_act++; 
        break; 
    }

    if(s_cmd_act == DANDY_JOB_CODE_END)
    {
        return -1; 
    }

    if(g_pshm_job->dwCmdLoadCount  <= g_i_prog_act)
    {        
        return -1;  // over count of cmd
    }

    return 0;  
}
#else
// Increments Cmd Index to Run. 
// Returns Normal(0) or End of Job
static int CmdIndexIncrement(void)
{
    int next; 
    
    next = NextCmdIndex(); 
    if(next < 0)        // no cmd to run
    {
        return -1; 
    }
    else                // next cmd exists. 
    {
        g_i_prog_act = next; 
        return 0; 
    }
}
#endif 

// Returns Next Index or End of Job(<0)(CMD_END or Line Over)
static int NextCmdIndex(void)
{   
    // program index is updated by CMD
    // program ctrl cmd is processed here
    // 's_cmd_act' must be used instead of 'g_rcmd[g_i_prog_act].nCode'.
    // for the current job program can be reloaded by CALL or RET. 

    UNT next; 

    if(s_cmd_act == DANDY_JOB_CODE_END)
    {
        return -1;         
    }

    switch(s_cmd_act)
    {
    case DANDY_JOB_CODE_JUMP:
    case DANDY_JOB_CODE_CALL:        
    case DANDY_JOB_CODE_RET:
        next = g_i_branch; 
        break; 
    default:
        next = g_i_prog_act + 1; 
        break; 
    }    

    if(g_pshm_job->dwCmdLoadCount <= next)
    {        
        return -1;  // over count of cmd
    }
    else
    {
        return next;     
    }
}
