////////////////////////////////////////////////////////////////////////////////
// T_RUNTIME.C is Runtime Thread Program. 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "taskexec_def.h"
#include "te_serv.h"
#include "dandy_echo.h"
#include "float.h"
#include "filemgr.h"
#include "coord.h"

extern UNT RestartShmSizeGet(void); 
extern void RestartShmCopy(UNT f_restart); 

volatile TIME_PRECISE t_update[10]; 

////////////////////////////////////////////////////////////////////////////////

static void UPDATE(void); 
static void RUN(void); 

static void ShmRead(void); 
static void ShmWrite(void); 
#if 0
static void Motor_To_Axis(int n_rob);
static void Axis_To_Motor(int n_rob);
#endif
static void FileWrite(int rob); 
static void AxisToCart(void); 
static void InitTargetPos(int n_rob); 
static void CopyActual(void);
    
// Checks Motor Needs Target Data
static BYTE IsMotorTargetNeed(void); 
// Returns Error Condition by Error Code
static int CheckErrorCond(void); 
// Returns Stop Condition by Error Code. 
static int CheckStopCond(void); 
// Returns New Mode Invalid Condition by Error Code
static int CheckNoNewCond(void); 
// Prints Actual Runtime Param
static void PrintRunParam(void); 
// Resets Requested Parameters
static void ResetRequest(void); 

static TIME_PRECISE s_time_act;
static TIME_PRECISE s_time_old;

#if 0
volatile TIME_PRECISE s_time_start;
volatile TIME_PRECISE s_time_msg;
volatile TIME_PRECISE s_time_calc;
volatile TIME_PRECISE s_time_vga;
#endif

////////////////////////////////////////////////////////////////////////////////

int g_mode = RUNMODE_NONE;       // Indicator for Actual Mode
static int s_stop = 0;                  // Indicator for Actual Stopping 
static int s_mode_req = RUNMODE_NONE;   // Requested Mode
static int s_stop_req = 0;              // Requested Stop Flag
static int s_stop_req_mod = 0;         // Requested Stop Mode. 0:normal ELSE:quick
static RUN_PARAM s_param_req; 

// Hardware Status
BYTE g_motor_err = 0;
BYTE g_estop = FALSE;
BYTE g_servo = TRUE;

UNT g_n_samp_over = 0;          // Over Sampling Count

////////////////////////////////////////////////////////////////////////////////

#if defined(__QNXNTO__)
TIME_PRECISE TimeGet(void)
{
    return ClockCycles()*1000000000/CPS;    
}
#else
TIME_PRECISE TimeGet(void)
{
    return TIME_GetPrecise();
}
#endif

THREAD_ENTRY_TYPE RuntimeThread(void* pParam)
{
    TE_MSG      msg; 
    MSG_INFO	msg_info; 
    int         rcvid; 
    int         f_exit = 0;     // exit requested flag
    volatile TIME_PRECISE time[10]; 
#if 0
    THREAD_SetSchedule(0, THREAD_POLICY_FIFO, THREAD_PRIO_REALTIME);
#endif

    R_VERB_MSG("RUNTIME thread Starts with priority %d\n", THREAD_GetPriority(0));
    
    if(Init_RuntimeInit())
    { 
        goto EXIT_PROCESS;
    }   
    InitTargetPos(0); 
    
    // Runtime Starts Indicate Setting 
	g_f_runtime_ready = TRUE; 
    R_VERB_MSG("'Runtime' thread completed initialization.\n"); 

    // Runtime Thread Start                                     
    R_VERB_MSG("'Runtime' thread Starts Main Loop...\n"); 
	while(1)
	{
        // Receive Blocking
		rcvid = MSG_Receive(g_ch_run, &msg, sizeof(msg), &msg_info);	
        if(rcvid < 0)
		{
			R_VERB_WRN("Failed MSG Receiption.\n");
            continue; 
		}

time[0] = TimeGet(); 

        // Check Exit Condition

        // Exit Request 
        if(msg.code == RUNSERV_EXIT)
        {	
            Run_Stop(0); 
            f_exit = TRUE; 
        }

        if(f_exit == TRUE && g_mode == RUNMODE_NONE)
        {
            goto EXIT_PROCESS; 
        }        
        else if(g_f_runtime_pause)
        {
            continue; 
        }
        // Timer, Runtime Original Main Process
        else if(msg.code == RUNSERV_TIMER)
        {   
            //      
time[1] = TimeGet();

            ShmRead();

time[2] = TimeGet();

            UPDATE(); 

time[3] = TimeGet();

            RUN();             

time[4] = TimeGet();

            ShmWrite();

time[5] = TimeGet();

            FileWrite(0); 

time[6] = TimeGet();
#if 0
            PrintEvent(); 
#endif

time[7] = TimeGet();

        }
        // Undefined 
        else
        {    
            R_VERB_WRN("Received unsupported service. code:%d value:%d\n", msg.code, msg.value);
            continue; 
        }
time[8] = TimeGet(); 

        // Runtime Thread Spend Time 
        g_time_spend    = time[8] - time[0]; 
        if(g_time_spend_max < g_time_spend)
        {
            g_time_spend_max = g_time_spend; 
            
#if TIMECHK_

            R_VERB_MSG("RUNTIME : Exit-Chk:%llu Shm-Read:%llu UPDATE:%llu RUN:%llu Shm-Wrt:%llu File-Wrt:%llu Print-Evt:%llu t7~8:%llu\n", 
                 time[1]-time[0], time[2]-time[1], time[3]-time[2], time[4]-time[3], time[5]-time[4], time[6]-time[5], time[7]-time[6], time[8]-time[7]); 
#endif
            R_VERB_TIMECHK("RUNTIME : Exit-Chk:%llu Shm-Read:%llu UPDATE:%llu RUN:%llu Shm-Wrt:%llu File-Wrt:%llu Print-Evt:%llu t7~8:%llu\n", 
                 time[1]-time[0], time[2]-time[1], time[3]-time[2], time[4]-time[3], time[5]-time[4], time[6]-time[5], time[7]-time[6], time[8]-time[7]); 
        }
        g_n_spend_over += (g_time_spend > MAX_SPEND_TIME)? 1 : 0;   
	}

EXIT_PROCESS:
                                             	
    // Runtime Exits Indicate Setting 
    R_VERB_MSG("Enters to the exit process.\n");
	Init_RuntimeUninit();
    g_f_runtime_exit = TRUE; 	                 
    R_VERB_MSG("Exits 'Runtime' thread.\n"); 
	return 0; 
}

////////////////////////////////////////////////////////////////////////////////

// Request Runtime Thread Mode to 'mode'.
int Run_ReqMode(int mode, void* param)
{   
    if(param == NULL)
    {
        VERBOSE_ERROR("Invalid Runmode Parameter\n"); 
        return -1; 
    }
        
    if(g_mode != RUNMODE_NONE)
    {        
        VERBOSE_WARNING("Runtime is busy. Trg:%s Act:%s\n", 
                        Str_RunMode(mode), Str_RunMode(g_mode));
        return -1; 
    }
    
    // Runtime Specific Parameter Setting First 
    switch(mode)
    {
    case RUNMODE_TIMETEST: 
        s_param_req.timetest = *(RUNPARAM_TIMETEST*)param;
        break; 
    case RUNMODE_JOG:
        s_param_req.jog = *(RUNPARAM_JOG*)param;
        break;
    case RUNMODE_PROG:
        s_param_req.prog = *(RUNPARAM_PROG*)param; 
        break; 
    case RUNMODE_NONE:
    default:
        break; 
    }

    // Mode Setting at Last 
    s_mode_req = mode; 

    VERBOSE_VERBOSE("Requests Runtime Mode. Trg:%s Act:%s\n", 
                    Str_RunMode(mode), Str_RunMode(g_mode));
    return 0; 
}

void Run_Stop(int f_quick)
{
    s_stop_req = TRUE; 
    s_stop_req_mod = (f_quick)? 1 : 0; 
}

// Checks Runtime is Jog running with 'nAxis'.
// Ret Running(1) Else(0)
int Run_IsJogRunning(int nAxis)
{
    if(g_mode == RUNMODE_JOG /*&& jog_param.axis == nAxis*/)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

////////////////////////////////////////////////////////////////////////////////
#include "cmd.h"

static void RUN(void)
{
    // Trajectory Running    
    if(Traj_IsRunning(&g_traj))
    {
        Traj_GetTarget(&g_traj, g_pos_trg, &g_profile, g_pos_prev, g_vel_prev);        
        ERROR_SET(SECT_RUN, (g_traj.error)? ERR_TRAJ_START|(0xff & g_traj.error) : 0); 
    }    
   
    RUNSERV_RUN(g_mode); 

    AxisToCart();      

        
    // working distance
#if 0
    if(g_weave.mode != WV_MODE_NONE) 
    {
        g_dist_weave = Wv_Dist(&g_weave, &g_bTt.p); 
    }
#else

    // Working Distance. 
    // Inverse Error makes Working Distance 0. 
    // If Inv. Error occurs, Traj type changes into Type of JOINT. 
#if 0
    if( (Cmd_IsLinMot(g_i_prog_act) && g_traj.type == TRAJ_TYPE_LINEAR) || 
        (Cmd_IsCirMot(g_i_prog_act) && g_traj.type == TRAJ_TYPE_CIRCLE) )
    {
        g_dist_work = g_traj.lmot.dist * g_profile; 
    }
#else 
    if(Cmd_IsLinMot(g_i_prog_act) && g_traj.type == TRAJ_TYPE_LINEAR) 
    {
        g_dist_work = g_traj.lmot.dist * g_profile; 
    }
    else if(Cmd_IsCirMot(g_i_prog_act) && g_traj.type == TRAJ_TYPE_CIRCLE) 
    {
        g_dist_work = g_traj.cmot.arc * g_profile; 
    }
#endif 
    else if((Cmd_IsWeaveCmd(g_i_prog_act)  && Wv_IsRunning(&g_weave)) || 
            (Cmd_IsCweaveCmd(g_i_prog_act) && Wv_IsRunning(&g_weave)) )
    {
#if 0
        g_dist_work = Wv_Dist(&g_weave, &g_bTt.p); 
#else
        g_dist_work = g_weave.d_act; 
#endif
    }
    else
    {
        ; // g_dist_work = 0; 
    }
#endif
}

static void UPDATE(void)
{
    int ret;    
    
t_update[0] = TimeGet(); 
    ////////////////////////////////////////////////////////////////////////////
    // Update Tick
    g_run_tick++;
    g_run_time = g_run_tick * g_t_samp; 
        
    // Update Time
    s_time_old = s_time_act;
    s_time_act = TimeGet();
    g_time_diff = s_time_act - s_time_old;
    g_n_samp_over += (g_time_diff >= g_t_samp*BASIC_SAMPLE + 1000)? 1 : 0;   // check the sampling over 1000ns.

    // 
    g_time_err = g_time_diff - g_t_samp*BASIC_SAMPLE; 
    g_time_err *= (g_time_err < 0)? -1 : 1;    // making +
    g_time_err_max = (g_time_err_max < g_time_err)? g_time_err : g_time_err_max; 

    // update Axis
    g_vel_prev[0] = (g_pos_trg[0] - g_pos_prev[0])/g_t_samp; 
    g_vel_prev[1] = (g_pos_trg[1] - g_pos_prev[1])/g_t_samp; 
    g_vel_prev[2] = (g_pos_trg[2] - g_pos_prev[2])/g_t_samp; 
    g_vel_prev[3] = (g_pos_trg[3] - g_pos_prev[3])/g_t_samp; 
    g_vel_prev[4] = (g_pos_trg[4] - g_pos_prev[4])/g_t_samp; 
    g_vel_prev[5] = (g_pos_trg[5] - g_pos_prev[5])/g_t_samp; 
    memcpy(g_pos_prev, g_pos_trg, sizeof(g_pos_trg)); 

t_update[1] = TimeGet();     

    ////////////////////////////////////////////////////////////////////////////
    // Error Condition Check 
    ret = CheckErrorCond(); 
    if(ret)
    {
        if(!g_error.code)
        {
            R_VERB_ERR("Error occured(%s, %s(0x%x))\n", Str_Sect(SECT_RUN), Str_Error(ret), ret);                
        }
        ERROR_SET(SECT_RUN, ret); 
        RUNSERV_STOP(1); 
        s_stop = TRUE; 
    }

t_update[2] = TimeGet(); 

    ////////////////////////////////////////////////////////////////////////////
    // Stop Condition Check
    ret = CheckStopCond();
    if(ret)
    {
        if(!s_stop)
        {                    
            R_VERB_WRN("Stop Condition(%d). %s\n", ret, Str_Error(ret));
        }

        if(s_stop_req && s_stop_req_mod==0)
        {
            RUNSERV_STOP(0);    
        }
        else
        {
            RUNSERV_STOP(1);    // quick stop
        }
        s_stop = TRUE; 
    }

t_update[3] = TimeGet(); 

    ////////////////////////////////////////////////////////////////////////////
    // Update Mode & End of Work, Update to Normal Mode
    ret = RUNSERV_UPDATE(g_mode);    
    // Check of On the mode running
    if(!ret)
    {
        ResetRequest(); 
t_update[4] = TimeGet(); 
        return; 
    }

t_update[4] = TimeGet(); 

    ////////////////////////////////////////////////////////////////////////////
    // End of One Mode...

    // end of a mode print
    if(g_mode != RUNMODE_NONE)
    {
#if TIMECHK_
        R_VERB_MSG("End of %s Mode\n", Str_RunMode(g_mode));
#endif
        R_VERB_TIMECHK("End of %s Mode\n", Str_RunMode(g_mode));
    }

t_update[5] = TimeGet(); 

    // No Another Mode Requested Check
    if(s_mode_req == RUNMODE_NONE)
    {
        g_mode = RUNMODE_NONE;
        ResetRequest(); 
t_update[5] = TimeGet(); 
        return; 
    }

t_update[6] = TimeGet(); 

    ////////////////////////////////////////////////////////////////////////////
    // New Mode Requested... 
   
    // Automatic Error & Stop Reset at New Mode Request
    ERROR_SET(SECT_NONE, 0); 
    s_stop = 0; 
    g_run_tick = 1;     // not 0, but 1. Start from 1
    g_run_time = g_run_tick * g_t_samp; 

    // update Axis
    CopyActual(); 

    // New Mode Start Check    
    ret = CheckNoNewCond(); 
    if(ret)
    {        
        R_VERB_WRN("Invalid New Mode Condition(0x%x). %s\n", ret, Str_Error(ret));         
        ResetRequest();         
        return; 
    }

    ////////////////////////////////////////////////////////////////////////////
    // New Mode Startable...

t_update[7]= TimeGet(); 

    // Inits Fail Check 
    ret = RUNSERV_INIT(s_mode_req, &s_param_req);   
    if(ret)
    {   
        // New Start Fail
        R_VERB_ERR("Failed to Init Runtime %s Mode. %s.\n",Str_RunMode(s_mode_req), Str_Error(ret)); 
        ERROR_SET(SECT_RUN, ret); 
        g_mode = RUNMODE_NONE;
        ResetRequest(); 
        return; 
    }
t_update[8] = TimeGet(); 

    ////////////////////////////////////////////////////////////////////////////
    // New Start Success
    g_mode      = s_mode_req;            
    ResetRequest(); 
    
#if TIMECHK_
     R_VERB_MSG("Runtime Mode(%s) Starts.\n", Str_RunMode(g_mode));
    // (g_print_off & 0x02)? 0 : VERBOSE_Format(VERBOSE_TYPE_MESSAGE,   "R/ ""Runtime Mode(%s) Starts.\n", Str_RunMode(g_mode)); 
//     (1)? 0 : VERBOSE_Format(VERBOSE_TYPE_MESSAGE,   "test\n"); 
#endif
    R_VERB_TIMECHK("Runtime Mode(%s) Starts.\n", Str_RunMode(g_mode));
    PrintRunParam();  


t_update[9] = TimeGet(); 

#if 0
    R_VERB_MSG("UPDATE Time(New Mode Req.) : t0~t1:%llu t1~t2:%llu t2~t3:%llu t3~t4:%llu t4~t5:%llu t5~t6:%llu t6~t7:%llu t7~t8:%llu t8~t9:%llu[ns]\n", 
        t_update[1]-t_update[0], t_update[2]-t_update[1], t_update[3]-t_update[2], t_update[4]-t_update[3], t_update[5]-t_update[4], t_update[6]-t_update[5], t_update[7]-t_update[6], t_update[8]-t_update[7], t_update[9]-t_update[8]); 
#endif
#if TIMECHK_
    R_VERB_MSG("UPDATE(..) (New Mode Req.) : Prepare:%llu Copy-Act:%llu Init:%llu Print-Param:%llu[ns]\n", 
        t_update[6]-t_update[0], t_update[7]-t_update[6], t_update[8]-t_update[7], t_update[9]-t_update[8]);     
#endif  
    R_VERB_TIMECHK("UPDATE(..) (New Mode Req.) : Prepare:%llu Copy-Act:%llu Init:%llu Print-Param:%llu[ns]\n", 
            t_update[6]-t_update[0], t_update[7]-t_update[6], t_update[8]-t_update[7], t_update[9]-t_update[8]);     
    return; 
}

UNT ERRCODE_TRAJ(unsigned err)
{
    return (err)? ERR_TRAJ_START|(0xff & err) : 0;
    // Trajectory error has all inverse errors. 
    // So it is no need to convert the error for inverse to the inverse error. 
} 

// weave error -> runtime error
UNT ERRCODE_WEAVE(unsigned err)
{
#if 0 // error 
    return (err)? ERR_WEAV_START|(0xff & err) : 0;
#else
    return (Wv_IsTrajErr(err))? ERRCODE_TRAJ(0xff & err)    : 
           (err)?               ERR_WEAV_START|(0xff & err) : 0;  
    // Weave error just has WV_ERR_TRAJ for the trajectory errors. 
    // So it is need to convert the error for TRAJ to TRAJ error. 
#endif
} 

UNT ERRCODE_INVERSE(unsigned err)
{
    return (err)? ERR_INVERSE_START|(0xff & err) : 0;
} 

void ERROR_SET(unsigned char sect, unsigned short code)
{   
    if(!g_error.code && code)                 
    {
        // general error info
        g_error.sect = sect; 
        g_error.code = code; 
        R_VERB_ERR("Error occured(%s, %s(0x%x))\n", Str_Sect(sect), Str_Error(code), code); 

        // trajectory error 
        if((g_error.code & 0xff00) == ERR_TRAJ_START)
        {
            R_VERB_ERR("TRAJ Error Pos[rad]   : %.2f  %.2f  %.2f  %.2f  %.2f  %.2f \n", 
                g_traj.err_pos[0], g_traj.err_pos[1], g_traj.err_pos[2], 
                g_traj.err_pos[3], g_traj.err_pos[4], g_traj.err_pos[5]); 

                // g_pos_trg[0], g_pos_trg[1], g_pos_trg[2], 
                // g_pos_trg[3], g_pos_trg[4], g_pos_trg[5]); 

            R_VERB_ERR("TRAJ Error Vel[rad/s] : %.2f  %.2f  %.2f  %.2f  %.2f  %.2f \n",                
                g_traj.err_vel[0]*1000, g_traj.err_vel[1]*1000, g_traj.err_vel[2]*1000, 
                g_traj.err_vel[3]*1000, g_traj.err_vel[4]*1000, g_traj.err_vel[5]*1000);
                // g_vel_prev[0]*1000, g_vel_prev[1]*1000, g_vel_prev[2]*1000, 
                // g_vel_prev[3]*1000, g_vel_prev[4]*1000, g_vel_prev[5]*1000);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

static void ShmRead(void)
{
    if(g_f_loop_back == TRUE)
    {       	
        UNT val; 

        g_motor_err = g_motor_err;
    	g_servo  	= g_servo;
    	g_estop     = g_estop;

        memcpy(g_pos_act, g_pos_trg, sizeof(g_pos_trg));   

        // If DO is set, DI regarded is set. 
        // For details, refer WELD_SET_ARC, WELD_GET_ARC, ... in 'taskexec_def.h'. 

        // arc sig
        val = !(g_do[g_idx_do_arc] ^ g_onlev_do_arc); 
        g_di[g_idx_di_arc] = !(g_onlev_di_arc ^ (val)); 
        // gas
        val = !(g_do[g_idx_do_gas] ^ g_onlev_do_gas); 
        g_di[g_idx_di_no_gas] = !(g_idx_di_no_gas ^ (!val)); 
        // touch process
        val = !(g_do[g_idx_do_tch_proc] ^ g_onlev_do_tch_proc); 
        g_di[g_idx_di_tch_proc] = !(g_onlev_di_tch_proc ^ (val)); 
        g_di[g_idx_di_tch_sig]  = !(g_onlev_di_tch_sig  ^ (val)); 
    }
    else if(g_pshm_sc_motor != NULL)
    {
        void* src = (void*)g_pshm_sc_motor->inputstate.dbActPos;         

        // sig
        g_servo  	= g_pshm_sc_motor->outputstate.fServoOnOutState;
        g_estop     = g_pshm_sc_motor->sysstate.fEStopState;
        g_motor_err = g_pshm_sc_motor->sysstate.fErrorState;
        
        // pos
        memcpy(g_pos_act, src, sizeof(g_pos_act));

        // io 
        memcpy(g_di, (int*)g_pshm_sc_motor->inputstate.nDinPortVal,  sizeof(g_di));         
        memcpy(g_ai, (DBL*)g_pshm_sc_motor->inputstate.dbAinPortVal, sizeof(g_ai));   

        memcpy(g_do_act, (int*)g_pshm_sc_motor->outputstate.nDoutPortVal,  sizeof(g_do_act));         
        memcpy(g_ao_act, (DBL*)g_pshm_sc_motor->outputstate.dbAoutPortVal, sizeof(g_ao_act));   

        // ADC 
        g_n_adc_data = shm_servo->ADC_gathering_index; 
    }
}

static void ShmWrite(void)
{
    // TE Shm Writing
    if(g_pshm_te_test != NULL)
    {
        int sect; 

        memcpy((double*)&g_pshm_te_test->xyzrpy_act, g_xyzrpy_act.dbMember, sizeof(g_xyzrpy_act.dbMember)); 
        g_pshm_te_test->coord_ref= g_coord_ref; 
                
        g_pshm_te_test->run_mode = g_mode;
        g_pshm_te_test->run_stop = s_stop; 
        g_pshm_te_test->run_error = g_error;         
        g_pshm_te_test->run_prog_idx = RunServProg_IndexGet(); 
        g_pshm_te_test->run_next_idx = RunServProg_NextGet();
        g_pshm_te_test->run_prog_mod = RunServProg_ModeGet(); 
        g_pshm_te_test->run_cmd_code = g_rcmd[g_i_prog_act].nCode;         
        g_pshm_te_test->run_f_weld = g_f_weld; 

        g_pshm_te_test->f_restart_exist = g_restart.f_restart; 
        g_pshm_te_test->dist_work = g_dist_work; 

        sect = Prof7_GetSect(&g_traj.profile, g_traj.t_start); 
        g_pshm_te_test->prof_sect = (sect == PROF7_SECT_ACC_J1)?  PROF_A1  :
                                    (sect == PROF7_SECT_ACC_U )?  PROF_AU  : 
                                    (sect == PROF7_SECT_ACC_J2)?  PROF_A2  : 
                                    (sect == PROF7_SECT_UNI   )?  PROF_U   :
                                    (sect == PROF7_SECT_DEC_J1)?  PROF_D1  : 
                                    (sect == PROF7_SECT_DEC_U )?  PROF_DU  : 
                                    (sect == PROF7_SECT_DEC_J2)?  PROF_D2  :
                                    (sect == PROF7_SECT_OVER  )?  PROF_OVER: 
                                                                  PROF_NONE; 
    }

    // SC SHM Writing
#if 0
    if(g_pshm_sc_motor != NULL)
    {   
        INT  i; 
        INT* dout; 
        DBL* aout; 
        INT* aout_type; 

        if(IsMotorTargetNeed())
        {
            void* trg = (void*)g_pshm_sc_motor->outputcmd.dbTrgPos; 
            memcpy(trg, g_pos_trg, sizeof(g_pos_trg)); 
        }         

        // IO Shm setting 
        // IO Target is not to SHM directly. Just writes when the value changed. 
        // Direct writing prohibits the io-access from anther process. 

        for(i=0 ; i<ROBOT_DO_PORT_COUNT ; i++)
        {
            dout  = (int*)&g_pshm_sc_motor->outputcmd.nDoutPortCmd[i];             
            if(g_do[i] != g_do_prev[i]) 
            {
                *dout = g_do[i]; 
            }            
        }
        for(i=0 ; i<ROBOT_AO_PORT_COUNT ; i++)
        {
            aout_type  = (int*)&g_pshm_sc_motor->sysstate.nAoutMappingType[i]; 
            if(g_ao_type[i] != g_ao_type_prev[i])
            {
                *aout_type = g_ao_type[i];                 
            }

            aout  = (DBL*)&g_pshm_sc_motor->outputcmd.dbAoutPortCmd[i]; 
            if(g_ao[i] != g_ao_prev[i])
            {
                *aout = g_ao[i]; 
            }            
        }
    }
    
    if(g_ao[g_idx_ao_volt] != g_ao_prev[g_idx_ao_volt] && g_ao_type[g_idx_ao_volt] == 1)
    {
        R_VERB_VRB("Welding Volt Set %.1f\n", g_ao[g_idx_ao_volt]); 
    }
    if(g_ao[g_idx_ao_curr] != g_ao_prev[g_idx_ao_curr] && g_ao_type[g_idx_ao_curr] == 1)        
    {
        R_VERB_VRB("Welding Curr Set %.1f\n", g_ao[g_idx_ao_curr]); 
    }

    // set previous val
    memcpy(g_do_prev, g_do, sizeof(g_do)); 
    memcpy(g_ao_prev, g_ao, sizeof(g_ao)); 
    memcpy(g_ao_type_prev, g_ao_type, sizeof(g_ao_type)); 
#endif 
    
    if(g_pshm_sc_motor != NULL)
    {   
        INT  i; 
        INT* dout; 
        DBL* aout; 
        INT* aout_type; 

        if(IsMotorTargetNeed())
        {
            void* trg = (void*)g_pshm_sc_motor->outputcmd.dbTrgPos; 
            memcpy(trg, g_pos_trg, sizeof(g_pos_trg)); 
        }         

        // IO Shm setting 
        // IO Target is not to SHM directly. Just writes when the value changed. 
        // Direct writing prohibits the io-access from anther process. 

        for(i=0 ; i<ROBOT_DO_PORT_COUNT ; i++)
        {
            dout  = (int*)&g_pshm_sc_motor->outputcmd.nDoutPortCmd[i];             
            if(g_do_exec[i]) 
            {
                *dout = g_do[i]; 
            }            
        }
        for(i=0 ; i<ROBOT_AO_PORT_COUNT ; i++)
        {
            aout_type  = (int*)&g_pshm_sc_motor->sysstate.nAoutMappingType[i]; 
            aout  = (DBL*)&g_pshm_sc_motor->outputcmd.dbAoutPortCmd[i]; 
            if(g_ao_exec[i])
            {
                *aout = g_ao[i]; 
                *aout_type = g_ao_type[i]; 
            }            
        }
    }
    
    if(g_ao_exec[g_idx_ao_volt] && g_ao_type[g_idx_ao_volt] == 1)
    {        
        R_VERB_VRB("Welding Volt Set %.1f\n", g_ao[g_idx_ao_volt]); 
    }
    if(g_ao_exec[g_idx_ao_curr] && g_ao_type[g_idx_ao_curr] == 1)        
    {
        R_VERB_VRB("Welding Curr Set %.1f\n", g_ao[g_idx_ao_curr]); 
    }

    // set previous val
    memset(g_do_exec, 0, sizeof(g_do)); 
    memset(g_ao_exec, 0, sizeof(g_ao)); 
    // memcpy(g_ao_type_prev, g_ao_type, sizeof(g_ao_type)); 
}

#if 0 
static void Motor_To_Axis(int n_rob)
{  
    int i;
    int ori, dir;
    unsigned pul;
    double red; 

    ASSERT_RETURN(IS_ROB_NUM(n_rob), ;);

    for(i=0 ; i<MAX_AXIS_COUNT; i++)    // axis
    {
        ori = g_mot_ori[i];
        dir = g_mot_dir[i];
        red = g_mot_red[i];
        pul = g_mot_pul[i];

        g_pos_act[i] = (2*PI / pul) * (g_mot_act[i] - ori) / (dir * red);
    }
}
#endif

#if 0
static void Axis_To_Motor(int n_rob)
{
    int i;
    int ori, dir; 
    unsigned pul;
    double red; 

    ASSERT_RETURN(IS_ROB_NUM(n_rob), ;);

    for(i=0 ; i<MAX_AXIS_COUNT ; i++)    // axis
    {
		ori = g_mot_ori[i];
		dir = g_mot_dir[i];
		red = g_mot_red[i];
		pul = g_mot_pul[i];

		g_mot_trg[i] = (int)(ori + (pul / (2*PI)) * dir * red * g_pos_trg[i]);
    }
}
#endif

static BYTE IsMotorTargetNeed(void)
{
    return (g_mode == RUNMODE_JOG   || 
            g_mode == RUNMODE_PROG  || 
            g_mode == RUNMODE_RESTART); 
}

// Returns Error Condition by Error Code
static int CheckErrorCond(void)
{  
    ASSERT_RETURN(!g_error.code,                        g_error.code); 
    ASSERT_RETURN(!g_motor_err,                         ERR_MOTOR_ERR); 
    ASSERT_RETURN(g_mode==RUNMODE_NONE || g_servo,      ERR_SERVO_OFF);   
    return 0; 
}

// Returns Stop Condition by Error Code. 
static int CheckStopCond(void)
{    
    if(s_stop_req)
    {
        return (s_stop_req_mod)? ERR_QUICK_STOP_REQ : ERR_STOP_REQ; 
    }
    return (g_estop)? ERR_ESTOP_ON : ERR_NONE; 
}

// Returns New Mode Invalid Condition by Error Code
static int CheckNoNewCond(void)
{
    // Reset Error for Development Convenient        
    if(g_f_auto_err_reset)
    {
        g_error.code = 0; 
        g_error.sect = 0; 
    }

    ASSERT_RETURN(!g_error.code, g_error.code); 
    ASSERT_RETURN(!s_stop_req,  ERR_STOP_REQ); 
    ASSERT_RETURN(!g_estop,     ERR_ESTOP_ON);
    ASSERT_RETURN(g_servo,      ERR_SERVO_OFF);     
    return 0; 
}

// Prints Actual Runtime Param
static void PrintRunParam(void)
{
    RUNPARAM_JOG* jog; 
    PROFILE7* prof; 
    double rat; 

    switch(g_mode)
    {
    case RUNMODE_JOG:
        
        jog = &s_param_req.jog;         

        R_VERB_VRB(
            "Jog Info. Coord:%s(%d) Axis:%s Dir:%s Vel:%f[/ms] Keep:%.0f[ms]\n",
            Str_Coord(jog->coord), jog->coord, Str_Axis(jog->coord, jog->axis), 
            Str_Dir(jog->dir), jog->vel, jog->keep); 

        if(jog->coord == COORDINDEX_JOINT)
        {  
            prof = &g_traj.profile; 
            rat  = g_traj.jmot[jog->axis].dbJointD;
#if 0
            R_VERB_VRB(        
		        "Motion Profile Info. in physic scale\n"
                "   TIME[ms] t_1a:%f t_ua:%f t_2a:%f t_u:%f t_1d:%f t_ud:%f t_2d:%f [ms]\n"
                "   POS[rad] p_1a:%f p_ua:%f p_2a:%f p_u:%f p_1d:%f p_ud:%f p_2d:%f \n"
                "   VEL[r/ms] v_1a:%f v_ua:%f v_2a:%f v_u:%f v_1d:%f v_ud:%f v_2d:%f \n"
                "    a_1a:%f a_ua:%f a_2a:%f a_u:%f a_1d:%f a_ud:%f a_2d:%f \n",		        
                prof->t_1a,     prof->t_ua,     prof->t_2a,     prof->t_u,     prof->t_1d,     prof->t_ud,     prof->t_2d,
                prof->p_1a*rat, prof->p_ua*rat, prof->p_2a*rat, prof->p_u*rat, prof->p_1d*rat, prof->p_ud*rat, prof->p_2d*rat,
                prof->v_1a*rat, prof->v_ua*rat, prof->v_2a*rat, prof->v_u*rat, prof->v_1d*rat, prof->v_ud*rat, prof->v_2d*rat,
                prof->a_1a*rat, prof->a_ua*rat, prof->a_2a*rat, prof->a_u*rat, prof->t_1d*rat, prof->a_ud*rat, prof->a_2d*rat);
#endif
            R_VERB_VRB("Joint Motion Info. Axis:%d, Disp:%.2f \n", 
                jog->axis, g_traj.jmot[jog->axis].dbJointD); 
                
            R_VERB_VRB("Profile Info\n"
                "             <j1a>     <ua>      <j2a>     <u>       <j1d>     <ud>      <j2d>\n"
                "  [ms]%09.2f %09.2f %09.2f %09.2f %09.2f %09.2f %09.2f\n"                
                "  [P ]%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n", 		        
                prof->t_1a, prof->t_ua, prof->t_2a, prof->t_u, prof->t_1d, prof->t_ud, prof->t_2d,
                prof->p_1a, prof->p_ua, prof->p_2a, prof->p_u, prof->p_1d, prof->p_ud, prof->p_2d);
                
#if 0
            R_VERB_VRB(
                "<j1a>  <ua> <j2a>    <u>      <j1d>     <ud>      <j2d>\n"
                "\t%05.2f %05.2f %05.2f %09.2f %09.2f %09.2f %09.2f T:ms\n"
                "\t%05.2f %05.2f %05.2f %09.2f %09.2f %09.2f %09.2f P:mm\n"
		        "\t%05.2f %05.2f %05.2f %09.2f %09.2f %09.2f %09.2f V:mm/ms\n", 
                
                prof->t_1a,     prof->t_ua,     prof->t_2a,     prof->t_u,     prof->t_1d,     prof->t_ud,     prof->t_2d,
                prof->p_1a*rat, prof->p_ua*rat, prof->p_2a*rat, prof->p_u*rat, prof->p_1d*rat, prof->p_ud*rat, prof->p_2d*rat,
                prof->v_1a*rat*1000., prof->v_ua*rat*1000., prof->v_2a*rat*1000., prof->v_u*rat*1000., prof->v_1d*rat*1000., prof->v_ud*rat*1000., prof->v_2d*rat*1000.); 
#endif      
        }  
        else 
        {
            R_VERB_VRB(
                "Cart Motion Info. wrt BASE\n"
                "  [Linear] Dir:%05.2f %05.2f %05.2f, Dis:%.2f[mm] \n"
                "  [Rotate] Dir:%05.2f %05.2f %05.2f, Ang:%.2f[rad] \n",
                g_traj.lmot.Sp.x, g_traj.lmot.Sp.y, g_traj.lmot.Sp.z, g_traj.lmot.dist, 
                g_traj.lmot.Sr.x, g_traj.lmot.Sr.y, g_traj.lmot.Sr.z, g_traj.lmot.ang); 

            prof = &g_traj.profile; 
            R_VERB_VRB("Profile Info\n"
                "             <j1a>     <ua>      <j2a>     <u>       <j1d>     <ud>      <j2d>\n"
                "  [T(ms)] %09.2f %09.2f %09.2f %09.2f %09.2f %09.2f %09.2f\n"                
                "  [P    ] %09.7f %09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n", 		        
                prof->t_1a, prof->t_ua, prof->t_2a, prof->t_u, prof->t_1d, prof->t_ud, prof->t_2d,
                prof->p_1a, prof->p_ua, prof->p_2a, prof->p_u, prof->p_1d, prof->p_ud, prof->p_2d);
        }
        return; 

    case RUNMODE_TIMETEST:        
    case RUNMODE_NONE:
    default:
        return; 
    }
}

static void ResetRequest(void)
{
    s_stop_req = 0; 
    s_stop_req_mod = 0; 
    s_mode_req = 0; 
}

static void FileWrite(int nRob)
{
    if(g_mode == RUNMODE_NONE)
    {
        File_Close(nRob); 
        return; 
    }
    // File Open at start of mode. 
    // FILE_Open doesn't open again if the file has opened already. 
    if(g_f_file)
    {
        File_Open(nRob); 
    }
    File_Write(NULL, g_pos_trg, NULL, g_pos_act, nRob); 
}

static void AxisToCart(void)
{
#if 0
    // target
    g_Forward(&g_bTe, g_pos_trg, g_dh);   
    g_bTt = TRANS_Multi_TRANS(g_bTe, g_traj.eTt); 
    g_xyzrpy = TRANS_GetXyzrpy(g_bTt); 
#else
    TRANS cTt;         
    // 
    g_Forward(&g_bTe, g_pos_trg, g_dh);   
    g_bTt = TRANS_Multi_TRANS(g_bTe, g_eTt);
    // 
    g_Forward(&g_bTe_act, g_pos_act, g_dh);  
    g_bTt_act = TRANS_Multi_TRANS(g_bTe_act, g_eTt); 

    // xyzrpy 
    if(g_coord_ref == COORDINDEX_JOINT)
    {
        // target
        memcpy(g_xyzrpy.dbMember,     g_pos_trg, sizeof(g_xyzrpy)); 
        // actual
        memcpy(g_xyzrpy_act.dbMember, g_pos_act, sizeof(g_xyzrpy)); 
    }
    else
    {
#if 0
        // target
        g_Forward(&g_bTe, g_pos_trg, g_dh);           
        TcpWrtActualCoord(&cTt, g_coord_ref); 
        g_xyzrpy = TRANS_GetXyzrpy(cTt);     
        // actual
        g_Forward(&g_bTe_act, g_pos_act, g_dh);   
        TcpWrtActualCoord(&cTt, g_coord_ref); 
        g_xyzrpy_act = TRANS_GetXyzrpy(cTt); 
#endif         
        // target
        TcpWrtTargetCoord(&cTt, g_coord_ref); 
        g_xyzrpy = TRANS_GetXyzrpy(cTt);     

        // actual        
        TcpWrtActualCoord(&cTt, g_coord_ref); 
        g_xyzrpy_act = TRANS_GetXyzrpy(cTt); 
    }
#if 0
    g_bTt = TRANS_Multi_TRANS(g_bTe, g_traj.eTt); 
#endif

#endif
}		

static void InitTargetPos(int n_rob)
{
    // Target Pos Init
    ShmRead();              // Read Act Motor Pulse
#if 0
    Motor_To_Axis(n_rob);   // Convert Motor Pulse to Axis (g_pos_act)
#endif
    CopyActual(); 
}

static void CopyActual(void)
{
    memcpy(g_pos_trg, g_pos_act, sizeof(g_pos_trg));    // copy
    memcpy(g_pos_prev, g_pos_act, sizeof(g_pos_prev)); 
    memset(g_vel_prev, 0, sizeof(g_vel_prev)); 
#if 0
    memcpy(g_do, g_do_prev, sizeof(g_do)); 
    memcpy(g_ao, g_ao_prev, sizeof(g_ao)); 
#endif
}

// weld condition 
extern WELD_COND_START g_weld_start; // start welding condition [mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_major; // major welding condition [mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_weave; // start-weaving welding condition[mm, ms, A, V]
#if 0   // Definition of EDIST Welding Condition is corrected. 
extern WELD_COND_MAIN  g_weld_major_edist; // major welding condition when edist of weaving is exist[mm, ms, A, V]
extern WELD_COND_MAIN  g_weld_weave_edist; // start-weaving welding conditionn when edist of weaving is exist[mm, ms, A, V]
#endif 
extern WELD_COND_MAIN  g_weld_major_edist; // EDIST welding condition [mm, ms, A, V]
extern WELD_COND_END   g_weld_final; // final welding condition [mm, ms, A, V]

