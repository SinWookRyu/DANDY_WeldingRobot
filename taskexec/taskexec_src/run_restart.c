#include "taskexec_def.h"
#include "cmd.h"
#include "call_stack.h"

static int MoveInit(void); 
static int CmotInit(void); 
static int WeaveMotInit(void); 
static int LastCmdInit(void); 
static VOD WeldIoReset(void); 
static int PrepurgeInit(void); 
static int ArconInit(void); 
static VOD ProgramInit(void); 
static VOD RecoverProgVar(void); 
static UNT WasVertCmd(int i_prog); 
#if 0
static DBL OverlapDistGet(volatile SHM_RESTART* restart); 
#endif
static DBL OverlapDistGet_Horz(volatile SHM_RESTART* restart); 
static DBL OverlapDistGet_Vert(volatile SHM_RESTART* restart); 
static void CweaveDataRecover(void); 
// Cweave Cmd Init : Weaving Part(weaving motion init) / Waiting Part(just start)
static int CweavCmdInit(void); 

static DBL s_overlap;               // actual overlap dist

////////////////////////////////////////////////////////////////////////////////

// Sets Restart Data 'g_restart'. 
// f_set : Flag for setting Shm restartable. 
// mode_prog : PROG_MODE_XXX
void RestartDataSet(UNT f_set, UNT mode_prog)
{    
    if(f_set == OFF)
    {
        g_restart.f_restart = OFF; // sets/resets restartable                
    }
     
    else
    {
        // flag for if g_restart is available
        g_restart.f_restart = ON; // sets/resets restartable

        // program control data
        g_restart.mode_prog = mode_prog; 
        g_restart.comp_result = g_comp_result; 
        g_restart.f_welding = g_f_weld; 
        g_restart.i_prog = g_i_prog_act;
        g_restart.weld_spd = g_weld_spd; 
        g_restart.dist_work = g_dist_work; 

        // for cweave
        g_restart.cweave_mode  = g_last_mode_cweave; // cwv_mode;     
        g_restart.cweave_conf = cwv_conf; 
        g_restart.cweave_y_dir= cwv_y_dir; 
        g_restart.cweave_t_rest = cwv_t_rest; 

        // welding curr & volt
        g_restart.ao_val_volt  = g_ao[g_idx_ao_volt];      
        g_restart.ao_val_curr  = g_ao[g_idx_ao_curr]; 
        g_restart.ao_type_curr = g_ao_type[g_idx_ao_curr];
        g_restart.ao_type_volt = g_ao_type[g_idx_ao_volt]; 

        // for arc sensor 
        g_restart.f_sensor = weld_idx.arc_sensor; 
        g_restart.ref_curr = ref_current; 
        g_restart.ref_cent = ref_weight; 
        // no meaning. 'ref_current & weight' are reseted Main Weaving Node starts. 
        // Main Weaving Node always new starts after restart is operated. 

        // last mode of cweave, weave, traj

        memcpy(g_restart.jnt_f, g_pos_trg, sizeof(g_restart.jnt_f)); 
        memcpy(&g_restart.stack, CallStack_Ptr(), sizeof(CALL_STACK)); 
    
        // last traj & weaving data save. 
        // The modes of each are saved manually because they are set to 0 at end.  
        g_restart.traj = g_traj; 
        g_restart.traj.type = g_last_mode_traj; 
        
        g_restart.weave = g_weave; 
        g_restart.weave.mode = g_last_mode_weav; 

        // for cweave
        g_restart.Tf_cwv  = cwv_Tf; 
        g_restart.Tv1_cwv = cwv_Tv1; 
        g_restart.Tv2_cwv = cwv_Tv2; 
        g_restart.wvf_cwv = cwv_wvf; 

        // welding file loaded finally
        g_restart.swf = g_weld_start;  
        g_restart.ewf = g_weld_final;  
        g_restart.mwf_pri = g_weld_major;  
        g_restart.mwf_sub = g_weld_weave;  
        g_restart.mwf_pri_ed = g_weld_major_edist; 
#if 0 // Definition of EDIST Welding Condition is corrected. 
        g_restart.mwf_sub_ed = g_weld_weave_edist; 
#endif 
    }

    // SHM 
    memcpy(((byte*)g_pshm_restart)+sizeof(SHM_RESTART), &g_restart, sizeof(g_restart)); 
}

// restart mode(step)
#define RES_STEP_NONE       0
#define RES_STEP_MOVE_INIT  1
#define RES_STEP_PREPURGE   2           
#define RES_STEP_ARCON      3
#define RES_STEP_LAST_CMD   4

static UNT s_step; 
static DBL s_time; 
static UNT s_f_stop;    // req-flag for stop
static UNT s_mod_stop;  // stop req. mode. 1:quick 0:normal

// Inits RESTART Runtime Service
// Returns Suc(0), Fail(Else).
int RunServRestart_Init(const RUNPARAM_RESTART* restart)
{
    int ret; 

    // check restartable
    if(g_restart.f_restart == OFF)
    {
        return ERR_NO_RESTART_DATA; 
    }

    // Traj Error(Including Inverse Error, Limit Error, ..) cannot be restarted. 
    if(g_restart.traj.error)
    {
        return ERR_TRAJ_ERR_NO_RESTART;         
    }

    // Init for moving to the restart pos & rot. 
    ret = MoveInit(); 
    if(ret)
    {
        return ret;         
    }

    // init setting
    s_f_stop = 0; 
    s_mod_stop = 0; 
    s_step = RES_STEP_MOVE_INIT;       
    R_VERB_VRB("RESTART starts MOVE-INIT step.\n"); 
    return 0; 
}

// Updates RESTART Runtime Service. 
// Returns RESTART is running(0), End of Running(Else)
int RunServRestart_Update(void)
{
    int ret; 

    if(s_step == RES_STEP_NONE)
    {
        return -1;
    }
    
    // stop request to Sub-modules
    if(s_f_stop)
    {
        Traj_Stop(&g_traj, s_mod_stop);    
        Wv_Stop(&g_weave, s_mod_stop); 
        Cmd_Stop(g_restart.i_prog, s_mod_stop); 
        s_time = -0.; 
    }

    // Updates as mode. 
    switch(s_step)
    {
    case RES_STEP_MOVE_INIT:
        
        // update trajectory 
        if(Traj_Update(&g_traj) == 0)
        {   
            return 0; 
        }

        // End of Trajectory ///////////////////////////////////////////////////

        // traj error check 
        if(g_traj.error)
        {
            ERROR_SET(SECT_RESTART, ERRCODE_TRAJ(g_traj.error)); 
            WeldIoReset(); 
            return -1;
        }

        // stop by stop req / error
        if(s_f_stop || g_error.code)
        {
            WeldIoReset(); 
            return -1; 
        }

        // If Welding was running, go to arc-on with start of prepurging.             
        if(g_restart.f_welding)
        {            
            ret = PrepurgeInit(); 
            if(ret)
            {                
                ERROR_SET(SECT_RESTART, ret); 
                WeldIoReset(); 
                return -1;
            }

            // set next step 
            R_VERB_VRB("RESTART starts Prepurge Step.\n");         
            s_step = RES_STEP_PREPURGE; 
            return 0; 
        }
        // If Welding was not running, go to simple restart.  
        else
        {
            ret = LastCmdInit(); 
            if(ret)
            {                
                ERROR_SET(SECT_RESTART, ret); 
                WeldIoReset(); 
                return -1;
            }

            R_VERB_VRB("RESTART starts Last-Cmd Step.\n");         
            s_step = RES_STEP_LAST_CMD; 
            return 0; 
        }
        
    case RES_STEP_PREPURGE:

        // update time
        s_time -= g_traj.t_samp; 
        if(0 < s_time)
        {            
            return 0; 
        }

        // End of Waiting Prepurging Time //////////////////////////////////////

        // stop by stop req / error
        if(s_f_stop || g_error.code)
        {
            WeldIoReset(); 
            return -1; 
        }

        // Inits Next Step
        ret = ArconInit(); 
        if(ret)
        {            
            ERROR_SET(SECT_RESTART, ret);
            WeldIoReset(); 
            return -1; 
        }

        // Sets Next Step
        s_step = RES_STEP_ARCON; 
        R_VERB_VRB("RESTART starts ARCON Step.\n");         
        return 0;

    case RES_STEP_ARCON:

        // update time
        s_time -= g_traj.t_samp; 
        if(0 < s_time)
        {
            return 0;
        }

        // END OF Start Arc ////////////////////////////////////////////////////

        // stop by stop req / error
        if(s_f_stop || g_error.code)
        {
            WeldIoReset(); 
            return -1; 
        }

        // Arc on sig fail Exit 
        if(WELD_GET_ARC == OFF)
        {
            WeldIoReset(); 
            ERROR_SET(SECT_RESTART, ERR_ARCON_SIG_FAIL);
            return -1; 
        }

        // Main Welding Condition Setting as Vert/Horz 
        if(WasVertCmd(g_restart.i_prog))
        {
            WELD_SET_VOLT(g_pshm_restart->vert_main_vol); 
            WELD_SET_CURR(g_pshm_restart->vert_main_cur); 
        }
        else
        {
            WELD_SET_VOLT(g_pshm_restart->hori_main_vol); 
            WELD_SET_CURR(g_pshm_restart->hori_main_cur); 
        }
                
        // Init Next Step 
        ret = LastCmdInit(); 
        if(ret)
        {            
            ERROR_SET(SECT_RESTART, ret);
            WeldIoReset(); 
            return -1; 
        }

        // Sets Next Step
        s_step = RES_STEP_LAST_CMD; 
        R_VERB_VRB("RESTART starts LAST-CMD Step.\n");         
        return 0;

    case RES_STEP_LAST_CMD:

        ret = Cmd_Update(g_restart.i_prog); 

        // It's wrong that Cmd ends normally. 
        if(ret)
        {
            s_step = RES_STEP_NONE; 
            WeldIoReset(); 
            return ret; 
        }

        ////////////////////////////////////////////////////////////////////////
        // RESTART END

        if(s_overlap <= g_dist_work + EPS_P)
        {            
            ProgramInit();             

            s_step = RES_STEP_NONE; 
            g_mode = RUNMODE_PROG; 
            R_VERB_MSG("RESTART Completed Overlap-dist(%.2f[mm]). Start PROG(%s mode)\n", 
                        s_overlap, Str_ProgMode(RunServProg_ModeGet())); 
            return 0; // !!!!! restart return 0 at end of work. !!!!!
        }
        return ret; 

    default:
        ERROR_SET(SECT_RESTART, ERR_UNDEF_STEP);
        return -1; 
    }
    return -1; 
}

void RunServRestart_Stop(int f_quick)
{   
    s_f_stop   = 1; 
    s_mod_stop = (f_quick)? 1:0;   // stop mode 0:normal, 1:quick
}

////////////////////////////////////////////////////////////////////////////////

extern void Cweave_Reset_Stop(void); 
extern void Cweave_Reset_Time(void); 
extern UNT Cweave_IsWeavingMode(int mode); 

// Initiates moving to the restart position & rotation. 
// The restart position is included the overlap dist to the stop pos If the res-
// tart cmd is overlap cmd 
// If the restart cmd is for WEAVE, It is on the center line of weaving. 
// Motion type can be the joint or linear motion as the request. 
// Return : error code. 
static int MoveInit(void)
{
    int ret; 
    TRANS  bTe_f, bTt_f;
    double vel; 
    unsigned conf; 
        
    //  config & velocity    
    conf = g_Config(g_restart.jnt_f, g_dh); 
    vel = g_pshm_restart->path_speed; 

    // Stop Pos & Rot
    g_Forward(&bTe_f, g_restart.jnt_f, g_dh); // bTe
    bTt_f = TRANS_Multi_TRANS(bTe_f, g_eTt);                 // bTt=bTe*eTt
        
    // restart pos & rot for cweave
    // on weaving overlap adjusted, on dwell no overlap adjusted. 
    if(Cmd_IsCweaveCmd(g_restart.i_prog))
    {        
        if(Cweave_IsWeavingMode(g_restart.cweave_mode))
        {
            WEAVE* wv;  
            double dist_done; 
            wv = &g_restart.weave; 
            
            s_overlap = (g_restart.cweave_mode == CWEAVE_MODE_VERT)? 
                        OverlapDistGet_Vert(g_pshm_restart): 
                        OverlapDistGet_Horz(g_pshm_restart); 
            dist_done = g_restart.dist_work - s_overlap; 
            bTt_f.p = POS_Plus_POS(wv->pw, POS_Multi_S(wv->xw, dist_done));   
        }
        else
        {
            s_overlap = 0; 
            bTt_f.p = bTt_f.p;
        }
    }
    // restart pos for weaving. 
    // It is on the center line of weaving & regars overlap dist
    else if(Cmd_IsWeaveCmd(g_restart.i_prog))
    {
        
        WEAVE* wv;  
        double dist_done; 
        wv = &g_restart.weave; 

        s_overlap = OverlapDistGet_Vert(g_pshm_restart); 
        dist_done = g_restart.dist_work - s_overlap;         
        bTt_f.p = POS_Plus_POS(wv->pw, POS_Multi_S(wv->xw, dist_done));   
    }  
    // Restart Pos for Linear Motion. It regards Overlap-dist. 
    else if(Cmd_IsLinMot(g_restart.i_prog))
    {
        TRAJ* traj;                
        traj = &g_restart.traj; 

        // Inv Err -> Motion type changes into Joint. 
        if(traj->type != TRAJ_TYPE_LINEAR)
        {
            return ERR_NO_LMOT_DATA_SAVED;  
        }

        s_overlap = OverlapDistGet_Horz(g_pshm_restart);
        bTt_f.p = POS_Plus_POS(bTt_f.p, POS_Multi_S(traj->lmot.Sp, -s_overlap));                      
    }
    // Restart Pos for anther Cmds. It is the final stop pos.  
    else
    {
        s_overlap = 0; 
        bTt_f.p = bTt_f.p;   // no work
    }
    
    // Init Motion of Linear Type
    if(g_pshm_restart->moving_type == RESTART_MOVE_TYPE_LIN)
    {   
        ret = Traj_Lmot_Set2(&g_traj, g_pos_act, &bTt_f, conf, vel, 0);
        return ERRCODE_TRAJ(ret); 

    }
    // Init Motion of Joint Type
    else
    {   
        double jnt_s[6];         
        
        // No overlap : The final joint is target joint. 
        // Direct Copy To avoid Inverse error in case of Zero joint of the final. 
        if(s_overlap == 0)
        {
            memcpy(jnt_s, g_restart.jnt_f, sizeof(jnt_s)); 
        }
        // Overlap
        else
        {
            // Target Joint by Inverse. 
            bTe_f = TRANS_Multi_TRANS(bTt_f, TRANS_Inv(g_eTt)); 
            ret = g_Inverse(jnt_s, &bTe_f, conf, g_dh, g_restart.jnt_f, OFF); 
            if(ret)
            {
                return ERRCODE_INVERSE(ret);
            }
        }
        // joint motion
        ret = Traj_Set_Jmot2(&g_traj, g_pos_act, jnt_s, vel); 
        return ERRCODE_TRAJ(ret);                     
    }  
}

static VOD RecoverProgVar(void)
{    
    // Copies Program Env. Params from buffer
    g_comp_result = g_restart.comp_result;         
    g_f_weld      = g_restart.f_welding;
    g_weld_spd    = g_restart.weld_spd; 

    // arc sensor 
    weld_idx.arc_sensor = g_restart.f_sensor; 
    ref_current = g_restart.ref_curr; 
    ref_weight  = g_restart.ref_cent; 

    memcpy(CallStack_Ptr(), (void*)&g_restart.stack, sizeof(CALL_STACK));     
    memcpy(&g_weld_start, (void*)&g_restart.swf, sizeof(g_weld_start));     
    g_weld_final = g_restart.ewf; 
    g_weld_major = g_restart.mwf_pri; 
    g_weld_weave = g_restart.mwf_sub; 
    g_weld_major_edist = g_restart.mwf_pri_ed; 
#if 0 // Definition of EDIST Welding Condition is corrected
    g_weld_weave_edist = g_restart.mwf_sub_ed; 
#endif 
}

static VOD ProgramInit(void)
{
    RUNPARAM_PROG prog; 

    // Final Welding Curr/Volt Recover
    if(g_restart.f_welding)
    {
        g_ao[g_idx_ao_volt] = g_restart.ao_val_volt;     
        g_ao[g_idx_ao_curr] = g_restart.ao_val_curr; 
        g_ao_exec[g_idx_ao_curr] = ON; 
        g_ao_exec[g_idx_ao_volt] = ON; 
        g_ao_type[g_idx_ao_curr] = g_restart.ao_type_curr; 
        g_ao_type[g_idx_ao_volt] = g_restart.ao_type_volt;        
    }

    // Recovers Program Mode. 
    prog.i_start  = g_restart.i_prog; 
    prog.mode_run = g_restart.mode_prog; 
    RunServProg_Reset(&prog);     
}

// Returns Horizontal Overlap Distance from SHM_RESTART. 
// Overlap Distance must not be shorter than the worked-distance. 
static DBL OverlapDistGet_Horz(volatile SHM_RESTART* restart)
{   
    return (g_pshm_restart->d_overlap_horz < g_restart.dist_work)? 
            g_pshm_restart->d_overlap_horz : g_restart.dist_work; 
}

// Returns Vertical Overlap Distance from SHM_RESTART. 
// Overlap Distance must not be shorter than the worked-distance. 
static DBL OverlapDistGet_Vert(volatile SHM_RESTART* restart)
{   
    return (g_pshm_restart->d_overlap_vert < g_restart.dist_work)? 
            g_pshm_restart->d_overlap_vert : g_restart.dist_work; 
}

static int CmotInit(void)
{    
    CMOT* cmot; 
    int ret; 
    UNT conf; 
    DBL prog; 
    DBL arc, rot; 
    
    // conf = g_Config(g_pos_trg, g_dh);   
    conf = g_restart.traj.config; 
    cmot = &g_restart.traj.cmot; 
    prog = g_restart.dist_work / cmot->arc; 
    arc = cmot->arc - cmot->arc * prog; 
    rot = cmot->ang - cmot->ang * prog; 

    ret = Traj_Set_Cmot(&g_traj, g_pos_trg, &cmot->Pc, &cmot->Sp, 
        arc, cmot->v_p, &cmot->Sr, rot, 0, conf);               

    return ERRCODE_TRAJ(ret); 
}

static int WeaveMotInit(void)
{  
    WEAVE* wv; 
    TRANS Tf; 
    double pitch_s, width_s, speed_s; 
    double d_done, d_total, d_start, d_angle, d_final; 
    int ret; 
    
    wv = &g_restart.weave; 
    if(!Wv_IsRunning(wv))
    {
        return ERR_NO_WEAVE_DATA_SAVED; 
    }

    // weaving working done distance
    d_done = g_restart.dist_work - s_overlap; 
    d_done = (0 <= d_done)? d_done : 0; 

    // total remained distance to work
    d_total = wv->d_total - d_done; 
    d_total = (0 <= d_total)? d_total : 0; 
    
    // angle dist to work
    // Actually no need to check the angle distance. It means the rotation is 
    // already over that 'd_angle' < 0. 
    d_angle = (wv->d_angle - d_done); 
    d_angle = (0 <= d_angle)? d_angle : 0; 
    
    // weaving start distance to work
    d_start = (Wv_StartDist(wv) - d_done); 
    d_start = (0 <= d_start)? d_start : 0; 
    
    // weaving final distance to work
    d_final = (wv->d_final < d_total)? wv->d_final : d_total; 

    // Final Tf to work. 
    Wv_FinalTrans(wv, &Tf.p, &Tf.R); 

    // The criteria of start condition is the final condition. 
    pitch_s = (wv->n_seg == WV_SEG_START)? wv->pitch_s : wv->pitch; 
    width_s = (wv->n_seg == WV_SEG_START)? wv->width_s : wv->width; 
    speed_s = (wv->n_seg == WV_SEG_START)? wv->v_pos_s : wv->v_pos;     

    // weaving init. 
    ret = Wv_Set(&g_weave,    &g_traj,        g_pos_trg,   
                 &Tf,         wv->config,     &wv->yw,    0., 
                 wv->pitch,   wv->width,      wv->v_pos, 
                 wv->pitch_s, wv->width_s,    wv->v_pos_s, 
                 wv->dwl_even,wv->dwl_even,   wv->dwl_cent, 
                 d_angle,     d_start,        d_final); 

    return ERRCODE_WEAVE(ret); 
}

static void CweaveDataRecover(void)
{   
    cwv_mode  = g_restart.cweave_mode;     
    cwv_conf  = g_restart.cweave_conf;  
    cwv_y_dir = g_restart.cweave_y_dir;  
    cwv_t_rest= g_restart.cweave_t_rest;     
    
    cwv_Tf = g_restart.Tf_cwv; 
    cwv_Tv1 = g_restart.Tv1_cwv; 
    cwv_Tv2 = g_restart.Tv2_cwv; 
    cwv_wvf = g_restart.wvf_cwv; 
}

// Cweave Cmd Init : Weaving Part(weaving motion init) / Waiting Part(just start)
static int CweavCmdInit(void)
{
    int ret; 

    // If Weaving-Motion was running, Weaving Motion Inits. 
    if(Cweave_IsWeavingMode(g_restart.cweave_mode))
    {   
        ret = WeaveMotInit(); 
        EXIST_RETURN(ret, ret); 
    }

    // Cweave Cmd Control Var's Recover & Reset
    CweaveDataRecover(); 
    Cweave_Reset_Time(); 
    Cweave_Reset_Stop(); 

    // Cweave Cmd Handler Set
    Cmd_ExtHandlerSet(CmdUpd_CWEAV, CmdStp_CWEAV, "CWEAV"); 
    return 0; 
}

static int LastCmdInit(void)
{  
    int ret; 

    // 1st restartable check is at Init. 
    if(g_restart.f_restart == OFF)
    {
        return ERR_NO_RESTART_DATA; 
    }

    // Recoverse Program Vars
    RecoverProgVar(); 

    // Cweaving Cmd Init
    if(Cmd_IsCweaveCmd(g_restart.i_prog))
    {
        ret = CweavCmdInit(); 
        EXIST_RETURN(ret, ret);         
        return 0; 
    }
    // Weaving Cmd Init
    else if(Cmd_IsWeaveCmd(g_restart.i_prog)) 
    {
        // New Weaving Init
        ret = WeaveMotInit();   
        EXIST_RETURN(ret, ret);         
        Cmd_ExtHandlerSet(CmdUpd_WEAV, CmdStp_WEAV, "WEAV"); 
        return 0; 
    }
    // MOVC Cmd 
    else if(Cmd_IsCirMot(g_restart.i_prog))
    {
        // Circular Motion Set
        ret = CmotInit();   
        EXIST_RETURN(ret, ret);   
        Cmd_ExtHandlerSet(CmdUpd_Mov, CmdStp_Mov, "CMOT"); 
        return 0; 
    }
    // Another Cmd, Just Cmd Init
    else
    {
        UNT f_dry;         
        f_dry = (g_restart.mode_prog == PROG_RUNMODE_DRY)? ON : OFF; 
        return Cmd_Init(g_restart.i_prog, g_pos_trg, f_dry);         
    }
}

static int PrepurgeInit(void)
{   
    DANDY_JOB_SWF start; 
    
    // welder on check     
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        return ERR_WELDER_POWER_FAIL; 
    }
    
    // get start welding file saved. 
    memcpy(&start, (void*)&g_restart.swf, sizeof(start)); 

    // arc-sensor data init
    // Here
   
    WELD_SET_TCH_MC(OFF); 
    WELD_SET_GAS(ON); 

    s_step = RES_STEP_PREPURGE;     
    s_time = start.dbPreflowTime; 
    return 0; 
}

static int ArconInit(void)
{
    DANDY_JOB_SWF start; 

    // gas on fail & exit
    if(WELD_GET_NO_GAS == ON)
    {
        return ERR_GASON_SIG_FAIL; 
    }

    // get start welding file saved. 
    memcpy(&start, (void*)&g_restart.swf, sizeof(start)); 
    
    // start welding Condition Setting as Vert/Horz 
    if(WasVertCmd(g_restart.i_prog))
    {
        WELD_SET_VOLT(g_pshm_restart->vert_start_vol); 
        WELD_SET_CURR(g_pshm_restart->vert_start_cur); 
    }
    else
    {
        WELD_SET_VOLT(g_pshm_restart->hori_start_vol); 
        WELD_SET_CURR(g_pshm_restart->hori_start_cur); 
    }

    WELD_SET_ARC(ON); 
    s_time = start.dbArcTime; 
    return 0; 
}

// Vertical Cmds are
// - UWEAVE
// - Vert. Weaving in CWEAVE
static UNT WasVertCmd(int i_prog)
{
    if(g_rcmd[i_prog].nCode == DANDY_JOB_CODE_UWEAVL || 
       (g_rcmd[i_prog].nCode == DANDY_JOB_CODE_CWEAV && 
        g_restart.cweave_mode == CWEAVE_MODE_VERT)     )
    {
        return ON; 
    }
    else
    {
        return OFF; 
    }
}   

static VOD WeldIoReset(void)
{   
    WELD_SET_ARC(OFF); 
#if 0
    WELD_SET_TCH_MC(ON); 
    WELD_SET_GAS(OFF);     
    WELD_SET_VOLT(0.); 
    WELD_SET_CURR(0.);     
#endif
}

