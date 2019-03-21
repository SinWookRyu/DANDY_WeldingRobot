////////////////////////////////////////////////////////////////////////////////

#include "taskexec_def.h"

// Print Additional Events
static void WeldCondCtrl(void); 

int RUNSERV_INIT(int mode, const RUN_PARAM* param)
{
    int ret; 
    TIME_PRECISE t[2]; 
    switch(mode)
    {            
    case RUNMODE_PROG:
t[0] = TimeGet(); 
        ret = RunServProg_Init(&param->prog); 
t[1] = TimeGet();
#if TIMECHK_
R_VERB_MSG("Init Prog:%llu[ns]\n", t[1]-t[0]); 
#endif
R_VERB_TIMECHK("Init Prog:%llu[ns]\n", t[1]-t[0]); 
        return ret; 

    case RUNMODE_RESTART:
t[0] = TimeGet(); 
        ret = RunServRestart_Init(&param->restart); 
t[1] = TimeGet();
#if TIMECHK_
R_VERB_MSG("Init Restart:%llu[ns]\n", t[1]-t[0]); 
#endif
R_VERB_TIMECHK("Init Restart:%llu[ns]\n", t[1]-t[0]); 
        return ret; 

    case RUNMODE_JOG:     
t[0] = TimeGet();
        ret = RunServJog_Init(&param->jog); 
t[1] = TimeGet();
#if TIMECHK_
R_VERB_MSG("Init Jog:%llu[ns]\n", t[1]-t[0]); 
#endif
R_VERB_TIMECHK("Init Restart:%llu[ns]\n", t[1]-t[0]); 
        return ret; 

    case RUNMODE_TIMETEST:
        return RunServTimerTest_Init(); 

    case RUNMODE_NONE:
    default:
        return RunServNormal_Init(); 
    }
}

int RUNSERV_UPDATE(int mode)
{
    int ret; 
    switch(mode)
    {
    case RUNMODE_PROG:
        ret = RunServProg_Update(); 
        WeldCondCtrl(); 
        return ret; 

    case RUNMODE_RESTART:
        return RunServRestart_Update(); 

    case RUNMODE_JOG:
        return RunServJog_Update(); 

    case RUNMODE_TIMETEST:
        return RunServTimerTest_Update(); 

    case RUNMODE_NONE:
    default:
        return RunServNormal_Update(); 
    }
}
void RUNSERV_RUN(int mode)
{
    switch(mode)
    {
    case RUNMODE_PROG:
        return; 
    case RUNMODE_RESTART:
        return; 
#if 1
    case RUNMODE_JOG:
        ;//RunServJog_Run();
        return; 
#else
    case RUNMODE_JOG:
        (g_mode_jog == 1)? RunServJogWeave_Run() : RunServJog_Run(); 
        return; 
#endif

    case RUNMODE_TIMETEST:
        RunServTimerTest_Run(); 
        return; 

    case RUNMODE_NONE:
    default:
        RunServNormal_Run(); 
        return; 
    }
}
#if 0
void RUNSERV_STOP(int mode)
{
    RunServProg_Stop(); 
    RunServJog_Stop();                 
    RunServTimerTest_Stop(); 
    RunServNormal_Stop();     
}
#else
void RUNSERV_STOP(int f_quick)
{
    RunServRestart_Stop(f_quick); 
    RunServProg_Stop(f_quick);     
    RunServJog_Stop(f_quick);                 
    RunServTimerTest_Stop(); 
    RunServNormal_Stop();     
}

#endif

////////////////////////////////////////////////////////////////////////////////

int RunServNormal_Init(void)
{return 0;}

int RunServNormal_Update(void)
{return -1;}

void RunServNormal_Run(void)
{return ;}

void RunServNormal_Stop(void)
{return ;}

////////////////////////////////////////////////////////////////////////////////
static void WeldCondCtrl(void)
{
    // Program Ctrl Var's Setting     
    g_f_skip  = (PROGCOND_SKIP_LEFT || PROGCOND_SKIP_RIGHT)?  ON : OFF; // skip condition        
    g_n_shift = (PROGCOND_GAP_HORZ)?  g_pshm_rm_sys->nHorzGapShift     :// weld cond shift by left/right/horz
                (PROGCOND_GAP_LEFT)?  g_pshm_rm_sys->nVertLeftGapShift : 
                (PROGCOND_GAP_RIGHT)? g_pshm_rm_sys->nVertRightGapShift: 0; 
    
    // Weaving Curr & Volt
    if(g_mode && g_weave.mode != WV_MODE_NONE && g_weave.f_seg_change)
    {   
        DBL volt = 0.;
        DBL curr = 0.; 

        // Welding Condition Modification During Weaving 

#if 0
        // On Start Wv Segment Start
        if(g_weave.n_seg==WV_SEG_START)
        {
            R_VERB_VRB("Starts Start Weaving Seg.\n");       
            if(g_f_weld)
            {
                if(g_weave.d_final > 10.) // WV has EDIST. 
                {
                    volt = (g_weld_weave_edist.dbVoltage); 
                    curr = (g_weld_weave_edist.dbCurrent);     
                    R_VERB_VRB("Weld Cond with End Dist Applied. ED=%.2f %.2f[V] %.2f[A]\n", g_weave.d_final, volt, curr); 
                }
                else
                {
                    volt = (g_weld_weave.dbVoltage); 
                    curr = (g_weld_weave.dbCurrent);                 
                    R_VERB_VRB("Weld Cond without End Dist Applied. ED=%.2f %.2f[V] %.2f[A]\n", g_weave.d_final, volt, curr); 
                }
                WELD_SET_VOLT(volt); 
                WELD_SET_CURR(curr);                 
            }
        }
        // On Main Wv Segment Start
        if(g_weave.n_seg==WV_SEG_MAIN)
        {
            R_VERB_VRB("Starts Main Weaving Seg.\n");       
            if(g_f_weld)
            {
                if(g_weave.d_final > 10.) // WV has EDIST. 
                {
                    volt = (g_weld_major_edist.dbVoltage); 
                    curr = (g_weld_major_edist.dbCurrent); 
                    R_VERB_VRB("Weld Cond with End Dist Applied. ED=%.2f %.2f[V] %.2f[A]\n", g_weave.d_final, volt, curr); 
                }
                else
                {
                    volt = (g_weld_major.dbVoltage); 
                    curr = (g_weld_major.dbCurrent); 
                    R_VERB_VRB("Weld Cond without End Dist Applied. ED=%.2f %.2f[V] %.2f[A]\n", g_weave.d_final, volt, curr); 
                }
                WELD_SET_VOLT(volt); 
                WELD_SET_CURR(curr);       

                // to save I, V data 
                if(weld_idx.arc_sensor == ON)
                {
                    shm_servo->Iw = curr; 
                    shm_servo->Vw = volt; 
                    shm_servo->pitch = g_weave.pitch; 
                    shm_servo->width = g_weave.width; 
                    shm_servo->speed = g_weave.v_pos; 
                    shm_servo->dwl_e = g_weave.dwl_even; 
                    shm_servo->dwl_o = g_weave.dwl_odd; 
                }
            }            
        }

        // On Final Wv Segment, Returns to the major welding condition. 
        if(g_weave.n_seg==WV_SEG_FINAL)
        {
            R_VERB_VRB("Starts End Weaving Seg.\n");       
        }        
#endif 
        // On Start Wv Segment Start
        if(g_weave.n_seg==WV_SEG_START)
        {
            R_VERB_VRB("Starts Start Weaving Seg.\n");       
            if(g_f_weld)
            {
                volt = (g_weld_weave.dbVoltage); 
                curr = (g_weld_weave.dbCurrent);                 
                R_VERB_VRB("Start Weaving Weld Cond is Applied. %.2f[V] %.2f[A]\n", volt, curr); 
                WELD_SET_VOLT(volt); 
                WELD_SET_CURR(curr);                 
            }
        }
        // On Main Wv Segment Start
        if(g_weave.n_seg==WV_SEG_MAIN)
        {
            R_VERB_VRB("Starts Main Weaving Seg.\n");       
            if(g_f_weld)
            {
                volt = (g_weld_major.dbVoltage); 
                curr = (g_weld_major.dbCurrent); 
                R_VERB_VRB("Main Weaving Weld Cond is Applied. %.2f[V] %.2f[A]\n", volt, curr); 
                WELD_SET_VOLT(volt); 
                WELD_SET_CURR(curr);       

                // to save I, V data 
                if(weld_idx.arc_sensor == ON)
                {
                    shm_servo->Iw = curr; 
                    shm_servo->Vw = volt; 
                    shm_servo->pitch = g_weave.pitch; 
                    shm_servo->width = g_weave.width; 
                    shm_servo->speed = g_weave.v_pos; 
                    shm_servo->dwl_e = g_weave.dwl_even; 
                    shm_servo->dwl_o = g_weave.dwl_odd; 
                }
            }            
        }

        // On Final Wv Segment, Returns to the major welding condition. 
        if(g_weave.n_seg==WV_SEG_FINAL)
        {
            R_VERB_VRB("Starts End Weaving Seg.\n");       

            if(g_weave.d_final > 10.) // WV has EDIST. 
            {
                volt = (g_weld_major_edist.dbVoltage); 
                curr = (g_weld_major_edist.dbCurrent); 
                R_VERB_VRB("End Weave Weld Condition is Applied. ED=%.2f %.2f[V] %.2f[A]\n", g_weave.d_final, volt, curr); 
                WELD_SET_VOLT(volt); 
                WELD_SET_CURR(curr);       
            }
            else
            {                
                R_VERB_VRB("Small End Dist Keeps on Weld Cond of Main Weaving to End Weaving. ED=%.2f\n", g_weave.d_final); 
            }
        }   
    }
}
