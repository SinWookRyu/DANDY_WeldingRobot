#include "taskexec_def.h"
#include "utility.h"
#include "arg.h"

#define ARCON_STEP_NONE         0
#define ARCON_STEP_PREPURGE     1
#define ARCON_STEP_ARCON        2

#define ARCOFF_STEP_NONE        0
#define ARCOFF_STEP_CRATOR      1
#define ARCOFF_STEP_POSTPURGE   2
#define ARCOFF_STEP_ARCOFF      3

static INT s_stop; 
static INT s_step; 
static DBL s_time;
static DBL s_time_wire; 
static DBL s_v_wire; 
static INT s_f_sensor;
static INT s_cmd_code; 
static INT s_f_touch;

static void ArcOnFailExit(int err); 
static void WireFailExit(int err);

////////////////////////////////////////////////////////////////////////////////

void CmdStp_WELD(int f_fast)
{
    s_stop = ON; 
}

////////////////////////////////////////////////////////////////////////////////

int CmdSet_ARCON(int index, DBL* jnt)
{  
    DANDY_JOB_ARG_ARCON* arg;     
    int ret; 
    
    // welder on check     
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        return ERR_WELDER_POWER_FAIL; 
    }
       
    arg = &g_rcmd[index].arg.argArcOn; 
    
    ret = SwfFromArg(&g_weld_start, &arg->valWeldStart, g_n_shift); 
    if(ret)
    {
        return ret; 
    }    

    ret = MwfFromArg(&g_weld_major, &arg->valWeldMain, g_n_shift); 
    if(ret)
    {
        return ret; 
    }    

    // Welding Condition during Start Weaving Period. 
    // If not defined Main Welding Condition is used. 
    ret = MwfFromArg(&g_weld_weave, &arg->valWeldWeave, g_n_shift); 
    if(ret)
    {
        g_weld_weave = g_weld_major; 
    }  

    // Major Welding Cond when weaving edist exists.
    ret = MwfFromArg(&g_weld_major_edist, &arg->valWeldMain, g_n_shift+WELDCOND_SHIFT_WV_EDIST); 
    if(ret)
    {
        g_weld_major_edist = g_weld_major; 
    }

#if 0   // Definition of EDIST Welding Condition is corrected. 
    // Welding Condition during Start Weaving Period EDIST Exist. 
    // If not defined Main Welding Condition is used. 
    ret = MwfFromArg(&g_weld_weave_edist, &arg->valWeldWeave, g_n_shift + WELDCOND_SHIFT_WV_EDIST); 
    if(ret)
    {
        g_weld_weave_edist = g_weld_weave;
    }
#endif 

    // Arc Sensor On/Off Setting. 
    // This setting is Off at ARC-OFF. 
    ret = Arg_Scalar(&s_f_sensor, NULL, &arg->valSensorUsed); 
    if(ret)
    {
        return ret; 
    }
    
    WELD_SET_TCH_MC(OFF); 
    WELD_SET_GAS(ON); 

    s_step = ARCON_STEP_PREPURGE; 
    s_stop = 0; 
    s_time = 0.; 

    return 0; 
}

int CmdUpd_ARCON(void)
{
    if(s_step == ARCON_STEP_NONE)
    {
        return -1;
    }

    // 
    s_time += g_traj.t_samp; 

    // welder power fail 
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        ArcOnFailExit(ERR_WELDER_POWER_FAIL); 
        return -1; 
    }

    if(s_stop)
    {
        ArcOnFailExit(0);
        return -1;
    }

    switch(s_step)
    {
    case ARCON_STEP_PREPURGE:
        if(s_time < g_weld_start.dbPreflowTime)
        {
            return 0;
        }

        // End of Waiting   ////////////////////////////////////////////////////

        // gas on fail & exit
        if(WELD_GET_NO_GAS == ON)
        {
            ArcOnFailExit(ERR_GASON_SIG_FAIL); 
            return -1;
        }

        // start welding Condition
        WELD_SET_VOLT(g_weld_start.dbVoltage + g_weld_major.dbVoltage); 
        WELD_SET_CURR(g_weld_start.dbCurrent + g_weld_major.dbCurrent); 
        WELD_SET_ARC(ON); 

        s_time = 0.; 
        s_step = ARCON_STEP_ARCON; 
        return 0; 

    case ARCON_STEP_ARCON:
        if(s_time < g_weld_start.dbArcTime)
        {
            return 0;
        }

        // END OF WAITING   ////////////////////////////////////////////////////

        // Arc on sig fail Exit 
        if(WELD_GET_ARC == OFF)
        {
            ArcOnFailExit(ERR_ARCON_SIG_FAIL); 
            return -1; 
        }

        // END OF CMD   ////////////////////////////////////////////////////////
        // Sets Main Conditions, Weld Flag, Weld Speed

        WELD_SET_VOLT(g_weld_major.dbVoltage); 
        WELD_SET_CURR(g_weld_major.dbCurrent); 
        g_f_weld = ON;  // weld mode ON
        g_weld_spd = g_weld_major.dbSpeed; 
       
        // arc-sensor data init
        weld_idx.arc_sensor = (byte)s_f_sensor;     
        weld_idx.I_main = weld_idx.I_main;  // no effective param
        weld_idx.V_main = weld_idx.V_main;  // no effective param

        return -1; 

    case ARCON_STEP_NONE:
    default:
        return -1; 
    }
}

////////////////////////////////////////////////////////////////////////////////

int CmdSet_ARCOFF(int index, DBL* jnt)
{  
    DANDY_JOB_ARG_ARCOFF* arg; 
    int ret; 
       
    arg = &g_rcmd[index].arg.argArcOff; 
    ret = EwfFromArg(&g_weld_final, &arg->valWeldEnd, g_n_shift); 
    if(ret)
    {
        return ret; 
    }
    
    WELD_SET_VOLT(g_weld_major.dbVoltage + g_weld_final.dbVoltage); 
    WELD_SET_CURR(g_weld_major.dbCurrent + g_weld_final.dbCurrent); 
    weld_idx.arc_sensor = OFF; 

    s_step = ARCOFF_STEP_CRATOR; 
    s_time = 0.; 
    s_stop = 0; 

    return 0; 
}

int CmdUpd_ARCOFF(void)
{
    if(s_step == ARCOFF_STEP_NONE)
    {
        return -1;
    }

    // 
    s_time += g_traj.t_samp; 

    if(s_stop)
    {
        ArcOnFailExit(0);
        return -1;
    }

    switch(s_step)
    {
    case ARCOFF_STEP_CRATOR:
        if(s_time < g_weld_final.dbCraterTime)
        {
            return 0;
        }

        // End of Waiting   ////////////////////////////////////////////////////
        WELD_SET_ARC(OFF); 
        s_time = 0.; 
        s_step = ARCOFF_STEP_POSTPURGE; 
        return 0; 

    case ARCOFF_STEP_POSTPURGE:
        if(s_time < g_weld_final.dbPostflowTime)
        {
            return 0;
        }

        // END OF WAITING   ////////////////////////////////////////////////////
        WELD_SET_GAS(OFF); 
        s_time = 0.; 
        s_step = ARCOFF_STEP_ARCOFF; 
        return 0; 
    
    case ARCOFF_STEP_ARCOFF:
        if(s_time < WAITTIME_IO_CHANGE)
        {
            return 0; 
        }

        // END OF WAITING  /////////////////////////////////////////////////////        
        WELD_SET_TCH_MC(ON); 
        WELD_SET_GAS(OFF); 
        WELD_SET_ARC(OFF); 
        WELD_SET_VOLT(0.); 
        WELD_SET_CURR(0.); 

        g_f_weld = OFF;  // Weld mode off
        s_step = ARCOFF_STEP_NONE; 
        return -1; 

    case ARCOFF_STEP_NONE:
    default:
        return -1; 
    }
}

////////////////////////////////////////////////////////////////////////////////

// change of main welding condition 
int CmdSet_ARCSET(int index, DBL* jnt)
{   
    DANDY_JOB_ARG_ARCON* arg; 
    int ret; 
     
    // If no welding status, No working this cmd. 
    if(!g_f_weld)
    {
        return 0; 
    }

    arg = &g_rcmd[index].arg.argArcOn; 
    
    ret = MwfFromArg(&g_weld_major, &arg->valWeldMain, g_n_shift); 
    if(ret)
    {
        return ret; 
    }

    // Major Welding Cond when weaving edist exists.
    ret = MwfFromArg(&g_weld_major_edist, &arg->valWeldMain, g_n_shift+WELDCOND_SHIFT_WV_EDIST); 
    if(ret)
    {
        g_weld_major_edist = g_weld_major; 
    }

    // Arc Sensor On/Off Setting. 
    // This setting is Off at ARC-OFF. 
    ret = Arg_Scalar((int*)&weld_idx.arc_sensor, NULL, &arg->valSensorUsed); 
    if(ret)
    {
        return ret; 
    }

    // reset main welding Condition
    WELD_SET_VOLT(g_weld_major.dbVoltage); 
    WELD_SET_CURR(g_weld_major.dbCurrent); 
    g_weld_spd = g_weld_major.dbSpeed; 

    return 0; 
}

////////////////////////////////////////////////////////////////////////////////
#define WIRE_STEP_NONE       (0)
#define WIRE_STEP_WAIT       (1)
#define WIRE_STEP_WIRE       (2)

int CmdSet_WIRE(int index, DBL* jnt)
{  
    DANDY_JOB_ARG_WIRE* arg; 
    int ret;         
    
    // welder power fail 
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        WireFailExit(ERR_WELDER_POWER_FAIL); 
        return -1; 
    }

    // welder on check     
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        return ERR_WELDER_POWER_FAIL; 
    }
       
    arg = &g_rcmd[index].arg.argWire; 

    ret = PosVelFromArg(&s_v_wire, &arg->valSpeed); 
    ASSERT_RETURN(!ret, ret); 

    ret = TimeFromArg(&s_time_wire, &arg->valTimeout); 
    ASSERT_RETURN(!ret, ret); 

    ret = Arg_Scalar(&s_f_touch, NULL, &arg->valTouchUsed); 
    ASSERT_RETURN(!ret, ret); 

    s_cmd_code = g_rcmd[index].nCode; 

    // as touch param, wire out 1st or touch prepare 1st
    
    if(!s_f_touch)
    {
        WELD_SET_WIRE(s_v_wire); 
        (s_cmd_code == DANDY_JOB_CODE_WIREB)? WELD_SET_WIREBACK(ON): 
                                              WELD_SET_WIREFEED(ON); 
        s_step = WIRE_STEP_WIRE; 
        s_time = s_time_wire; 
    }
    else
    {
        WELD_SET_TCH_MC(ON);
        WELD_SET_TCH_PROC(ON);
        s_step = WIRE_STEP_WAIT; 
        s_time = WAITTIME_IO_CHANGE; 
    }
    s_stop = 0; 
    return 0; 
}

int CmdUpd_WIRE(void)
{  
    s_time -= g_traj.t_samp; 

    // welder power fail 
    // All during ARCON, WELDER POWER must be ON. 
    if(WELD_GET_PWR_FAIL == ON)
    {
        WireFailExit(ERR_WELDER_POWER_FAIL); 
        return -1; 
    }

    // stop check 
    if(s_stop)
    {
        WireFailExit(0);
        return -1; 
    }

    switch(s_step)
    {
    case WIRE_STEP_WAIT: 
        if(s_time <= 0.)
        {  
            // Check Wire Touch Already
            if(WELD_GET_TCH_SIG == ON)
            {
                WireFailExit(ERR_TOUCH_ALREADY);
                return -1; 
            }

            WELD_SET_WIRE(s_v_wire); 
            (s_cmd_code == DANDY_JOB_CODE_WIREB)? WELD_SET_WIREBACK(ON): 
                                                  WELD_SET_WIREFEED(ON); 
            s_step = WIRE_STEP_WIRE; 
            s_time = s_time_wire; 
        }
        return 0; 

    case WIRE_STEP_WIRE:

        // In Touch Case, Touch Ready Check 
        if(s_f_touch && WELD_GET_TCH_PROC == OFF)
        {   
            WireFailExit(ERR_TCH_NOT_READY);           
            return -1; 
        }

        // In Touch Case, Touch Check. 
        if(s_f_touch && WELD_GET_TCH_SIG == ON)
        {
#if 0
            R_VERB_MSG("Wire Feeding Touched.\n"
                        "  s_f_touch : %d \n"
                        "  g_di[g_idx_di_tch_sig] : %d \n"
                        "  g_onlev_di_tch_sig : %d\n"
                        "  g_idx_di_tch_sig : %d\n"
                        "  WELD_GET_TCH_SIG : %d \n", 
                        s_f_touch, g_di[g_idx_di_tch_sig], g_onlev_di_tch_sig, 
                        g_idx_di_tch_sig, WELD_GET_TCH_SIG); 
#endif 
            R_VERB_MSG("Wire Feeding Touched.\n");                         
            WireFailExit(0); 
            return -1; 
        }

        // Time out check. In wire, No Touch is not error. 
        if(s_time <= 0.)
        {   
            int err = (s_f_touch == ON && WELD_GET_TCH_SIG == OFF)? 
                      ERR_TOUCH_FAIL:0;             
            WireFailExit(err);             
            s_step = WIRE_STEP_NONE; 
            return -1; 
        }
        return 0; 
        
    case WIRE_STEP_NONE: 
    default: 
        return -1; 
    }
}

////////////////////////////////////////////////////////////////////////////////

static void ArcOnFailExit(int err)
{   
    ERROR_SET(SECT_CMD_WELD, err); 

    WELD_SET_TCH_MC(ON); 
    WELD_SET_GAS(OFF); 
    WELD_SET_ARC(OFF); 

    WELD_SET_VOLT(0.); 
    WELD_SET_CURR(0.); 

    WELD_SET_WIREBACK(OFF); 
    WELD_SET_WIREFEED(OFF); 

    s_step = ARCON_STEP_NONE; 
}

static void WireFailExit(int err)
{
    WELD_SET_WIRE(0.); 
    WELD_SET_WIREBACK(OFF); 
    WELD_SET_WIREFEED(OFF); 
    if(s_f_touch)
    {
        WELD_SET_TCH_PROC(OFF); 
    }
    ERROR_SET(SECT_CMD_WELD, err); 
}
