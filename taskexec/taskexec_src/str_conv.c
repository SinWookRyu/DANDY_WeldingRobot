////////////////////////////////////////////////////////////////////////////////
// STR_CONV.C is the string convert program. 
// 2013-05-08 mrch0
// 

#include "taskexec_def.h"
#include "cmd.h"

// Returns Coord Name. Not includes the index of USER(Not USR_11, But USR only)
const char* Str_Coord(char i_coord)
{    
    switch(i_coord)
    {
    case COORDINDEX_JOINT:
        // CRT_strcpy(str, size, "JNT");
    	return "JNT";           

    case COORDINDEX_WORLD:
        return "WLD"; 

    case COORDINDEX_BASE:
        return "BAS"; 

    case COORDINDEX_END: 
        return "END"; 

    case COORDINDEX_TCP:
        return "TCP"; 
        
    case COORDINDEX_SENSOR:
        return "SEN"; 

    default:
        if(i_coord < 0)
        {
            return "NOD"; 
        }
        else
        {
            return "USR";             
        }        
    }          
}

// Returns Axis Name String 
const char* Str_Axis(char i_coord, BYTE i_axis)
{
    switch(i_axis)
    {
    case AXIS_LIN_X:
        return (i_coord == COORDINDEX_JOINT)? "JNT_0":"LIN_X";              

    case AXIS_LIN_Y:
        return (i_coord == COORDINDEX_JOINT)? "JNT_1":"LIN_Y";                 
        
    case AXIS_LIN_Z:
        return (i_coord == COORDINDEX_JOINT)? "JNT_2":"LIN_Z";                 
        
    case AXIS_ROT_X:
        return (i_coord == COORDINDEX_JOINT)? "JNT_3":"ROT_X";                 
        
    case AXIS_ROT_Y:
        return (i_coord == COORDINDEX_JOINT)? "JNT_4":"ROT_Y";                 

    case AXIS_ROT_Z:
        return (i_coord == COORDINDEX_JOINT)? "JNT_5":"ROT_Z";                 

    default:
        return "NODEF";        
    }
}

const char* Str_RunMode(int mode_run)
{
    switch(mode_run)
    {
    case RUNMODE_TIMETEST:
        return "TIME"; 

    case RUNMODE_JOG:
    	return "JOG ";

    case RUNMODE_PROG:        
        return "PROG"; 

    case RUNMODE_RESTART:
        return "RES "; 

    case RUNMODE_NONE:
    default:
        return "NONE"; 
    }
}

const char* Str_ProgMode(int mode_prog)
{
    return (mode_prog == PROG_RUNMODE_THRU)? "THRU":
           (mode_prog == PROG_RUNMODE_STEP)? "STEP":
           (mode_prog == PROG_RUNMODE_DRY)?  "DRY ": "----"; 
}

// Returns "ON", "OFF" as 'on' value 1, 0.
const char* Str_OnOff(unsigned on)
{
	return (!on)? "OF" : "ON";    
}

// Not only error message but also includes Normal Event. 
const char* Str_Error(unsigned short code)
{
    ////////// Special Module Errors ////////// 

    // Trajectory Error 
    if((code & 0xff00) == ERR_TRAJ_START)
    {
        return Traj_ErrorStr(code & 0xff); 
    }
    // Trajectory Error 
    if((code & 0xff00) == ERR_WEAV_START)
    {
        return Wv_ErrorStr(code & 0xff); 
    }
    // Inverse Error
    if((code & 0xff00) == ERR_INVERSE_START)
    {
        switch(code & 0xff)
        {
        case ERR_INVALID_PARAM:
            return "Inverse Err. Wrong Params"; 
        case ERR_SINGULAR_NEAR_Z0:
            return "Inverse Err. Singular by TCP Near Z0"; 
        case ERR_SINGULAR_TH5:
            return "Inverse Err. Singular by Th5 0/PI"; 
        case ERR_UNREACHABLE:
            return "Inverse Err. Unreachable"; 
        case ERR_LARGE_VEL:
            return "Inverse Err. Large Diff from Prev. Pos"; 
        default:
            return "Undef"; 
        }
    }

    //////////  TASKEXEC Module Errors ////////// 
    switch(code)
    {    
    case ERR_NONE:         
        return "None"; 
    case ERR_NULL_PTR:
        return "NULL Pointer"; 
    case ERR_NO_JOBSHM:
        return "No Job SHM"; 
    case ERR_CMDINIT_FAIL:
        return "Job Cmd Init Fail"; 
    case ERR_UNSUPPORT_CMD:
        return "Unsupport Job Cmd"; 
    case ERR_INVALID_ARGVAL_TYPE:
        return "Invalid Job Arg Type"; 
    case ERR_UNSUPPORT_VAL:
        return "Unsupport Job Value"; 
    case ERR_NO_JNT_ACCESS:
        return "No Joint Mode Access"; 
    case ERR_NO_CART_ACCESS:
        return "No Cart Mode Access"; 
    case ERR_UNSUPPORT_DBL_OPER:
        return "Unsupport Dbl Oper"; 
    case ERR_UNSUPPORT_OPER_CMD:
        return "Unsupport Oper Cmd";
    case ERR_UNSUPPORT_BRANCH_CMD:
        return "Unsupport Branch Cmd"; 
    case ERR_UNDEF_STEP: 
        return "Undefined Step Tried"; 
    case ERR_STOP_REQ:
        return "Stop Requested"; 
    case ERR_QUICK_STOP_REQ:
        return "Quick Stop Requested"; 
    case ERR_ESTOP_ON: 
        return "Estop";
    case ERR_MOTOR_ERR:
        return "Driver-Err";
    case ERR_SERVO_OFF:
        return "Servo-off"; 
    case ERR_NO_RESTART_DATA:
        return "No Restart Data"; 
    case ERR_NO_LMOT_DATA_SAVED:
        return "No Lmot Data Saved(Inv. Err Possible)"; 
    case ERR_NO_WEAVE_DATA_SAVED:
        return "No Weave Data Saved";    
    case ERR_TRAJ_ERR_NO_RESTART:        
        return "Traj Error(including Inverse Error, Lim Stop) cannot be restarted."; 
    case ERR_INVALID_AXIS:
        return "Invalid Axis"; 
    case ERR_INVALID_COORD:
        return "Invalid Coord"; 
    case ERR_INVALID_DIR:
        return "Invalid Dir"; 
    case ERR_WATCHDOG_OVER:
        return "Jog Watch-dog Fired";      
    case ERR_TIME_OVER:
        return "Time-over"; 
    case ERR_STACK_NOMORE_PUSH:
        return "Stack No More Push"; 
    case ERR_STACK_NOMORE_POP:
        return "Stack No More Pop"; 
    case ERR_JOBLOAD_FAIL:
        return "Job Load Fail"; 
    case ERR_LESS_ARG_COUNT:
        return "Less Argument Count"; 
    case ERR_TCH_NOT_READY:
        return "Touch Sensor Ready Fail"; 
    case ERR_TOUCH_SENSOR_RESET_FAIL:
        return "Touch Sensor Reset Fail"; 
    case ERR_TOUCH_FAIL:
        return "Touch Failed"; 
    case ERR_TOUCH_ALREADY:
        return "Touch Already"; 
    case ERR_WELDER_POWER_FAIL:
        return "Welder Power Fail"; 
    case ERR_ARCON_SIG_FAIL:
        return "Arc-on Sig Fail"; 
    case ERR_GASON_SIG_FAIL:
        return "Gas-on Sig Fail"; 
    case ERR_CWEAV_SHORT_HORZ:
        return "Too Short Horz. Dist. of Cweave"; 
    case ERR_ARG_ELEM_MISMATCH:
        return "Job Prog Arg Element Mismatch"; 
    case ERR_DIV_BY_0:
        return "Divided by 0"; 
    default:        
        return "Undefined Event"; 
    }
}

// Returns Section Name of Each Sect. 
const char* Str_Sect(unsigned short sect)
{
    switch(sect)
    {
    case SECT_RUN:
        return "RUN"; 
            
    case SECT_JOG:
        return "JOG";
        
    case SECT_PROG:
        return "PRG"; 

    case SECT_CMD_MOV:
        return "MOV"; 

    case SECT_CMD_TOUCH:
        return "TCH"; 
                
    case SECT_CMD_WELD:
        return "WLD"; 

    case SECT_CMD_PORT:
        return "POT";

    case SECT_CMD_BRANCH:
        return "BRN"; 
        
    case SECT_CMD_CWEAV:
        return "CWV";     

    case SECT_RESTART:
        return "RES"; 

    case SECT_NONE:
    default:
        return "NON";
    }
}

char* Str_WeldIoMap(void)
{   
    static char str[80*10] = ""; 
    int n = 0; 

    n = CRT_sprintf(str+n,   sizeof(str)-n, 
        "  [DOUT]       <ARC>   <GAS>   <M/C>   <TCH-RUN>   <WIR-F> <WIR-B>\n"    
        "      PORT i     %-7d %-7d %-7d   %-10d %-7d %-7d \n"
        "      ON LEV     %-7d %-7d %-7d   %-10d %-7d %-7d \n",        
        g_idx_do_arc,   g_idx_do_gas,   g_idx_do_tch_mc,   g_idx_do_tch_proc,   g_idx_do_wirefeed,   g_idx_do_wireback, 
        g_onlev_do_arc, g_onlev_do_gas, g_onlev_do_tch_mc, g_onlev_do_tch_proc, g_onlev_do_wirefeed, g_onlev_do_wireback); 

    n += CRT_sprintf(str+n, sizeof(str)-n,
        "  [D-IN]       <ARC>   <TCH-RUN> <TCH-SIG>   <PWR-FAIL> <NO-GAS>\n"
        "      PORT i     %-7d  %-10d  %-10d %-7d   %-7d\n"
        "      ON LEV     %-7d  %-10d  %-10d %-7d   %-7d\n", 
        g_idx_di_arc,   g_idx_di_tch_proc,   g_idx_di_tch_sig,   g_idx_di_pwr_fail,   g_idx_di_no_gas, 
        g_onlev_di_arc, g_onlev_di_tch_proc, g_onlev_di_tch_sig, g_onlev_di_pwr_fail, g_onlev_di_no_gas);         

    n += CRT_sprintf(str+n, sizeof(str)-n,
        "  [AOUT]       <VOLT>  <CURR>  <WIRE>\n"
        "      PORT i     %-7d %-7d %-7d\n", 
        g_idx_ao_volt, g_idx_ao_curr, g_idx_ao_wire); 
    return str; 
}

void Str_PrintArg(void)
{   
    int i; 
    printf("TASKEXEC(ver:%s build:%s) ", TE_VERSION, TE_BUILD); 
    for(i=1 ; i<g_argc ; i++)
    {
        printf("%s ", g_argv[i]); 
    }    
    printf("\n"); 
}

char* Str_WeaveSeg(unsigned n_seg)
{
    return (n_seg == WV_SEG_START)? "START" : 
           (n_seg == WV_SEG_MAIN)?  "MAJOR" : 
           (n_seg == WV_SEG_FINAL)? "FINAL" : "UNDEF"; 
}

char* Str_ProgCtrlVar(void)
{   
    static char str[80*10] = ""; 
    int n = 0; 

    n = CRT_sprintf(str+n,   sizeof(str)-n,             
        "  [Prog Skip Left ] Usage:%s, Var:B_%03d\n"
        "  [Prog Skip Right] Usage:%s, Var:B_%03d\n"
        "  [Gap Cond Modify] Usage:%s, Var:B_%03d\n", 
        Str_OnOff(g_pshm_rm_sys && g_pshm_rm_sys->fLeftSkip),  g_i_bvar_left, 
        Str_OnOff(g_pshm_rm_sys && g_pshm_rm_sys->fRightSkip), g_i_bvar_right, 
        Str_OnOff(g_pshm_sys_conf && g_pshm_sys_conf->robot->weld_func.fGapRefVarUsed), g_i_bvar_gap_sel); 
    return str; 
}

char* Str_CweaveMode(int mode)
{
    switch(mode)
    {
    case CWEAVE_MODE_VERT: 
        return "VERT";         
    case CWEAVE_MODE_DWELL_V: 
        return "DWL_V"; 
    case CWEAVE_MODE_ROUND: 
        return "ROUND"; 
    case CWEAVE_MODE_DWELL_H: 
        return "DWL_H"; 
    case CWEAVE_MODE_HORZ:
        return "HORZ"; 
    case CWEAVE_MODE_NONE: 
    default:
        return "NONE"; 
    }
}