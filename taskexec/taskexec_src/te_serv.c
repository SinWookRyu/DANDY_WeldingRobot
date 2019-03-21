////////////////////////////////////////////////////////////////////////////
// TE_SERV.C is for the service handlers of TE

int IpcInit(void); 
void IpcUninit(void); 

#include "taskexec_def.h"
#include "te_serv.h"
#include "runtime.h"
#include "string.h"
#include "filemgr.h"
#include "coord.h"
#include "trajectory.h"
#include "weave.h"

extern void RestartDataSet(UNT f_set, UNT mode_prog);  

static void TeServPrint(void); 

// Jog Speed Service Data[%] Converter from 'char' to 'double'[-1.0~1.0]
// -100 <= n_spd <= 100 : -1.0 <= spd <= 1.0
//  100  < n_spd        : spd = (n_spd - 100) * 0.1 * 0.01
//         n_spd < -100 : spd = (n_spd + 100) * 0.1 * 0.01
static double GetJogVal_Spd(char n_spd); 
static POS GetJogVal_DirLin(VALUE_JOG val); 
static POS GetJogVal_DirRot(VALUE_JOG val); 

static struct te_serv
{    
    int code; 
    int (*Handler)(void* buff, int* p_size, const TE_MSG* p_msg); 
    const char* name; 
    const char* help;
} s_serv[] = 
{
    // Definition,      Handler,            Description,  Specification
    {TESERV_EXIT,       TeServ_Exit,        "Exit      ", "-"                 }, 
    {TESERV_VERSION,    TeServ_Vers,        "Version   ", "-"                 },     
    {TESERV_INIT,       TeServ_Init,        "IPC Init  ", "-"                 }, 
    {TESERV_LIFE_CHK,   TeServ_LifeCheck,   "Life Check", "0:Off Els:On"      },
    {TESERV_STOP,       TeServ_Stop,        "Stop      ", "0:Norm Els:Qucik"  }, 
    {TESERV_JOG,        TeServ_Jog,         "Jog Run   ", "VALUE_JOG"         }, 
    {TESERV_PROG_THRU,  TeServ_ProgThru,    "Prog Thru ", "Start Index"       }, 
    {TESERV_PROG_STEP,  TeServ_ProgStep,    "Prog Step ", "Start Index"       }, 
    {TESERV_PROG_DRY,   TeServ_ProgDry,     "Prog Dry  ", "Start Index"       }, 
    {TESERV_RESTART,    TeServgRestart,     "Restart   ", "-"                 }, 
    {TESERV_DISP_TOGGLE,TeServ_DispToggle,  "Disp Toggl", "0:Help Els:Object" }, 
    {TESERV_SAMPTIME,   TeServ_SampTime,    "Samp Time ", "Sampling Time[ms]" }, 
    {TESERV_FILE,       TeServ_File,        "File Write", "0:Off Els:On"      },   
    {TESERV_TIMETEST,   TeServ_TimeTest,    "Timer Test", "-"                 }, 
    {TESERV_RESET_ERR,  TeServ_ResetErr,    "Reset Err ", "-"                 }, 
#if 0
    {TESERV_TRAJ_INFO,  TeServ_TrajInfo,    "Traj Info ", "-"                 }, 
#endif 
    {TESERV_INFO_PRINT, TeServ_InfoPrint,   "Print Info", "0:help 1:T 2:W 3:R"}, 
    {TESERV_JOG_MODE,   TeServ_JogMode,     "Jog Mode  ", "1:Weav Els:Normal" },     
    {TESERV_JNT_2_CART, TeServ_Jnt2Cart,    "Jnt 2 Cart", "Val:CRD Data:Joint"}, 
    {TESERV_DISP_COORD, TeServ_DispCoord,   "Disp Coord", "CRD_IDX(W:-2,T:-5)"}, 
    {TESERV_TIMER_SKIP, TeServ_TimerSkip,   "Timer Skip", "-"                 },
    {TESERV_PRG_VAR_CLR,TeServ_PrgVarClr,   "PrgVar Clr", "01:Stack 02:Restrt"},
    {TESERV_WELD_IO_MAP,TeServ_WeldIoMap,   "WeldIO Map", "-"                 },  
    {TESERV_PRG_CTR_VAR,TeServ_ProgCtrlVar, "Prog Vars ", "Program Ctrl Vars" },
    {TESERV_RUN_PAUSE,  TeServ_RuntimePause,"Run Pause ", "0:Run Els:Pause"   },
#if 0
    {TESERV_RESTART_PRT,TeServ_RestartPrint,"Res Print ", "-"                 },
#endif 
}; 

static int s_n_serv = 0;     // Count of Te Service

////////////////////////////////////////////////////////////////////////////////
// Implemetations 

// Checks mapping of Te Service Registry & TE_SERV_CODE. 
// Strongly recommened to check before calling each TE Services. 
// Suc(-1), Fail(TE_SERV_CODE)
int TeServ_CheckMapping(void)
{
    int i; 
    div_t temp;
    
	temp = div(sizeof(s_serv), sizeof(s_serv[0]));    
    s_n_serv = temp.quot; 

    for(i=0 ; i<s_n_serv ; i++)
    {
        if(s_serv[i].code != i)
        {        	
            return i; 
        }        
    }
    return -1; 
}

// Returns Service Name. 
const char* TeServ_GetName(TESERV_CODE code)
{    
    ASSERT_RETURN(0 <= code && code < s_n_serv, "Undefined "); 
    return s_serv[code].name; 
}

// Service Handler of TE 
// 1) Sets data in reply packet 
// 2) Handles Service
// 3) Returns the result of handling. Suc(0), Fail(Else)
// 
// TE Service Hanlder 
// Basic Form : TeServ_xxx(void* buff, int* size, const TE_MSG* p_msg); 
// - buff : Output body data to reply. 
// - size : buff(body) size. Reply Size = size + Header Size. 
// - p_msg: Message data requested. 
// 
// Returns Reply Result(Normally 0) or Unsupported(-1)
int TeServ_Handler(TESERV_CODE code, void* buff, int* p_size, const TE_MSG* p_msg)
{    
    *p_size = 0; 
    ASSERT_RETURN(0 <= code && code < s_n_serv, -1); 
    return s_serv[code].Handler(buff, p_size, p_msg); 
}

////////////////////////////////////////////////////////////////////////////////
// Handlers 

// Service Handler of TE ///////////////////////////////////////////////////////
// 1) Sets data in reply packet 
// 2) Handles Service
// 3) Returns the result of handling. Suc(0), Fail(Else)

// TE Service Hanlder 
// Basic Form : TeServ_xxx(void* buff, int* size, const TE_MSG* p_msg); 
// - buff : Output body data to reply. 
// - size : buff(body) size. Reply Size = size + Header Size. 
// - p_msg: Message data requested. 

int TeServ_Vers(void* buff, int* p_size, const TE_MSG* p_msg)
{	
    TE_REPLYDATA_VERS* p_ver; 
    
    // Reply Data Setting
    if(p_size)
    {
        *p_size = sizeof(TE_REPLYDATA_VERS);
    }
    if(buff)
    {
        p_ver = buff;
        CRT_strcpy(p_ver->build, sizeof(p_ver->build), TE_BUILD);
        CRT_strcpy(p_ver->vers, sizeof(p_ver->vers), TE_VERSION); 
    }   

    // work
    Str_PrintArg(); 
    TeServPrint(); 
    VERBOSE_VERBOSE("Service Excutes. Code:VERSION(%d).\n", TESERV_VERSION);	
	return 0; 
}

int TeServ_Exit(void* buff, int* p_size, const TE_MSG* p_msg)
{		
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
	VERBOSE_VERBOSE("Service Excutes. Code:EXIT(%d).\n", TESERV_EXIT);
    return 0; 
}

int TeServ_Init(void* buff, int* p_size, const TE_MSG* p_msg)
{	
    int ret; 
	VERBOSE_VERBOSE("Service Excutes. Code:INIT(%d).\n", TESERV_INIT);		

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
     
    // First Stop Init Wait Exit Timer
    if(g_htmr_exit != -1)
    {
        TIME_PauseTimerPulse(g_htmr_exit);
        VERBOSE_VERBOSE("Pauses waiting timer for RM Init MSG.\n"); 
    }

    // First Stop Init Wait Exit Timer
    if(g_htmr_run != -1)
    {
        TIME_PauseTimerPulse(g_htmr_run);
        VERBOSE_VERBOSE("Pauses waiting timer for RUNTIME.\n"); 
    }

    // runtime thread pause request
     g_f_runtime_pause = ON;     
    // Waits stopping of Runtime Thread that uses Global & Shm Resosurces. 
    THREAD_Sleep(WAITTIME_SLICE); 
    
    // Runtime Erorr & Some Var's Reset & Init IPC
    memset(&g_error, 0, sizeof(g_error)); 
    g_time_spend_max = 0; 
    g_n_spend_over = 0; 
    g_time_err_max = 0; 

    ret = Init_IpcInit(); 

    // Waits stopping of Runtime Thread that uses Global & Shm Resosurces. 
    THREAD_Sleep(WAITTIME_SLICE); 
    // runtime thread pause reset
    g_f_runtime_pause = OFF; 

    return ret; 
}

int TeServ_TimeTest(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_TIMETEST param; 

    VERBOSE_VERBOSE("Service Excutes. Code:TIMETEST(%d).\n", TESERV_TIMETEST);			

    if(p_size)
    {
        *p_size = 0; 
    }

    return Run_ReqMode(RUNMODE_TIMETEST, &param); 
}
    
int TeServ_Stop(void* buff, int* p_size, const TE_MSG* p_msg)
{       
    VERBOSE_VERBOSE("Service Excutes. Code:STOP(%d).Quick:%s\n", TESERV_STOP, Str_OnOff(p_msg->value));			

    if(p_size)
    {
        *p_size = 0; 
    }

    Run_Stop(p_msg->value);     // p_msg->value : quick stop flag
	return 0; 
}

int TeServ_Jog(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_JOG jog; 
    VALUE_JOG* p_jog_val;    
    double spd; 
        
    if(p_size)
    {
        *p_size = 0; 
    }

    // Jog Value Analysis
    p_jog_val = (VALUE_JOG*)&p_msg->value;
    spd = GetJogVal_Spd(p_jog_val->nSpeed); 

    // Jog is running, Resets watch-dog only. 
    if(Run_IsJogRunning(p_jog_val->axis))
    {
        // Resets watch-dog. 
#if 1
        Jog_WatchDogReset(); 	    
#else
        (g_mode_jog == 1)? JogWeave_WatchDogReset() : Jog_WatchDogReset(); 	    
#endif
        return 0; 
    }

    VERBOSE_MESSAGE("JOG Service(%d) Requested.\n"
                    "\tRob:%d Coord:%s(%d) Axis:%s KeepTime:%dms Speed:%.1f%%\n", 
        TESERV_JOG, p_jog_val->nRobNum, Str_Coord(p_jog_val->coord), p_jog_val->coord, 
        Str_Axis(p_jog_val->coord, p_jog_val->axis), (1<<p_jog_val->t_keep), spd*100);			
  
    // Setting Runtime Jog Param 
    jog.coord = p_jog_val->coord;    
    jog.axis  = p_jog_val->axis;
    jog.dir   = (spd<0.)? -1.0 : 1.0;
    jog.vel   = fabs(spd*g_vel_axis_max[p_jog_val->axis]);
    jog.keep  = (1<<p_jog_val->t_keep);

    // for cartesian jog 
    jog.s_lin = GetJogVal_DirLin(*p_jog_val); 
    jog.s_rot = GetJogVal_DirRot(*p_jog_val); 
    jog.v_lin = fabs(spd*g_vel_lin_max); 
    jog.v_rot = fabs(spd*g_vel_ori_max); 
        
    return Run_ReqMode(RUNMODE_JOG, &jog); 
}

// Program Run Thru
int TeServ_ProgThru(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_PROG prog; 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // Work   
    VERBOSE_MESSAGE("Program Run Thru Mode Starts(%d). Start Index:%d\n", TESERV_PROG_THRU, p_msg->value);             
    prog.i_start  = p_msg->value; 
    prog.mode_run = PROG_RUNMODE_THRU;     
    return Run_ReqMode(RUNMODE_PROG, &prog); 
}

// Program Run Step
int TeServ_ProgStep(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_PROG prog; 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // Work   
    VERBOSE_MESSAGE("Program Run Step Mode Starts(%d). Index:%d\n", TESERV_PROG_STEP, p_msg->value);             
    prog.i_start  = p_msg->value; 
    prog.mode_run = PROG_RUNMODE_STEP;     
    return Run_ReqMode(RUNMODE_PROG, &prog); 
}

// Dry Prog (Thru)
int TeServ_ProgDry(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_PROG prog; 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // Work   
    VERBOSE_MESSAGE("Dry Run(Thru) Mode Starts(%d). Start Index:%d\n", TESERV_PROG_DRY, p_msg->value);             
    prog.i_start  = p_msg->value; 
    prog.mode_run = PROG_RUNMODE_DRY;     
    return Run_ReqMode(RUNMODE_PROG, &prog); 
}

// 
int TeServgRestart(void* buff, int* p_size, const TE_MSG* p_msg)
{
    RUNPARAM_RESTART restart; 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // Work   
    if(g_restart.f_restart == OFF)
    {
        VERBOSE_WARNING("%s\n", Str_Error(ERR_NO_RESTART_DATA));             
        return 0; 
    }
    else
    {
        VERBOSE_MESSAGE("Restart Mode Starts(%d). Prog Mode:%s Prog Index:%d\n", 
           TESERV_RESTART, Str_ProgMode(g_restart.mode_prog), g_restart.i_prog);            

        restart.temp = 0; 
        return Run_ReqMode(RUNMODE_RESTART, &restart); 
    }
}

int TeServ_LifeCheck(void* buff, int* p_size, const TE_MSG* p_msg)
{   
    if(p_size)
    {
        *p_size = 0; 
    }
    return 0; 
}

#if 0
int TeServ_EcatPdoCtrl(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    VERBOSE_VERBOSE("Unsupported\n"); 
    return 0; 
}
#endif

#if 0
int TeServ_Vga(void* buff, int* p_size, const TE_MSG* p_msg)
#else
int TeServ_DispToggle(void* buff, int* p_size, const TE_MSG* p_msg)
#endif
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

#if 0
    // Service Action 
    g_f_vga_on = p_msg->value; 

    VERBOSE_VERBOSE("Service Excutes. Code:TESERV_VGA(%d). Val:%s\n", 
        TESERV_VGA, Str_OnOff(p_msg->value));
#endif
    // toggling bit of print option
    g_print_off = g_print_off ^ p_msg->value; 

#if 0
    // message ordering    
    g_print_off |= (g_print_off & BIT_RUNERR_OFF)? (BIT_RUNWRN_OFF | BIT_RUNMSG_OFF | BIT_RUNVRB_OFF): 
                   (g_print_off & BIT_RUNWRN_OFF)? (BIT_RUNMSG_OFF | BIT_RUNVRB_OFF): 
                   (g_print_off & BIT_RUNMSG_OFF)? (BIT_RUNVRB_OFF) : 0;   
#endif
    if(p_msg->value == BIT_RUNERR_OFF)
    {
    }

    // 0, help print
    if(!p_msg->value)
    {
        printf(
            "<Display Toggle Service> (Usage : tep %d [value])\n"
            "[value] can be '+' operation.\n"            
            "  %2d : VGA             (%s)\n"             
            "  %2d : Time Use Check  (%s)\n"
            "  %2d : Runtime Verbose (%s)\n"        
            "  %2d : Runtime Message (%s)\n"             
            "  %2d : Runtime Warning (%s)\n"             
            "  %2d : Runtime Error   (%s)\n",            
            TESERV_DISP_TOGGLE, BIT_VGA_OFF, Str_OnOff(g_print_off & BIT_VGA_OFF), 
            BIT_TIMECHK_OFF, Str_OnOff(g_print_off & BIT_TIMECHK_OFF), 
            BIT_RUNVRB_OFF,  Str_OnOff(g_print_off & BIT_RUNVRB_OFF), 
            BIT_RUNMSG_OFF,  Str_OnOff(g_print_off & BIT_RUNMSG_OFF), 
            BIT_RUNWRN_OFF,  Str_OnOff(g_print_off & BIT_RUNWRN_OFF), 
            BIT_RUNERR_OFF,  Str_OnOff(g_print_off & BIT_RUNERR_OFF));
    }
    else
    {
        printf(
            "<Display Off Status>\n"                  
            "  VGA             : %s \n"             
            "  Time Use Check  : %s \n" 
            "  Runtime Verbose : %s \n"             
            "  Runtime Message : %s \n"             
            "  Runtime Warning : %s \n"             
            "  Runtime Error   : %s \n",                       
            Str_OnOff(g_print_off & BIT_VGA_OFF), 
            Str_OnOff(g_print_off & BIT_TIMECHK_OFF), 
            Str_OnOff(g_print_off & BIT_RUNVRB_OFF), 
            Str_OnOff(g_print_off & BIT_RUNMSG_OFF), 
            Str_OnOff(g_print_off & BIT_RUNWRN_OFF), 
            Str_OnOff(g_print_off & BIT_RUNERR_OFF)); 
    }
    return 0; 
}

#if 0
int TeServ_RestartPrint(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    printf(
        "<Restart Data>\n"        
        "  f_restart : %s         \t mode_prog : %s\n"
        "  i_prog    : %d         \t dist_work : %.2f[mm]\n"
        "  comparison: %.2f       \t cweav_mode: %s\n"
        "  cweav_y   : %.2f       \t f_welding : %s\n"
        "  weld_spd  : %.2f[mm/ms]\t f_sensor  : %s \n"
        "  traj.type : %s         \t traj.err  : %d(%s) \n", 
        Str_OnOff(g_restart.f_restart), Str_ProgMode(g_restart.mode_prog), 
        g_restart.i_prog,               g_restart.dist_work, 
        g_restart.comp_result,          Str_CweaveMode(g_restart.cweave_mode), 
        g_restart.cweave_y_dir,         Str_OnOff(g_restart.f_welding), 
        g_restart.weld_spd,             Str_OnOff(g_restart.f_sensor), 
        Traj_TypeStr(g_restart.traj.type), g_restart.traj.error, Traj_ErrorStr(g_restart.traj.error)); 

    return 0;
}
#endif 
int TeServ_SampTime(void* buff, int* p_size, const TE_MSG* p_msg)
{
    int nRet; 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // Setting Timer
    nRet = Init_TimerInit(); 
    if(nRet)
    {
        VERBOSE_ERROR("Failed Runtime Sampling Time Setting : %u[ms]\n", g_t_samp);             
        return -1; 
    }
    else
    {
        // Validation Value
        g_t_samp = (p_msg->value <= 1)? 1 : p_msg->value;

        Traj_Set_Dynamics(&g_traj, g_t_samp, 
            g_traj.jerk_joint,  g_traj.acc_joint, g_traj.dec_stop_joint, g_traj.dec_fast_joint, 
            g_traj.jerk_pos,    g_traj.acc_pos,   g_traj.dec_stop_pos,   g_traj.dec_fast_pos, 
            g_traj.jerk_rot,    g_traj.acc_rot,   g_traj.dec_stop_rot,   g_traj.dec_fast_rot, 
            g_traj.Inverse,     g_traj.Forward,   g_traj.Config, g_traj.dh, &g_traj.eTt); 

        VERBOSE_VERBOSE("Runtime Sampling Time Setting : %u[ms]\n", g_t_samp);
        return 0;
    }
}

// Opens Target Writing File 
int TeServ_File(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // File Open 
    g_f_file = (p_msg->value)? TRUE : FALSE; 
    VERBOSE_MESSAGE("File Write Mode %s.\n", Str_OnOff(g_f_file));             
    return 0; 
}

// Resets Error 
int TeServ_ResetErr(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    memset(&g_error, 0, sizeof(g_error)); 
    VERBOSE_MESSAGE("Resets Error. Error:%d\n", g_error.code);             
    return 0; 
}

#if 0
int TeServ_TrajInfo(void* buff, int* p_size, const TE_MSG* p_msg)
#endif 
int TeServ_InfoPrint(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    switch(p_msg->value)
    {    
    case 1: // Traj
        printf("\nTraj Info's. Unit:[rad], [rad/s], [rad/s^2]\n%s", Traj_Print(&g_traj));          
        return 0; 
    case 2: // Weaving
        printf("\n%s", Wv_Print(&g_weave));          
        return 0; 
    case 3: // Restart
        printf(
            "<Restart Data>\n"        
            "  f_restart : %s         \t mode_prog : %s\n"
            "  i_prog    : %d         \t dist_work : %.2f[mm]\n"
            "  comparison: %.2f       \t cweav_mode: %s\n"
            "  cweav_y   : %.2f       \t f_welding : %s\n"
            "  weld_spd  : %.2f[mm/ms]\t f_sensor  : %s \n"
            "  traj.type : %s         \t traj.err  : %d(%s) \n\n", 
            Str_OnOff(g_restart.f_restart), Str_ProgMode(g_restart.mode_prog), 
            g_restart.i_prog,               g_restart.dist_work, 
            g_restart.comp_result,          Str_CweaveMode(g_restart.cweave_mode), 
            g_restart.cweave_y_dir,         Str_OnOff(g_restart.f_welding), 
            g_restart.weld_spd,             Str_OnOff(g_restart.f_sensor), 
            Traj_TypeStr(g_restart.traj.type), g_restart.traj.error, Traj_ErrorStr(g_restart.traj.error)); 
        return 0; 
    case 0: // Instruction
    default:
        printf("Print Infomation Service. USAGE : tep 15 [OBJ]\n"
            "  OBJ : 0 (Help), 1 (TRAJ), 2 (WEAVE), 3 (RESTART)\n"); 
        return 0; 
    }
    return 0; 
}

int TeServ_JogMode(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    // Work
#if 0
    g_mode_jog = p_msg->value; 
    VERBOSE_MESSAGE("Jog Mode Starts : (%d)\n", g_mode_jog);             
#else    
    VERBOSE_MESSAGE("Jog Mode Starts \n");             
#endif
    return 0; 
}

int TeServ_Jnt2Cart(void* buff, int* p_size, const TE_MSG* p_msg)
{	    
    TRANS cTt; 
    XYZRPY xyzrpy; 
    
    // Reply Data Setting
    if(p_size)
    {
        *p_size = sizeof(double) * 6;
    }

    TcpWrtActualCoord(&cTt, p_msg->value);  
    xyzrpy = TRANS_GetXyzrpy(cTt); 

    if(buff)
    {
        memcpy(buff, &xyzrpy, sizeof(double)*6);         
    }       
	return 0; 
}

int TeServ_DispCoord(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    
    // Work    
    VERBOSE_MESSAGE("Display Coord Change : %s(%d) -> %s(%d)\n", 
        Str_Coord(g_coord_ref), g_coord_ref, Str_Coord(p_msg->value), p_msg->value);             
 
    g_coord_ref = p_msg->value; 
    return 0; 
}

int TeServ_TimerSkip(void* buff, int* p_size, const TE_MSG* p_msg)
{
    extern void CmdStp_TIMER(int f_estop); 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    // 2013.12.19
    // CmdStp_TIMER sets 's_stop' variable in 'cmd_port.c'. 
    // And CmdStp_TIMER is for stopping of 
    //     DANDY_JOB_CODE_TIMER, DANDY_JOB_CODE_WAIT, DANDY_JOB_CODE_UNTIL, DANDY_JOB_CODE_PAUSE.     
    CmdStp_TIMER(0); 

    VERBOSE_MESSAGE("Timer(Timer, Wait, Until) Skips\n");         
    return 0; 
}

// Clear Program Var's Service
int TeServ_PrgVarClr(void* buff, int* p_size, const TE_MSG* p_msg)
{
    extern void CallStack_Init(void); 

    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }

    if(p_msg->value & 0x01)
    {
        CallStack_Init(); 
    }
    if(p_msg->value & 0x02)
    {
        RestartDataSet(OFF, RunServProg_ModeGet());
    }

    VERBOSE_MESSAGE("Call Stack Cleared\n");         
    return 0; 
}

// Show the Wedler IO Map
int TeServ_WeldIoMap(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    
    // Work
    printf("\nWelder IO Mapping \n%s", Str_WeldIoMap());          
    return 0; 
}

// Show Program Control Var's 
int TeServ_ProgCtrlVar(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    
    // Work
    printf("\n<Program Ctrl Vars>\n%s", Str_ProgCtrlVar()); 
    return 0; 
}

int TeServ_RuntimePause(void* buff, int* p_size, const TE_MSG* p_msg)
{
    // No Reply Data Setting
    if(p_size)
    {
        *p_size = 0; 
    }
    
    // Work
    g_f_runtime_pause = p_msg->value; 
    VERBOSE_MESSAGE("Runtime Thread %s.\n", (g_f_runtime_pause)? "Pauses" : "Runs");
    return 0; 
}


////////////////////////////////////////////////////////////////////////////////

// Jog Speed Service Data[%] Converter from 'char' to 'double'[-1.0~1.0]
// -100 <= n_spd <= 100 (-100 ~ 100%) : spd = n_spd * 0.01                
//  100  < n_spd        ( 0.1 ~ 2.7%) : spd = (n_spd - 100) * 0.1 * 0.01
//         n_spd < -100 (-2.8 ~-0.1%) : spd = (n_spd + 100) * 0.1 * 0.01
static double GetJogVal_Spd(char n_spd)
{
    double spd; 

    if(n_spd == 0)
    {
        spd = 0.0; 
    }
    else if(n_spd > 100)
    {
        spd = (n_spd - 100) * 0.001; 
    }
    else if(n_spd < -100)
    {
        spd = (n_spd + 100) * 0.001; 
    }
    else
    {
        spd = n_spd * 0.01; 
    }

    return spd; 
}

// Returns Dir Vect wrt BASE. 
static POS GetJogVal_DirLin(VALUE_JOG val)
{    
    TRANS T; 
    POS dir = {{0,0,1.}}; 
    double sign; 

    // Get Unit Vector wrt each Coord
    sign = (val.nSpeed < 0)? -1. : 1.; 

    switch(val.axis)
    {
    case AXIS_LIN_X:    
        dir.x = 1 * sign; 
        dir.y = 0; 
        dir.z = 0; 
        break; 
        
    case AXIS_LIN_Y:    
        dir.x = 0; 
        dir.y = 1 * sign; 
        dir.z = 0; 
        break; 

    case AXIS_LIN_Z:
        dir.x = 0; 
        dir.y = 0; 
        dir.z = 1 * sign; 
        break; 
    
    default:
    case AXIS_ROT_X:
    case AXIS_ROT_Z:
    case AXIS_ROT_Y:
        dir.x = 0; 
        dir.y = 0; 
        dir.z = 0; 
        break; 
    }

    // apply coord
    switch(val.coord)
    {
    case COORDINDEX_BASE:    
        break; // No Transfrom

    case COORDINDEX_WORLD:
        // bP = bTw*wP
        dir = ROT_Multi_POS(g_bTw.R, dir); 
        break; 

    case COORDINDEX_END:
        // bP = bTe*eP
        dir = ROT_Multi_POS(g_bTe.R, dir); 
        break; 

    case COORDINDEX_TCP:
        // bP = bTt*eP = bTe*eTt*eP
        T = TRANS_Multi_TRANS(g_bTe, g_eTt); 
        dir = ROT_Multi_POS(T.R, dir); 
        break; 

    case COORDINDEX_SENSOR:
        // bP = bTt*eP = bTe*eTt*tTs*sP
        T = TRANS_Multi_TRANS(g_bTe, g_eTt); 
        T = TRANS_Multi_TRANS(T, g_tTs); 
        dir = ROT_Multi_POS(T.R, dir); 
        break; 
    default:
        // User Coord
        // bP = bTu*uP = bTw*wTu*uP
        if(0 <= val.coord && val.coord < MAX_USER_COORD_COUNT)
        {
            XYZRPY user; 
            COORD_EULER src; 
            
            src = g_pshm_sys_conf->robot[val.nRobNum].user[(int)val.coord];
            user.x = src.x; 
            user.y = src.y; 
            user.z = src.z;
            user.roll  = src.rol;  
            user.pitch = src.pit; 
            user.yaw   = src.yaw; 

            T = TRANS_Xyzrpy(user);        // wTu
            T = TRANS_Multi_TRANS(g_bTw, T);    // bTw * wTu
            dir = ROT_Multi_POS(T.R, dir); 
        }
        // default, No Transfrom
        else
        {
            ; 
        }
    }
    return dir; 
}

// Returns Dir Vect wrt BASE. 
static POS GetJogVal_DirRot(VALUE_JOG val)
{    
    TRANS T; 
    POS dir; 
    double sign; 

    sign = (val.nSpeed < 0)? -1. : 1.; 

    switch(val.axis)
    {
    case AXIS_ROT_X:    
        dir.x = 1 * sign; 
        dir.y = 0; 
        dir.z = 0; 
        break; 
        
    case AXIS_ROT_Y:    
        dir.x = 0; 
        dir.y = 1 * sign; 
        dir.z = 0; 
        break; 

    case AXIS_ROT_Z:
        dir.x = 0; 
        dir.y = 0; 
        dir.z = 1 * sign; 
        break; 
    
    default:
    case AXIS_LIN_X:
    case AXIS_LIN_Y:
    case AXIS_LIN_Z:
        dir.x = 0; 
        dir.y = 0; 
        dir.z = 0; 
        break; 
    }

    // apply coord
    switch(val.coord)
    {
    case COORDINDEX_BASE:    
        break; // No Transfrom

    case COORDINDEX_WORLD:
        // bP = bTw*wP
        dir = ROT_Multi_POS(g_bTw.R, dir); 
        break; 

    case COORDINDEX_END:
        // bP = bTe*eP
        dir = ROT_Multi_POS(g_bTe.R, dir); 
        break; 

    case COORDINDEX_TCP:
        // bP = bTt*eP = bTe*eTt*eP
        T = TRANS_Multi_TRANS(g_bTe, g_eTt); 
        dir = ROT_Multi_POS(T.R, dir); 
        break; 

    case COORDINDEX_SENSOR:
        // bP = bTt*eP = bTe*eTt*tTs*sP
        T = TRANS_Multi_TRANS(g_bTe, g_eTt); 
        T = TRANS_Multi_TRANS(T, g_tTs); 
        dir = ROT_Multi_POS(T.R, dir); 
        break; 
    default:
        // User Coord
        // bP = bTu*uP = bTw*wTu*uP
        if(0 <= val.coord && val.coord < MAX_USER_COORD_COUNT)
        {
            XYZRPY user; 
            COORD_EULER src; 
            
            src = g_pshm_sys_conf->robot[val.nRobNum].user[(int)val.coord];            
            user.x = src.x; 
            user.y = src.y; 
            user.z = src.z;
            user.roll  = src.rol;  
            user.pitch = src.pit; 
            user.yaw   = src.yaw; 

            T = TRANS_Xyzrpy(user);        // wTu
            T = TRANS_Multi_TRANS(g_bTw, T);    // bTw * wTu
            dir = ROT_Multi_POS(T.R, dir); 
        }
        // default, No Transfrom
        else
        {
            ; 
        }
    }
    return dir; 
}
static void TeServPrint(void)
{
    int i; 
    div_t temp;

	temp = div(sizeof(s_serv), sizeof(s_serv[0])*2);

    printf("\nSERVICE    | CODE | VALUE   %10s", " "); 
    printf("  SERVICE    | CODE | VALUE\n"); 

    for(i=0 ; i<temp.quot ; i++)
    {        
        int j = (temp.rem)? i + temp.quot + 1 : i + temp.quot; 
        printf(" %s   %03d   %-18s  ", s_serv[i].name, s_serv[i].code, s_serv[i].help);         
        printf(" %s   %03d   %-18s\n", s_serv[j].name, s_serv[j].code, s_serv[j].help); 
    }
    if(temp.rem)
    {
        printf(" %s   %03d   %-18s\n", s_serv[i].name, s_serv[i].code, s_serv[i].help);         
    }
}
