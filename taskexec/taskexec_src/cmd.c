#include "taskexec_def.h"
#include "coord.h"
#include "arg.h"
#include "cmd.h"
#include "weave.h"

////////////////////////////////////////////////////////////////////////////////
static int CmdSet_Default(int index, double jnt_start[6])  {return 0;}
static int CmdUpd_Default(void) {return -1;}
static VOD CmdStp_Default(int f_estop)   {;}

static int (*s_CmdSet)(int index, double jnt_start[6]) = CmdSet_Default; 
static int (*s_CmdUpd)(void) = CmdUpd_Default; 
static VOD (*s_CmdStp)(int f_estop) = CmdStp_Default; 

int Cmd_Init(int index, double jnt_start[6], UNT f_dry)
{    
    int ret; 

    // Cmds 
    switch(g_rcmd[index].nCode)
    {    
    case DANDY_JOB_CODE_NOP:
    case DANDY_JOB_CODE_COMMENT:
        s_CmdSet = CmdSet_Default; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default;   
        g_CmdStr = "NOP"; 
        break; 
    
    case DANDY_JOB_CODE_END:
        s_CmdSet = CmdSet_Default; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default;   
        g_CmdStr = "END"; 
        break; 

    // For CMD MOVE
    case DANDY_JOB_CODE_MOVJ:        
        s_CmdSet = CmdSet_MOVJ; 
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov; 
        g_CmdStr = "MOVJ"; 
        break;  

    case DANDY_JOB_CODE_MOVL:  
        s_CmdSet = CmdSet_MOVL;
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov;   
        g_CmdStr = "MOVL";
        break;  
    
    case DANDY_JOB_CODE_MOVO:         
        s_CmdSet = CmdSet_MOVO; 
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov;          
        g_CmdStr = "MOVO"; 
        break;  
        
    case DANDY_JOB_CODE_MOVC:         
        s_CmdSet = CmdSet_MOVC; 
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov; 
        g_CmdStr = "MOVC"; 
        break;  
        
    case DANDY_JOB_CODE_IMOVL:         
        s_CmdSet = CmdSet_MOVI; 
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov; 
        g_CmdStr = "IMOV"; 
        break;  
        
    case DANDY_JOB_CODE_HOME:         
        s_CmdSet = CmdSet_HOME; 
        s_CmdUpd = CmdUpd_Mov; 
        s_CmdStp = CmdStp_Mov; 
        g_CmdStr = "HOME"; 
        break;          

    case DANDY_JOB_CODE_UWEAVL:
        s_CmdSet = CmdSet_WEAV; 
        s_CmdUpd = CmdUpd_WEAV; 
        s_CmdStp = CmdStp_WEAV; 
        g_CmdStr = "WEAV"; 
        break;
        
    case DANDY_JOB_CODE_CWEAV:
        s_CmdSet = CmdSet_CWEAV; 
        s_CmdUpd = CmdUpd_CWEAV; 
        s_CmdStp = CmdStp_CWEAV; 
        g_CmdStr = "CWEAV"; 
        break;

    // For CMD COORD
    
    case DANDY_JOB_CODE_GETPOS:         
        s_CmdSet = Cmd_Set_GETP; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "GETP"; 
        break;          

    case DANDY_JOB_CODE_SET:
        s_CmdSet = CmdSet_SET; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "SET"; 
        break; 

    case DANDY_JOB_CODE_ADD:
        s_CmdSet = CmdSet_ADD; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "ADD"; 
        break; 

    case DANDY_JOB_CODE_SUB:
        s_CmdSet = CmdSet_SUB; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "SUB"; 
        break; 
        
    case DANDY_JOB_CODE_MUL:
        s_CmdSet = CmdSet_MUL; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "MUL"; 
        break; 
        
    case DANDY_JOB_CODE_DIV:
        s_CmdSet = CmdSet_DIV; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "DIV"; 
        break; 
        
    case DANDY_JOB_CODE_MOD:
        s_CmdSet = CmdSet_MOD; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "MOD"; 
        break; 
        
    case DANDY_JOB_CODE_AND:
        s_CmdSet = CmdSet_AND; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "AND"; 
        break; 
        
    case DANDY_JOB_CODE_OR:
        s_CmdSet = CmdSet_OR; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "OR"; 
        break; 
        
    case DANDY_JOB_CODE_XOR:
        s_CmdSet = CmdSet_XOR; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "XOR"; 
        break; 
        
    case DANDY_JOB_CODE_NOT:
        s_CmdSet = CmdSet_NOT; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "NOT"; 
        break; 
        
    case DANDY_JOB_CODE_COMP:
        s_CmdSet = CmdSet_COMP; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "COMP"; 
        break; 
        
    case DANDY_JOB_CODE_LABEL:
        s_CmdSet = CmdSet_Default;
        s_CmdUpd = CmdUpd_Default;
        s_CmdStp = CmdStp_Default;
        g_CmdStr = "LABEL";
        break; 

    case DANDY_JOB_CODE_JUMP:
        s_CmdSet = CmdSet_JUMP;
        s_CmdUpd = CmdUpd_Default;
        s_CmdStp = CmdStp_Default;
        g_CmdStr = "JUMP";
        break; 
        
    case DANDY_JOB_CODE_CALL:
        s_CmdSet = CmdSet_CALL;
        s_CmdUpd = CmdUpd_CALL_RET;
        s_CmdStp = CmdStp_Default;
        g_CmdStr = "CALL";
        break; 
    
    case DANDY_JOB_CODE_RET:
        s_CmdSet = CmdSet_RET;
        s_CmdUpd = CmdUpd_CALL_RET;
        s_CmdStp = CmdStp_Default;
        g_CmdStr = "RET";
        break; 

    case DANDY_JOB_CODE_TOUCH:
        s_CmdSet = CmdSet_TOUCH;
        s_CmdUpd = CmdUpd_TOUCH;
        s_CmdStp = CmdStp_TOUCH;
        g_CmdStr = "TOUCH";
        break; 

    case DANDY_JOB_CODE_ARCON:
        s_CmdSet = (f_dry)? CmdSet_Default : CmdSet_ARCON;
        s_CmdUpd = (f_dry)? CmdUpd_Default : CmdUpd_ARCON;
        s_CmdStp = (f_dry)? CmdStp_Default : CmdStp_WELD ;
        g_CmdStr = (f_dry)? "D_AON" : "ARCON";
        break; 

    case DANDY_JOB_CODE_ARCOFF:
        s_CmdSet = (f_dry)? CmdSet_Default : CmdSet_ARCOFF;
        s_CmdUpd = (f_dry)? CmdUpd_Default : CmdUpd_ARCOFF;
        s_CmdStp = (f_dry)? CmdStp_Default : CmdStp_WELD  ;
        g_CmdStr = (f_dry)? "D_AOF" : "ARCOF";
        break; 

    case DANDY_JOB_CODE_ARCSET:
        s_CmdSet = CmdSet_ARCSET;
        s_CmdUpd = CmdUpd_Default;
        s_CmdStp = CmdStp_Default;
        g_CmdStr = "WSET";
        break; 

    case DANDY_JOB_CODE_WIREF:
        s_CmdSet = CmdSet_WIRE;
        s_CmdUpd = CmdUpd_WIRE;
        s_CmdStp = CmdStp_WELD;
        g_CmdStr = "WIREF";
        break; 

    case DANDY_JOB_CODE_WIREB:
        s_CmdSet = CmdSet_WIRE;
        s_CmdUpd = CmdUpd_WIRE;
        s_CmdStp = CmdStp_WELD;
        g_CmdStr = "WIREB";
        break; 

    case DANDY_JOB_CODE_INP:
        s_CmdSet = CmdSet_PORT; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "INP"; 
        break; 
        
    case DANDY_JOB_CODE_OUTP:
        s_CmdSet = CmdSet_PORT; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default; 
        g_CmdStr = "OUTP"; 
        break; 
        
    case DANDY_JOB_CODE_TIMER:
        s_CmdSet = CmdSet_TIMER;
        s_CmdUpd = CmdUpd_TIMER; 
        s_CmdStp = CmdStp_TIMER; 
        g_CmdStr = "TIMER"; 
        break; 
        
    case DANDY_JOB_CODE_WAIT:
        s_CmdSet = CmdSet_TIMER; 
        s_CmdUpd = CmdUpd_WAIT; 
        s_CmdStp = CmdStp_TIMER; 
        g_CmdStr = "WAIT";
        break; 
        
    case DANDY_JOB_CODE_UNTIL:
        s_CmdSet = CmdSet_TIMER; 
        s_CmdUpd = CmdUpd_UNTIL; 
        s_CmdStp = CmdStp_TIMER; 
        g_CmdStr = "UNTIL";
        break; 

    case DANDY_JOB_CODE_PAUSE:
        s_CmdSet = CmdSet_PAUSE; 
        s_CmdUpd = CmdUpd_PAUSE; 
        s_CmdStp = CmdStp_TIMER; 
        g_CmdStr = "PAUSE";
        break; 

    default : 
        s_CmdSet = CmdSet_Default; 
        s_CmdUpd = CmdUpd_WIRE; 
        s_CmdStp = CmdStp_Default;   
        g_CmdStr = "???"; 
        return ERR_UNSUPPORT_CMD;         
    }       

    // Skips All Cmd Except SET. 
    if(g_f_skip && s_CmdSet != CmdSet_SET)
    {
        s_CmdSet = CmdSet_Default; 
        s_CmdUpd = CmdUpd_Default; 
        s_CmdStp = CmdStp_Default;   
        g_CmdStr = (PROGCOND_SKIP_LEFT)? "SkipL" : "SkipR"; 
    }

#if 0 // PRINT
    if( g_rcmd[index].nCode == DANDY_JOB_CODE_MOVL)
    {
        printf("[AAA] %f %f %f\n", jnt_start[0], jnt_start[1], jnt_start[2]); 
    }
#endif 

    ret = s_CmdSet(index, jnt_start); 
    if(ret)
    {
        R_VERB_ERR("Failed to Init CMD. Idx:%d Cmd:%s(%d).Err(%d):%s\n",index, g_CmdStr, g_rcmd[index].nCode, ret, Str_Error(ret));  
    }
    else
    {
        R_VERB_VRB("Initted CMD. Idx:%d Cmd:%s(%d)\n",index, g_CmdStr, g_rcmd[index].nCode);  
    }
    return ret;
}

// To Use Extra Cmd Handler Sets External Cmd Handlers. 
// Init Call must be done seperately. 
void Cmd_ExtHandlerSet(int (*CmdUpd)(void), VOD (*CmdStp)(int f_estop), char* str_cmd)
{
    s_CmdUpd = (CmdUpd)? CmdUpd : s_CmdUpd; 
    s_CmdStp = (CmdStp)? CmdStp : s_CmdStp; 
    g_CmdStr = str_cmd; 
}

int Cmd_Update(int index)
{
    return s_CmdUpd(); 
}

void Cmd_Stop(int index, int f_estop)
{  
    s_CmdStp(f_estop);     
} 

// returns if 'cmd_code' is the linear motion cmd. 
int Cmd_IsLinMot(int i_prog)
{ 
    ASSERT_RETURN(g_rcmd, OFF); 
    return (g_rcmd[i_prog].nCode == DANDY_JOB_CODE_MOVL  || 
            g_rcmd[i_prog].nCode == DANDY_JOB_CODE_IMOVL )? ON : OFF;    
}

int Cmd_IsCirMot(int i_prog)
{ 
    ASSERT_RETURN(g_rcmd, OFF); 
    return (g_rcmd[g_i_prog_act].nCode == DANDY_JOB_CODE_MOVC)? ON : OFF;    
}

UNT Cmd_IsWeaveCmd(int i_prog)
{       
    ASSERT_RETURN(g_rcmd, OFF); 
    return (g_rcmd[i_prog].nCode == DANDY_JOB_CODE_UWEAVL)? ON : OFF; 
}      

UNT Cmd_IsCweaveCmd(int i_prog)
{
    ASSERT_RETURN(g_rcmd, OFF); 
    return (g_rcmd[i_prog].nCode == DANDY_JOB_CODE_CWEAV)? ON : OFF; 
}