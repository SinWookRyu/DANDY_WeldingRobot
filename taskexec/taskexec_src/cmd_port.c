// 
#include "taskexec_def.h"
#include "arg.h"

static DBL s_time; 
static int s_stop; 
static DANDY_JOB_ARG_IO* s_arg; 

double TimerCountDown(void)
{
    return s_time; 
}

int CmdSet_PORT(int index, DBL* jnt)
{
    DANDY_JOB_ARG_IO* arg; 
    int src; 
    int ret; 
    
    s_stop = 0; 

    arg = &g_rcmd[index].arg.argIo; 

    // target
    ret = Arg_Scalar(&src, NULL, &arg->valSource); 
    if(ret)
    {
        return ret;
    }

    // save
    return Arg_Set_Int(&arg->valTarget, src);
}

int CmdSet_TIMER(int index, DBL* jnt)
{       
    s_stop = 0; 
    s_arg = &g_rcmd[index].arg.argIo;     
    return TimeFromArg(&s_time, &s_arg->valTimeout); 
#if 0
    if(ret)
    {   
        return ret; 
    }
    else
    {
        // Wait time is less than 0, waits infinitely
        s_time = (wait_time < 0)? DBL_MAX : wait_time; 
        return 0; 
    }
#endif
}

int CmdUpd_TIMER(void)
{
    // time over check
    s_time -= g_traj.t_samp;     
    if(s_time <= 0)
    {
        return -1; 
    }
    // stop request, exits immediately
    if(s_stop)
    {
        return -1;
    }
    // time expired
    return 0; 
}

void CmdStp_TIMER(int f_estop)
{
    s_stop = 1; 
}

// Set : CmdSet_TIMER(..)
// Exits if the condition is satisfied. 
int CmdUpd_WAIT(void)
{
    int ret; 
    int left, right; 
    
    // time expired condition
    s_time -= g_traj.t_samp;     
    if(s_time <= 0)
    {
        return -1; 
    }

    // stop request, exits immediately
    if(s_stop)
    {
        return -1;
    }

    // left
    ret = Arg_Scalar(&left,  NULL, &s_arg->valTarget); 
    if(ret)
    {
        ERROR_SET(SECT_CMD_PORT, ret); 
        return -1; 
    }

    // right
    ret = Arg_Scalar(&right, NULL, &s_arg->valSource); 
    if(ret)
    {
        ERROR_SET(SECT_CMD_PORT, ret); 
        return -1; 
    }
        
    // Comparison exit condition. Ex) GT : left is greater than right. 
    // Expires if the condition is satisfied. 
    if( (s_arg->nWaitCond == DANDY_JOB_OPCOND_EQ && left == right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_NE && left != right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_LT && left <  right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_LE && left <= right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_GT && left >  right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_GE && left >= right) )
    {
        return -1; 
    }
    else
    {
        return 0; 
    }
}

// Waits until the condition is satisfied
int CmdUpd_UNTIL(void)
{
    int ret; 
    int left, right; 

    // time expired condition
    s_time -= g_traj.t_samp;     
    if(s_time <= 0)
    {
        return -1; 
    }

    // stop requested 
    if(s_stop)
    {
        return -1; 
    }
    
    // left
    ret = Arg_Scalar(&left, NULL, &s_arg->valTarget); 
    if(ret)
    {
        ERROR_SET(SECT_CMD_PORT, ret); 
        return -1; 
    }

    // right
    ret = Arg_Scalar(&right, NULL, &s_arg->valSource); 
    if(ret)
    {
        ERROR_SET(SECT_CMD_PORT, ret); 
        return -1; 
    }
        
    // Comparison. Ex) GT : left is greater than right. 
    // Expires if the condition is not satisfied. 
    if(  s_arg->nWaitCond == DANDY_JOB_OPCOND_NONE || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_EQ && left == right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_NE && left != right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_LT && left <  right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_LE && left <= right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_GT && left >  right) || 
        (s_arg->nWaitCond == DANDY_JOB_OPCOND_GE && left >= right) )
    {
        return 0; 
    }
    else
    {
        return -1; 
    }
}

// PAUSE : Waits infinite until s_stop activated. 

int CmdSet_PAUSE(int index, DBL* jnt)
{       
    s_stop = 0; 
    return 0; 
}

int CmdUpd_PAUSE(void)
{
    // stop request, exits immediately
    if(s_stop)
    {
        return -1;
    }
    // time expired
    return 0; 
}
