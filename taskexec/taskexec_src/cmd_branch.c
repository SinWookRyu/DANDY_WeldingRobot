// CMD_BRANCH.C is CMD Program related to the Job Program Control. 
// CMD_BRANCH's subject is setting 'g_i_branch' 
// which is refered by Index Update Module of 'RUN_PROG'. 

////////////////////////////////////////////////////////////////////////////////

#include "taskexec_def.h"
#include "call_stack.h"
#include "arg.h"

static int IndexFromArg(int* i_next, UCH* f_extern, char** path_ext, int i_act); 
static int s_f_external; 
static DBL s_t_start; 

////////////////////////////////////////////////////////////////////////////////

int CmdSet_JUMP(int index, double jnt_start[6])
{
    return IndexFromArg(&g_i_branch, NULL, NULL, index); 
}

int CmdSet_CALL(int index, double jnt_start[6])
{
    int ret; 
    UCH f_extern;     
    STR path_ext; 

    // Branch info's 
    ret = IndexFromArg(&g_i_branch, &f_extern, &path_ext, index);     
    if(ret)
    {
        return ret; 
    }

    // Stack of Return Info's
    // +1 : Next of Actual is excuted when returned
    ret = CallStack_Push(index+1, f_extern, (const STR)g_pshm_rm_sys->szCurrJobFileName);     
    if(ret)
    {
        return ERR_STACK_NOMORE_PUSH;         
    }

    // external call setting 
    if(f_extern)
    {
        // Job load Request SHM Setting & Pulse Sednig 
        g_pshm_rm_sys->fJobLoadDone = 0;              
        CRT_strcpy((STR)g_pshm_rm_sys->szTargJobFileName, PATH_NAME_BUFFER_SIZE, path_ext);     
        MSG_SendPulse(g_co_rm, RSVC_SERV_JOBLOAD, 0); 

        s_f_external = 1; 
    }
    else
    {
        s_f_external = 0; 
    }
    s_t_start = 0; 

    return 0; 
}

int CmdUpd_CALL_RET(void)
{
    s_t_start += g_traj.t_samp; 

    // Waits Call(Ret) Flag is set. 
    if(s_f_external)
    {
        ASSERT_RETURN(g_pshm_rm_sys, ERR_NULL_PTR); 

        // Wait time over
        if(WAITTIME_JOBLOAD < s_t_start)
        {   
            ERROR_SET(SECT_CMD_BRANCH, ERR_TIME_OVER); 
            return ERR_TIME_OVER; 
        }
        if(g_pshm_rm_sys->fJobLoadDone == 0)
        {
            return 0; 
        }
        else if(g_pshm_rm_sys->fJobLoadDone == 1)
        {
            return -1; 
        }
        else
        {
            ERROR_SET(SECT_CMD_BRANCH, ERR_JOBLOAD_FAIL); 
            return -1;         
        }
    }
    // Internal Call
    else
    {        
        return -1; 
    }
}

int CmdSet_RET(int index, double jnt_start[6])
{
    int ret; 
    UCH f_ext_call; 
    CHR path_ret[PATH_NAME_BUFFER_SIZE]; 

    // Return Info's
    ret = CallStack_Pop(&g_i_branch, &f_ext_call, path_ret); 
    if(ret)
    {
        return ERR_STACK_NOMORE_POP; 
    }

    if(f_ext_call)
    {
        // Job load Request SHM Setting & Pulse Sednig 
        g_pshm_rm_sys->fJobLoadDone = 0;      
        CRT_strcpy((STR)g_pshm_rm_sys->szTargJobFileName, PATH_NAME_BUFFER_SIZE, path_ret);     
        MSG_SendPulse(g_co_rm, RSVC_SERV_JOBLOAD, 0); 
        
        s_f_external = 1; 
    }
    else
    {
        s_f_external = 0; 
    }
    s_t_start = 0.; 
    return 0; 
}

////////////////////////////////////////////////////////////////////////////////

#if 0
static int IndexFromArg(int* i_next, UCH* f_extern, char** path_ext, int i_act)
{
    DANDY_JOB_ARG_BRANCH* arg; 
    int ret; 

    arg = &g_rcmd[i_act].arg.argBranch;     
        
    // Branch Condistion is satisfied, Go to the index of Argument
    if( (arg->nBranchCond == DANDY_JOB_OPCOND_NONE) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_EQ && g_comp_result == 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_NE && g_comp_result != 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_LT && g_comp_result <  0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_LE && g_comp_result <= 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_GT && g_comp_result >  0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_GE && g_comp_result >= 0.0) )
    {   
        // External Branch
#if 0
        if(arg->szName[0])
#else
        if(arg->valAddr.nValueType == DANDY_JOB_VAL_TYPE_NONE)
#endif
        {
            if(f_extern)
            {
                *f_extern = 1; 
            }
            if(path_ext)
            {
                *path_ext = arg->szName; 
            }
            if(i_next)
            {
                *i_next = 0; 
            }
        }
        // Internal Branch
        else
        {
            if(f_extern)
            {
                *f_extern = 0; 
            }
            if(i_next)
            {
                ret = Arg_Scalar(i_next, NULL, &arg->valAddr); 
                if(ret)
                {
                    return ret; 
                }
            }
        }
        return 0;
    }
    // Branch Condistion is unsatisfied, Go to the next. 
    else
    {
        // Get Increment Number
        *i_next = i_act + 1;   

        // external flag setting
        if(f_extern)
        {
            *f_extern = 0;
        }
        // path for external setting
        if(path_ext)
        {
            *path_ext = ""; 
        }
        return 0; 
    }
}
#endif 

static int IndexFromArg(int* i_next, UCH* f_extern, char** path_ext, int i_act)
{
    DANDY_JOB_ARG_BRANCH* arg; 
    int ret;     
    double L, R; 
    double comp_result; 

    arg = &g_rcmd[i_act].arg.argBranch;     
        
    // Check Local Compare Mode & Calc Local Result    
    if(Arg_Scalar(NULL, &L, &arg->valIfL) == 0)         
    {
        if(Arg_Scalar(NULL, &R, &arg->valIfR) == 0)
        {
            comp_result = L - R;                        // L-Val & R-val Exist, 
        }                                               // Compare is by Local. 
        else
        {
            return ERR_LESS_ARG_COUNT;                  // L-Val Exist, No R-val 
        }
    }
    else
    {
        if(Arg_Scalar(NULL, &R, &arg->valIfR) == 0)
        {
            return ERR_LESS_ARG_COUNT;                  // No L-Val, R-val Exist
        }
        else
        {
            comp_result = g_comp_result;                // No L-val, No R-val
        }                                               // Compare is by Global. 
    }
    
    // Branch Condistion is satisfied, Go to the index of Argument
    if( (arg->nBranchCond == DANDY_JOB_OPCOND_NONE) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_EQ && comp_result == 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_NE && comp_result != 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_LT && comp_result <  0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_LE && comp_result <= 0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_GT && comp_result >  0.0) || 
        (arg->nBranchCond == DANDY_JOB_OPCOND_GE && comp_result >= 0.0) )
    {   
        // External Branch
        if(arg->valAddr.nValueType == DANDY_JOB_VAL_TYPE_NONE)
        {
            if(f_extern)
            {
                *f_extern = 1; 
            }
            if(path_ext)
            {
                *path_ext = arg->szName; 
            }
            if(i_next)
            {
                *i_next = 0; 
            }
        }
        // Internal Branch
        else
        {
            if(f_extern)
            {
                *f_extern = 0; 
            }
            if(i_next)
            {
                ret = Arg_Scalar(i_next, NULL, &arg->valAddr); 
                if(ret)
                {
                    return ret; 
                }
            }
        }
        return 0;
    }
    // Branch Condistion is unsatisfied, Go to the next. 
    else
    {
        // Get Increment Number
        *i_next = i_act + 1;   

        // external flag setting
        if(f_extern)
        {
            *f_extern = 0;
        }
        // path for external setting
        if(path_ext)
        {
            *path_ext = ""; 
        }
        return 0; 
    }
}



