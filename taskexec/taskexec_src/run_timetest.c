#include "taskexec_def.h"

// Rule of Cmd Executor 
// 
// Cmd Excutor Format : Init() -> Process() -> PreProcess() -> Process() -> ..
// Request is setting Req Var's & Execution of Req. is in PreProcess
// All Cmd & Step have 1 sampling at least. 
// 
// In PreProcess()
// - Ret -1(End of Cmd), 0(Running)
// - Transient Process
// - Step Over Process
// - Init Process : 
// - All Interruptable Request Handling (init, stop,...)
// - Justifies Go on or Quit
// - Check Error 
// 
// In Process()
// - Calculation Target on the time even if error case
// - No Return
// - 

// 
#define STEP_TIMETEST_NONE  0
#define STEP_TIMETEST_START 1
#define STEP_TIMETEST_RUN   2
#define STEP_TIMETEST_STOP  3

#define TIMETEST_START_TIME 2000
#define TIMETEST_STOP_TIME  2000

static unsigned s_step = STEP_TIMETEST_NONE; 
static unsigned s_stop;                         // Stop Request

static unsigned s_i_start; 
static unsigned s_i_run; 
static unsigned s_i_stop; 

static TIME_PRECISE s_time; 
static TIME_PRECISE s_time_old; 

static unsigned s_time_max; 
static unsigned s_time_min; 

static double s_time_dif; 
static double s_time_avg; 
static double s_time_avg_old; 
static double s_time_std; 
static double s_time_std_old; 

// return : 0(suc), ELSE(fail)
int RunServTimerTest_Init(void)
{
    //if(s_step != STEP_TIMETEST_NONE)
    // {
    //     return -1; 
    // }

    // General Control Var's    
    s_stop = 0; 
    s_step = STEP_TIMETEST_START; 
    
    // Specific Control Var's
    s_i_start = 0; 
    s_i_stop = 0; 
    s_i_run = 0; 
    
    // Implematation Var's
    s_time = 0;    
    s_time_dif = 0; 
    s_time_avg = 0; 
    s_time_std = 0;     

    s_time_old = 0; 
    s_time_avg_old = 0; 
    s_time_std_old = 0; 
    
    s_time_max = 0; 
    s_time_min = 0xFFFFFFFF; 

#if 0
    TIME_InitPrecise();     
#endif
    return 0; 
}

// return : 0(running), ELSE(end of process)
int RunServTimerTest_Update(void)
{   
    // Update Data 
    s_time_old = s_time; 
    s_time_avg_old = s_time_avg; 
    s_time_std_old = s_time_std; 
    
    // (E)Stop Request & Stop Init
    if(s_stop && s_step != STEP_TIMETEST_STOP)
    {
        s_step = STEP_TIMETEST_STOP;       
        s_i_stop = 0; 
    }
        
    switch(s_step)
    {
    case STEP_TIMETEST_START:
        
        s_i_run = 0; 
        s_i_start++; 
        
        // Next Step Conditin
        if( s_i_start * g_t_samp >= TIMETEST_START_TIME ) 
        {    
            s_i_run = 1; 
            s_step = STEP_TIMETEST_RUN;              
        }
        return 0; 

    case STEP_TIMETEST_RUN:        
        s_i_run++; 
        return 0; 

    case STEP_TIMETEST_STOP:

        s_i_stop++; 

        // Next Step Conditin
        if( s_i_stop * g_t_samp  >= TIMETEST_STOP_TIME ) 
        {
            s_step = STEP_TIMETEST_NONE;              
            return -1; 
        }
        return 0;

    case STEP_TIMETEST_NONE:
    default:
        return -1;
    }
}

// 
void RunServTimerTest_Run(void)
{   
    double n; 

    s_time =  TIME_GetPrecise();   

    switch(s_step)
    {
    case STEP_TIMETEST_START:
        // Waits Start Time
        s_time_avg = (double)(s_time - s_time_old); 
        s_time_std = 0; 
        break;        

    case STEP_TIMETEST_RUN:
        // Calculate Time of This Sampling 

        n = s_i_run;  // for double calculation

        s_time_dif = (double)(s_time - s_time_old); 
        s_time_avg = (s_time_avg_old*(n-1)/n + s_time_dif/n); 
        s_time_std = (s_time_std_old*(n-1)/n + (s_time_avg-s_time_dif)/n); 

        if(s_time_dif > g_t_samp*BASIC_SAMPLE)
        {
            R_VERB_MSG("At:%llx, diff:%llx\n", s_time, s_time-s_time_old);
        }
        s_time_max = (s_time_dif > s_time_max)? (unsigned)s_time_dif : s_time_max; 
        s_time_min = (s_time_dif < s_time_min)? (unsigned)s_time_dif : s_time_min; 
        
        break; 
        
    case STEP_TIMETEST_STOP:
        // Waits Stop Time
        break; 
        
    case STEP_TIMETEST_NONE:
    default:
        break; 
    }

    VGA_printf(0, g_i_vga,"      Test[ns] tck:%09u max:%09u min:%09u    ", 
            s_i_run, s_time_max, s_time_min); 
    VGA_printf(1, g_i_vga, "               dif:%09.0f avg:%09.0f std:%09.0f     ", 
            s_time_dif, s_time_avg, s_time_std);    
}


// Stop Request
void RunServTimerTest_Stop(void)
{
    s_stop = 1; 
#if 0
    TIME_InitPrecise(); 
#endif   
}
