#ifndef RUN_SERV_H__
#define RUN_SERV_H__

#include "taskexec_def.h"
#include "trajectory.h"

#if defined (__cplusplus)
extern "C" {
#endif

#pragma pack(push,1)   
typedef struct 
{
    char        coord;
    BYTE        axis;
    double      vel;    // [/ms] (>0)
    double      dir;    // Pos:1.0 Neg:-1.0
    double      keep;   // [ms] keeping time
    double 		dog; 	// [ms] watchdog timer (jog_keep<jog_dog, Stops Jog)

    // for cartesian jog 
    POS     s_lin;      // wrt BASE
    double  v_lin;      
    POS     s_rot;      // wrt BASE
    double  v_rot;      // [rad/ms]
}RUNPARAM_JOG; 
#pragma pack(pop)

#pragma pack(push,1)   
typedef struct 
{
    int     i_start;    // start program index    
    int     mode_run;   // PROG_RUNMODE_XXX
}RUNPARAM_PROG; 
#pragma pack(pop)

#pragma pack(push,1)   
typedef struct 
{
    int     temp;
}RUNPARAM_RESTART; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{
    int test; 
}RUNPARAM_TIMETEST; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
    RUNPARAM_TIMETEST   timetest; 
    RUNPARAM_JOG        jog; 
    RUNPARAM_PROG       prog; 
    RUNPARAM_RESTART    restart; 
}RUN_PARAM; 
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////    
// 
int RUNSERV_INIT(int mode, const RUN_PARAM* param); 
int RUNSERV_UPDATE(int mode); 
VOD RUNSERV_RUN(int mode); 
VOD RUNSERV_STOP(int f_quick); 

////////////////////////////////////////////////////////////////////////////////
// Formal Services for Runtime Module
int RunServNormal_Init(void); 
int RunServNormal_Update(void); 
VOD RunServNormal_Run(void); 
VOD RunServNormal_Stop(void); 

int RunServTimerTest_Init(void); 
int RunServTimerTest_Update(void); 
VOD RunServTimerTest_Run(void); 
VOD RunServTimerTest_Stop(void); 

int RunServJog_Init(const RUNPARAM_JOG* jog);
int RunServJog_Update(void);
VOD RunServJog_Stop(int f_quick);

int RunServProg_Init(const RUNPARAM_PROG* prog); 
int RunServProg_Update(void);
VOD RunServProg_Stop(int f_quick);
// Resets Program Mode with 'prog'
VOD RunServProg_Reset(const RUNPARAM_PROG* prog); 

int RunServRestart_Init(const RUNPARAM_RESTART* prog); 
int RunServRestart_Update(void);
VOD RunServRestart_Stop(int f_quick);

// Returns Running Program Index
int RunServProg_IndexGet(void);
// Returns Running Program Mode
int RunServProg_ModeGet(void); 
// Returns Next Running Program Index
// Next Index is useful at Normal Mode. 
int RunServProg_NextGet(void); 

////////////////////////////////////////////////////////////////////////////////
// Specific Services 

extern RUNPARAM_JOG jog_param; 
// Resets Jog Watch-dog
void Jog_WatchDogReset(void);

// Resets Weave Jog Watch-dog
void JogWeave_WatchDogReset(void); 

#if defined (__cplusplus)
}
#endif

#endif
