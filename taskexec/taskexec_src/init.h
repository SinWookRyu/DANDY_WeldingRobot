////////////////////////////////////////////////////////////////////////////////
// INIT.H is initialzation program for TASKEXEC
// 2013-05-19 mrch0

#ifndef INIT_H_2013_05_19_
#define INIT_H_2013_05_19_

#if defined(__cplusplus)
extern "C" {
#endif

int Init_Argument(int argc, char* argv[]); 

void Init_VerboseInit(void); 

void Init_VerboseUninit(void); 

int Init_TeResourceInit(void);  

void Init_TeResourceUninit(void); 

int Init_IpcInit(void); 

void Init_IpcUninit(void); 

void Init_SetExitFlag(void); 

void Init_RobotInfo(void);

// Inits for Runtime Thread 

int Init_RuntimeInit(void); 

void Init_RuntimeUninit(void); 

// 
int Init_TimerInit(void);  // [ms]

void Init_TimerUninit(void); 

#if defined(__cplusplus)
}
#endif
#endif
