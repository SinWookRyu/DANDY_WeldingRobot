#ifndef RUNTIME_H_20130708_
#define RUNTIME_H_20130708_

#include "dandy_platform.h"
#include "taskexec_def.h"

#if defined(__cplusplus)
extern "C" {
#endif

THREAD_ENTRY_TYPE RuntimeThread(void* pParam); 

int Run_ReqMode(int mode, void* param); 

void Run_Stop(int f_quck); 

// Checks Runtime is Jog running with 'nAxis'.
// Ret Running(1) Else(0)
int Run_IsJogRunning(int nAxis); 

#if defined(__cplusplus)
}
#endif

#endif
