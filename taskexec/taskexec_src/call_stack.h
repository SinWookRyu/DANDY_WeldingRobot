#ifndef CALL_STACK_H_
#define CALL_STACK_H_

// #include "taskexec_def.h"
#include "utility.h"
#include "ipc_robotmgr.h"

#define CALL_DEPTH              64

#pragma pack(push, 1)
typedef struct call_stack_t
{
    int i; 

    struct 
    {
        int i_call;     // return index of job program
        UCH f_extern;   // flag for external return
        CHR path[PATH_NAME_BUFFER_SIZE];    // return path 
    } call[CALL_DEPTH];
} CALL_STACK; 
#pragma pack(pop)

#if defined(__cplusplus)
extern "C" {
#endif

CALL_STACK* CallStack_Ptr(void); 
VOD CallStack_Init(void); 
int CallStack_IsEmpty(void); 
int CallStack_IsFull(void);
// Returns the count of current stack. 
int CallStack_Count(void); 
// i_call : next index to be excuted when it returned. 
int CallStack_Push(int i_call, UCH f_extern, const STR path); 
int CallStack_Pop(int* i_call, UCH* f_extern, STR path); 

#if defined(__cplusplus)
}
#endif

#endif