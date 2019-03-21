#include "call_stack.h"
#include "CRT.h"

static CALL_STACK s_call_stack; 

////////////////////////////////////////////////////////////////////////////////

CALL_STACK* CallStack_Ptr(void)
{
    return &s_call_stack; 
}

void CallStack_Init(void)
{
    s_call_stack.i = 0; 
}

int CallStack_IsEmpty(void)
{
    return (s_call_stack.i)? 0 : 1; 
}

// returns the actual count of the stack. 
int CallStack_Count(void)
{
    return s_call_stack.i;
}

int CallStack_IsFull(void)
{
    return (s_call_stack.i >= CALL_DEPTH)? 1 : 0; 
}

// i_call : next index to be excuted when it returned. 
int CallStack_Push(int i_call, UCH f_extern, const STR path)
{
    ASSERT_RETURN(path, -1); 
    if(CallStack_IsFull())
    {
        return -1; 
    }

    s_call_stack.call[s_call_stack.i].i_call   = i_call; 
    s_call_stack.call[s_call_stack.i].f_extern = f_extern; 
    CRT_strcpy(s_call_stack.call[s_call_stack.i].path, PATH_NAME_BUFFER_SIZE, path);  
    s_call_stack.i++; 
    return 0; 
}

int CallStack_Pop(int* i_call, UCH* f_extern, STR path)
{
    if(CallStack_IsEmpty())
    {
        return -1; 
    }

    s_call_stack.i--;
    
    if(i_call)
    {
        *i_call = s_call_stack.call[s_call_stack.i].i_call; // 1 : next next line
    }
    
    if(f_extern)
    {
        *f_extern = s_call_stack.call[s_call_stack.i].f_extern; 
    }
    if(path)
    {
        STR path_src; 
        path_src = s_call_stack.call[s_call_stack.i].path; 
        CRT_strcpy(path, PATH_NAME_BUFFER_SIZE, path_src); 
    }
    
    return 0; 
}
