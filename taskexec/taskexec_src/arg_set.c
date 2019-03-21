// ARG_SET.C is the program for setting JOB PROGRAM Arguments
// ARG doesn't convert units from DEG to RAD and so on. 
// User convert after/before Value Get/Set.
// 'Arg_Set_xxx' represents for ARG_SET Program. Type of xxx. 
// 'Arg_xxxElem_Set' represents for Program of Setting Argument's Element.
// 2013-10-25 mrch0

#include "taskexec_def.h"
#include "arg.h"

int Arg_Set_Byte(const ARG_VALUE *pArgVal, BYTE val)         
{
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 
    
    switch(pArgVal->nValueType)
    { 
    case VALUE_TYPE_B_VAR:
        *(g_varB + pArgVal->value.nVarIndex) = (BYTE)val; 
        return 0; 

    case VALUE_TYPE_I_VAR:
        *(g_varI + pArgVal->value.nVarIndex) = (int)val; 
        return 0; 

    case VALUE_TYPE_R_VAR:
        *(g_varR + pArgVal->value.nVarIndex) = (double)val; 
        return 0;   

    case VALUE_TYPE_DO_VAR:
#if 0
        g_do[pArgVal->value.nVarIndex] = (int)val; 
#endif 
        DO_SET(pArgVal->value.nVarIndex, (int)val); 
        return 0; 
        
    case VALUE_TYPE_AO_VAR:
#if 0
        g_ao[pArgVal->value.nVarIndex] = (double)val; 
#endif 
        AO_SET(pArgVal->value.nVarIndex, (double)val); 
        return 0; 
            
    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_Int(const ARG_VALUE *pArgVal, int val)         
{    
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 
    
    switch(pArgVal->nValueType)
    { 
    case VALUE_TYPE_B_VAR:
        *(g_varB + pArgVal->value.nVarIndex) = (BYTE)val; 
        return 0; 

    case VALUE_TYPE_I_VAR:
        *(g_varI + pArgVal->value.nVarIndex) = (int)val; 
        return 0; 

    case VALUE_TYPE_R_VAR:
        *(g_varR + pArgVal->value.nVarIndex) = (double)val; 
        return 0; 
        
    case VALUE_TYPE_DO_VAR:
#if 0
        g_do[pArgVal->value.nVarIndex] = (int)val; 
#endif 
        DO_SET(pArgVal->value.nVarIndex, (int)val); 
        return 0; 
        
    case VALUE_TYPE_AO_VAR:
#if 0
        g_ao[pArgVal->value.nVarIndex] = (double)val; 
        g_ao_type[pArgVal->value.nVarIndex] = 0; 
#endif 
        AO_SET(pArgVal->value.nVarIndex, (double)val); 
        return 0; 

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_Double(const ARG_VALUE *pArgVal, double val)         
{
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 
    
    switch(pArgVal->nValueType)
    { 
    case VALUE_TYPE_B_VAR:
        *(g_varB + pArgVal->value.nVarIndex) = (BYTE)val; 
        return 0; 

    case VALUE_TYPE_I_VAR:
        *(g_varI + pArgVal->value.nVarIndex) = (int)val; 
        return 0; 

    case VALUE_TYPE_R_VAR:
        *(g_varR + pArgVal->value.nVarIndex) = (double)val; 
        return 0;   
        
    case VALUE_TYPE_DO_VAR:
#if 0
        g_do[pArgVal->value.nVarIndex] = (int)val; 
#endif 
        DO_SET(pArgVal->value.nVarIndex, (int)val); 
        return 0; 
        
    case VALUE_TYPE_AO_VAR:
#if 0
        g_ao[pArgVal->value.nVarIndex] = (double)val; 
#endif 
        AO_SET(pArgVal->value.nVarIndex, (double)val); 
        return 0; 

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_RobPos(const ARG_VALUE *pArgVal, const ROBOT_POS *pRobPos)
{
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR);
    ASSERT_RETURN(pRobPos, ERR_NULL_PTR);

    switch(pArgVal->nValueType)
    {            
    case VALUE_TYPE_T_VAR:
        memcpy(g_varT + pArgVal->value.nVarIndex, pRobPos, sizeof(ROBOT_POS));                
        return 0; 

    case VALUE_TYPE_P_VAR:       
        memcpy(g_varP + pArgVal->value.nVarIndex, pRobPos, sizeof(ROBOT_POS));                
        return 0; 

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_WeavePar(const ARG_VALUE *pArgVal, const WEAVE_PAR *pWeavePar)
{
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR);
    ASSERT_RETURN(pWeavePar, ERR_NULL_PTR);

    switch(pArgVal->nValueType)
    {            
    case VALUE_TYPE_WVF_VAR:
        memcpy(g_wvf + pArgVal->value.nVarIndex, pWeavePar, sizeof(WEAVE_PAR));                
        return 0; 

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_WeldCondStart(const ARG_VALUE *arg, const WELD_COND_START* start)                     
{
    ASSERT_RETURN(arg, ERR_NULL_PTR);
    ASSERT_RETURN(start, ERR_NULL_PTR); 

    switch(arg->nValueType)
    {            
    case VALUE_TYPE_SWF_VAR:        
        memcpy(g_swf + arg->value.nVarIndex, start, sizeof(WELD_COND_START));                
        return 0;

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_WeldCondMain(const ARG_VALUE *arg, const WELD_COND_MAIN* pri)                     
{
    ASSERT_RETURN(arg, ERR_NULL_PTR);
    ASSERT_RETURN(pri, ERR_NULL_PTR); 

    switch(arg->nValueType)
    {            
    case VALUE_TYPE_MWF_VAR:        
        memcpy(g_mwf + arg->value.nVarIndex, pri, sizeof(WELD_COND_MAIN));                
        return 0;

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

int Arg_Set_WeldCondEnd(const ARG_VALUE *arg, const WELD_COND_END* end)                     
{
    ASSERT_RETURN(arg, ERR_NULL_PTR);
    ASSERT_RETURN(end, ERR_NULL_PTR); 

    switch(arg->nValueType)
    {            
    case VALUE_TYPE_EWF_VAR:        
        memcpy(g_ewf + arg->value.nVarIndex, end, sizeof(WELD_COND_MAIN));                
        return 0;

    default:
        return ERR_INVALID_ARGVAL_TYPE;
    }
}
// elem : Bit Index Number
int Arg_BitElem_Set(const ARG_VALUE *arg, int elem, int value)
{
    int obj; 
    int ret; 

    // Get Source in type of Bit. 
    value = (value)? (0x00000001 << elem) : 0; 

    // Get basic int data
    ret = Arg_Scalar(&obj, NULL, arg); 
    if(ret)
    {
        return ret;        
    }

    // Operation
    obj = obj & ~(0x00000001 << elem);            
    obj = obj | value; 

    return Arg_Set_Int(arg, obj); 
}

int Arg_JntElem_Set(const ARG_VALUE *arg, int elem, double value)
 {       
    ROBOT_POS obj;     
    int ret; 

    // Get Value
    ret = Arg_RobPos(&obj, arg);             
    if(ret)
    {
        return ret;        
    }

    // Check Joint Type of Robot Pos
    if(obj.nConfig != ROBPOS_TYPE_JOINT)
    {
        return ERR_NO_JNT_ACCESS; 
    }

    obj.pos[elem] = value;  
    return Arg_Set_RobPos(arg, &obj); 
}

int Arg_CartElem_Set(const ARG_VALUE *arg, int elem, double value)
 {       
    ROBOT_POS obj;     
    int ret; 

    // Get Value
    ret = Arg_RobPos(&obj, arg);             
    if(ret)
    {
        return ret;        
    }

    // Check Joint Type of Robot Pos
    if(obj.nConfig != ROBPOS_TYPE_CART)
    {
        return ERR_NO_CART_ACCESS; 
    }

    obj.pos[elem] = value;  
    return Arg_Set_RobPos(arg, &obj); 
}