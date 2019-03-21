// ARG_GET.C is the program for getting JOB PROGRAM Arguments
// ARG doesn't convert units from DEG to RAD and so on. 
// User convert after/before Value Get/Set.
// 'Arg_Get' represents for ARG_GET Program. 
// 2013-10-25 mrch0

// Definition of WELDFILE_VAR_READABLE (2014-06-23)
// - DANDY_JOB_VAL::nVarOffset is used to apply reading WeldFile, WeavFile, and RobPos by INT Variable. 
// - DANDY_JOB_VAL::nVarOffset is for keeping compatibility of WeldFile & WeavFile btw DANDY1 & DANDY2. 
// - Method : FileIndex = I_var + DANDY_JOB_VAL::nVarOffset
// - DANDY_JOB_VAL::nVarOffset is used only for reading not for writing. 
#define WELDFILE_VAR_READABLE

#include "taskexec_def.h"
#include "coord.h"
#include "arg.h"

int Arg_Scalar(int             *pIntValue,       //[out] value in forms of int
               double          *pDBValue,        //[out] value in forms of double               
               const ARG_VALUE *pArgVal)         //[in]                 
{
    void            *pTemp;
    int             i;
    int             nVal = 0;
    double          dbVal = 0.0;    

    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 

    switch(pArgVal->nValueType)
    {
        // simple const(real, int, byte)
        case VALUE_TYPE_R_CONST:
            dbVal = pArgVal->value.RealConst;
            nVal = (int)dbVal;
            break;
        
        case VALUE_TYPE_I_CONST:
            nVal = pArgVal->value.IntConst;
            dbVal = (double)nVal;
            break;

        case VALUE_TYPE_B_CONST:
            nVal = pArgVal->value.ByteConst;
            dbVal = (double)nVal;
            break;          

        // variable & ROBOT_POS
        case VALUE_TYPE_R_VAR:
            pTemp = g_varR + pArgVal->value.nVarIndex;
            dbVal = (double)(*(double*)pTemp);
            nVal = (int)dbVal;
            break;

        case VALUE_TYPE_I_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            nVal = (*(int*)pTemp);
            dbVal = (double)nVal;
            break;       

        case VALUE_TYPE_B_VAR:
            pTemp = g_varB + pArgVal->value.nVarIndex;
            nVal = (*(BYTE*)pTemp);
            dbVal = (double)nVal;
            break;  
       
        // I type variable & I type ROBOT_POS
        case VALUE_TYPE_RI_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
            pTemp = g_varR + i;
            dbVal = (double)*(double*)pTemp;
            nVal = (int)dbVal;
            break;

        case VALUE_TYPE_II_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
            pTemp = g_varI + i;
            nVal = *(int*)pTemp;    
            dbVal = (double)nVal;
            break;

        case VALUE_TYPE_BI_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
            pTemp = g_varB + i;
            nVal = *(BYTE*)pTemp;    
            dbVal = (double)nVal;
            break;

        case VALUE_TYPE_DI_VAR:
            nVal = g_di[pArgVal->value.nVarIndex]; 
            dbVal = nVal; 
            break;
            
        case VALUE_TYPE_DO_VAR:
#if 0
            nVal = g_do[pArgVal->value.nVarIndex]; 
#endif 
            nVal = g_do_act[pArgVal->value.nVarIndex]; 
            dbVal = nVal; 
            break;
            
        case VALUE_TYPE_AI_VAR:
            dbVal = g_ai[pArgVal->value.nVarIndex]; 
            nVal = (int)dbVal; 
            break;
            
        case VALUE_TYPE_AO_VAR:
#if 0
            dbVal = g_ao[pArgVal->value.nVarIndex]; 
#endif 
            dbVal = g_ao_act[pArgVal->value.nVarIndex]; 
            nVal = (int)dbVal; 
            break;
            
        case VALUE_TYPE_DII_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
            nVal = g_di[i]; 
            dbVal = (double)nVal; 
            break;
            
        case VALUE_TYPE_DOI_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
#if 0
            nVal = g_do[i]; 
#endif 
            nVal = g_do_act[i]; 
            dbVal = (double)nVal; 
            break;
            
        case VALUE_TYPE_AII_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
            dbVal = g_ai[i]; 
            nVal = (int)dbVal; 
            break;
            
        case VALUE_TYPE_AOI_VAR:
            pTemp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)pTemp;
#if 0
            dbVal = g_ao[i]; 
#endif 
            dbVal = g_ao_act[i]; 
            nVal = (int)dbVal; 
            break;

        default:
            // Unsupport Type of Arg_Value
            return ERR_INVALID_ARGVAL_TYPE;
    }
    
    if(pIntValue != NULL)
    {
        *pIntValue = nVal;
    }
    if(pDBValue != NULL)
    {
        *pDBValue = dbVal;
    }

    return 0;
}

int Arg_RobPos(ROBOT_POS       *pRobPos,    //[out] value in forms of ROBOT_POS                          
               const ARG_VALUE *pArgVal)    //[in]                 
{
    int         i; 
    int         type; 
    void        *pTemp;
    ROBOT_POS   rob_pos;    

    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 

    switch(pArgVal->nValueType)
    {
    case VALUE_TYPE_T_VAR:
            pTemp = g_varT + pArgVal->value.nVarIndex;
            rob_pos = (*(ROBOT_POS*)pTemp);
            type = rob_pos.nConfig;
            break;

    case VALUE_TYPE_P_VAR:
        pTemp = g_varP + pArgVal->value.nVarIndex;
        rob_pos = (*(ROBOT_POS*)pTemp);
        break;

    case VALUE_TYPE_PI_VAR:
        pTemp = g_varI + pArgVal->value.nVarIndex;
        i = *(int*)pTemp;
#ifdef WELDFILE_VAR_READABLE
        i += pArgVal->nVarOffset; 
#endif 
        pTemp = g_varP + i;
        rob_pos = *(ROBOT_POS*)pTemp;            
        break;

    case VALUE_TYPE_TI_VAR:
        pTemp = g_varI + pArgVal->value.nVarIndex;
        i = *(int*)pTemp;
#ifdef WELDFILE_VAR_READABLE
        i += pArgVal->nVarOffset; 
#endif 
        pTemp = g_varT + i;
        rob_pos = *(ROBOT_POS*)pTemp;            
        break;

    default:
        return ERR_INVALID_ARGVAL_TYPE; 
    }

    if(pRobPos != NULL)
    {
        *pRobPos = rob_pos;
    }

    return 0; 
}

// nShift : Shifed Index of Pointer
int Arg_WeldCond_Start(WELD_COND_START* pWelCon, 
                       const ARG_VALUE* pArgVal, 
                       const unsigned   nShift)
{
    int              i;
    void            *temp;    
    WELD_COND_START  cond;

    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 
    
    switch(pArgVal->nValueType)
    {        
        case VALUE_TYPE_SWF_VAR:
            temp = g_swf + pArgVal->value.nVarIndex + nShift;
            cond = (*(WELD_COND_START*)temp);
            break;        

        // I type variable & I type ROBOT_POS
        case VALUE_TYPE_SWFI_VAR:
            temp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)temp;
#ifdef WELDFILE_VAR_READABLE
            i += pArgVal->nVarOffset; 
#endif 
            temp = g_swf + i + nShift;
            cond = *(WELD_COND_START*)temp;            
            break;

        default:
            return ERR_INVALID_ARGVAL_TYPE;
    }
  
    if(pWelCon != NULL)
    {
        *pWelCon = cond;
    }
    
    return 0;
}

// nShift : Shifed Index of Pointer
int Arg_WeldCond_Main(WELD_COND_MAIN*  pWelCon, 
                      const ARG_VALUE* pArgVal, 
                      const unsigned   nShift)
{
    int              i;
    void            *temp;    
    WELD_COND_MAIN  cond;
    
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 

    switch(pArgVal->nValueType)
    {   
        case VALUE_TYPE_MWF_VAR:
            temp = g_mwf + pArgVal->value.nVarIndex + nShift;
            cond = (*(WELD_COND_MAIN*)temp);
            break;        

        // I type variable & I type ROBOT_POS
        case VALUE_TYPE_MWFI_VAR:
            temp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)temp;
#ifdef WELDFILE_VAR_READABLE
            i += pArgVal->nVarOffset; 
#endif 
            temp = g_mwf + i + nShift;
            cond = *(WELD_COND_MAIN*)temp;            
            break;

        default:
            return ERR_INVALID_ARGVAL_TYPE;
    }
  
    if(pWelCon != NULL)
    {
        *pWelCon = cond;
    }
    
    return 0;
}

// nShift : Shifed Index of Pointer
int Arg_WeldCond_End(WELD_COND_END*   pWelCon, 
                     const ARG_VALUE* pArgVal, 
                     const unsigned   nShift)
{
    int             i;
    void           *temp;    
    WELD_COND_END   cond;
    
    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 

    switch(pArgVal->nValueType)
    {  
        case VALUE_TYPE_EWF_VAR:
            temp = g_ewf + pArgVal->value.nVarIndex + nShift;
            cond = (*(WELD_COND_END*)temp);
            break;        

        // I type variable & I type ROBOT_POS
        case VALUE_TYPE_EWFI_VAR:
            temp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)temp;
#ifdef WELDFILE_VAR_READABLE
            i += pArgVal->nVarOffset; 
#endif           
            temp = g_ewf + i + nShift;
            cond = *(WELD_COND_END*)temp;            
            break;

        default:
            return ERR_INVALID_ARGVAL_TYPE;
    }
  
    if(pWelCon != NULL)
    {
        *pWelCon = cond;
    }
    
    return 0;
}

// Gets values for weaving from ARG_VALUE & SHM_ROBOT_JOB.
// nShift : Shifed Index of Pointer
int Arg_WeavePar(WEAVE_PAR          *pWeavePar, //[out]                    
                 const ARG_VALUE    *pArgVal,   //[in]                     
                 const unsigned     nShift)
{
    int         i; 
    void       *temp;
    WEAVE_PAR   cond;

    ASSERT_RETURN(pArgVal, ERR_NULL_PTR); 

    switch(pArgVal->nValueType)
    {  
        case VALUE_TYPE_WVF_VAR:
            temp = g_wvf + pArgVal->value.nVarIndex + nShift;
            cond = (*(WEAVE_PAR*)temp);
            break;        

        // I type variable & I type ROBOT_POS
        case VALUE_TYPE_WVFI_VAR:
            temp = g_varI + pArgVal->value.nVarIndex;
            i = *(int*)temp;
#ifdef WELDFILE_VAR_READABLE
            i += pArgVal->nVarOffset; 
#endif 
            temp = g_wvf + i + nShift;
            cond = *(WEAVE_PAR*)temp;            
            break;

        default:
            return ERR_INVALID_ARGVAL_TYPE;
    }

    if(pWeavePar != NULL)
    {
        *pWeavePar = cond;
    }
    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

// Bit Element case, 
int Arg_BitElem_Get(int* out, const ARG_VALUE *arg, int index) 
{
    int ret; 
    int val; 

    ASSERT_RETURN(arg, ERR_NULL_PTR); 

    // Get basic int data
    ret = Arg_Scalar(&val, NULL, arg); 
    if(ret)
    {
        return ret;        
    }

    val = val & (0x00000001 << index);
    val = val >> index;

    if(out)
    {
        *out = val; 
    }
    return 0; 
}
       
int Arg_JntElem_Get(double* out, const ARG_VALUE* arg, int index)
{
    int ret; 
    ROBOT_POS val; 

    ASSERT_RETURN(arg, ERR_NULL_PTR); 

    // Get Robot Pos
    ret = Arg_RobPos(&val, arg); 
    if(ret)
    {
        return ret;        
    }

    // Check Joint Type of Robot Pos
    if(val.nConfig != ROBPOS_TYPE_JOINT)
    {
        return ERR_NO_JNT_ACCESS; 
    }

    if(out)
    {
        *out = val.pos[index];         
    }
    return 0; 
}

int Arg_CartElem_Get(double* out, const ARG_VALUE* arg, int index)
{
    int ret; 
    ROBOT_POS val; 

    ASSERT_RETURN(arg, ERR_NULL_PTR); 

    // Get Robot Pos
    ret = Arg_RobPos(&val, arg); 
    if(ret)
    {
        return ret;        
    }

    // Check Cart Type of Robot Pos
    if(val.nConfig != ROBPOS_TYPE_CART)
    {
        return ERR_NO_CART_ACCESS; 
    }

    if(out)
    {
        *out = val.pos[index];         
    }
    return 0; 
}
