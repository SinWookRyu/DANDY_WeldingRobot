// CMD_OP.C is the program for operational job cmds. 
// 2013.10.25 mrch0

#include "arg.h"

#pragma pack(push, 1)
typedef enum op_type_t 
{
    OPV_TYPE_NONE = 0,
    OPV_TYPE_INT, 
    OPV_TYPE_DOUBLE, 
    OPV_TYPE_RPOS,
    OPV_TYPE_WEAV, 
    OPV_TYPE_WCS, 
    OPV_TYPE_WCM, 
    OPV_TYPE_WCE    
}OP_VALUE_TYPE;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct calc_value_t
{
    OP_VALUE_TYPE type; 
    union 
    {
        int             nValue;
        double          dbValue;
        DANDY_JOB_POS   posValue;
        DANDY_JOB_WEAV  weaValue;
        DANDY_JOB_SWF   wcsValue;
        DANDY_JOB_MWF   wcmValue;
        DANDY_JOB_EWF   wceValue;
    } object; 
} OP_VALUE;
#pragma pack(pop)

#define IS_BIT_ELEMENT(ele_)  (DANDY_JOB_VAL_ELE_B0 <= (ele_) && (ele_) <= DANDY_JOB_VAL_ELE_B31)
#define IS_JNT_ELEMENT(ele_)  (DANDY_JOB_VAL_ELE_J0 <= (ele_) && (ele_) <= DANDY_JOB_VAL_ELE_J5)
#define IS_CART_ELEMENT(ele_) (DANDY_JOB_VAL_ELE_X  <= (ele_) && (ele_) <= DANDY_JOB_VAL_ELE_RZ)

#define BIT_ELEM_2_INDEX(ele_)     ((ele_) - DANDY_JOB_VAL_ELE_B0)
#define JNT_ELEM_2_INDEX(ele_)     ((ele_) - DANDY_JOB_VAL_ELE_J0)
#define CART_ELEM_2_INDEX(ele_)    ((ele_) - DANDY_JOB_VAL_ELE_X)

////////////////////////////////////////////////////////////////////////////////

static int Operation(int index, int cmd_code); 

static int TypeCast_Dbl(OP_VALUE* val); 

static int TypeCast_Int(OP_VALUE* val); 

// Reads OP_VALUE out of OPV_LVAL
// If 'pArgValue' is 'int' & 'nElement' is bit, output is 1 or 0.
// If 'pArgValue' is 'ROBOT_POS' & 'nElement' exist, output is double type.
// If 'nElement' is not correct type of 'pArgValue', Output is undefined. 
// Returns suc(0), fail(-1)
static int OpVal_from_ArgVal(OP_VALUE *val, const ARG_VALUE *arg, int elem);

static int ArgVal_from_OpVal(ARG_VALUE* arg, int elem, const OP_VALUE* val);

////////////////////////////////////////////////////////////////////////////////

int CmdSet_SET(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_ARITH* arg;
    OP_VALUE R; 
    int ret; 

    arg = &g_rcmd[index].arg.argArith; 

    // Read Value 
    ret = OpVal_from_ArgVal(&R, &arg->valSource, arg->nSourceElement);
    if(ret)
    {
        return ret; 
    }

    // Save L_Value    
    return ArgVal_from_OpVal(&arg->valTarget, arg->nTargetElement, &R);
}

int CmdSet_ADD(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_ADD); 
}
int CmdSet_SUB(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_SUB); 
}
int CmdSet_MUL(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_MUL); 
}
int CmdSet_DIV(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_DIV); 
}
int CmdSet_MOD(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_MOD); 
}
int CmdSet_AND(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_AND); 
}
int CmdSet_OR(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_OR); 
}
int CmdSet_XOR(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_XOR); 
}
int CmdSet_NOT(int index, double jnt_start[6])
{
    return Operation(index, DANDY_JOB_CODE_NOT); 
}


int CmdSet_COMP(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_ARITH* arg;
    OP_VALUE L, R;     
    int ret; 

    arg = &g_rcmd[index].arg.argArith; 

    // Read Value 1
    ret = OpVal_from_ArgVal(&L, &arg->valTarget, arg->nTargetElement);
    if(ret)
    {
        return ret; 
    }

    // Read Value 2
    ret = OpVal_from_ArgVal(&R, &arg->valSource, arg->nSourceElement);
    if(ret)
    {
        return ret; 
    }

    // Double Type Cast 
    ret = TypeCast_Dbl(&L); 
    if(ret)
    {
        return ret;
    }    
    ret = TypeCast_Dbl(&R); 
    if(ret)
    {
        return ret;
    }

    g_comp_result = L.object.dbValue - R.object.dbValue; 

    return 0; 
}

////////////////////////////////////////////////////////////////////////////////

static int Operation(int index, int cmd_code)
{
    DANDY_JOB_ARG_ARITH* arg;
    OP_VALUE L, R; 
    div_t temp;
    int ret; 

    arg = &g_rcmd[index].arg.argArith; 

    // Read Value 1
    ret = OpVal_from_ArgVal(&L, &arg->valTarget, arg->nTargetElement);
    if(ret)
    {
        return ret; 
    }

    // Read Value 2
    ret = OpVal_from_ArgVal(&R, &arg->valSource, arg->nSourceElement);
    if(ret)
    {
        return ret; 
    }

    // Double Type Cast & Operation
    if(L.type == OPV_TYPE_DOUBLE || R.type == OPV_TYPE_DOUBLE) 
    {
        ret = TypeCast_Dbl(&L); 
        if(ret)
        {
            return ret;
        }
        ret = TypeCast_Dbl(&R); 
        if(ret)
        {
            return ret;
        }

        L.type = OPV_TYPE_DOUBLE; 

        switch(cmd_code)
        {
        case DANDY_JOB_CODE_ADD:
            L.object.dbValue = L.object.dbValue + R.object.dbValue;                     
            break;

        case DANDY_JOB_CODE_SUB:
            L.object.dbValue = L.object.dbValue - R.object.dbValue;                     
            break;   

        case DANDY_JOB_CODE_MUL:
            L.object.dbValue = L.object.dbValue * R.object.dbValue;                     
            break;   

        case DANDY_JOB_CODE_DIV:
            if(R.object.dbValue < EPSILON && -EPSILON < R.object.dbValue)
            {
                return ERR_DIV_BY_0; 
            }
            L.object.dbValue = L.object.dbValue / R.object.dbValue;                     
            break;   

        case DANDY_JOB_CODE_MOD:  
            temp = div((int)L.object.dbValue, (int)R.object.dbValue);    
            L.object.dbValue = temp.rem; 
            break;

        case DANDY_JOB_CODE_AND:            
        case DANDY_JOB_CODE_OR:            
        case DANDY_JOB_CODE_XOR:            
        case DANDY_JOB_CODE_NOT:
            return ERR_UNSUPPORT_DBL_OPER; 
        default:
            return ERR_UNSUPPORT_OPER_CMD; 
        }
    }
    // Int Type Cast & Operation
    else
    {
        ret = TypeCast_Int(&L); 
        if(ret)
        {
            return ret;
        }
        ret = TypeCast_Int(&R); 
        if(ret)
        {
            return ret;
        }

        L.type = OPV_TYPE_INT; 

        switch(cmd_code)
        {   
        case DANDY_JOB_CODE_ADD:
            L.object.nValue = L.object.nValue + R.object.nValue;                     
            break;

        case DANDY_JOB_CODE_SUB:
            L.object.nValue = L.object.nValue - R.object.nValue;                     
            break;
            
        case DANDY_JOB_CODE_MUL:
            L.object.nValue = L.object.nValue * R.object.nValue;                     
            break;
        
        case DANDY_JOB_CODE_DIV:
            if(R.object.nValue < EPSILON && -EPSILON < R.object.nValue)
            {
                return ERR_DIV_BY_0; 
            }
            L.object.nValue = L.object.nValue / R.object.nValue;                     
            break;
            
        case DANDY_JOB_CODE_MOD:   
            temp = div(L.object.nValue, R.object.nValue);    
            L.object.nValue = temp.rem; 
            break; 

        case DANDY_JOB_CODE_AND:   
            L.object.nValue = L.object.nValue & R.object.nValue;  
            break; 

        case DANDY_JOB_CODE_OR:      
            L.object.nValue = L.object.nValue | R.object.nValue;   
            break; 

        case DANDY_JOB_CODE_XOR:            
            L.object.nValue = L.object.nValue ^ R.object.nValue;   
            break; 

        case DANDY_JOB_CODE_NOT:
            L.object.nValue = !(R.object.nValue);   
            break; 

        default:             
            return ERR_UNSUPPORT_OPER_CMD; 
        }
    }

    // Save L_Value    
    return ArgVal_from_OpVal(&arg->valTarget, arg->nTargetElement, &L);
}

////////////////////////////////////////////////////////////////////////////////

static int TypeCast_Dbl(OP_VALUE* val)
{
    if(val->type == OPV_TYPE_INT)
    {
        double temp; 
        temp = val->object.nValue; 
        val->object.dbValue = (double)temp; 
        val->type = OPV_TYPE_DOUBLE; 
        return 0; 
    }

    if(val->type == OPV_TYPE_DOUBLE)
    {        
        return 0; 
    }
    
    return ERR_INVALID_ARGVAL_TYPE; 
}

static int TypeCast_Int(OP_VALUE* val)
{
    if(val->type == OPV_TYPE_INT)
    {   
        return 0; 
    }

    if(val->type == OPV_TYPE_DOUBLE)
    {
        double temp; 
        temp = val->object.dbValue; 
        val->object.nValue = (int)temp; 
        val->type = OPV_TYPE_INT;
        return 0; 
    }
    
    return ERR_INVALID_ARGVAL_TYPE; 
}

// Reads OP_VALUE out of OPV_LVAL
// If 'pArgValue' is 'int' & 'nElement' is bit, output is 1 or 0.
// If 'pArgValue' is 'ROBOT_POS' & 'nElement' exist, output is double type.
// If 'nElement' is not correct type of 'pArgValue', Output is undefined. 
// Returns suc(0), fail(-1)
static int OpVal_from_ArgVal(OP_VALUE *val, const ARG_VALUE *arg, int elem)
{    
    ASSERT_RETURN(arg, ERR_NULL_PTR); 

    // Get operand
    switch(arg->nValueType)
    {
        // Int type case. 
    case VALUE_TYPE_B_CONST:
    case VALUE_TYPE_B_VAR:
    case VALUE_TYPE_BI_VAR:

    case VALUE_TYPE_I_CONST:
    case VALUE_TYPE_I_VAR:
    case VALUE_TYPE_II_VAR:

    case VALUE_TYPE_DI_VAR:
    case VALUE_TYPE_DO_VAR:
    case VALUE_TYPE_AI_VAR:
    case VALUE_TYPE_AO_VAR:
    case VALUE_TYPE_DII_VAR:
    case VALUE_TYPE_DOI_VAR:
    case VALUE_TYPE_AII_VAR:
    case VALUE_TYPE_AOI_VAR:

        // Set Value Type
        

        // Get basic int data
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_INT;
            return Arg_Scalar(&val->object.nValue, NULL, arg); 
        }

        // Bit Element case, 
        if(IS_BIT_ELEMENT(elem))
        {
            val->type = OPV_TYPE_INT;
            return Arg_BitElem_Get(&val->object.nValue, arg, BIT_ELEM_2_INDEX(elem)); 
        }
        return ERR_ARG_ELEM_MISMATCH;

        // Real type case
    case VALUE_TYPE_R_CONST:    
    case VALUE_TYPE_R_VAR:    
    case VALUE_TYPE_RI_VAR:        

        // Get basic int data
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_DOUBLE;
            return Arg_Scalar(NULL, &val->object.dbValue, arg); 
        }

        return ERR_ARG_ELEM_MISMATCH;

        // ROBOT_POS type case. 
    case VALUE_TYPE_T_VAR:
    case VALUE_TYPE_P_VAR:
    case VALUE_TYPE_TI_VAR:        
    case VALUE_TYPE_PI_VAR:

        // Get basic int data
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_RPOS;
            return Arg_RobPos(&val->object.posValue, arg); 
        }

        // RobPos Type & Element Type Check 
        // Cart RobPos & Joint Element is not allowed. 
        // Joint RobPos & Cart Element is not allowed.

        if(IS_CART_ELEMENT(elem))
        {
            val->type = OPV_TYPE_DOUBLE;
            return Arg_CartElem_Get(&val->object.dbValue, arg, CART_ELEM_2_INDEX(elem)); 
        }

        if(IS_JNT_ELEMENT(elem))
        {
            val->type = OPV_TYPE_DOUBLE;
            return Arg_JntElem_Get(&val->object.dbValue, arg, JNT_ELEM_2_INDEX(elem)); 
        }

        return ERR_ARG_ELEM_MISMATCH;

        // ROBOT_WEAV type
    case VALUE_TYPE_WVF_VAR:
    case VALUE_TYPE_WVFI_VAR:

        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_WEAV;
            return Arg_WeavePar(&val->object.weaValue, arg, g_n_shift); 
        }
        return ERR_ARG_ELEM_MISMATCH;

        // WELD_COND_STAR type
    case VALUE_TYPE_SWF_VAR:
    case VALUE_TYPE_SWFI_VAR:

        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_WCS;
            return Arg_WeldCond_Start(&val->object.wcsValue, arg, g_n_shift); 
        }
        return ERR_ARG_ELEM_MISMATCH;

        // WELD_COND_MAIN type
    case VALUE_TYPE_MWF_VAR:
    case VALUE_TYPE_MWFI_VAR:

        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_WCM;
            return Arg_WeldCond_Main(&val->object.wcmValue, arg, g_n_shift); 
        }
        return ERR_ARG_ELEM_MISMATCH;

        // WELD_COND_END type
    case VALUE_TYPE_EWF_VAR:
    case VALUE_TYPE_EWFI_VAR:

        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            val->type = OPV_TYPE_WCE;
            return Arg_WeldCond_End(&val->object.wceValue, arg, g_n_shift); 
        }
        return ERR_ARG_ELEM_MISMATCH;

    default:        
        return ERR_INVALID_ARGVAL_TYPE;
    }
}

static int ArgVal_from_OpVal(ARG_VALUE* arg, int elem, const OP_VALUE* val)
{    
    int nVal; 
    DBL dVal; 

    ASSERT_RETURN(arg, ERR_NULL_PTR); 
    ASSERT_RETURN(val, ERR_NULL_PTR);

    // Get operand
    switch(val->type)
    {
    case OPV_TYPE_INT:

        nVal = val->object.nValue; 

        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_Int(arg, nVal); 
        }

        if(IS_BIT_ELEMENT(elem))
        {
            return Arg_BitElem_Set(arg, BIT_ELEM_2_INDEX(elem), nVal);
        }

        if(IS_CART_ELEMENT(elem))            
        {               
            return Arg_CartElem_Set(arg, CART_ELEM_2_INDEX(elem), (double)nVal);
        }

        if(IS_JNT_ELEMENT(elem))
        {
            return Arg_JntElem_Set(arg, JNT_ELEM_2_INDEX(elem), (double)nVal); 
        }
              
    case OPV_TYPE_DOUBLE:

        dVal = val->object.dbValue; 

        // no element -> set val directly
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_Double(arg, dVal); 
        }

        // obj is bit element 
        if(IS_BIT_ELEMENT(elem))
        {
            return Arg_BitElem_Set(arg, BIT_ELEM_2_INDEX(elem), (int)dVal); 
        }

        // obj is robot pos element(cart/joint)
        if(IS_CART_ELEMENT(elem))            
        {   
            return Arg_CartElem_Set(arg, CART_ELEM_2_INDEX(elem), dVal); 
        }

        // obj is robot pos element(cart/joint)
        if(IS_JNT_ELEMENT(elem))
        {  
            return Arg_JntElem_Set(arg, JNT_ELEM_2_INDEX(elem), dVal); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    case OPV_TYPE_RPOS:
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_RobPos(arg, &val->object.posValue); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    case OPV_TYPE_WEAV:
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_WeavePar(arg, &val->object.weaValue); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    case OPV_TYPE_WCS:
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_WeldCondStart(arg, &val->object.wcsValue); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    case OPV_TYPE_WCM:
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_WeldCondMain(arg, &val->object.wcmValue); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    case OPV_TYPE_WCE:
        if(DANDY_JOB_VAL_ELE_NONE == elem)
        {
            return Arg_Set_WeldCondEnd(arg, &val->object.wceValue); 
        }
        return ERR_ARG_ELEM_MISMATCH; 

    default:
        return ERR_UNSUPPORT_VAL; 
    }
}
