/////////////////////////////////////////////////////////////////////////////
//
//  confpars_motor.c: Motor Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigMotor()
//

int SYSC_LoadConfigMotor(int nMotor, const char* pszKey, const char* pszValue)
{
    int nType, nCount;
    int nValue;
    double dbValue;

    char szBuffer[256];
    const char* pszNext;
    double rgdbVal[2];

    DANDY_ASSERT(nMotor >= 0 && nMotor < MAX_MOTOR_COUNT);

    ///////////////////////////////////////////////////////
    // name : 'NAME = The name of the motor'
    if (stricmp(pszKey, SYSCONF_KEY_MOTOR_NAME) == 0)
    {
        CRT_strcpy(g_rgszMotorName[nMotor], MOTOR_NAME_LEN, pszValue);
        g_rgszMotorName[nMotor][MOTOR_NAME_LEN-1] = 0;
    }
    ///////////////////////////////////////////////////////
    // 'TYPE = sigma_v'
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_TYPE) == 0)
    {
        if (stricmp(pszValue, "sigma_v") == 0 ||
            stricmp(pszValue, "sigma_5") == 0 ||
            stricmp(pszValue, "SIGMA-V") == 0 ||
            stricmp(pszValue, "SIGMA-5") == 0)
        {
            g_rgnMotorType[nMotor] = MOTTYPE_SIGMA_V;
        }
        else if (stricmp(pszValue, "minas") == 0 ||
            stricmp(pszValue, "MINAS") == 0)
        {
            g_rgnMotorType[nMotor] = MOTTYPE_MINAS;
        }
        else
        {
            g_rgnMotorType[nMotor] = MOTTYPE_NONE;
            VERBOSE_ERROR("Unknown Motor type : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }
    }
    ///////////////////////////////////////////////////////
    // ID = 1
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_ID) == 0)
    {
        nType = PARAM_ConvNum(pszValue, &nValue, NULL);

        if (nType != CONFIG_NUM_INT || nValue < 0 || nValue > 32767)
        {
            VERBOSE_ERROR("Invalid Motor identifier : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        g_rgnMotorIndex[nMotor] = nValue;
    }
    ///////////////////////////////////////////////////////
    // ENCODER_RES = 8192
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ENCODER_RES) == 0)
    {
        nType = PARAM_ConvNum(pszValue, &nValue, NULL);

        if (nType != CONFIG_NUM_INT || nValue == 0)
        {
            VERBOSE_ERROR("Invalid encoder resolution specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        g_rgnEncoderRes[nMotor] = nValue;
        g_rgMotorConfig[nMotor].nEncRes = nValue;
    }
    ///////////////////////////////////////////////////////
    // ABS_ENCODER_RES = 131072   <-- 2^17
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ABS_ENCODER_RES) == 0)
    {
        nType = PARAM_ConvNum(pszValue, &nValue, NULL);

        if (nType != CONFIG_NUM_INT || nValue < 2 || (nValue & 0x1))
        {
            VERBOSE_ERROR("Invalid ABS encoder resolution specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        g_rgdwAbsEncoderRes[nMotor] = (unsigned long) nValue;
    }
    ///////////////////////////////////////////////////////
    // ENCODER_TYPE = INC(or ABS)
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ENCODER_TYPE) == 0)
    {
        pszNext = pszValue;

        pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);

        if (pszNext == NULL)
        {
            VERBOSE_ERROR("Invalid syntax : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(szBuffer, "inc") == 0 || stricmp(szBuffer, "INC") == 0)
        {
            g_rgnEncoderType[nMotor] = ENCTYPE_INC;
        }
        else if (stricmp(szBuffer, "abs") == 0 || stricmp(szBuffer, "ABS") == 0)
        {
            g_rgnEncoderType[nMotor] = ENCTYPE_ABS;
        }
        else
        {
            VERBOSE_ERROR("Invalid encoder type is specified : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }
    }
    ///////////////////////////////////////////////////////
    // HW_HOME = 3
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_HW_HOME) == 0)
    {
        nType = PARAM_ConvNum(pszValue, &nValue, NULL);

        if (stricmp(pszValue, "disable") == 0)
        {
            g_rgnHwHome[nMotor] = -1;
        }
        else
        {
            nType = PARAM_ConvNum(pszValue, &nValue, NULL);

            if (nType != CONFIG_NUM_INT || nValue < 0)
            {
                VERBOSE_ERROR("Invalid HW Home : '%s=%s'\n",
                                  pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
                return RESULT_ERROR;
            }

            g_rgnHwHome[nMotor] = nValue;
        }
    }
    ///////////////////////////////////////////////////////
    // MAX_VEL = 20
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_MAX_VEL) == 0)
    {
        nType = PARAM_ConvNum(pszValue, NULL, &dbValue);

        if ((nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT) ||
            dbValue < 1.e-8)
        {
            VERBOSE_ERROR("Invalid max velocity specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        g_rgdbMotorMaxVel[nMotor] = dbValue;
        g_rgMotorConfig[nMotor].vellim_max = dbValue;
    }
    ///////////////////////////////////////////////////////
    // MAX_ACCEL = 20
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_MAX_ACCEL) == 0)
    {
        nType = PARAM_ConvNum(pszValue, NULL, &dbValue);

        if ((nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT) ||
            dbValue < 1.e-8)
        {
            VERBOSE_ERROR("Invalid max accel specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        g_rgdbMotorMaxAccel[nMotor] = dbValue;
    }
    ///////////////////////////////////////////////////////
    // xxxxx      = time
    // JERK       = 400
    // ACCEL      = 400
    // STOP       = 300
    // ERROR_STOP = 
    // ESTOP      =
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_JERK) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_MOTOR_ACCEL) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_MOTOR_DECEL) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_MOTOR_ERROR_STOP) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_MOTOR_ESTOP) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdbVal, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid motor control parameter was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 5)
        {
            VERBOSE_ERROR("Too many motor control parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ACCEL) == 0)
        {
            g_rgMotorConfig[nMotor].acc = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_DECEL) == 0)
        {
            g_rgMotorConfig[nMotor].dec = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ERROR_STOP) == 0)
        {
            g_rgMotorConfig[nMotor].dec_error = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ESTOP) == 0)
        {
            g_rgMotorConfig[nMotor].dec_estop = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_JERK) == 0)
        {
            g_rgMotorConfig[nMotor].jrk = rgdbVal[0];
        }
    }

    else
    {
        VERBOSE_ERROR("unknown motor configuration specified : %s\n", pszKey);
        SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_MOTOR_PARAM);
        return RESULT_ERROR;
    }
    
    return 0;
}