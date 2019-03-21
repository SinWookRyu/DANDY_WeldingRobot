/////////////////////////////////////////////////////////////////////////////
//
//  confpars_axis.c: Axis Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigAxis()
//

int SYSC_LoadConfigAxis(int nAxis, const char* pszKey, const char* pszValue)
{
    int i, nType, nCount;
    int nValue;
    double dbValue;

    char szBuffer[256];
    const char* pszNext;
    double rgdbVal[2];

    DANDY_ASSERT(nAxis >= 0 && nAxis < MAX_AXIS_COUNT);

    ///////////////////////////////////////////////////////
    // name : 'NAME = The name of the axis'
    if (stricmp(pszKey, SYSCONF_KEY_AXIS_NAME) == 0)
    {
        CRT_strcpy(g_rgszAxisName[nAxis], AXIS_NAME_LEN, pszValue);
        g_rgszAxisName[nAxis][AXIS_NAME_LEN-1] = 0;
    }
    ///////////////////////////////////////////////////////
    // 'TYPE = REVOLUTE'
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_TYPE) == 0)
    {
        if (stricmp(pszValue, "REVOLUTE") == 0 ||
            stricmp(pszValue, "REV") == 0 ||
            stricmp(pszValue, "ROTARY") == 0)
        {
            g_rgnAxisType[nAxis] = AXISTYPE_REVOLUTE;
        }
        else if (stricmp(pszValue, "PRISMATIC") == 0 ||
            stricmp(pszValue, "PRIS") == 0 ||
            stricmp(pszValue, "LINEAR") == 0)
        {
            g_rgnAxisType[nAxis] = AXISTYPE_PRISMATIC;
        }
        else
        {
            g_rgnAxisType[nAxis] = AXISTYPE_NONE;
            VERBOSE_ERROR("Unknown Axis type : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
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
            VERBOSE_ERROR("Invalid Axis identifier : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
            return RESULT_ERROR;
        }

        g_rgnAxisIndex[nAxis] = nValue;
    }
    ///////////////////////////////////////////////////////
    // HW_LIMIT = -3670, disable
    // HW_LIMIT = -3540, 6570
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_HW_LIMIT) == 0)
    {
        pszNext = pszValue;

        for (i = 0; i < 2; i++)
        {
            if (*pszNext == 0)
                break;

            pszNext = PARAM_ParseArrayConfig(pszNext,
                                            szBuffer,
                                            sizeof(szBuffer) / sizeof(szBuffer[0]));

            if (pszNext == NULL)
                break;

            if (stricmp(szBuffer, "disable") == 0)
            {
                g_rgfHwLimitUsed[nAxis][i] = 0;
            }
            else
            {
                nType = PARAM_ConvNum(szBuffer, NULL, &dbValue);

                if (nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT)
                {
                    VERBOSE_ERROR("Invalid HW limit : '%s=%s'\n",
                             pszKey, pszValue);
                    SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
                    return RESULT_ERROR;
                }

                g_rgfHwLimitUsed[nAxis][i] = 1;
                g_rgdbHwLimit[nAxis][i] = dbValue;
            }
        }
    }
    ///////////////////////////////////////////////////////
    // SW_LIMIT = -3670, disable
    // SW_LIMIT = -3540, 6570
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_SW_LIMIT) == 0)
    {
        pszNext = pszValue;

        for (i = 0; i < 2; i++)
        {
            if (*pszNext == 0)
                break;

            pszNext = PARAM_ParseArrayConfig(pszNext,
                                            szBuffer,
                                            sizeof(szBuffer) / sizeof(szBuffer[0]));

            if (pszNext == NULL)
                break;

            if (stricmp(szBuffer, "disable") == 0)
            {
                g_rgfSwLimitUsed[nAxis][i] = 0;
            }
            else
            {
                nType = PARAM_ConvNum(szBuffer, NULL, &dbValue);

                if (nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT)
                {
                    VERBOSE_ERROR("Invalid SW limit : '%s=%s'\n",
                             pszKey, pszValue);
                    SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
                    return RESULT_ERROR;
                }

                g_rgfSwLimitUsed[nAxis][i] = 1;
                g_rgdbSwLimit[nAxis][i] = dbValue;
            }
        }
    }
    ///////////////////////////////////////////////////////
    // GEAR_RATIO = 33.33
    // GEAR_RATIO = 100, 3      # 100 / 3
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_GEAR_RATIO) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdbVal, 2);

        if (nCount == 1)
        {
            if (rgdbVal[0] < 1.e-8)
            {
                VERBOSE_ERROR("Invalid gear ratio specified : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
                return RESULT_ERROR;
            }

            g_rgdbGearRatio[nAxis] = rgdbVal[0];
        }
        else if (nCount == 2)
        {
            if (rgdbVal[0] < 1.e-8 || rgdbVal[1] < 1.e-8)
            {
                VERBOSE_ERROR("The denominator is too samll : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
                return RESULT_ERROR;
            }

            g_rgdbGearRatio[nAxis] = rgdbVal[0] / rgdbVal[1];
        }
        else
        {
            VERBOSE_ERROR("Invalid gear ratio specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
            return RESULT_ERROR;
        }
    }
    ///////////////////////////////////////////////////////
    // TERM_DIST = 10
    // TERM_DIST = 50, PI       # 50 * 3.141592...
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_TERM_DIST) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdbVal, 2);

        if (nCount == 1)
        {
            g_rgdbRotaionDist[nAxis] = rgdbVal[0];
        }
        else if (nCount == 2)
        {
            g_rgdbRotaionDist[nAxis] = rgdbVal[0] * rgdbVal[1];
        }
        else
        {
            VERBOSE_ERROR("Invalid gear ratio specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
            return RESULT_ERROR;
        }
    }
    ///////////////////////////////////////////////////////
    // ENCODER_ORIGIN = INC, disable, 100 (type, reset value, home value)
    else if (stricmp(pszKey, SYSCONF_KEY_MOTOR_ENCODER_ORIGIN) == 0)
    {
        pszNext = pszValue;

        pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);
                
        if (pszNext != NULL)
        {
            if (stricmp(szBuffer, "disable") == 0)
            {
                g_rgnEcnoderHomeVal[nAxis] = INT_MIN;
            }
            else
            {
                nType = PARAM_ConvNum(szBuffer, &nValue, NULL);

                if (nType != CONFIG_NUM_INT || nValue == INT_MIN)
                {
                    VERBOSE_ERROR("Invalid encoder home value is specified : '%s=%s'\n",
                                  pszKey, pszValue);
                    SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
                    return RESULT_ERROR;
                }

                g_rgnEcnoderHomeVal[nAxis] = nValue;
            }
        }
    }
    ///////////////////////////////////////////////////////
    // DIRECTION = 1 (or -1)
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_DIRECTION) == 0)
    {
        nType = PARAM_ConvNum(pszValue, NULL, &dbValue);

        if ((nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT) ||
            (dbValue != 1 && dbValue != -1))
        {
            VERBOSE_ERROR("Invalid axis direction specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
            return RESULT_ERROR;
        }

        g_rgnAxisDirection[nAxis] = (int) dbValue;
    }
    ///////////////////////////////////////////////////////
    // MOTOR_CNT = 1 (or more)
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_MOTOR_COUNT) == 0)
    {
        nType = PARAM_ConvNum(pszValue, NULL, &dbValue);

        if ((nType != CONFIG_NUM_INT && nType != CONFIG_NUM_FLOAT) ||
            dbValue <= 0)
        {
            VERBOSE_ERROR("Invalid motor count specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
            return RESULT_ERROR;
        }

        g_nMotorCount[nAxis] = (int) dbValue;
    }

    else
    {
        VERBOSE_ERROR("unknown axis configuration specified : %s\n", pszKey);
        SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_AXIS_PARAM);
        return RESULT_ERROR;
    }
    
    return 0;
}