/////////////////////////////////////////////////////////////////////////////
//
//  confpars_welder.c: Welder Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"


double g_dbVoltRealTimeCmdOffsetUnit = 0;
double g_dbCurrRealTimeCmdOffsetUnit = 0;

double g_dbUpperBoundY_Volt = 0;
double g_dbLowerBoundY_Volt = 0;
double g_dbUpperBoundX_Volt = 0;
double g_dbLowerBoundX_Volt = 0;
double g_dbUpperBoundY_Curr = 0;
double g_dbLowerBoundY_Curr = 0;
double g_dbUpperBoundX_Curr = 0;
double g_dbLowerBoundX_Curr = 0;

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigWelder()
//

int SYSC_LoadConfigWelder(int nWelder, const char* pszKey, const char* pszValue)
{
    static const char* rgpszInportKey[MAX_WELD_DIN_PORT_COUNT] = {
            "Arc",          // 0
            "NoGas",        // 1
            "NoWire",       // 2
            "PowerFail",    // 3
            "TouchProc",    // 4
            "TouchSig",     // 5
            "Weld",         // 6
            "ArcFail"};     // 7
    static const char* rgpszOutportKey[MAX_WELD_DOUT_PORT_COUNT] = {
            "ArcOn",        // 0
            "GasOn",        // 1
            "WireF",        // 2
            "WireB",        // 3
            "TouchStart",   // 4
            "MCTouchMode",  // 5
            "WireCut",      // 6
            "WeldPwr"};     // 7

    int i, nLen;
    int nValue;
    double rgdb[32];
    int nCount;

    char szBuffer[256];
    const char* pszNext;
    const char* pszSubValue;

    DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);

    ///////////////////////////////////////////////////////
    // name : 'NAME = The name of the welder'
    if (stricmp(pszKey, SYSCONF_KEY_WELDER_NAME) == 0)
    {
        CRT_strncpy(g_rgWelderConfig[nWelder].szName,
                    sizeof(g_rgWelderConfig[nWelder].szName) + 1,
                    pszValue,
                    WELDER_NAME_LEN);
        g_rgWelderConfig[nWelder].szName[WELDER_NAME_LEN-1] = 0;
    }
    ///////////////////////////////////////////////////////
    // 'TYPE = Hyosung_UR501R'
    // 'TYPE = Daihen_DM500'
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_TYPE) == 0)
    {
        if (strnicmp(pszValue, "Hyosung_UR", 10) == 0)
        {
            g_rgWelderConfig[nWelder].nType = WELDER_TYPE_HYOSUNG_UR;
        }
        else if (strnicmp(pszValue, "Daihen_DM", 9) == 0)
        {
            g_rgWelderConfig[nWelder].nType = WELDER_TYPE_DAIHEN_DM;
        }
        else
        {
            g_rgWelderConfig[nWelder].nType = WELDER_TYPE_NONE;

            VERBOSE_WARNING("Unknown welder type specified : '%s'\n",
                               pszKey);
        }
    }
    ///////////////////////////////////////////////////////
    // 'ABILITY = co2, mig, mag, tig'
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_ABILITY) == 0)
    {
        int nAbility;

        nAbility = 0;

        pszNext = pszValue;
        while (pszNext != NULL && *pszNext)
        {
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, sizeof(szBuffer));

            if (stricmp(szBuffer, "co2") == 0)
                nAbility |= WELDER_ABIL_CO2;
            else if (stricmp(szBuffer, "mig") == 0)
                nAbility |= WELDER_ABIL_MIG;
            else if (stricmp(szBuffer, "mag") == 0)
                nAbility |= WELDER_ABIL_MAG;
            else if (stricmp(szBuffer, "tig") == 0)
                nAbility |= WELDER_ABIL_TIG;
            else
            {
                VERBOSE_ERROR("Invalid welder ability specified (co2/mig/mag/tig) : '%s'\n",
                                 szBuffer);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }
        }

        g_rgWelderConfig[nWelder].nAbility = nAbility;
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_INPORT_NO) == 0)
    {
        // abstract inport number
        pszNext = PARAM_ParseArrayConfig(pszValue, szBuffer, sizeof(szBuffer));

        if (pszNext == NULL)
        {
            VERBOSE_ERROR("Invalid welder inport specified : '%s'\n",
                             pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (PARAM_ConvNum(szBuffer, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= MAX_WELDER_COUNT)
        {
            VERBOSE_ERROR("Invalid welder inport port number : '%s'\n",
                             szBuffer);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nDinSlaveNo = nValue;

        while (pszNext != NULL && *pszNext != 0)
        {
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, sizeof(szBuffer));

            if ((pszSubValue=strchr(szBuffer, ':')) == NULL ||
                pszSubValue == szBuffer)
            {
               VERBOSE_ERROR("Invalid welder inport arguments : '%s'\n",
                                 szBuffer);
               SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }

            nLen = pszSubValue - szBuffer;
            pszSubValue++;     // skip ':'

            for (i = 0; i < MAX_WELD_DIN_PORT_COUNT; i++)
            {
                if (strlen(rgpszInportKey[i]) == (unsigned)nLen &&
                    strnicmp(rgpszInportKey[i], szBuffer, nLen) == 0)
                {
                    break;
                }
            }

            if (i >= MAX_WELD_DIN_PORT_COUNT)
            {
                VERBOSE_ERROR("Invalid welder inport sub-item '%s'\n",
                                 szBuffer);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }

            if (PARAM_ConvNum(pszSubValue, &nValue, NULL) != CONFIG_NUM_INT ||
                nValue < 0 || nValue >= 32)
            {
                VERBOSE_ERROR("Invalid welder inport sub-item value '%s:%s'\n",
                                 rgpszInportKey[i], pszSubValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
             }

            g_rgWelderConfig[nWelder].rgndin_portno[i] = nValue;
        }
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_INPORT_LEVEL) == 0)
    {
        pszNext = pszValue;

        while (pszNext != NULL && *pszNext != 0)
        {
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, sizeof(szBuffer));

            if ((pszSubValue=strchr(szBuffer, ':')) == NULL ||
                pszSubValue == szBuffer)
            {
               VERBOSE_ERROR("Invalid welder inport level : '%s'\n",
                                 szBuffer);
               SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
               return RESULT_ERROR;
            }

            nLen = pszSubValue - szBuffer;
            pszSubValue++;     // skip ':'

            for (i = 0; i < MAX_WELD_DIN_PORT_COUNT; i++)
            {
                if (strlen(rgpszInportKey[i]) == (unsigned)nLen &&
                    strnicmp(rgpszInportKey[i], szBuffer, nLen) == 0)
                {
                    break;
                }
            }

            if (i >= MAX_WELD_DIN_PORT_COUNT)
            {
                VERBOSE_ERROR("Invalid welder inport sub-item '%s'\n",
                                 szBuffer);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }

            if (PARAM_ConvNum(pszSubValue, &nValue, NULL) != CONFIG_NUM_INT ||
                (nValue != 0 && nValue != 1))
            {
                VERBOSE_ERROR("Invalid welder inport sub-item value '%s:%s'\n",
                                 rgpszInportKey[i], pszSubValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
             }

            g_rgWelderConfig[nWelder].rgndin_actlev[i] = nValue;
        }
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_OUTPORT_NO) == 0)
    {
        // abstract outport number
        pszNext = PARAM_ParseArrayConfig(pszValue, szBuffer, sizeof(szBuffer));

        if (pszNext == NULL)
        {
            VERBOSE_ERROR("Invalid welder outport specified : '%s'\n",
                             pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (PARAM_ConvNum(szBuffer, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= MAX_WELDER_COUNT)
        {
            VERBOSE_ERROR("Invalid welder outport port number : '%s'\n",
                             szBuffer);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nDoutSlaveNo = nValue;

        while (pszNext != NULL && *pszNext != 0)
        {
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, sizeof(szBuffer));

            if ((pszSubValue=strchr(szBuffer, ':')) == NULL ||
                pszSubValue == szBuffer)
            {
               VERBOSE_ERROR("Invalid welder outport arguments : '%s'\n",
                                szBuffer);
               SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
               return RESULT_ERROR;
            }

            nLen = pszSubValue - szBuffer;
            pszSubValue++;     // skip ':'

            for (i = 0; i < MAX_WELD_DOUT_PORT_COUNT; i++)
            {
                if (strlen(rgpszOutportKey[i]) == (unsigned)nLen &&
                    strnicmp(rgpszOutportKey[i], szBuffer, nLen) == 0)
                {
                    break;
                }
            }

            if (i >= MAX_WELD_DOUT_PORT_COUNT)
            {
                VERBOSE_ERROR("Invalid welder outport sub-item '%s'\n",
                                 szBuffer);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }

            if (PARAM_ConvNum(pszSubValue, &nValue, NULL) != CONFIG_NUM_INT ||
                nValue < 0 || nValue >= 32)
            {
                VERBOSE_ERROR("Invalid welder outport sub-item value '%s:%s'\n",
                                 rgpszOutportKey[i], pszSubValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
             }

            g_rgWelderConfig[nWelder].rgndout_portno[i] = nValue;
        }
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_OUTPORT_LEVEL) == 0)
    {
        pszNext = pszValue;

        while (pszNext != NULL && *pszNext != 0)
        {
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, sizeof(szBuffer));

            if ((pszSubValue=strchr(szBuffer, ':')) == NULL ||
                pszSubValue == szBuffer)
            {
               VERBOSE_ERROR("Invalid welder outport level : '%s'\n",
                                szBuffer);
               SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
               SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);return RESULT_ERROR;
            }

            nLen = pszSubValue - szBuffer;
            pszSubValue++;     // skip ':'

            for (i = 0; i < MAX_WELD_DOUT_PORT_COUNT; i++)
            {
                if (strlen(rgpszOutportKey[i]) == (unsigned)nLen &&
                    strnicmp(rgpszOutportKey[i], szBuffer, nLen) == 0)
                {
                    break;
                }
            }

            if (i >= MAX_WELD_DOUT_PORT_COUNT)
            {
                VERBOSE_ERROR("Invalid welder outport sub-item '%s'\n",
                                 szBuffer);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
            }
            
            if (PARAM_ConvNum(pszSubValue, &nValue, NULL) != CONFIG_NUM_INT ||
                (nValue != 0 && nValue != 1))
            {
                VERBOSE_ERROR("Invalid welder outport sub-item value '%s:%s'\n",
                                 rgpszOutportKey[i], pszSubValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
                return RESULT_ERROR;
             }

            g_rgWelderConfig[nWelder].rgndout_actlev[i] = nValue;
        }
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_VOLT_IN) == 0)
    {
        if (PARAM_ConvNum(pszValue, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= 32)
        {
            VERBOSE_ERROR("Invalid welder voltage input port value '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nVoltageInPortNo = nValue;
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_CURR_IN) == 0)
    {
        if (PARAM_ConvNum(pszValue, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= 32)
        {
            VERBOSE_ERROR("Invalid welder current input port value '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nCurrentInPortNo = nValue;
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_VOLT_OUT) == 0)
    {
        if (PARAM_ConvNum(pszValue, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= 32)
        {
            VERBOSE_ERROR("Invalid welder voltage output port value '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nVoltageOutPortNo = nValue;
    }
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_CURR_OUT) == 0)
    {
        if (PARAM_ConvNum(pszValue, &nValue, NULL) != CONFIG_NUM_INT ||
            nValue < 0 || nValue >= 32)
        {
            VERBOSE_ERROR("Invalid welder current output port value '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_rgWelderConfig[nWelder].nCurrentOutPortNo = nValue;
    }
    ///////////////////////////////////////////////////////
    // VOLT_OFFSET_UNIT = 0000.00
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_VOLT_OFFSET_UNIT) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid volt offset unit was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many volt offset unit were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of volt offset unit
        if (rgdb[0] < 0)
        {
            VERBOSE_ERROR("invalid volt offset unit range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbVoltRealTimeCmdOffsetUnit= rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // CURR_OFFSET_UNIT = 0000.00
    else if (stricmp(pszKey, SYSCONF_KEY_WELDER_CURR_OFFSET_UNIT) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid curr offset unit was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many curr offset unit were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of curr offset unit
        if (rgdb[0] < 0)
        {
            VERBOSE_ERROR("invalid curr offset unit range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbCurrRealTimeCmdOffsetUnit= rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // VOLT_CALIB_UPPER_Y = 38.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_VOLT_CALIB_UPPER_Y) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid volt calib upper Y was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many volt calib upper Y were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of volt calib upper Y
        if (rgdb[0] < 0 || rgdb[0] > HYOSUNG_WELD_MAX_VOLT_VAL)
        {
            VERBOSE_ERROR("invalid volt calib upper Y range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbUpperBoundY_Volt = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // VOLT_CALIB_LOWER_Y = 20.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_VOLT_CALIB_LOWER_Y) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid volt calib lower Y was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many volt calib lower Y were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of volt calib lower Y
        if (rgdb[0] < 0 || rgdb[0] > HYOSUNG_WELD_MAX_VOLT_VAL)
        {
            VERBOSE_ERROR("invalid volt calib lower Y range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbLowerBoundY_Volt = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // VOLT_CALIB_UPPER_X = 10.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_VOLT_CALIB_UPPER_X) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid volt calib upper X was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many volt calib upper X were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of volt calib upper X
        if (rgdb[0] < 0 || rgdb[0] > MAX_WELDOUT_VOLTAGE)
        {
            VERBOSE_ERROR("invalid volt calib upper X range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbUpperBoundX_Volt = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // VOLT_CALIB_LOWER_X = 0.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_VOLT_CALIB_LOWER_X) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid volt calib lower X was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many volt calib lower X were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of volt calib lower X
        if (rgdb[0] < 0 || rgdb[0] > MAX_WELDOUT_VOLTAGE)
        {
            VERBOSE_ERROR("invalid volt calib lower X range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbLowerBoundX_Volt = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // CURR_CALIB_UPPER_Y = 38.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_CURR_CALIB_UPPER_Y) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid curr calib upper Y was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many curr calib upper Y were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of curr calib upper Y
        if (rgdb[0] < 0 || rgdb[0] > HYOSUNG_WELD_MAX_CURR_VAL)
        {
            VERBOSE_ERROR("invalid curr calib upper Y range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbUpperBoundY_Curr = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // CURR_CALIB_LOWER_Y = 20.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_CURR_CALIB_LOWER_Y) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid curr calib lower Y was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many curr calib lower Y were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of curr calib lower Y
        if (rgdb[0] < 0 || rgdb[0] > HYOSUNG_WELD_MAX_CURR_VAL)
        {
            VERBOSE_ERROR("invalid curr calib lower Y range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbLowerBoundY_Curr = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // CURR_CALIB_UPPER_X = 10.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_CURR_CALIB_UPPER_X) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid curr calib upper X was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many curr calib upper X were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of curr calib upper X
        if (rgdb[0] < 0 || rgdb[0] > MAX_WELDOUT_VOLTAGE)
        {
            VERBOSE_ERROR("invalid curr calib upper X range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbUpperBoundX_Curr = rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // CURR_CALIB_LOWER_X = 0.0
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_CURR_CALIB_LOWER_X) == 0)
    {
        // 1 means no of element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid curr calib lower X was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many curr calib lower X were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        // check validation of curr calib lower X
        if (rgdb[0] < 0 || rgdb[0] > MAX_WELDOUT_VOLTAGE)
        {
            VERBOSE_ERROR("invalid curr calib lower X range\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
            return RESULT_ERROR;
        }

        g_dbLowerBoundX_Curr = rgdb[0];
    }

    else
    {
        VERBOSE_ERROR("unknown welder configuration specified : %s\n", pszKey);
        SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_WELDER_PARAM);
        return RESULT_ERROR;
    }

    return 0;
}