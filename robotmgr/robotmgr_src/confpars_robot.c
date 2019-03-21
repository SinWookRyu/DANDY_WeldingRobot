/////////////////////////////////////////////////////////////////////////////
//
//  confpars_robot.c: Robot Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"

int g_nLeftVertSkipBvar     = 0;
int g_nRightVertSkipBvar    = 0;
int g_nLeftCollarSkipBvar   = 0;
int g_nRightCollarSkipBvar  = 0;
int g_nHorizontalSkipBvar   = 0;
int g_nLeftBracketSkipBvar  = 0;
int g_nRightBracketSkipBvar = 0;

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigRobot()
//

int SYSC_LoadConfigRobot(int nRobot, const char* pszKey, const char* pszValue)
{
    int rgn[32];
    double rgdb[32];
    double rgdbVal[2];
    int i, nType;
    int nVal, nCount;

    char szBuffer[256];
    const char* pszNext;

    DANDY_ASSERT(nRobot >= 0 && nRobot < MAX_ROBOT_COUNT);

    ///////////////////////////////////////////////////////
    // name : 'NAME = The name of the robot'
    if (stricmp(pszKey, SYSCONF_KEY_ROBOT_NAME) == 0)
    {
        CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, pszValue);
        g_rgszRobotName[nRobot][ROBOT_NAME_LEN-1] = 0;
    }
    ///////////////////////////////////////////////////////
    // type : 'TYPE = DR6'
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_TYPE) == 0)
    {
        if (stricmp(pszValue, "void") == 0 ||
            stricmp(pszValue, "VOID") == 0 ||
            stricmp(pszValue, "no axis") == 0 ||
            stricmp(pszValue, "EXCEPT") == 0 ||
            stricmp(pszValue, "except") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_EXCEPT;
        }
        else if (stricmp(pszValue, "CART") == 0 ||
            stricmp(pszValue, "CARTESIAN") == 0 ||
            stricmp(pszValue, "RECT") == 0 ||
            stricmp(pszValue, "RECTANGLE") == 0 ||
            stricmp(pszValue, "RECTANGULAR") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_RECTANGULAR;
        }
        else if (stricmp(pszValue, "10KG") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_10KG;
        }
        else if (stricmp(pszValue, "DR6") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_DR6;
        }
        else if (stricmp(pszValue, "DANDY") == 0 ||
                 stricmp(pszValue, "DANDY-I") == 0 ||
                 stricmp(pszValue, "DANDY-1") == 0 ||
                 stricmp(pszValue, "DANDY-i") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_DANDY;
        }
        else if (stricmp(pszValue, "DANDY-II") == 0 ||
                 stricmp(pszValue, "DANDY-2") == 0 ||
                 stricmp(pszValue, "DANDY-ii") == 0)
        {
            g_rgnRobotType[nRobot] = ROBTYPE_DANDY_II;
        }
        else
        {
            g_rgnRobotType[nRobot] = ROBTYPE_NONE;
            VERBOSE_ERROR("unknown Robot Type specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }
    }
    ///////////////////////////////////////////////////////
    // COMM_PORT = RS485, com5
    // COMM_PORT = RS485, /dev/ser5
    else if (stricmp(pszKey, SYSCONF_KEY_AXIS_COMM_PORT) == 0)
    {
        pszNext = pszValue;

        pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);

        if (pszNext == NULL)
        {
            VERBOSE_ERROR("Invalid syntax : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(szBuffer, "RS232") == 0)
        {
            g_rgnCommProtocol[nRobot] = ROBOT_COMM_RS232;
        }
        else if (stricmp(szBuffer, "RS485") == 0)
        {
            g_rgnCommProtocol[nRobot] = ROBOT_COMM_RS485;
        }
        else
        {
            VERBOSE_ERROR("Invalid comm protocol specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        // com port
        pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);

        if (pszNext == NULL)
        {
            VERBOSE_ERROR("Invalid syntax : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (strlen(szBuffer) >= ROBOT_COMM_PORT_LEN)
        {
            VERBOSE_ERROR("comm port is too long : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        CRT_strncpy(g_rgszCommPort[nRobot], sizeof(g_rgszCommPort[nRobot]) + 1,
                    szBuffer, ROBOT_COMM_PORT_LEN);
        g_rgszCommPort[nRobot][ROBOT_COMM_PORT_LEN-1] = 0;
    }
    ///////////////////////////////////////////////////////
    // axis : AXES = 0, 1, 2, 3, 4, 5, ...
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_AXES) == 0)
    {
        nCount = PARAM_ConvArrayNumInt(pszValue, rgn, ROB_AXIS_COUNT);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Axis was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }
        
        if (nCount > ROB_AXIS_COUNT)
        {
            VERBOSE_ERROR("Too many axes were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        for (i = 0; i < nCount; i++)
        {
            if (rgn[i] < 0 || rgn[i] >= ROB_AXIS_COUNT)
            {
                VERBOSE_ERROR("Invalid Axes was specified : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
                return RESULT_ERROR;
            }

            g_rgRobotAxis[nRobot][i].nMotorCount = rgn[i];
        }

        g_rgnRobotAxisCount[nRobot] = nCount;
    }
    ///////////////////////////////////////////////////////
    // link param : 'LINK_th = th0, th1, th2, ...'      <- theta
    //              'LINK_d  = d0,  d1,  d2, ...'       <- d
    //              'LINK_al = al0, al1, al2, ...'      <- alpha
    //              'LINK_l  = l0,  l1,  l2, ...'       <- l
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_th) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_d) == 0  ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_al) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_l) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, ROB_AXIS_COUNT);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid parameter was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > ROB_AXIS_COUNT)
        {
            VERBOSE_ERROR("Too many parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        for (i = 0; i < ROB_AXIS_COUNT; i++)
        {
            if (stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_th) == 0)
                g_rgRobotDHParam[nRobot][i].th = rgdb[i];
            else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_d) == 0)
                g_rgRobotDHParam[nRobot][i].d = rgdb[i];
            else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_al) == 0)
                g_rgRobotDHParam[nRobot][i].al = rgdb[i];
            else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_LINK_l) == 0)
                g_rgRobotDHParam[nRobot][i].l = rgdb[i];
            else
            {
                DANDY_ASSERT(0);
                g_rgRobotDHParam[nRobot]->l = rgdb[i];
            }
        }
    }
    ///////////////////////////////////////////////////////
    // JOG_SPEED = 10
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_JOG_SPEED) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nVal, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid jog percentage was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_rgnRobotDefJogPercent[nRobot] = nVal;
    }
    ///////////////////////////////////////////////////////
    // EXJOG_SPEED = 10
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_EXJOG_SPEED) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nVal, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 1000)
        {
            VERBOSE_ERROR("Invalid exjog percentage was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_rgnRobotDefExtraJogPercent[nRobot] = nVal;
    }
    ///////////////////////////////////////////////////////
    // MAX_JOINT_SPEED = 0.72, 0.72, 0.72, 0.72, 0.72, 0.72, ...
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_MAX_JOINT_SPEED) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, ROB_AXIS_COUNT);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Max Joint Speed was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }
        
        if (nCount > ROB_AXIS_COUNT)
        {
            VERBOSE_ERROR("Too many max joint speed were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        for (i = 0; i < nCount; i++)
        {
            if (rgdb[i] < 0 || rgdb[i] >= ROB_AXIS_COUNT)
            {
                VERBOSE_ERROR("Invalid Max Joint Speed was specified : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
                return RESULT_ERROR;
            }
        }

        for (i = 0; i < nCount; i++)
        {
            g_rgRobotMotion[nRobot].dbMaxJointSpeed[i] = rgdb[i];
        }
    }
    ///////////////////////////////////////////////////////
    // 'MAX_SPEED = <LinearSpeed>, <OrientaionSpeed>'
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_MAX_SPEED) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 2);

        if (nCount < 2 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid max speed was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 2)
        {
            VERBOSE_ERROR("Too many max speed were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_rgRobotMotion[nRobot].dbMaxLinearSpeed = rgdb[0];
        g_rgRobotMotion[nRobot].dbMaxOrientSpeed = rgdb[1];
    }
    ///////////////////////////////////////////////////////
    // xxxxx      = time
    // ACCEL      = 400
    // STOP       = 300
    // ERROR_STOP = 
    // ESTOP      =
    // TSTOP      = 
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_ACCEL) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_DECEL) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_ERROR_STOP) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_ESTOP) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_TSTOP) == 0)
    {
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdbVal, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid robot motion parameter was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 5)
        {
            VERBOSE_ERROR("Too many robot motion parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_ACCEL) == 0)
        {
            g_rgRobotMotion[nRobot].dbAccel = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_DECEL) == 0)
        {
            g_rgRobotMotion[nRobot].dbDecel = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_ERROR_STOP) == 0)
        {
            g_rgRobotMotion[nRobot].dbDecel_Error = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_ESTOP) == 0)
        {
            g_rgRobotMotion[nRobot].dbDecel_Estop = rgdbVal[0];
        }
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_TSTOP) == 0)
        {
            g_rgRobotMotion[nRobot].dbDecel_Touch = rgdbVal[0];
        }
    }
    ///////////////////////////////////////////////////////
    // JERK  =
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_JERK) == 0)
    {
        static double s_dbJerkTime;
        
        nCount = PARAM_ConvArrayNumFloat(pszValue, &s_dbJerkTime, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Jerk time is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (s_dbJerkTime < 0 ||
            s_dbJerkTime > g_rgRobotMotion[nRobot].dbAccel)
        {
            VERBOSE_ERROR("Jerk time is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_rgRobotMotion[nRobot].dbJerk = s_dbJerkTime;
    }
    ///////////////////////////////////////////////////////
    // HOME_SPEED = 30
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_HOME_SPEED) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nVal, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nVal < 1 || nVal > 100)
        {
            VERBOSE_ERROR("Invalid home speed was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_nHomeSpeed[nRobot] = nVal;
    }
    ///////////////////////////////////////////////////////
    // WELDERS = 0, 1, 2
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_WELDERS) == 0)
    {
        nCount = PARAM_ConvArrayNumInt(pszValue, rgn, MAX_WELDER_COUNT);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid welder list was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }
        
        if (nCount > MAX_WELDER_COUNT)
        {
            VERBOSE_ERROR("Too many welders were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        for (i = 0; i < nCount; i++)
        {
            if (rgn[i] < 0 || rgn[i] >= MAX_WELDER_COUNT)
            {
                VERBOSE_ERROR("Invalid welder was specified : '%s=%s'\n",
                                 pszKey, pszValue);
                SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
                return RESULT_ERROR;
            }

            g_rgnRobotWelderList[nRobot][i] = rgn[i];
        }

        g_rgnRobotWelderCount[nRobot] = nCount;
    }
    ///////////////////////////////////////////////////////
    // CMD_SIZE = 8192
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_CMD_SIZE) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nCount < MIN_CMD_COUNT || nCount > MAX_CMD_COUNT)
        {
            VERBOSE_ERROR("Invalid code count was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_rgdwCmdSize[nRobot] = nCount;
    }
    ///////////////////////////////////////////////////////
    // TVA_SIZE = 256
    // PVA_SIZE = 1024
    // BVA_SIZE = 1024
    // IVA_SIZE = 1024
    // RVA_SIZE = 1024
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_TVA_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_PVA_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_BVA_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_IVA_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_RVA_SIZE) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nCount < 0 || nCount > MAX_CMD_COUNT)
        {
            VERBOSE_ERROR("Invalid the number was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        //nCount++;   // subscript 0 is used for internal
                    // so it is needed one more

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_TVA_SIZE) == 0)
            g_rgdwTVarSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_PVA_SIZE) == 0)
            g_rgdwPVarSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_BVA_SIZE) == 0)
            g_rgdwBVarSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_IVA_SIZE) == 0)
            g_rgdwIVarSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_RVA_SIZE) == 0)
            g_rgdwRVarSize[nRobot] = nCount;
        else
            DANDY_ASSERT(0);
    }
    ///////////////////////////////////////////////////////
    // CWEAV_TOUCHUPDIS  = 5
    else if (stricmp(pszKey, SYSCONF_KEY_CWEAV_TOUCHUPDIS) == 0)
    {
        static double s_dbCWeavTouchUpDis;
        
        nCount = PARAM_ConvArrayNumFloat(pszValue, &s_dbCWeavTouchUpDis, 1);

        if (nCount < 0 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("CWeav Touch Up Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (s_dbCWeavTouchUpDis < 0)
        {
            VERBOSE_ERROR("CWeav Touch Up Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_Weld_Function[nRobot].dbCWeavTouchUpDis = s_dbCWeavTouchUpDis;
    }
    ///////////////////////////////////////////////////////
    // CWEAV_HORMARGIN  = 12
    else if (stricmp(pszKey, SYSCONF_KEY_CWEAV_HORMARGIN) == 0)
    {
        static double s_dbCWeavHorMargin;
        
        nCount = PARAM_ConvArrayNumFloat(pszValue, &s_dbCWeavHorMargin, 1);

        if (nCount < 0 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("CWeav Horizontal Margin Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (s_dbCWeavHorMargin < 0)
        {
            VERBOSE_ERROR("CWeavHorizontal Margin Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_Weld_Function[nRobot].dbCWeavHorMargin = s_dbCWeavHorMargin;
    }
    ///////////////////////////////////////////////////////
    // CWEAV_WELDLEGDIS  = 3
    else if (stricmp(pszKey, SYSCONF_KEY_CWEAV_WELDLEGDIS) == 0)
    {
        static double s_dbCWeavWeldLegDis;
        
        nCount = PARAM_ConvArrayNumFloat(pszValue, &s_dbCWeavWeldLegDis, 1);

        if (nCount < 0 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("CWeav Weld Leg Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (s_dbCWeavWeldLegDis < 0)
        {
            VERBOSE_ERROR("CWeav Weld Leg Distance is invalid : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_Weld_Function[nRobot].dbCWeavWeldLegDis = s_dbCWeavWeldLegDis;
    }
    ///////////////////////////////////////////////////////
    // GAP_REF_BVAR_USED = 1 or 0
    else if (stricmp(pszKey, SYSCONF_KEY_GAP_REF_BVAR_USED) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || (nCount != 1 && nCount != 0))
        {
            VERBOSE_ERROR("Invalid Gap Ref Var Used was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        g_Weld_Function[nRobot].fGapRefVarUsed = nCount;
    }
    ///////////////////////////////////////////////////////
    // GAP_REF_BVAR    = 4
    // LEFT_WELD_BVAR  = 10
    // RIGHT_WELD_BVAR = 20
    else if (stricmp(pszKey, SYSCONF_KEY_GAP_REF_BVAR)    == 0 ||
             stricmp(pszKey, SYSCONF_KEY_LEFT_WELD_BVAR)  == 0 ||
             stricmp(pszKey, SYSCONF_KEY_RIGHT_WELD_BVAR) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nCount > (int) g_rgdwBVarSize[nRobot])
        {
            VERBOSE_ERROR("Invalid the number was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_GAP_REF_BVAR) == 0)
        {
            g_Weld_Function[nRobot].nGapRefBvar    = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                 SYSCONF_KEY_GAP_REF_BVAR, pszKey, pszValue);
                g_Weld_Function[nRobot].fGapRefVarUsed = 0;
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_LEFT_WELD_BVAR) == 0)
        {
            g_Weld_Function[nRobot].nLeftWeldBvar  = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                 SYSCONF_KEY_LEFT_WELD_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_RIGHT_WELD_BVAR) == 0)
        {
            g_Weld_Function[nRobot].nRightWeldBvar  = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                 SYSCONF_KEY_RIGHT_WELD_BVAR, pszKey, pszValue);
            }
        }
        else
        {
            DANDY_ASSERT(0);
        }
    }
    ///////////////////////////////////////////////////////
    // LEFT_VERT_SKIP_REF_BVAR     = 10
    // RIGHT_VERT_SKIP_REF_BVAR    = 20
    // LEFT_COLLAR_SKIP_REF_BVAR   = 30
    // RIGHT_COLLAR_SKIP_REF_BVAR  = 40
    // HORIZONTAL_SKIP_REF_BVAR    = 50
    // LEFT_BRACKET_SKIP_REF_BVAR  = 70
    // RIGHT_BRACKET_SKIP_REF_BVAR = 80
    else if (stricmp(pszKey, SYSCONF_KEY_LEFT_VERT_SKIP_REF_BVAR)     == 0 ||
             stricmp(pszKey, SYSCONF_KEY_RIGHT_VERT_SKIP_REF_BVAR)    == 0 ||
             stricmp(pszKey, SYSCONF_KEY_LEFT_COLLAR_SKIP_REF_BVAR)   == 0 ||
             stricmp(pszKey, SYSCONF_KEY_RIGHT_COLLAR_SKIP_REF_BVAR)  == 0 ||
             stricmp(pszKey, SYSCONF_KEY_HORIZONTAL_SKIP_REF_BVAR)    == 0 ||
             stricmp(pszKey, SYSCONF_KEY_LEFT_BRACKET_SKIP_REF_BVAR)  == 0 ||
             stricmp(pszKey, SYSCONF_KEY_RIGHT_BRACKET_SKIP_REF_BVAR) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nCount > (int) g_rgdwBVarSize[nRobot])
        {
            VERBOSE_ERROR("Invalid the Skip Ref Bvar was specified : '%s=%s'\n",
                          pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_LEFT_VERT_SKIP_REF_BVAR) == 0)
        {
            g_nLeftVertSkipBvar  = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_LEFT_VERT_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_RIGHT_VERT_SKIP_REF_BVAR) == 0)
        {
            g_nRightVertSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_RIGHT_VERT_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_LEFT_COLLAR_SKIP_REF_BVAR) == 0)
        {
            g_nLeftCollarSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_LEFT_COLLAR_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_RIGHT_COLLAR_SKIP_REF_BVAR) == 0)
        {
            g_nRightCollarSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_RIGHT_COLLAR_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_HORIZONTAL_SKIP_REF_BVAR) == 0)
        {
            g_nHorizontalSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_HORIZONTAL_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_LEFT_BRACKET_SKIP_REF_BVAR) == 0)
        {
            g_nLeftBracketSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_LEFT_BRACKET_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else if (stricmp(pszKey, SYSCONF_KEY_RIGHT_BRACKET_SKIP_REF_BVAR) == 0)
        {
            g_nRightBracketSkipBvar = nCount;
            if (nCount < 0)
            {
                VERBOSE_WARNING("'%s' is set to 'no use' : '%s=%s'\n",
                                SYSCONF_KEY_RIGHT_BRACKET_SKIP_REF_BVAR, pszKey, pszValue);
            }
        }
        else
        {
            DANDY_ASSERT(0);
        }
    }
    ///////////////////////////////////////////////////////
    // WVF_SIZE = 32
    // SWF_SIZE = 128
    // MWF_SIZE = 128
    // EWF_SIZE = 128
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_WVF_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_SWF_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_MWF_SIZE) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_EWF_SIZE) == 0)
    {
        nType = PARAM_ConvArrayOneNum(pszValue, &nCount, NULL, NULL);

        if (nType != CONFIG_NUM_INT || nCount < 0 || nCount > MAX_CMD_COUNT)
        {
            VERBOSE_ERROR("Invalid the number was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
            return RESULT_ERROR;
        }

        //nCount++;   // subscript 0 is used for internal
                    // so it is needed one more

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_WVF_SIZE) == 0)
            g_rgdwWeaveSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_SWF_SIZE) == 0)
            g_rgdwSWFSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_MWF_SIZE) == 0)
            g_rgdwMWFSize[nRobot] = nCount;
        else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_EWF_SIZE) == 0)
            g_rgdwEWFSize[nRobot] = nCount;
        else
            DANDY_ASSERT(0);
    }

    else
    {
        VERBOSE_ERROR("unknown robot configuration specified : %s\n", pszKey);
        SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_ROBOT_PARAM);
        return RESULT_ERROR;
    }
    
    return 0;
}