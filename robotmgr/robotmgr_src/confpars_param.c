/////////////////////////////////////////////////////////////////////////////
//
//  confpars_param.c: User Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigRobotParameter()
//

int SYSC_LoadConfigRobotParameter(int nRobot, const char* pszKey, const char* pszValue)
{
    double rgdb[32];
    int i;
    int nCount;
    int nUserCoordIdx;
    int nHomeIdx;
    int nVoltTuneInIdx, nVoltTuneOutIdx;
    int nCurrTuneInIdx, nCurrTuneOutIdx;
    int nWeldMeasureIdx;

    DANDY_ASSERT(nRobot >= 0 && nRobot < MAX_ROBOT_COUNT);

    ///////////////////////////////////////////////////////
    // JOB_NAME = xxxx.pgm or xxxx
    if (stricmp(pszKey, SYSCONF_KEY_LOADED_JOB_NAME) == 0)
    {
        if (g_pszLoadedJobFileName != NULL)
        {
            VERBOSE_ERROR("Job File Name is already specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        //g_pszLoadedJobFileName = (char *)DEBUG_MALLOC(strlen(pszValue)+1);
        g_pszLoadedJobFileName = (char *)DEBUG_MALLOC(JOB_MODULE_NAME_SIZE);

        if (g_pszLoadedJobFileName == NULL)
        {
            VERBOSE_ERROR("Memory allocation failed for Job File Name\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        CRT_strcpy(g_pszLoadedJobFileName, strlen(pszValue)+1, pszValue);
    }
    ///////////////////////////////////////////////////////
    // TCP      = x, y, z, roll, pitch, yaw
    // WORLD    = x, y, z, roll, pitch, yaw
    // CART     = x, y, z, roll, pitch, yaw
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_TCP) == 0   ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_WORLD) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_CART) == 0)
    {
        // 6 means no of euler parameters
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 6);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid coord information was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 6)
        {
            VERBOSE_ERROR("Too many coord information were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_TCP) == 0)
        {
            g_rgCoordInfo[nRobot].coordTool.x   = rgdb[0];
            g_rgCoordInfo[nRobot].coordTool.y   = rgdb[1];
            g_rgCoordInfo[nRobot].coordTool.z   = rgdb[2];
            g_rgCoordInfo[nRobot].coordTool.rol = rgdb[3];
            g_rgCoordInfo[nRobot].coordTool.pit = rgdb[4];
            g_rgCoordInfo[nRobot].coordTool.yaw = rgdb[5];
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_WORLD) == 0)
        {
            g_rgCoordInfo[nRobot].world.x   = rgdb[0];
            g_rgCoordInfo[nRobot].world.y   = rgdb[1];
            g_rgCoordInfo[nRobot].world.z   = rgdb[2];
            g_rgCoordInfo[nRobot].world.rol = rgdb[3];
            g_rgCoordInfo[nRobot].world.pit = rgdb[4];
            g_rgCoordInfo[nRobot].world.yaw = rgdb[5];
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_CART) == 0)
        {
            g_CartOffset.x   = rgdb[0];
            g_CartOffset.y   = rgdb[1];
            g_CartOffset.z   = rgdb[2];
            g_CartOffset.rol = rgdb[3];
            g_CartOffset.pit = rgdb[4];
            g_CartOffset.yaw = rgdb[5];
        }
    }
    ///////////////////////////////////////////////////////
    // USER     = Index(0~127)> x, y, z, roll, pitch, yaw
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_USER) == 0)
    {
        // 6 means no of euler parameters + 1 User Coordinate Parameter Index
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 7);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid user coord information was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 7)
        {
            VERBOSE_ERROR("Too many user coord information were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_USER) == 0)
        {
            nUserCoordIdx = (int) rgdb[0];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].x   = rgdb[1];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].y   = rgdb[2];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].z   = rgdb[3];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].rol = rgdb[4];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].pit = rgdb[5];
            g_rgCoordInfo[nRobot].user[nUserCoordIdx].yaw = rgdb[6];
        }
    }
    ///////////////////////////////////////////////////////
    // HOME     = Index(0~31)> joint1, joint2, joint3, joint4, joint5, joint6
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_HOME) == 0)
    {
        // 6 means no of euler parameters + 1 User Coordinate Parameter Index
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 7);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid home position information was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 7)
        {
            VERBOSE_ERROR("Too many home position information were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_ROBOT_COORD_HOME) == 0)
        {
            nHomeIdx = (int) rgdb[0];
            for (i = 1; i < nCount; i++)
            {
                g_rgdbHomePosVal[nHomeIdx][i-1] = rgdb[i];  // Unit: deg
            }
        }
    }
    ///////////////////////////////////////////////////////
    // TUNE_APPLY_IDX = IN, OUT
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_INDEX) == 0)
    {
        // 2 means no of tuning input parameters
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 2);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Weld Tuning Parameter Index was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 2)
        {
            VERBOSE_ERROR("Too many Weld Tuning Parameter Index were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        g_nWeldTuneInParamApplyIndex  = (int) rgdb[0];
        g_nWeldTuneOutParamApplyIndex = (int) rgdb[1];
    }
    ///////////////////////////////////////////////////////
    // TUNE_VOLT_IN = Idx> Volt_a, Volt_b, Volt_Scale, Volt_Offset
    // TUNE_CURR_IN = Idx> Curr_a, Curr_b, Curr_Scale, Curr_Offset
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_VOLT_IN) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_CURR_IN) == 0)
    {
        // 4 means no of tuning input parameters + 1 Parameter Index
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 5);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Weld Input Tuning Parameters was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 5)
        {
            VERBOSE_ERROR("Too many Weld Input Tuning Parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_VOLT_IN) == 0)
        {
            nVoltTuneInIdx = (int) rgdb[0];
            g_rgWeldTuneParam[nVoltTuneInIdx].input.dbVolt_a     = rgdb[1];
            g_rgWeldTuneParam[nVoltTuneInIdx].input.dbVolt_b     = rgdb[2];
            g_rgWeldTuneParam[nVoltTuneInIdx].input.dbVoltScale  = rgdb[3];
            g_rgWeldTuneParam[nVoltTuneInIdx].input.dbVoltOffset = rgdb[4];
        }

        if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_CURR_IN) == 0)
        {
            nCurrTuneInIdx = (int) rgdb[0];
            g_rgWeldTuneParam[nCurrTuneInIdx].input.dbCurr_a     = rgdb[1];
            g_rgWeldTuneParam[nCurrTuneInIdx].input.dbCurr_b     = rgdb[2];
            g_rgWeldTuneParam[nCurrTuneInIdx].input.dbCurrScale  = rgdb[3];
            g_rgWeldTuneParam[nCurrTuneInIdx].input.dbCurrOffset = rgdb[4];
        }
    }
    ///////////////////////////////////////////////////////
    // TUNE_VOLT_OUT = Idx> Volt_a, Volt_b, Volt_c, Volt_Scale, Volt_Offset
    // TUNE_CURR_OUT = Idx> Curr_a, Curr_b, Curr_c, Curr_Scale, Curr_Offset
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_VOLT_OUT) == 0 ||
             stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_CURR_OUT) == 0)
    {
        // 5 means no of tuning output parameters + 1 Parameter Index
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 6);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Weld Output Tuning Parameters was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 6)
        {
            VERBOSE_ERROR("Too many Weld Output Tuning Parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_VOLT_OUT) == 0)
        {
            nVoltTuneOutIdx = (int) rgdb[0];
            g_rgWeldTuneParam[nVoltTuneOutIdx].output.dbVolt_a     = rgdb[1];
            g_rgWeldTuneParam[nVoltTuneOutIdx].output.dbVolt_b     = rgdb[2];
            g_rgWeldTuneParam[nVoltTuneOutIdx].output.dbVolt_c     = rgdb[3];
            g_rgWeldTuneParam[nVoltTuneOutIdx].output.dbVoltScale  = rgdb[4];
            g_rgWeldTuneParam[nVoltTuneOutIdx].output.dbVoltOffset = rgdb[5];
        }

        if (stricmp(pszKey, SYSCONF_KEY_WELD_TUNE_CURR_OUT) == 0)
        {
            nCurrTuneOutIdx = (int) rgdb[0];
            g_rgWeldTuneParam[nCurrTuneOutIdx].output.dbCurr_a     = rgdb[1];
            g_rgWeldTuneParam[nCurrTuneOutIdx].output.dbCurr_b     = rgdb[2];
            g_rgWeldTuneParam[nCurrTuneOutIdx].output.dbCurr_c     = rgdb[3];
            g_rgWeldTuneParam[nCurrTuneOutIdx].output.dbCurrScale  = rgdb[4];
            g_rgWeldTuneParam[nCurrTuneOutIdx].output.dbCurrOffset = rgdb[5];
        }
    }
    ///////////////////////////////////////////////////////
    // WELD_MEASURE_VAL = Idx> Volt_Cmd, Volt_Measure, Curr_Cmd, Curr_Measure
    else if (stricmp(pszKey, SYSCONF_KEY_WELD_MEASURE_VAL) == 0)
    {
        // 4 means no of weld measure parameters + 1 Parameter Index
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 5);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Weld Measure Parameters was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 5)
        {
            VERBOSE_ERROR("Too many Weld Measure Parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        nWeldMeasureIdx = (int) rgdb[0];
        g_dbControllerCmdVolt[nWeldMeasureIdx] = rgdb[1];
        g_dbWelderMeasureVolt[nWeldMeasureIdx] = rgdb[2];
        g_dbControllerCmdCurr[nWeldMeasureIdx] = rgdb[3];
        g_dbWelderMeasureCurr[nWeldMeasureIdx] = rgdb[4];
    }
    ///////////////////////////////////////////////////////
    // RESTART_INFO =  Moving Type,
    //                 Hroz_OverLapDistance, Vert_OverLapDistance, PathSpeed,
    //                 Horiz_Start_Volt, Horiz_Start_Curr,
    //                 Horiz_Main_Volt,  Horiz_Main_Curr,
    //                 Vert_Start_Volt,  Vert_Start_Curr,
    //                 Vert_Main_Volt,   Vert_Main_Curr
    else if (stricmp(pszKey, SYSCONF_KEY_ROBOT_RESTART_INFO) == 0)
    {
        // 12 means no of tuning output parameters
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 12);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid Restart Parameters was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 12)
        {
            VERBOSE_ERROR("Too many Restart Parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        g_rgRestartParam[nRobot].moving_type    = (unsigned) rgdb[0];
        g_rgRestartParam[nRobot].d_overlap_horz = rgdb[1];
        g_rgRestartParam[nRobot].d_overlap_vert = rgdb[2];

        if(g_rgRestartParam[nRobot].moving_type == RESTART_MOVE_TYPE_LIN)
        {
            g_rgRestartParam[nRobot].path_speed = rgdb[3] * 0.001;
        }
        else if(g_rgRestartParam[nRobot].moving_type == RESTART_MOVE_TYPE_JNT)
        {
            g_rgRestartParam[nRobot].path_speed = rgdb[3] * 0.01;
        }

        if(g_rgRestartParam[nRobot].path_speed <= 0)
        {
            VERBOSE_ERROR("Invalid Restart Parameters(path speed) was specified : %s\n",
                           pszValue);
            g_rgRestartParam[nRobot].path_speed = 0.01;

            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        g_rgRestartParam[nRobot].hori_start_vol = rgdb[4];
        g_rgRestartParam[nRobot].hori_start_cur = rgdb[5];
        g_rgRestartParam[nRobot].hori_main_vol  = rgdb[6];
        g_rgRestartParam[nRobot].hori_main_cur  = rgdb[7];
        g_rgRestartParam[nRobot].vert_start_vol = rgdb[8];
        g_rgRestartParam[nRobot].vert_start_cur = rgdb[9];
        g_rgRestartParam[nRobot].vert_main_vol  = rgdb[10];
        g_rgRestartParam[nRobot].vert_main_cur  = rgdb[11];
    }
    ///////////////////////////////////////////////////////
    // SAVE_NODE =  1, 8, 200
    else if (stricmp(pszKey, SYSCONF_KEY_ARCSENSOR_SAVENODE) == 0)
    {
        // 3 means no of arcsensor parameters
        nCount = PARAM_ConvArrayNumFloat(pszValue, rgdb, 3);

        if (nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid ArcSensor Parameters was specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many ArcSensor Parameters were specified : '%s=%s'\n",
                             pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_USER_PARAM);
            return RESULT_ERROR;
        }

        g_rgArcSensorParam[nRobot].fSaveArcSensorData = (unsigned) rgdb[0];
        g_rgArcSensorParam[nRobot].nStartSaveNodeNo   = (unsigned) rgdb[1];
        g_rgArcSensorParam[nRobot].nSaveNodeCount     = (unsigned) rgdb[2];
    }


    return 0;
}