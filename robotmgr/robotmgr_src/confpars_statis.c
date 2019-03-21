/////////////////////////////////////////////////////////////////////////////
//
//  confpars_statis.c: Statistics Parameter Config Information Load
//                                            2014.04.01  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"
#include "statistics.h"

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadSystemStatistics()
//

int SYSC_LoadSystemStatistics(int nRobot, const char* pszKey, const char* pszValue)
{
    double rgdb[32];
    int nCount;

    DANDY_ASSERT(nRobot >= 0 && nRobot < MAX_ROBOT_COUNT);

    ///////////////////////////////////////////////////////
    // RESET_DATE = yy:mm:dd
    if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_RESET_DATE) == 0)
    {
        // 3 means no of date element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid reset date was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many reset date were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
#if 1
        // check validation of the date
        if (rgdb[0] < 2014 ||
            rgdb[1] < 1    || rgdb[1] > 12 ||
            rgdb[2] < 1    || rgdb[2] > 31)
        {
            VERBOSE_ERROR("invalid date range(reset date)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
#endif
        g_StatisData.DateReset.nYear   = (int) rgdb[0];
        g_StatisData.DateReset.nMonth  = (int) rgdb[1];
        g_StatisData.DateReset.nDay    = (int) rgdb[2];
    }
    ///////////////////////////////////////////////////////
    // RESET_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_RESET_TIME) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid reset time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many reset time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
#if 1
        // check validation of the time
        if (rgdb[0] < 0 || rgdb[0] > 23 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(reset time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
#endif
        g_StatisData.TimeReset.nHour    = (int) rgdb[0];
        g_StatisData.TimeReset.nMinute  = (int) rgdb[1];
        g_StatisData.TimeReset.nSecond  = (int) rgdb[2];
    }
    ///////////////////////////////////////////////////////
    // PWRON_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_PWRON) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid power on time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many power on time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(power on time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimePowerOn.nHour    = (int) rgdb[0];
        g_StatisData.TimePowerOn.nMinute  = (int) rgdb[1];
        g_StatisData.TimePowerOn.nSecond  = (int) rgdb[2];
        
        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_POWERON] = 
                (g_StatisData.TimePowerOn.nHour   * 3600) +
                (g_StatisData.TimePowerOn.nMinute * 60) +
                (g_StatisData.TimePowerOn.nSecond);
    }
    ///////////////////////////////////////////////////////
    // SRVON_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_SRVON) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid servo on time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many servo on time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(servo on time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeServoOn.nHour    = (int) rgdb[0];
        g_StatisData.TimeServoOn.nMinute  = (int) rgdb[1];
        g_StatisData.TimeServoOn.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_SERVOON] = 
                (g_StatisData.TimeServoOn.nHour   * 3600) +
                (g_StatisData.TimeServoOn.nMinute * 60) +
                (g_StatisData.TimeServoOn.nSecond);
    }
    ///////////////////////////////////////////////////////
    // JOBEXE_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_JOBEXE) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid job exec time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many job exec time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(job exec time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeJobExec.nHour    = (int) rgdb[0];
        g_StatisData.TimeJobExec.nMinute  = (int) rgdb[1];
        g_StatisData.TimeJobExec.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_JOBEXEC] = 
                (g_StatisData.TimeJobExec.nHour   * 3600) +
                (g_StatisData.TimeJobExec.nMinute * 60) +
                (g_StatisData.TimeJobExec.nSecond);
    }
    ///////////////////////////////////////////////////////
    // ERROR_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_ERROR) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid error time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many error time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(error time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeError.nHour    = (int) rgdb[0];
        g_StatisData.TimeError.nMinute  = (int) rgdb[1];
        g_StatisData.TimeError.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_ERROR] = 
                (g_StatisData.TimeError.nHour   * 3600) +
                (g_StatisData.TimeError.nMinute * 60) +
                (g_StatisData.TimeError.nSecond);
    }
    ///////////////////////////////////////////////////////
    // STOP_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_STOP) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid stop time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many stop time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(stop time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeStop.nHour    = (int) rgdb[0];
        g_StatisData.TimeStop.nMinute  = (int) rgdb[1];
        g_StatisData.TimeStop.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_STOP] = 
                (g_StatisData.TimeStop.nHour   * 3600) +
                (g_StatisData.TimeStop.nMinute * 60) +
                (g_StatisData.TimeStop.nSecond);
    }
    ///////////////////////////////////////////////////////
    // MOVE_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_MOVE) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid stop time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many stop time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(move time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeMove.nHour    = (int) rgdb[0];
        g_StatisData.TimeMove.nMinute  = (int) rgdb[1];
        g_StatisData.TimeMove.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_MOVE] = 
                (g_StatisData.TimeMove.nHour   * 3600) +
                (g_StatisData.TimeMove.nMinute * 60) +
                (g_StatisData.TimeMove.nSecond);
    }
    ///////////////////////////////////////////////////////
    // WELD_TIME = hh:mm:ss
    else if (stricmp(pszKey, STAT_KEY_TIME_ROBOT_WELD) == 0)
    {
        // 3 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 3);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid weld time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 3)
        {
            VERBOSE_ERROR("Too many weld time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of the time
        if (rgdb[0] < 0 ||
            rgdb[1] < 0 || rgdb[1] > 59 ||
            rgdb[2] < 0 || rgdb[2] > 59)
        {
            VERBOSE_ERROR("invalid time range(weld time)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.TimeWeld.nHour    = (int) rgdb[0];
        g_StatisData.TimeWeld.nMinute  = (int) rgdb[1];
        g_StatisData.TimeWeld.nSecond  = (int) rgdb[2];

        g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_WELD] = 
                (g_StatisData.TimeWeld.nHour   * 3600) +
                (g_StatisData.TimeWeld.nMinute * 60) +
                (g_StatisData.TimeWeld.nSecond);
    }
    ///////////////////////////////////////////////////////
    // FILLET_WELD_DIST = 0000.00
    else if (stricmp(pszKey, STAT_KEY_DIST_ROBOT_FILLETWELD) == 0)
    {
        // 1 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid fillet weld distance was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many fillet weld distance were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of distance
        if (rgdb[0] < 0)
        {
            VERBOSE_ERROR("invalid distance range(fillet weld)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.dbDistFilletWeld = rgdb[0];
        g_dbRobotStatDist.rgdbDistance[ROBOT_STAT_DIST_FILLETWELD] =
                                                g_StatisData.dbDistFilletWeld;
    }
    ///////////////////////////////////////////////////////
    // WEAVE_WELD_DIST = 0000.00
    else if (stricmp(pszKey, STAT_KEY_DIST_ROBOT_WEAVEWELD) == 0)
    {
        // 1 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid weave weld distance was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many weave weld distance were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        // check validation of distance
        if (rgdb[0] < 0)
        {
            VERBOSE_ERROR("invalid distance range(weave weld)\n");
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_StatisData.dbDistFilletWeld = rgdb[0];
        g_dbRobotStatDist.rgdbDistance[ROBOT_STAT_DIST_WEAVWELD] =
                                                g_StatisData.dbDistFilletWeld;
    }
    ///////////////////////////////////////////////////////
    // ERR_STACK_TOP = 0 ~ MAX_ERROR_STACK_SIZE
    else if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_ST_TOP) == 0)
    {
        // 1 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid error history stack top index was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many error history stack top index were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
        g_ErrCodeStack.nTop = (int) rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // ERR_CNT = 0 ~ MAX_ERROR_STACK_SIZE
    else if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_COUNT) == 0)
    {
        // 1 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid error history count was specified : '%s=%s'\n",
                           pszKey, pszValue);
            
            g_ErrCodeStack.nErrCnt = g_ErrCodeStack.nTop;
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many error history count were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
        g_ErrCodeStack.nErrCnt = (int) rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // ERR_INDEX = 0 ~ MAX_ERROR_STACK_SIZE
    else if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_INDEX) == 0)
    {
        // 1 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 1);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid error history index was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 1)
        {
            VERBOSE_ERROR("Too many error history index were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
        g_ErrCodeStack.nErrHistoryIdx = (int) rgdb[0];
    }
    ///////////////////////////////////////////////////////
    // ERR_TIME = 2014-04-07,19:42:56
    if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_TIME) == 0)
    {
        CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[g_ErrCodeStack.nErrHistoryIdx],
                   SYSTIME_DATA_LEN,
                   pszValue);

        // 6 means no of date element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 6);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
#if 0
            VERBOSE_ERROR("Invalid error date/time was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
#endif
            g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nYear   = -1;
            g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nMonth  = -1;
            g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nDay    = -1;

            g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nHour   = -1;
            g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nMinute = -1;
            g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nSecond = -1;
        }

        if (nCount > 6)
        {
            VERBOSE_ERROR("Too many error date/time were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }
#if 0
        // check validation of the date
        if(g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nYear  != -1 &&
           g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nMonth != -1 &&
           g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nDay   != -1)
        {
            if (rgdb[0] != 0 && rgdb[1] != 0 && rgdb[2] != 0)
            {
                if (rgdb[0] < 2014 ||
                    rgdb[1] < 1    || rgdb[1] > 12 ||
                    rgdb[2] < 1    || rgdb[2] > 31)
                {
                    VERBOSE_ERROR("invalid error date range(error date(idx: %d))\n",
                                  g_ErrCodeStack.nErrHistoryIdx);
                    SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
                    return RESULT_ERROR;
                }
            }
        }

        // check validation of the time
        if(g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nHour   != -1 &&
           g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nMinute != -1 &&
           g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nSecond != -1)
        {
            if (rgdb[3] != 0 && rgdb[4] != 0 && rgdb[5] != 0)
            {
                if (rgdb[3] < 0 || rgdb[3] > 23 ||
                    rgdb[4] < 0 || rgdb[4] > 59 ||
                    rgdb[5] < 0 || rgdb[5] > 59)
                {
                    VERBOSE_ERROR("invalid error time range(error time(idx: %d))\n",
                                  g_ErrCodeStack.nErrHistoryIdx);
                    SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
                    return RESULT_ERROR;
                }
            }
        }
#endif   
        g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nYear   = (int) rgdb[0];
        g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nMonth  = (int) rgdb[1];
        g_ErrCodeStack.ErrDate[g_ErrCodeStack.nErrHistoryIdx].nDay    = (int) rgdb[2];

        g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nHour   = (int) rgdb[3];
        g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nMinute = (int) rgdb[4];
        g_ErrCodeStack.ErrTime[g_ErrCodeStack.nErrHistoryIdx].nSecond = (int) rgdb[5];
    }
    ///////////////////////////////////////////////////////
    // ERR_CODE = 18874410
    else if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_CODE) == 0)
    {
        // 2 means no of time element
        nCount = PARAM_ConvArrayNumFloatTime(pszValue, rgdb, 2);

        if (nCount < 1 || nCount == CONFIG_NUM_ERROR)
        {
            VERBOSE_ERROR("Invalid error history code was specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        if (nCount > 2)
        {
            VERBOSE_ERROR("Too many error history code were specified : '%s=%s'\n",
                           pszKey, pszValue);
            SVC_DefineErrorState(ON, SYS_ERR_SYNTAX_STATISTICS_PARAM);
            return RESULT_ERROR;
        }

        g_ErrCodeStack.nErrStack[g_ErrCodeStack.nErrHistoryIdx]         = (int) rgdb[0];
        g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nErrHistoryIdx] = (int) rgdb[1];
    }
    ///////////////////////////////////////////////////////
    // ERR_DETAIL = JOBASM_ERR_FILE_OPEN
    if (stricmp(pszKey, STAT_KEY_ERROR_HISTORY_DETAIL) == 0)
    {
        CRT_strcpy(g_ErrCodeStack.g_szErrContent[g_ErrCodeStack.nErrHistoryIdx],
                   ERROR_NAME_LEN,
                   pszValue);
    }
    
    return 0;
}