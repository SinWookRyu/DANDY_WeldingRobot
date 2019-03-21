/////////////////////////////////////////////////////////////////////////////
//
//  sysmon_statis.c: System Statistics Monitoring
//                                            2013.06.18  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"
#include "service.h"
#include "statistics.h"

///////////////////////////////////////
int g_fEnableWriteStatistics = OFF;
int g_fInitGetCurrTime = OFF;
int g_nDiffRTCSec = 0;
int g_nDiffRTCHour = 0;
double rgdbTmpStatDistance[ROBOT_STAT_DIST_COUNT];
int g_fArcOffTiming[ROBOT_STAT_DIST_COUNT] = {OFF, OFF};
int g_nWeldMode = -1;
int g_fOldWeldRun = OFF;

ROBOT_STAT_TIME     SysStatTimeCnt;

/////////////////////////////////////////////////////////////////////////////

#if 0   //for statistics data, use incremental timer
/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_IncSysUsingTime()
//
static void _loc_IncSysUsingTime(int iMember)
{
    DANDY_ASSERT(iMember >= 0 && iMember < ROBOT_STAT_TIME_COUNT);

    SysStatTimeCnt.rgdwTime[iMember] += ROBOMAN_SYSTIMER_INTERVAL;

    if (SysStatTimeCnt.rgdwTime[iMember] >= 1000)
    {
        g_dwRobotStatTime.rgdwTime[iMember]++;      //Unit: Sec
        SysStatTimeCnt.rgdwTime[iMember] -= 1000;
    }
}
#endif


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_AutoWriteStatisticsData()
//
static void _loc_AutoWriteStatisticsData(void)
{
    if(g_fEnableWriteStatistics == ON)
    {
        Fn_WriteStatisticsData(OPT_QUITE);
        SVC_SaveStatisticsDataToFile(ROBOT_0_INDEX, OPT_QUITE);

        Fn_GetSystemTime();
#if 0
        VERBOSE_MESSAGE("Auto Write Statistics Data Done! ("
                        STAT_FORMAT_DATE" "STAT_FORMAT_TIME")\n",
                        curr_time.nYear,
                        curr_time.nMonth,
                        curr_time.nDay,
                        curr_time.nHour,
                        curr_time.nMinute,
                        curr_time.nSec);
#endif
        g_fEnableWriteStatistics = OFF;
        
        DANDY_SLEEP(1);
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_GetkDiffRTCTime()
//
static void _loc_GetkDiffRTCTime(void)
{
    int nCurrSec, nOldSec;
    int nCurrHour, nOldHour;

    if(g_fInitGetCurrTime == OFF)
    {
        Fn_GetSystemTime();
        nOldHour = curr_time.nHour;
        g_fInitGetCurrTime = ON;
    }

    nOldHour  = curr_time.nHour;
    nOldSec   = curr_time.nSec;

    Fn_GetSystemTime();

    nCurrHour = curr_time.nHour;
    nCurrSec  = curr_time.nSec;

    g_nDiffRTCSec  = (abs) (nCurrSec  - nOldSec);
    g_nDiffRTCHour = (abs) (nCurrHour - nOldHour);
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_CheckStatTime()
//
static void _loc_CheckStatTime(int iMember)
{
    DANDY_ASSERT(iMember >= 0 && iMember < ROBOT_STAT_TIME_COUNT);

    // increase statistics time by second
    if(g_nDiffRTCSec >= 1)
    {
        g_dwRobotStatTime.rgdwTime[iMember]++;      //Unit: Sec
    }

    // time(hour) check for auto statistics data saving
    if(g_nDiffRTCHour >= 1)
    {
        g_fEnableWriteStatistics = ON;
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_CheckStatDistance()
//
static void _loc_CheckStatDistance(int iMember)
{
    DANDY_ASSERT(iMember >= 0 && iMember < ROBOT_STAT_DIST_COUNT);

    if(g_pShmemTEStatus != NULL)
    {
        // at arc off timing
        if(g_fArcOffTiming[iMember] == ON)
        {
            // save statistics distance to var
            g_dbRobotStatDist.rgdbDistance[iMember] =
                g_dbRobotStatDist.rgdbDistance[iMember] + rgdbTmpStatDistance[iMember];

            g_fArcOffTiming[iMember] = OFF;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckStatisticsData(void)
//
static int _loc_SYSMON_CheckStatisticsData(void)
{
    _loc_GetkDiffRTCTime();

    // system power-on time
    {
        _loc_CheckStatTime(ROBOT_STAT_TIME_POWERON);
    }

    // servo-on time
    if(g_pShmemSC != NULL)
    {
        if(g_pShmemSC->outputstate.fServoOnOutState == ON)
        {
            _loc_CheckStatTime(ROBOT_STAT_TIME_SERVOON);
        }
    }

    // job execution time
    if(g_pShm_SysStatus != NULL)
    {
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB &&
           g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
        {
            _loc_CheckStatTime(ROBOT_STAT_TIME_JOBEXEC);
        }
    }

    // error time
    if(g_pShm_SysStatus != NULL)
    {
        if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR)
        {
            _loc_CheckStatTime(ROBOT_STAT_TIME_ERROR);
        }
    }

    // stop time
    if(g_pShm_SysStatus != NULL)
    {
        if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_ENTRY ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_INIT  ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_TERMINATE)
        {
            _loc_CheckStatTime(ROBOT_STAT_TIME_STOP);
        }
    }

    // move time
    if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL)
    {
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB &&
           g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
        {
            if(g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVJ  ||
              (g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVL  &&
               g_pShmemTEStatus->run_f_weld == OFF) ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVC  ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVO  ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_IMOVL ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_HOME  ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_TOUCH)
            {
                _loc_CheckStatTime(ROBOT_STAT_TIME_MOVE);
            }
        }
    }

    // weld time
    if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL)
    {
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB &&
           g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
        {
            if(g_pShmemTEStatus->run_f_weld == ON &&
              (g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVL   ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_UWEAVL ||
               g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_CWEAV))
            {
                _loc_CheckStatTime(ROBOT_STAT_TIME_WELD);
            }
        }
    }

    // get arc off timing
    if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL)
    {
        // in case of job exec mode
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB)
        {
            // in case of arc on
            if(g_pShmemTEStatus->run_f_weld == ON)
            {
                // in case of MOVL
                if(g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_MOVL)
                {
                    rgdbTmpStatDistance[ROBOT_STAT_DIST_FILLETWELD] = g_pShmemTEStatus->dist_work;
                    g_nWeldMode = ROBOT_STAT_DIST_FILLETWELD;
                }
                // in case of WEAV
                else if(g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_UWEAVL ||
                        g_pShmemTEStatus->run_cmd_code == DANDY_JOB_CODE_CWEAV)
                {
                    rgdbTmpStatDistance[ROBOT_STAT_DIST_WEAVWELD] = g_pShmemTEStatus->dist_work;
                    g_nWeldMode = ROBOT_STAT_DIST_WEAVWELD;
                }

                g_fArcOffTiming[ROBOT_STAT_DIST_FILLETWELD] = OFF;
                g_fArcOffTiming[ROBOT_STAT_DIST_WEAVWELD] = OFF;
            }
            
            // get arc off timing by flag
            else if(g_fOldWeldRun == ON && g_pShmemTEStatus->run_f_weld == OFF)
            {
                if(g_nWeldMode == ROBOT_STAT_DIST_FILLETWELD)
                {
                    g_fArcOffTiming[ROBOT_STAT_DIST_FILLETWELD] = ON;
                }
                else if(g_nWeldMode == ROBOT_STAT_DIST_WEAVWELD)
                {
                    g_fArcOffTiming[ROBOT_STAT_DIST_WEAVWELD] = ON;
                }
            }

            g_fOldWeldRun = g_pShmemTEStatus->run_f_weld;
        }
    }

    // jillet joint distance
    if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL)
    {
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB)
        {
            _loc_CheckStatDistance(ROBOT_STAT_DIST_FILLETWELD);
        }
    }

    // weave distance
    if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL)
    {
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB)
        {
            _loc_CheckStatDistance(ROBOT_STAT_DIST_WEAVWELD);
        }
    }

    // automatically write statistic data (every 1 hour)
    if(g_pShm_SysStatus != NULL)
    {
        if(g_pShm_SysStatus->nSystemMode != MODE_STAT_ENTRY ||
           g_pShm_SysStatus->nSystemMode != MODE_STAT_INIT ||
           g_pShm_SysStatus->nSystemMode != MODE_STAT_TERMINATE)
        {
            _loc_AutoWriteStatisticsData();
        }
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SYSMON_SysStatisticsCheckThread()
//

THREAD_ENTRY_TYPE SYSMON_SysStatisticsCheckThread(void* pParam)
{
    while(g_fSysStatThRun == RUN)
	{
        _loc_SYSMON_CheckStatisticsData();

        DANDY_SLEEP(500);
    }

    g_fSysStatCheckThExit = TRUE;

    return 0;
}