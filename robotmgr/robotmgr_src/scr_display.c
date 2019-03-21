/////////////////////////////////////////////////////////////////////////////
//
//  scr_display.c: Screen Display
//                                            2013.06.18  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"
#include "statistics.h"

///////////////////////////////////////
#define VERBOSE_NAME    "RM_VERBOSE_"

int g_nDisYPos = 0;
int g_fDisStatisticsData = OFF;

/////////////////////////////////////////////////////////////////////////////
//
//  Function: DSP_InitDisplay()
//      - initialize verbose, vga display
//
int DSP_InitDisplay(void)
{
    ///////////////////////////////////
    //
    //  config verbose
    //

    // set verbose prefix tag
    VERBOSE_CleanDirtyFile(VERBOSE_NAME);
    VERBOSE_Create(VERBOSE_NAME, "[RM] ");
    ECHO_OpenClient(VERBOSE_NAME);
    
    //Enables display for the specific type channel
    VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);

    if (g_Arg.bVerbose)
    {
        // -verbose argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_IGNORE,  TRUE);
    }
    else if (g_Arg.bQuiet)
    {
        // -quiet argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_IGNORE,  FALSE);

        VERBOSE_VERBOSE("dddddddddddd\n");
    }
    else
    {
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_IGNORE,  FALSE);
    }

    // change verbose color
    VERBOSE_SetColor(VERBOSE_TYPE_VERBOSE,
                     ECHO_COLOR_LIGHTGREEN | ECHO_COLOR_INTENSITY,
                     ECHO_COLOR_BLACK);
#if defined(__DEBUG)
    VERBOSE_SetColor(VERBOSE_TYPE_NOTIFY,
                     ECHO_COLOR_LIGHTBLUE | ECHO_COLOR_INTENSITY,
                     ECHO_COLOR_WHITE | ECHO_COLOR_LIGHTGRAY);
#endif
    ///////////////////////////////////
    //
    //  initialize vga display
    //
    if(VGA_MapVGAMemory(0) == -1)
    {
        VERBOSE_ERROR("Map Error\n");
        return RESULT_ERROR;
    }

    // config default vga display color
    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_BLACK));

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: DSP_FinalizeDisplay()
//      - finalize verbose, vga display
//
int DSP_FinalizeDisplay(void)
{
    VGA_UnmapVGAMemory();

    VERBOSE_WaitForComplete();

    VERBOSE_Destroy();

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  DSP_DispDivider()
//

void DSP_DispDivider(void)
{
    VERBOSE_MESSAGE("--------------------------------------------\n\n");
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_DSP_VGA_SystemInform(int x, int y,
//                                      char targ, int fcolor, int bcolor)
//      - Display System Information

static void _loc_DSP_VGA_SystemInform(int nPosX, int nPosY,
                                      char *szPrintTarget,
                                      int nForeColor, int nBackColor)
{
    // config vga display color
    VGA_SetAttr(VGA_MAKE_ATTR(nForeColor, nBackColor));

    VGA_printf(nPosX, nPosY, "%s", szPrintTarget); 
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SVC_VGADisplayStatisticsData()
//
int SVC_VGADisplayStatisticsData(int nDisYPos)
{
    if(nDisYPos <= 0)
    {
        g_fDisStatisticsData = OFF;
        VERBOSE_MESSAGE("Statistics Data VGA Display OFF!\n");
    }
    else
    {
        g_fDisStatisticsData = ON;
        VERBOSE_MESSAGE("Statistics Data VGA Display ON!\n");
    }

    if(g_fDisStatisticsData == OFF)
    {
        g_nDisYPos = 0;
    }
    else
    {
        g_nDisYPos = nDisYPos;
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: DSP_VGADisplyThread()
//
#define FirstLine     0
#define SecondLine    1

THREAD_ENTRY_TYPE DSP_VGADisplyThread(void* pParam)
{
    static int s_nDSPPosX;
    static int s_nCnt;

    while(g_fVGA_DSPThRun == RUN && g_Arg.bVGA == FALSE)
	{
        // first line display define: 1. system mode 2. execution state
        //                            3. work type   4. error state     5. e-stop code
        // 1. system mode
        s_nDSPPosX = 0;
        if(g_pShm_SysStatus != NULL)
        {
            if(g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR)
            {
                VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_WHITE, VGA_COLOR_LIGHTGRAY));
                VGA_printf(s_nDSPPosX, FirstLine, "%-8s", g_szSystemMode);
            }
            else
            {
                VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_RED));
                VGA_printf(s_nDSPPosX, FirstLine, "%-8s", g_szSystemMode);
            }
        }

        // 2. execution state
        s_nDSPPosX = s_nDSPPosX + SYSMODE_NAME_LEN - 1;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_DARKGRAY));
        VGA_printf(s_nDSPPosX, FirstLine, "%-11s", g_szExecStat);

        // 3. work type
        s_nDSPPosX = s_nDSPPosX + EXECSTATE_NAME_LEN;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_BROWN, VGA_COLOR_WHITE));
        VGA_printf(s_nDSPPosX, FirstLine, "%-10s", g_szWorkType);
        
        // 4-1. error code
        s_nDSPPosX = s_nDSPPosX + WORKTYPE_NAME_LEN;
        if(g_pShm_SysStatus != NULL)
        {
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_RED));
            VGA_printf(s_nDSPPosX, FirstLine, "%-7x", g_pShm_SysStatus->nErrCode);
        }

        // 4-2. error content
        s_nDSPPosX = s_nDSPPosX + 8;
        // display error contents
        if(g_pShm_SysStatus != NULL)
        {
            if(g_pShm_SysStatus->fErrorState == TRUE)
            {
                SYSMON_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);

                // process for flickering
                if(s_nCnt <= 10)
                {
                    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTRED, VGA_COLOR_RED));
                    VGA_printf(s_nDSPPosX, FirstLine, "%-20s", g_szErrContent);
                    s_nCnt++;
                }
                else if(s_nCnt >= 10 && s_nCnt <= 15)
                {
                    _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine,
                                              "                    ",
                                              VGA_COLOR_LIGHTRED,
                                              VGA_COLOR_RED);
                    DANDY_SLEEP(1);
                    s_nCnt++;
                }
                else if(s_nCnt > 15)
                {
                    s_nCnt = 0;
                }
            }
            else if(g_pShm_SysStatus->fErrorState == FALSE)
            {
                SYSMON_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);

                s_nCnt = 0;
                _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine,
                                          g_szErrContent,
                                          VGA_COLOR_LIGHTMAGENTA,
                                          VGA_COLOR_BLACK);
            }
        }

        // 4-2. error mode
        s_nDSPPosX = s_nDSPPosX + ERROR_NAME_LEN - 1;
        if(g_pShm_SysStatus != NULL)
        {
            if(g_pShm_SysStatus->fErrorState == TRUE)
            {
                SYSMON_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);

                VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTRED, VGA_COLOR_BLACK));
                VGA_printf(s_nDSPPosX, FirstLine, "%-7s", g_szErrModeContent);
            }
            else if(g_pShm_SysStatus->fErrorState == FALSE)
            {
                SYSMON_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);

                s_nCnt = 0;
                VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTRED, VGA_COLOR_BLACK));
                VGA_printf(s_nDSPPosX, FirstLine, "%-7s", g_szErrModeContent);
            }
        }

        // 6. E-stop Code
        s_nDSPPosX = s_nDSPPosX + ERROR_MODE_NAME_LEN;
        
        if(g_pShm_SysStatus != NULL)
        {
            SYSMON_ParceEstopCode(g_pShm_SysStatus->nEstopCode);
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTMAGENTA, VGA_COLOR_BLACK));
            VGA_printf(s_nDSPPosX, FirstLine, "%-11s", g_szEstopContent);
        }

        // second line display define: 1. service name        2. RM state
        //                             3. TE state            4. SC state
        //                             5. Job File Name       6. exec line
        //                             7. auto ecat reset cnt
        // 1. service name
        s_nDSPPosX = 0;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_BLUE, VGA_COLOR_WHITE));
        VGA_printf(s_nDSPPosX, SecondLine, "%-18s", g_szServContent); 

        // 2. RM state
        s_nDSPPosX = s_nDSPPosX + SERV_NAME_LEN;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_DARKGRAY, VGA_COLOR_LIGHTCYAN));
        VGA_printf(s_nDSPPosX, SecondLine, "%-9s", g_szRMState); 

        // 3. TE state
        s_nDSPPosX = s_nDSPPosX + SIBLINGSTATE_NAME_LEN;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_DARKGRAY, VGA_COLOR_LIGHTCYAN));
        VGA_printf(s_nDSPPosX, SecondLine, "%-9s", g_szTEState); 

        // 4. SC state
        s_nDSPPosX = s_nDSPPosX + SIBLINGSTATE_NAME_LEN;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_DARKGRAY, VGA_COLOR_LIGHTCYAN));
        VGA_printf(s_nDSPPosX, SecondLine, "%-9s", g_szSCState); 

        // 5. Job File Name
        s_nDSPPosX = s_nDSPPosX + SIBLINGSTATE_NAME_LEN;
        if(g_fAseembleDone == TRUE && g_fSysAliveThRun == RUN)
        {
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_BLACK, VGA_COLOR_LIGHTGREEN));
            VGA_printf(s_nDSPPosX, SecondLine, "%-18s", g_rgszLoadedJobModName); 
        }
        else if(g_fAseembleDone != TRUE && g_fSysAliveThRun == RUN)
        {
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_BLACK, VGA_COLOR_LIGHTGREEN));
            VGA_printf(s_nDSPPosX, SecondLine, "%-18s", "NO JOB LOADED"); 
        }

         // 6. job exec line
        s_nDSPPosX = s_nDSPPosX + 19;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_DARKGRAY));
        VGA_printf(s_nDSPPosX, SecondLine, "%-3d/%-3d",
                   g_nJobExecLineIdx + 1, g_nCmdLoadCount);

         // 7. ecat auto restart count
        s_nDSPPosX = s_nDSPPosX + 8;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_RED));
        VGA_printf(s_nDSPPosX, SecondLine, "%-2d",
                   g_nEcatAutoResetCnt);

        // Statistics Data (By Service)
        if(g_fDisStatisticsData == ON)
        {
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_RED));

            VGA_printf(0, g_nDisYPos, "PWR:%-10ld",
                       g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_POWERON]);
            VGA_printf(0, g_nDisYPos + 1, "("STAT_FORMAT_TIME")",
                        g_StatisData.TimePowerOn.nHour,
                        g_StatisData.TimePowerOn.nMinute,
                        g_StatisData.TimePowerOn.nSecond);

            VGA_printf(12, g_nDisYPos, "SRV:%-10ld",
                       g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_SERVOON]);
            VGA_printf(12, g_nDisYPos + 1, "("STAT_FORMAT_TIME")",
                        g_StatisData.TimeServoOn.nHour,
                        g_StatisData.TimeServoOn.nMinute,
                        g_StatisData.TimeServoOn.nSecond);

            VGA_printf(24, g_nDisYPos, "JOB:%-10ld",
                       g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_JOBEXEC]);
            VGA_printf(24, g_nDisYPos + 1, "("STAT_FORMAT_TIME")",
                        g_StatisData.TimeJobExec.nHour,
                        g_StatisData.TimeJobExec.nMinute,
                        g_StatisData.TimeJobExec.nSecond);

            VGA_printf(36, g_nDisYPos, "ERR:%-10ld",
                       g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_ERROR]);
            VGA_printf(36, g_nDisYPos + 1, "("STAT_FORMAT_TIME")",
                        g_StatisData.TimeError.nHour,
                        g_StatisData.TimeError.nMinute,
                        g_StatisData.TimeError.nSecond);

            VGA_printf(48, g_nDisYPos, "WLD:%-10ld",
                       g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_WELD]);
            VGA_printf(48, g_nDisYPos + 1, "("STAT_FORMAT_TIME")",
                        g_StatisData.TimeWeld.nHour,
                        g_StatisData.TimeWeld.nMinute,
                        g_StatisData.TimeWeld.nSecond);

            VGA_printf(60, g_nDisYPos, "FLT:"STAT_FORMAT_DIST,
                       g_dbRobotStatDist.rgdbDistance[ROBOT_STAT_DIST_FILLETWELD]);

            VGA_printf(60, g_nDisYPos + 1, "WEV:"STAT_FORMAT_DIST,
                       g_dbRobotStatDist.rgdbDistance[ROBOT_STAT_DIST_WEAVWELD]);
        }

        //DANDY_SLEEP(150);
        DANDY_SLEEP(200);
        //VGA_SetRefreshType(150, DANDY_FALSE);
    }

    g_fVGA_DSPThExit = TRUE;

    return 0;
}
