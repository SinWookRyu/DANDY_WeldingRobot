/////////////////////////////////////////////////////////////////////////////
//
//  sysmon_err.c: System Error Monitoring
//                                            2013.06.18  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"
#include "service.h"

///////////////////////////////////////
int g_nCurrTimeDay = 0;
int g_nCurrTimeHour = 0;
int g_nCurrTimeMin = 0;
int g_fAutoEcatRestartDone = OFF;
int g_fAutoEcatRestartCond = OFF;
int g_nEcatAutoResetCnt = 0;

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckSiblingProcessError(char* szProcessName)
//
static int _loc_SYSMON_CheckSiblingProcessError(char* szProcessName)
{
    /* Check RM Error State */
    if(strcmp(szProcessName, ROBOMAN_ABB_NAME) == 0)
    {
        if(g_pShm_SysStatus != NULL && g_pShmemTEStatus != NULL && g_pShmemSC != NULL)
        {
            if((g_pShm_SysStatus->fErrorState == ON &&
                g_pShmemTEStatus->run_error.code == ERR_NONE &&
                g_pShmemSC->sysstate.fErrorState == OFF) &&
               g_pShm_SysStatus->nErrCode != SYS_ERR_OK)
            {
                //SVC_DefineErrorState(ON, g_pShm_SysStatus->nErrCode);
                ;
            }
        }
    }

    /* Check TE Error State */
    else if(strcmp(szProcessName, TASKEXEC_ABB_NAME) == 0)
    {
        if(g_pShmemTEStatus != NULL && g_pShm_SysStatus->fInitProcTE == TRUE)
        {
            if(g_pShmemTEStatus->run_error.code != ERR_NONE)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode & 0x0000000;

                // Set Error Owner
                g_pShm_SysStatus->nErrCode = g_pShmemTEStatus->run_error.code | ERR_OWNER_FROM_TE;
                
                // Set Error Mode
                if((g_pShmemTEStatus->run_error.sect & 0xff) != 0x00)
                {
                    g_pShm_SysStatus->nErrCode =
                            g_pShm_SysStatus->nErrCode | (g_pShmemTEStatus->run_error.sect << 16);
                }
                g_pShm_SysStatus->fErrorState = TRUE;

                // Set Error Code
                SVC_Write_ErrorHistory(ON, g_pShm_SysStatus->nErrCode);
            }
        }
    }

    /* Check SC Error State */
    else if(strcmp(szProcessName, SERVOCON_ABB_NAME) == 0)
    {
        if(g_pShmemSC != NULL && g_pShm_SysStatus->fInitProcSC == TRUE)
        {
            if(g_pShmemSC->sysstate.fErrorState == TRUE)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode & 0x0000000;

                if(g_pShmemSC->sysstate.nErrorAxis == 0)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS1 | ERR_OWNER_FROM_SC;
                }
                else if(g_pShmemSC->sysstate.nErrorAxis == 1)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS2 | ERR_OWNER_FROM_SC;
                }
                else if(g_pShmemSC->sysstate.nErrorAxis == 2)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS3 | ERR_OWNER_FROM_SC;
                }
                else if(g_pShmemSC->sysstate.nErrorAxis == 3)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS4 | ERR_OWNER_FROM_SC;
                }
                else if(g_pShmemSC->sysstate.nErrorAxis == 4)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS5 | ERR_OWNER_FROM_SC;
                }
                else if(g_pShmemSC->sysstate.nErrorAxis == 5)
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_AXIS6 | ERR_OWNER_FROM_SC;
                }
                else
                {
                    g_pShm_SysStatus->nErrCode = 
                            g_pShmemSC->sysstate.nErrorCode | ERR_MOD_FROM_NOTAXIS | ERR_OWNER_FROM_SC;
                }

                g_pShm_SysStatus->fErrorState = TRUE;

                // Set Error Code
                SVC_Write_ErrorHistory(ON, g_pShm_SysStatus->nErrCode);
            }
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SYSMON_SysErrorCheckThread()
//

THREAD_ENTRY_TYPE SYSMON_SysErrorCheckThread(void* pParam)
{
    while(g_fSysErrorStatusThRun == RUN)
	{
        if(g_pShm_SysStatus != NULL)
        {
            // set check process error state (RM)
            _loc_SYSMON_CheckSiblingProcessError(ROBOMAN_ABB_NAME);
            DANDY_SLEEP(5);

            // set check process error state (TE)
            _loc_SYSMON_CheckSiblingProcessError(TASKEXEC_ABB_NAME);
            DANDY_SLEEP(5);

            // set check process error state (SC)
            _loc_SYSMON_CheckSiblingProcessError(SERVOCON_ABB_NAME);
            DANDY_SLEEP(5);
        }
        
        DANDY_SLEEP(10);
    }
    
    g_fSysErrorStatusThExit = TRUE;

    return 0;
}


///////////////////////////////////////
//
//  Function: _loc_SVC_localInit()
//

int _loc_SVC_localInit(char* szProcessName)
{
    static int nRet = RESULT_OK;
    
    // TE Connection Init
    if(strcmp(szProcessName, TASKEXEC_ABB_NAME) == 0)
    {
        g_coidTEalive = MSG_AttachNamedConnection(TE_CHANNEL_NAME);

        if(g_coidTEalive != INVALID_COID)
            VERBOSE_VERBOSE("%s alive connected! coid = %d\n",
                                                              TE_CHANNEL_NAME,
                                                              g_coidTEalive);
        nRet = MSG_SendPulse(g_coidTEalive, TESERV_LIFE_CHK, 0);

        if(nRet != 1)
        {
            ;
        }
        
        g_coidTE = MSG_AttachNamedConnection(TE_CHANNEL_NAME);

        if(g_coidTE!= INVALID_COID)
            VERBOSE_VERBOSE("%s connected! coid = %d\n", TE_CHANNEL_NAME, g_coidTE);
    }
    // SC Connection Init
    else if(strcmp(szProcessName, SERVOCON_ABB_NAME) == 0)
    {
        g_coidSCalive = MSG_AttachNamedConnection(SC_CHANNEL_NAME);

        if(g_coidSCalive != INVALID_COID)
            VERBOSE_VERBOSE("%s alive connected! coid = %d\n",
                                                              SC_CHANNEL_NAME,
                                                              g_coidSCalive);
        nRet = MSG_SendPulse(g_coidSCalive, SC_SERV_ALIVE, 0);

        if(nRet != 1)
        {
            ;
        }

        g_coidSC = MSG_AttachNamedConnection(SC_CHANNEL_NAME);

        if(g_coidSC!= INVALID_COID)
            VERBOSE_VERBOSE("%s connected! coid = %d\n", SC_CHANNEL_NAME, g_coidSC);
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: _loc_Fn_GetCurrentTime()
//      - Function to get system time hour
static int _loc_Fn_GetCurrentTime(const struct tm* ptm)
{
    g_nCurrTimeDay  = ptm->tm_mday;
    g_nCurrTimeHour = ptm->tm_hour;
    g_nCurrTimeMin  = ptm->tm_min;

    return g_nCurrTimeHour;
}


///////////////////////////////////////
//
//  Function: _loc_CheckEcatAutoRestartCond()
//      - Function to automatically restart ecat master
static void _loc_EcatAutoRestart(int nIndex)
{
    int nResetflagHour;

    /* Set Reset flag hour - before 1hour set falg to OFF */
    if(g_rgnEcatMasterResetHours[nIndex] == 0)
    {
        nResetflagHour = 23;        // if time value is zero, minus 1hour is 23
    }
    else
    {
        nResetflagHour = g_rgnEcatMasterResetHours[nIndex] - 1;
    }

    /* Reset check flag */
    if(g_nCurrTimeHour == nResetflagHour)
    {
        g_fAutoEcatRestartDone = OFF;
    }
    
    if(g_pShm_SysStatus != NULL)
    {
        if(g_nCurrTimeHour == g_rgnEcatMasterResetHours[nIndex] &&
           g_nCurrTimeMin > g_nEcatMasterResetMin &&
           g_fAutoEcatRestartDone == OFF)
        {
            if(g_pShmemTEStatus != NULL)
            {
                // check auto ecat restart condition
#if 0
                if(g_pShmemTEStatus->run_mode == RUNMODE_NONE ||
                   (g_pShm_SysStatus->nExecStat == EXEC_STAT_TERMINATING &&
                    g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB))
#endif
                if(g_pShmemTEStatus->run_mode == RUNMODE_NONE &&
                  (g_pShm_SysStatus->nExecStat == EXEC_STAT_TERMINATING ||
                   g_pShm_SysStatus->nExecStat == EXEC_STAT_PREPARATION ||
                   g_pShm_SysStatus->nExecStat == EXEC_STAT_IDLE))
                {
                    g_fAutoEcatRestartCond = ON;
                }
                else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP)
                {
                    g_fAutoEcatRestartCond = ON;
                }
            }
        }
    }

    /* Auto Restart Master */
    if(g_pShm_SysStatus != NULL)
    {
        if(g_fAutoEcatRestartCond == ON)
        {
            if(g_nCurrTimeHour == g_rgnEcatMasterResetHours[nIndex] &&
               g_fAutoEcatRestartDone == OFF)
            {
                VERBOSE_MESSAGE("EtherCAT Master Auto Restart Done!(dd:hh:mm - %2d:%2d:%2d)\n",
                                g_nCurrTimeDay, g_nCurrTimeHour, g_nCurrTimeMin);

                MSG_SendPulse(g_coidSC, SC_SERV_ECAT_RESTART, 0);
                g_nEcatAutoResetCnt = g_nEcatAutoResetCnt + 1;

                g_fAutoEcatRestartDone = ON;
                g_fAutoEcatRestartCond = OFF;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SYSMON_SysAliveCheckThread()
//      Error Code: SYS_ERR_PROC_ALIVE_RM
//                  SYS_ERR_PROC_ALIVE_TE
//                  SYS_ERR_PROC_ALIVE_SC

THREAD_ENTRY_TYPE SYSMON_SysAliveCheckThread(void* pParam)
{
    static int nRet = RESULT_OK;
    static int retSCcoid = 0;
    static int retTEcoid = 0;
    static int nForPrintTE = 0;
    static int nForPrintSC = 0;
    static int s_nCnt = 0;
    static int s_nChkTimeCnt = 0;
    struct tm tm;
    int iCnt;

    nForPrintTE = nForPrintSC = 0;
    retSCcoid = retTEcoid = -1;
    
    g_coidTEalive = MSG_AttachNamedConnection(TE_CHANNEL_NAME);
    g_coidSCalive = MSG_AttachNamedConnection(SC_CHANNEL_NAME);

    while(g_fSysAliveThRun == RUN)
	{
        if(g_pShm_SysStatus != NULL)
        {
            // check TE process alive
            if(g_pShm_SysStatus->fInitProcTE == TRUE && 
               g_pShm_SysStatus->fExitProcTE == FALSE && 
               g_coidTEalive != INVALID_COID)
            {
                nRet = MSG_SendPulse(g_coidTEalive, TESERV_LIFE_CHK, 0);

                if(nRet == -1)
                {
                    SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_TE);
                    if(nForPrintTE ==0)
                    {
                        VERBOSE_VERBOSE("%s currently Not Alive! [ERR CODE: %d]\n",
                                        TE_PROCESS_NAME, SYS_ERR_PROC_ALIVE_TE);
                        nForPrintTE++;
                    }
                }
            }
            // If Alive Check is Failed, Define Error State and Try to Connection Init
            else if(g_coidTEalive == INVALID_COID && g_pShm_SysStatus->fExitProcTE != TRUE)
            {
                SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_TE);
                if (g_Arg.fNoAutoInit == FALSE)
                {
                    _loc_SVC_localInit(TASKEXEC_ABB_NAME);
                }
            }

            // check SC process alive
            if(g_pShm_SysStatus->fInitProcSC == TRUE &&
               g_pShm_SysStatus->fExitProcSC == FALSE && 
               g_coidSCalive != INVALID_COID)
            {
                nRet = MSG_SendPulse(g_coidSCalive, SC_SERV_ALIVE, 0);

                if(nRet == -1)
                {
                    // Process for Simultaneous Display
                    if(g_pShm_SysStatus->nErrCode == SYS_ERR_PROC_ALIVE_TE && s_nCnt > 1)
                    {
                        SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_TE);
                        s_nCnt = 0;
                    }
                    else
                    {
                        SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_SC);
                    }
                    s_nCnt++;

                    if(nForPrintSC ==0)
                    {
                        VERBOSE_VERBOSE("%s currently Not Alive! [ERR CODE: %d]\n",
                                        SC_PROCESS_NAME, SYS_ERR_PROC_ALIVE_SC);
                        nForPrintSC++;
                    }
                }
            }
            // If Alive Check is Failed, Define Error State and Try to Connection Init
            else if(g_coidSCalive == INVALID_COID && g_pShm_SysStatus->fExitProcSC != TRUE)
            {
                if (g_Arg.fNoAutoInit == FALSE)
                {
                    _loc_SVC_localInit(SERVOCON_ABB_NAME);
                }

                if(g_pShm_SysStatus->nErrCode == SYS_ERR_PROC_ALIVE_TE && s_nCnt > 1)
                {
                    SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_TE);
                    s_nCnt = 0;
                }
                else
                {
                    SVC_DefineErrorState(ON, SYS_ERR_PROC_ALIVE_SC);
                }
                s_nCnt++;
            }
        }

        if (g_Arg.fNoAutoInit == FALSE)
        {
            if(g_pShm_SysStatus != NULL)
            {
                if(g_pShm_SysStatus->fExitProcTE == TRUE)
                {
                    MSG_CloseSiblingConnection(g_coidTE, g_coidTEalive);
                }

                if(g_pShm_SysStatus->fExitProcSC == TRUE)
                {
                    MSG_CloseSiblingConnection(g_coidSC, g_coidSCalive);
                }
            }
        }

        while(g_fSysAliveThRun == PAUSE)
        {
            //DANDY_SLEEP(500);
            DANDY_SLEEP(100);
        }

        /* Auto Restart Ecat Master */
        //if(s_nChkTimeCnt == 2)
        if(s_nChkTimeCnt == 5)
        {
            TIME_GetLocalTime(&tm);
            g_nCurrTimeHour = _loc_Fn_GetCurrentTime(&tm);
            
            if(g_nEcatMasterResetHoursCount != 0)
            {
                for (iCnt = 0; iCnt < g_nEcatMasterResetHoursCount; iCnt++)
                {
                    _loc_EcatAutoRestart(iCnt);
                }
            }

            s_nChkTimeCnt = 0;
        }
        s_nChkTimeCnt++;

        DANDY_SLEEP(1500);
        //DANDY_SLEEP(500);
    }

    g_fSysAliveThExit = TRUE;

    return 0;
}

