/////////////////////////////////////////////////////////////////////////////
//
//  svc_rcon.c: Robot Control Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable
int g_fHomeExecSVCAct   = OFF;
int g_fWireCutSVCAct    = OFF;
int g_fStepExecSVCAct   = OFF;
int g_fSkipBvarModifySVCAct = OFF;
int g_fGapSkipBvarModifySVCAct = OFF;
int g_nHomeExecIndex  = 0;

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_Error_Reset()
//      - Service Name: RCON_SERV_ERROR_RESET

int SVC_Error_Reset(void)
{
    int nRet =0;
    int nRetSC, nRetTE;

    nRetSC = MSG_SendPulse(g_coidSC, SC_SERV_ALARM_RESET, 0);
    DANDY_SLEEP(100);

    nRetTE = MSG_SendPulse(g_coidTE, TESERV_RESET_ERR, 0);
    DANDY_SLEEP(100);
	VERBOSE_VERBOSE("Error Reset Service Executed!\n");

#if 0
    if(g_pShmemSC != NULL)
    {
        g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset = 0;
        g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset = 0;
    }
#endif

    if(nRetTE == RESULT_OK && nRetSC == RESULT_OK)
    {
        nRet = SVC_DefineErrorState(OFF, SYS_ERR_OK);
    }

    g_fErrorStop = OFF;

    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_TimerTest()
//

int SVC_TimerTest(void)
{
    MSG_SendPulse(g_coidTE, TESERV_TIMETEST, 0); 
	VERBOSE_VERBOSE("Timer Test Start!\n"); 

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_RCON_Home_Move()
//      - Service Name: RCON_SERV_HOME_MOVE

int SVC_RCON_Home_Move(int nHomeIndex)
{
    SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;
    DANDY_JOB_CMD*      pCmdStart;
    int                 nLineIndex;
    unsigned long       dwLoadedJobCmdCount;
    unsigned long       dwHomeModeCmdCount;

    // check Exec State
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
    {
        SVC_DefineErrorState(ON, SVC_ERR_JOB_ALREADY_EXEC);
        VERBOSE_ERROR("Home Move Already Executed!\n");
        g_fHomeExecSVCAct = OFF;
        return SVC_ERR_JOB_ALREADY_EXEC;
    }

    // check Error state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fErrorState == ON)
    {
        VERBOSE_ERROR("Cannot Execute DryRun. Check Error State.\n");
        g_fHomeExecSVCAct = OFF;
        return RESULT_ERROR;
    }

    // check E-stop state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        VERBOSE_ERROR("Cannot Execute DryRun. Check E-Stop State.\n");
        g_fHomeExecSVCAct = OFF;
        return RESULT_ERROR;
    }

    // check Servo-On state
    if(g_Arg.bNoDeadMan == FALSE)
    {
        if(g_pShmemSC != NULL)
        {
            if(g_pShmemSC->outputstate.fServoOnOutState == OFF)
            {
                SVC_DefineErrorState(ON, SVC_ERR_SERVO_OFF);
                VERBOSE_ERROR("Now, Servo Off State! Confirm Servo State!\n");
                g_fHomeExecSVCAct = OFF;
                return SVC_ERR_SERVO_OFF;
            }
        }
    }

    pCmdStart = GET_SHM_JOB_CMD_BUFFER(pShmem);
    
    // save currently loaded cmd count
    dwLoadedJobCmdCount = pShmem->dwCmdLoadCount;

    // set line index = loaded cmd count + 1
    nLineIndex = dwLoadedJobCmdCount + 1;

    // set new cmd count for home cmd process
    dwHomeModeCmdCount = 2;
    pShmem->dwCmdLoadCount = dwHomeModeCmdCount;

    pCmdStart[nLineIndex].nCode = DANDY_JOB_CODE_HOME;
    
    pCmdStart[nLineIndex].nLineId = 0;
    pCmdStart[nLineIndex].nLineIndex = 0;

    pCmdStart[nLineIndex].arg.argHome.valSpeed.nValueType = DANDY_JOB_VAL_TYPE_I_CONST;
    pCmdStart[nLineIndex].arg.argHome.valSpeed.value.IntConst = g_nHomeSpeed[0];

    pCmdStart[nLineIndex].arg.argHome.valHomeFile.nValueType = DANDY_JOB_VAL_TYPE_I_CONST;
    pCmdStart[nLineIndex].arg.argHome.valHomeFile.value.IntConst = nHomeIndex;

    pCmdStart[nLineIndex+1].nCode = DANDY_JOB_CODE_END;
    
    MSG_SendPulse(g_coidTE, TESERV_PROG_STEP, nLineIndex);
    
    VERBOSE_MESSAGE("Target HomePos[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nHomeIndex,
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][0] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][1] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][2] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][3] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][4] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][5] * (180/M_PI));

    VERBOSE_VERBOSE("Home Move Execute!\n");
    
    g_pShm_SysStatus->nWorkType = WORK_TYPE_HOME;

    g_pShm_SysStatus->nSystemMode = MODE_STAT_DRYRUN;
    g_nSystemMode = MODE_STAT_DRYRUN;

    g_fJobLoadDoneCheck = OFF;

    g_fJobExecRun = ON;

    DANDY_SLEEP(100);
    
    if(g_pShmemTEStatus != NULL)
    {
        while(g_pShmemTEStatus->run_mode == RUNMODE_PROG)
        {
            //DANDY_SLEEP(10);
            THREAD_Sleep(20);
        }
    }

    // recover cmd load count
    if(pShmem->dwCmdLoadCount == dwHomeModeCmdCount)
    {
        pCmdStart[nLineIndex].nCode = DANDY_JOB_CODE_NOP;
        pCmdStart[nLineIndex + 1].nCode = DANDY_JOB_CODE_NOP;
        pShmem->dwCmdLoadCount = dwLoadedJobCmdCount;
        VERBOSE_VERBOSE("Reset Cmd Count!(%ld -> %ld)\n", 
                        dwHomeModeCmdCount,
                        pShmem->dwCmdLoadCount);
    }

    g_fHomeExecSVCAct = OFF;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_RCON_WireCut_JobCall()
//      - Service Name: RCON_WIRECUT_JOB_CALL

int SVC_RCON_WireCut_JobCall(void)
{
    static char* s_pszOrigJobFileName;

    s_pszOrigJobFileName = (char *)DEBUG_MALLOC(PATH_NAME_BUFFER_SIZE);

    // save original job name
    memcpy(s_pszOrigJobFileName, g_pszLoadedJobFileName, PATH_NAME_BUFFER_SIZE);

    g_fSavedJobLoad = TRUE;

    // load wire cut job
    SVC_LoadJobData(g_pszWireCutJobFileName, JOBASM_AF_DANDY1996);

    SVC_ExecuteJobProgAuto(0);  // start at 1st line

    if(g_pShm_SysStatus == NULL)
    {
        g_fWireCutSVCAct = OFF;
        return RESULT_ERROR;
    }

    g_pShm_SysStatus->nWorkType = WORK_TYPE_WIRECUT;

    g_pShm_SysStatus->nSystemMode = MODE_STAT_DRYRUN;
    g_nSystemMode = MODE_STAT_DRYRUN;

    g_fJobLoadDoneCheck = OFF;

    g_fJobExecRun = ON;

    DANDY_SLEEP(200);

    // wait for wire cut job exec done
    while(g_pShm_SysStatus->nExecStat != EXEC_STAT_TERMINATING)
    {
        THREAD_Sleep(20);
    }

    // reload original job
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_TERMINATING)
    {
        DANDY_SLEEP(1000);
        SVC_LoadJobData(s_pszOrigJobFileName,  JOBASM_AF_DANDY1996);
    }

    g_fSavedJobLoad = FALSE;

    DEBUG_FREE(s_pszOrigJobFileName);
    s_pszOrigJobFileName = NULL;

    g_fWireCutSVCAct = OFF;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ModifyBvarForSkipCond()
//      - Service Name: RSVC_SERV_SET_SKIPCOND

int SVC_ModifyBvarForSkipCond(void)
{
    static int nRetmsg;

    RM_packet.nCode  = RSVC_JOB_CONSTVAREDIT;
    RM_packet.nValue = DANDY_JOB_VAL_TYPE_B_VAR;
    RM_packet.Data.const_var_val.nVarIndex = g_nSkipBvarIdx;
    RM_packet.Data.const_var_val.value.ByteVal = g_fSkip;
    RM_packet.nDataSize = sizeof(RM_packet.Data.const_var_val);

    nRetmsg = MSG_Send(g_coidRM,
                       &RM_packet,
                       sizeof(RM_packet), //RMGR_PACKET_HEAD_LEN,
                       &RM_reply_packet,
                       sizeof(RM_reply_packet)); //RMGR_REPLY_PACKET_HEAD_LEN);

    RM_reply_packet.Data.const_var_val.nVarIndex = g_nSkipBvarIdx;
    RM_reply_packet.Data.const_var_val.value.ByteVal = g_fSkip;
    RM_reply_packet.nCode     = RM_packet.nCode;
    RM_reply_packet.nValue    = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);

    g_fSkip = 0;
    g_nSkipBvarIdx = -1;

    g_fSkipBvarModifySVCAct = OFF;

    return 0;
}


#if 0
///////////////////////////////////////
//
//  Function: SVC_ModifyBvarForGapSkipCond()
//      - Service Name: RSVC_SERV_SET_GAPSKIPCOND

int SVC_ModifyBvarForGapSkipCond(int nGapSkipBvarIdx, int nGapSkipCond)
{
    static int nRetmsg;

    RM_packet.nCode  = RSVC_JOB_CONSTVAREDIT;
    RM_packet.nValue = DANDY_JOB_VAL_TYPE_B_VAR;
    RM_packet.Data.const_var_val.nVarIndex = nGapSkipBvarIdx;
    RM_packet.Data.const_var_val.value.ByteVal = nGapSkipCond;
    RM_packet.nDataSize = sizeof(RM_packet.Data.const_var_val);

    nRetmsg = MSG_Send(g_coidRM,
                       &RM_packet,
                       sizeof(RM_packet),
                       &RM_reply_packet,
                       sizeof(RM_reply_packet));

    RM_reply_packet.Data.const_var_val.nVarIndex = nGapSkipBvarIdx;
    RM_reply_packet.Data.const_var_val.value.ByteVal = nGapSkipCond;
    RM_reply_packet.nCode     = RM_packet.nCode;
    RM_reply_packet.nValue    = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);

    return 0;
}
#endif


///////////////////////////////////////
//
//  Function: SVC_ModifyBvarForGapSkipCond()
//      - Service Name: RSVC_SERV_SET_GAPSKIPCOND

int SVC_ModifyBvarForGapSkipCond(int nGapSkipBvarIdx, int nGapSkipCond)
{

    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    BYTE*           pBVarStart;

    pBVarStart = GET_SHM_JOB_BVA_BUFFER(pShmem);

    if(pBVarStart[nGapSkipBvarIdx] != nGapSkipCond)
    {
        VERBOSE_VERBOSE("[B%d] Var Modified %d -> %d\n",
                        nGapSkipBvarIdx,
                        (int) pBVarStart[nGapSkipBvarIdx],
                        nGapSkipCond);
    }

    pBVarStart[nGapSkipBvarIdx] = (BYTE) nGapSkipCond;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SVC_ExtraRobotControlSVCThread()
//

THREAD_ENTRY_TYPE SVC_ExtraRobotControlSVCThread(void* pParam)
{
    int nRet = RESULT_OK;

    while(g_fExtraRconSVCThRun == RUN)
	{
        if(g_fHomeExecSVCAct == ON)
        {
            nRet = SVC_RCON_Home_Move(g_nHomeExecIndex);

            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_HOME_MOVE);
                nRet = RESULT_ERROR;
            }
        }

        if(g_fWireCutSVCAct == ON)
        {
            nRet = SVC_RCON_WireCut_JobCall();

            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_WIRECUT_JOB_CALL);
                nRet = RESULT_ERROR;
            }
        }

        if(g_fStepExecSVCAct == ON)
        {
            nRet = SVC_ExecuteJobProgStep(g_nStepModeLineIndex);

            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOBEXECSTEP);
                nRet = RESULT_ERROR;
            }
        }

        if(g_fSkipBvarModifySVCAct == ON)
        {
            SVC_ModifyBvarForSkipCond();
        }
                
        if(g_fGapSkipBvarModifySVCAct == ON)
        {
            SVC_ModifyBvarForGapSkipCond(g_nLeftVertSkipBvar,     g_nLeftVertGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nRightVertSkipBvar,    g_nRightVertGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nLeftCollarSkipBvar,   g_nLeftCollarGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nRightCollarSkipBvar,  g_nRightCollarGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nHorizontalSkipBvar,   g_nHorizontalGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nLeftBracketSkipBvar,  g_nLeftBracketGapSkipCond);
            SVC_ModifyBvarForGapSkipCond(g_nRightBracketSkipBvar, g_nRightBracketGapSkipCond);

            g_fGapSkipBvarModifySVCAct = OFF;
        }
        
        DANDY_SLEEP(10);
    }

    g_fExtraRconSVCThExit = TRUE;

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_RCON_SetVoltageRealTimeOffset()
//      - Service Name: RCON_VOLT_REALTIME_OFFSET

int SVC_RCON_SetVoltageRealTimeOffset(int nOpt)
{
    if(nOpt == 1)       //increase
    {
        if(g_pShmemSC != NULL)
        {
            g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset =
                g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset + g_dbVoltRealTimeCmdOffsetUnit;
        }
    }
    else if(nOpt == -1) //decrease
    {
        if(g_pShmemSC != NULL)
        {
            g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset =
                g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset - g_dbVoltRealTimeCmdOffsetUnit;
        }
    }
    else if(nOpt == 0)  //for just data send
    {
        ;
    }
    
    if(g_pShmemSC != NULL)
    {
        RM_reply_packet.Data.volt_offset.dbVoltRealTimeCmdOffsetUnit = 
                                g_dbVoltRealTimeCmdOffsetUnit;
        RM_reply_packet.Data.volt_offset.dbVoltRealTimeCmdOffset = 
                                g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.volt_offset);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_RCON_SetCurrentRealTimeOffset()
//      - Service Name: RCON_CURR_REALTIME_OFFSET

int SVC_RCON_SetCurrentRealTimeOffset(int nOpt)
{
    if(nOpt == 1)       //increase
    {
        if(g_pShmemSC != NULL)
        {
            g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset =
                g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset + g_dbCurrRealTimeCmdOffsetUnit;
        }
    }
    else if(nOpt == -1) //decrease
    {
        if(g_pShmemSC != NULL)
        {
            g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset =
                g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset - g_dbCurrRealTimeCmdOffsetUnit;
        }
    }
    else if(nOpt == 0)  //for just data send
    {
        ;
    }
    
    if(g_pShmemSC != NULL)
    {
        RM_reply_packet.Data.curr_offset.dbCurrRealTimeCmdOffsetUnit = 
                                g_dbCurrRealTimeCmdOffsetUnit;
        RM_reply_packet.Data.curr_offset.dbCurrRealTimeCmdOffset = 
                                g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.curr_offset);

    return RESULT_OK;
}
