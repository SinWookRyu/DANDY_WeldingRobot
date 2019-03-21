/////////////////////////////////////////////////////////////////////////////
//
//  svc_job_exc.c: Job Execution Related Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#include "service.h"
#include "statistics.h"


///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int  g_nJobExecLineIdx = 0;
int  g_fSVCExecThRun = RUN;
int  g_fJobExecRun = OFF;
int  g_fJobExecStepMode = OFF;
int  g_nStepModeLineIndex = 0;
int  g_nHomeSpeed[MAX_ROBOT_COUNT];
int  g_fJobLoadDoneCheck = OFF;
int  g_nCmdLoadCount;
int  g_fJobExecStepFinalMode = OFF;
int  g_fJobStopRequest = OFF;
int  g_nJobLoadCnt = 0;
int  g_fRestartStepMode = OFF;
unsigned long g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_COUNT]  = {0, 0, 0};
unsigned long g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_COUNT]  = {0, 0, 0};
unsigned long g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_COUNT] = {0, 0, 0};
int  g_nOldRunReqLineIdx[ROBOT_RUN_REQ_COUNT] = {-1, -1, -1};


/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_LoadJobData()
//      - Service Name: RSVC_SERV_JOBLOAD
//                      RSVC_SERV_GENMAPFILE

int SVC_LoadJobData(const char* pszCompileTargetFileName, int nOpt)
{
    static int nRet = RESULT_OK;
    const char* pszCompileOpt;
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;

    // Check Compile Option (compiler supported option)
    if(nOpt & JOBASM_AF_ONEPASS && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_ONEPASS);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_CHECK_ONLY && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_CHECK_ONLY);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_STRICT && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_STRICT);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_DANDY1996 && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_DANDY1996);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_NO_COMMENT && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_NO_COMMENT);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_IGNORE_ERROR && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_IGNORE_ERROR);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_NO_EMPTY_LINE && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_NO_EMPTY_LINE);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_NO_EMPTY_ADJ && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_NO_EMPTY_ADJ);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_NO_CINDEX && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_NO_CINDEX);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }
    if(nOpt & JOBASM_AF_CAS && nOpt != 255)
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_CAS);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }

    // default: DANDY1996, no option: 0xff(255)
    if(nOpt == 0)   // force to default compile DANDY1996 format
    {
        nOpt = JOBASM_AF_DANDY1996;
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_DANDY1996);
        VERBOSE_WARNING("Compile Option(Default): %s\n", pszCompileOpt);
    }
    if(nOpt == 255) // set no compile option
    {
        pszCompileOpt = DANDY_DEFINE_NAME_STR(JOBASM_AF_NONE);
        VERBOSE_WARNING("Compile Option: %s\n", pszCompileOpt);
    }

    g_nAssembleOpt = nOpt;

    // set compile target job name (reply message packet)
    memcpy(Job_msg_data.szJobFileName,
           pszCompileTargetFileName,
           sizeof(Job_msg_data.szJobFileName));

    VERBOSE_MESSAGE("Received Job Name: %s\n", Job_msg_data.szJobFileName);

    // disassemble old job data & delete old job file
    if(g_fAseembleDone == TRUE)
    {
        JOB_DoJobDisassemble();
    }

    // compile job data & load job shared memory
    nRet = JOB_DoJobAssemble();

    // set compile target job name (shared memory)
    if(nRet == RESULT_OK)
    {
        memcpy(g_pShm_SysStatus->szCurrJobFileName,
               pszCompileTargetFileName,
               sizeof(g_pShm_SysStatus->szCurrJobFileName));
    }

    // define reply message packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    // if job load by not call command, clear call stack
    if(g_fCallModeJobLoad == FALSE)
    {
        // Clear Call Stack, in case of New Job Load
        if(g_fSavedJobLoad != TRUE)
        {
            MSG_SendPulse(g_coidTE, TESERV_PRG_VAR_CLR, 3);
        }

        memcpy(RM_reply_packet.Data.job_load.szJobFileName,
               pszCompileTargetFileName,
               sizeof(Job_msg_data.szJobFileName));
        RM_reply_packet.nDataSize = sizeof(RMGR_JOB_LOAD_DATA);
    }

    // if job load successful
    if(nRet == RESULT_OK)
    {
        g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;
        g_fJobLoadDoneCheck = ON;
        g_nJobExecLineIdx = 0;
        g_fJobExecRun = OFF;
        g_fJobExecStepMode = OFF;
        g_nJobLoadCnt++;
        
        //if job load by not call command, init run index
        if(g_fCallModeJobLoad == FALSE && g_pShmemTEStatus != NULL)
        {
            g_nTERunNextIdx = INIT_JOBRUNIDX;
        }

        // set command count
        if(pShmem != NULL)
        {
            g_nCmdLoadCount = pShmem->dwCmdLoadCount;
        }

        // if job load by call command
        if(g_fCallModeJobLoad == TRUE)
        {
            g_pShm_SysStatus->fJobLoadDone = 1;
            g_fJobExecRun = ON; //for loading by call cmd, sustain run mode
            g_fJobLoadDoneCheck = OFF;
        }
        else
        {
            // if job load by not call command, real time weld condition data initialized
            if(g_pShmemSC != NULL)
            {
                g_pShmemSC->outputcmd.dbVoltRealTimeCmdOffset = 0;
                g_pShmemSC->outputcmd.dbCurrRealTimeCmdOffset = 0;
            }
        }

        // set current loaded job name
        CRT_strcpy(g_pszLoadedJobFileName,
                   strlen(pszCompileTargetFileName)+1,
                   pszCompileTargetFileName);
        
        //file save currently loaded job name
        nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
        if(nRet == RESULT_OK)
        {
            VERBOSE_MESSAGE("Loaded Job File Name File Save Done!\n");
        }
    }
    else if(nRet == RESULT_ERROR)
    {
        //SVC_DefineErrorState(ON, SVC_ERR_JOB_COMPILE);
        g_fJobLoadDoneCheck = OFF;
        g_fJobExecRun = OFF;
        if(g_fCallModeJobLoad == TRUE)
        {
            g_pShm_SysStatus->fJobLoadDone = -1;
        }
    }

    g_fCallModeJobLoad = FALSE;

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_DumpJobShmData()
//      - Service Name: RSVC_SERV_JOBSHMDUMP

int SVC_DumpJobShmData(int nOpt)
{
    static int nRet = RESULT_OK;

    // Dump Job shared memory data
    nRet = JOB_DumpJobShmem(nOpt);

    // Define Reply Message
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RMGR_JOB_LOAD_DATA);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_JobCmdContinue()
//      - Service Name: RSVC_JOB_CMDCONTINUE

int SVC_JobCmdContinue(void)
{
    int nRet = RESULT_OK;

    // request continue to TE
    nRet = MSG_SendPulse(g_coidTE, TESERV_TIMER_SKIP, 0);
    
    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ExecuteJobProgAuto()
//      - Service Name: RSVC_SERV_JOBEXECAUTO

int SVC_ExecuteJobProgAuto(int nLineIdx)
{
    int nRet = RESULT_OK;
    int iCnt = 0;
    int nStartIdx = 0;
    
    /* check Exec Request Time */
        // calculate difference from new requested time and previous requested time
    g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_AUTORUN] = 
                            g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_POWERON];
    g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_AUTORUN] =
                     abs((int) (g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_AUTORUN] -
                                g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_AUTORUN]));
    g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_AUTORUN] =
                            g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_AUTORUN];
    
        // if run command receive less than THRES_RUN_DUPLICATE_REQ_SEC, ignore second command
    if((g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_AUTORUN] >= 0 &&
        g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_AUTORUN] < THRES_RUN_DUPLICATE_REQ_SEC) &&
       nLineIdx == g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_AUTORUN])
    {
        VERBOSE_WARNING("Too Fast Auto Job Run Request, Job Already Executed!\n"
                        "(ReqDiff: %ld < %d && ReqIdx: %d == OldReqIdx: %d)\n",
                        g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_AUTORUN],
                        THRES_RUN_DUPLICATE_REQ_SEC,
                        nLineIdx,
                        g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_AUTORUN]);

        return RESULT_OK;
    }

        // save old requested run index
    g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_AUTORUN] = nLineIdx;

    // check Exec State
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
    {
        SVC_DefineErrorState(ON, SVC_ERR_JOB_ALREADY_EXEC);
        VERBOSE_ERROR("Job Already Executed!\n");
        return SVC_ERR_JOB_ALREADY_EXEC;
    }

    // check Shock Sensor state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        MSG_SendPulse(g_coidSC, SC_RELEASE_SHOCKSENSOR, OFF);
    }

    // check cart cylinder state
    if(g_pShmemSC != NULL &&
       g_pShmemSC->inputstate.fCartCylinderState == ON)
    {
        SVC_DefineErrorState(ON, SVC_ERR_CART_CYLINDER_STATE);
        VERBOSE_ERROR("Cannot Execute AutoRun. Check Cart Cylinder State.\n");
        return SVC_ERR_CART_CYLINDER_STATE;
    }

    // Request Servo On to SC
    nRet = MSG_SendPulse(g_coidSC, SC_SERV_SERVO, ON);
        
    DANDY_SLEEP(200);
    
    // check Error state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fErrorState == ON)
    {
        VERBOSE_ERROR("Cannot Execute AutoRun. Check Error State.\n");
        return RESULT_ERROR;
    }

    // check E-stop state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        VERBOSE_ERROR("Cannot Execute AutoRun. Check E-Stop State.\n");
        return RESULT_ERROR;
    }

    // Wait for Servo On State
    if(g_pShmemSC != NULL)
    {
        while(g_pShmemSC->outputstate.fServoOnOutState == OFF)
        {
            iCnt++;
            DANDY_SLEEP(10);

            if(iCnt >= 100 || g_pShmemSC->outputstate.fServoOnOutState == ON)
            {
                if(iCnt >= 100)
                {
                    VERBOSE_ERROR("Exceeded Servo On Time Limit!\n");
                }
                iCnt = 0;
                break;
            }
        }
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
                return SVC_ERR_SERVO_OFF;
            }
        }
    }

    // reset next run index
    g_nTERunNextIdx = INIT_JOBRUNIDX;

    // Request Job Start to TE
    RM_packet.nCode  = TESERV_PROG_THRU;
    RM_packet.nValue = nLineIdx;
    RM_packet.nDataSize = 0;

    if(g_coidTE != INVALID_COID)
    {
        nRet = MSG_Send(g_coidTE,
                        &RM_packet,
                        sizeof(RM_packet),
                        &RM_reply_packet,
                        sizeof(RM_reply_packet));
    }
    
    nStartIdx = g_pShmemTEStatus->run_prog_idx;

    // if TE reaction delay, set error state
    if(RM_reply_packet.nCode != RM_packet.nCode)
    {
        VERBOSE_WARNING("Exceeded TE State Change Time Limit!(Start: %d, TEIdx: %d)\n",
                        nStartIdx, g_pShmemTEStatus->run_prog_idx);

        MSG_SendPulse(g_coidSC, SC_SERV_SERVO, OFF);

        SVC_DefineErrorState(ON, SVC_ERR_JOBEXE_TE_REACT);
        return SVC_ERR_JOBEXE_TE_REACT;
    }

	VERBOSE_VERBOSE("Request Job Prog Execute!(Start Line: %d)\n", nLineIdx);

    // set AutoRun mode
    g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;

    g_pShm_SysStatus->nSystemMode = MODE_STAT_AUTORUN;
    g_nSystemMode = MODE_STAT_AUTORUN;
    
    g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;

    /* Can't check TE react state (in case of TE executing time short) */
    // ;
    //DANDY_SLEEP(100);
    DANDY_SLEEP(500);

    g_fJobExecRun = ON;

    // Clear Job Load Done Flag for receive new request
    g_fJobLoadDoneCheck = OFF;

    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ExecuteJobProgDry()
//      - Service Name: RSVC_SERV_JOBEXECDRY

int SVC_ExecuteJobProgDry(int nLineIdx)
{
    int nRet = RESULT_OK;
    int nStartIdx = 0;

    /* check Exec Request Time */
        // calculate difference from new requested time and previous requested time
    g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_DRYRUN] = 
                            g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_POWERON];
    g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_DRYRUN] =
                     abs((int) (g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_DRYRUN] -
                                g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_DRYRUN]));
    g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_DRYRUN] =
                            g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_DRYRUN];
    
        // if run command receive less than THRES_RUN_DUPLICATE_REQ_SEC, ignore second command
    if((g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_DRYRUN] >= 0 &&
        g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_DRYRUN] < THRES_RUN_DUPLICATE_REQ_SEC) &&
       nLineIdx == g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_DRYRUN])
    {
        VERBOSE_WARNING("Too Fast Dry Job Run Request, Job Already Executed!\n"
                        "(ReqDiff: %ld < %d && ReqIdx: %d == OldReqIdx: %d)\n",
                        g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_DRYRUN],
                        THRES_RUN_DUPLICATE_REQ_SEC,
                        nLineIdx,
                        g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_DRYRUN]);

        return RESULT_OK;
    }
        // save old requested run index
    g_nOldRunReqLineIdx[ROBOT_RUN_REQ_TIME_DRYRUN] = nLineIdx;

    // check Exec State
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
    {
        SVC_DefineErrorState(ON, SVC_ERR_JOB_ALREADY_EXEC);
        VERBOSE_ERROR("Job Already Executed!\n");
        return SVC_ERR_JOB_ALREADY_EXEC;
    }

    // check Shock Sensor state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        MSG_SendPulse(g_coidSC, SC_RELEASE_SHOCKSENSOR, OFF);
    }

    // check cart cylinder state
    if(g_pShmemSC != NULL &&
       g_pShmemSC->inputstate.fCartCylinderState == ON)
    {
        SVC_DefineErrorState(ON, SVC_ERR_CART_CYLINDER_STATE);
        VERBOSE_ERROR("Cannot Execute AutoRun. Check Cart Cylinder State.\n");
        return SVC_ERR_CART_CYLINDER_STATE;
    }

    // check Error state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fErrorState == ON)
    {
        VERBOSE_ERROR("Cannot Execute DryRun. Check Error State.\n");
        return RESULT_ERROR;
    }

    // check E-stop state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        VERBOSE_ERROR("Cannot Execute DryRun. Check E-Stop State.\n");
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
                return SVC_ERR_SERVO_OFF;
            }
        }
    }

    // reset next run index
    g_nTERunNextIdx = INIT_JOBRUNIDX;

    // Request Job Start to TE
    RM_packet.nCode  = TESERV_PROG_DRY;
    RM_packet.nValue = nLineIdx;
    RM_packet.nDataSize = 0;

    if(g_coidTE != INVALID_COID)
    {
        nRet = MSG_Send(g_coidTE,
                        &RM_packet,
                        sizeof(RM_packet),
                        &RM_reply_packet,
                        sizeof(RM_reply_packet));
    }
    
    nStartIdx = g_pShmemTEStatus->run_prog_idx;

    // if TE reaction delay, set error state
    if(RM_reply_packet.nCode != RM_packet.nCode)
    {
        VERBOSE_WARNING("Exceeded TE State Change Time Limit!(Start: %d, TEIdx: %d)\n",
                        nStartIdx, g_pShmemTEStatus->run_prog_idx);

        MSG_SendPulse(g_coidSC, SC_SERV_SERVO, OFF);

        SVC_DefineErrorState(ON, SVC_ERR_JOBEXE_TE_REACT);
        return SVC_ERR_JOBEXE_TE_REACT;
    }

	VERBOSE_VERBOSE("Request Job Prog Execute!(Start Line: %d)\n", nLineIdx);
    
    // DryRun mode
    g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;

    g_pShm_SysStatus->nSystemMode = MODE_STAT_DRYRUN;
    g_nSystemMode = MODE_STAT_DRYRUN;

    g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;

    /* Can't check TE react state (in case of TE executing time short) */
    // ;
    //DANDY_SLEEP(100);
    DANDY_SLEEP(500);

    g_fJobExecRun = ON;

    // Clear Job Load Done Flag for receive new request
    g_fJobLoadDoneCheck = OFF;

    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ExecuteJobProgStep()
//      - Service Name: RSVC_SERV_JOBEXECSTEP

int SVC_ExecuteJobProgStep(int nLineIdx)
{
    int nRet = RESULT_OK;

    // check Shock Sensor state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        MSG_SendPulse(g_coidSC, SC_RELEASE_SHOCKSENSOR, OFF);
    }

    // check Error state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fErrorState == ON)
    {
        VERBOSE_ERROR("Cannot Execute StepRun. Check Error State.\n");
        g_fStepExecSVCAct = OFF;
        g_nJobExecLineIdx = g_nStepModeLineIndex;

        return RESULT_ERROR;
    }

    // check E-stop state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        VERBOSE_ERROR("Cannot Execute StepRun. Check E-Stop State.\n");
        g_fStepExecSVCAct = OFF;
        g_nJobExecLineIdx = g_nStepModeLineIndex;

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
                g_fStepExecSVCAct = OFF;
                g_nJobExecLineIdx = g_nStepModeLineIndex;

                return SVC_ERR_SERVO_OFF;
            }
        }
    }

    // check Exec State
    if(g_fJobExecStepMode == ON &&
      (g_nJobExecLineIdx  == g_nStepModeLineIndex))
    {
        // in case of step mode, SVC_ERR_JOB_ALREADY_EXEC error ignored
        VERBOSE_WARNING("Job Already Executed!(ExeStat: %d, StepMode: %d)\n",
                        g_pShm_SysStatus->nExecStat,
                        g_fJobExecStepMode);
        
        g_fStepExecSVCAct = OFF;
        return RESULT_OK;
    }

    /* 1. Start Execution */
	VERBOSE_VERBOSE("Request Job Prog Step Execute!\n");
    SVC_JobProgLineDisplay(nLineIdx);
    
    RM_packet.nCode  = TESERV_PROG_STEP;
    RM_packet.nValue = nLineIdx;
    RM_packet.nDataSize = 0;

    if(g_coidTE != INVALID_COID)
    {
        nRet = MSG_Send(g_coidTE,
                        &RM_packet,
                        sizeof(RM_packet),
                        &RM_reply_packet,
                        sizeof(RM_reply_packet));
    }
    
    /* 2. Set Execution State Variables */
    g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;

    /* Can't check TE react state (in case of TE executing time short) */
    // ;
    DANDY_SLEEP(100);

    // Define Step execution line index
    g_nStepModeLineIndex = nLineIdx;
    g_nJobExecLineIdx = g_nStepModeLineIndex;

    // Define Step mode
    g_fJobExecRun = ON;
    g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;

    g_pShm_SysStatus->nSystemMode = MODE_STAT_STEP;
    g_nSystemMode = MODE_STAT_STEP;

    g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;
    g_fJobExecStepMode = ON;
    g_fJobLoadDoneCheck = OFF;

    if(g_fJobExecStepMode == ON)
    {
        VERBOSE_VERBOSE("Step mode ON (%d)!\n", g_fJobExecStepMode);
    }

    /* 3. Wait for Execution */
    if(g_pShmemTEStatus != NULL)
    {
        while(g_pShmemTEStatus->run_mode    == RUNMODE_PROG &&          //check runmode
             (g_pShm_SysStatus->fErrorState  == OFF  &&                 // rm error
             (g_pShmemSC != NULL       && g_pShmemSC->sysstate.fErrorState == OFF) &&       // sc error
             (g_pShmemTEStatus != NULL && g_pShmemTEStatus->run_error.code == ERR_NONE)) && // te error
             (g_pShm_SysStatus->fEStopState == OFF          ||                  // rm estop
             (g_pShmemSC != NULL && g_pShmemSC->sysstate.fEStopState == OFF)))  // sc estop
        {
            THREAD_Sleep(10);
        }
    }

    THREAD_Sleep(100);
    //THREAD_Sleep(50);

    /* 4. Exec done */
    if((g_pShm_SysStatus->fErrorState  == OFF  &&                   // rm error
       (g_pShmemSC != NULL       && g_pShmemSC->sysstate.fErrorState == OFF) &&         // sc error
       (g_pShmemTEStatus != NULL && g_pShmemTEStatus->run_error.code == ERR_NONE)) &&   // te error
       (g_pShm_SysStatus->fEStopState  == OFF &&                            // rm estop
       (g_pShmemSC != NULL && g_pShmemSC->sysstate.fEStopState == OFF)))    // sc estop
    {
        // normal state
        g_fJobExecStepFinalMode = ON;

        if(g_pShmemTEStatus->run_stop != ON)
        {
            // in step mode line index control
            if(g_nStepModeLineIndex < g_nCmdLoadCount - 1 &&
               g_nTERunNextIdx != END_OF_JOBRUNIDX)
            {
                g_nJobExecLineIdx = g_pShmemTEStatus->run_next_idx;
            }
            if(g_nStepModeLineIndex < g_nCmdLoadCount - 1 &&
               g_nTERunNextIdx == END_OF_JOBRUNIDX)
            {
                g_nJobExecLineIdx = 0;
            }
            else if(g_nStepModeLineIndex >= g_nCmdLoadCount - 1 &&
                    g_nTERunNextIdx != END_OF_JOBRUNIDX)
            {
                g_nJobExecLineIdx =g_nTERunNextIdx;
            }
            else if(g_nStepModeLineIndex >= g_nCmdLoadCount - 1 ||
                    g_nTERunNextIdx == END_OF_JOBRUNIDX)
            {
                g_nJobExecLineIdx = 0;
            }
        }
        else
        {
            VERBOSE_WARNING("TE Stop Request ON! Not increased Job Index!\n");
        }

        VERBOSE_MESSAGE("Step Exec Done!(ExeIdx: %d, StepReq: %d, NextIdx: %d)\n",
                        g_nJobExecLineIdx,
                        g_nStepModeLineIndex,
                        g_nTERunNextIdx);

        THREAD_Sleep(800);
        //THREAD_Sleep(400);

        g_fJobExecRun = OFF;
        g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
        g_nSystemMode = MODE_STAT_MANUAL;
        
        g_fJobExecStepMode = OFF;
        g_fJobExecStepFinalMode = OFF;
    }
    else
    {
        // error state
        if(g_pShm_SysStatus->fErrorState == ON ||
          (g_pShmemSC != NULL       && g_pShmemSC->sysstate.fErrorState == ON) ||       // sc error
          (g_pShmemTEStatus != NULL && g_pShmemTEStatus->run_error.code != ERR_NONE))   // te error
        {
            THREAD_Sleep(500);  // prevent from too fast mode change
            VERBOSE_WARNING("Step Exec Fail[ERROR MODE]!(Idx Set to: %d, Step Req: %d)\n",
                            g_nJobExecLineIdx,
                            g_nStepModeLineIndex);

            g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
            g_pShm_SysStatus->nSystemMode = MODE_STAT_ERROR;
            g_nSystemMode = MODE_STAT_ERROR;
        }

        // estop state
        if(g_pShm_SysStatus->fEStopState == ON ||
          (g_pShmemSC != NULL && g_pShmemSC->sysstate.fEStopState == ON))
        {
            THREAD_Sleep(500);  // prevent from too fast mode change
            VERBOSE_WARNING("Step Exec Fail[ESTOP MODE]!(Idx Set to: %d, Step Req: %d)\n",
                            g_nJobExecLineIdx,
                            g_nStepModeLineIndex);

            g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
            g_pShm_SysStatus->nSystemMode = MODE_STAT_ESTOP;
            g_nSystemMode = MODE_STAT_ESTOP;
        }
        
        g_fJobExecRun = OFF;
        g_fJobExecStepMode = OFF;
    }

    g_fStepExecSVCAct = OFF;

    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobRestart()
//      - Service Name: RSVC_JOB_RESTART

int SVC_JobRestart(void)
{
    int nRet = RESULT_OK;
    int iCnt = 0;
    int nStartIdx = 0;

    /* check Exec Request Time */
        // calculate difference from new requested time and previous requested time
    g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_RESTART] = 
                            g_dwRobotStatTime.rgdwTime[ROBOT_STAT_TIME_POWERON];
    g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_RESTART] =
                     abs((int) (g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_RESTART] -
                                g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_RESTART]));
    g_rgdwExecReqTimeOld[ROBOT_RUN_REQ_TIME_RESTART] =
                            g_rgdwExecReqTimeNew[ROBOT_RUN_REQ_TIME_RESTART];
    
        // if run command receive less than THRES_RUN_DUPLICATE_REQ_SEC, ignore second command
    if(g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_RESTART] >= 0 &&
       g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_RESTART] < THRES_RUN_DUPLICATE_REQ_SEC)
    {
        VERBOSE_WARNING("Too Fast Restart Job Run Request, Job Already Executed!\n"
                        "(ReqDiff: %ld < %d)\n",
                        g_rgdwExecReqTimeDiff[ROBOT_RUN_REQ_TIME_RESTART],
                        THRES_RUN_DUPLICATE_REQ_SEC);

        return RESULT_OK;
    }

    // check Exec State
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
    {
        SVC_DefineErrorState(ON, SVC_ERR_JOB_ALREADY_EXEC);
        VERBOSE_ERROR("Job Already Executed!\n");
        return SVC_ERR_JOB_ALREADY_EXEC;
    }

    // check Shock Sensor state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        MSG_SendPulse(g_coidSC, SC_RELEASE_SHOCKSENSOR, OFF);
    }

    // check cart cylinder state
    if(g_pShmemSC != NULL &&
       g_pShmemSC->inputstate.fCartCylinderState == ON)
    {
        SVC_DefineErrorState(ON, SVC_ERR_CART_CYLINDER_STATE);
        VERBOSE_ERROR("Cannot Execute AutoRun. Check Cart Cylinder State.\n");
        return SVC_ERR_CART_CYLINDER_STATE;
    }

    // Request Servo On to SC
    nRet = MSG_SendPulse(g_coidSC, SC_SERV_SERVO, ON);
        
    DANDY_SLEEP(200);
    
    // check Error state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fErrorState == ON)
    {
        VERBOSE_ERROR("Cannot Execute AutoRun. Check Error State.\n");
        return RESULT_ERROR;
    }

    // check E-stop state
    if(g_pShm_SysStatus != NULL &&
       g_pShm_SysStatus->fEStopState == ON)
    {
        VERBOSE_ERROR("Cannot Execute AutoRun. Check E-Stop State.\n");
        return RESULT_ERROR;
    }

    // Wait for Servo On State
    if(g_pShmemSC != NULL)
    {
        while(g_pShmemSC->outputstate.fServoOnOutState == OFF)
        {
            iCnt++;
            DANDY_SLEEP(10);

            if(iCnt >= 100 || g_pShmemSC->outputstate.fServoOnOutState == ON)
            {
                if(iCnt >= 100)
                {
                    VERBOSE_ERROR("Exceeded Servo On Time Limit!\n");
                }
                iCnt = 0;
                break;
            }
        }
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
                return SVC_ERR_SERVO_OFF;
            }
        }
    }

    // Request Restart Service to TE
    if(g_pShmemTEStatus != NULL)
    {
        if(g_pShmemTEStatus->f_restart_exist == ON)
        {
            RM_packet.nCode  = TESERV_RESTART;
            RM_packet.nValue = 0;
            RM_packet.nDataSize = 0;

            if(g_coidTE != INVALID_COID)
            {
                nRet = MSG_Send(g_coidTE,
                                &RM_packet,
                                sizeof(RM_packet),
                                &RM_reply_packet,
                                sizeof(RM_reply_packet));
            }
            
            // for check TE prog mode
            DANDY_SLEEP(100);

            nStartIdx = g_pShmemTEStatus->run_prog_idx;

            // Set System mode (get from TE)
            g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;

            if(g_pShmemTEStatus->run_prog_mod == PROG_RUNMODE_THRU)
            {
                g_pShm_SysStatus->nSystemMode = MODE_STAT_AUTORUN;
                g_nSystemMode = MODE_STAT_AUTORUN;
                VERBOSE_VERBOSE("Set Sysmode AutoRun!\n");
            }
            else if(g_pShmemTEStatus->run_prog_mod == PROG_RUNMODE_DRY)
            {
                g_pShm_SysStatus->nSystemMode = MODE_STAT_DRYRUN;
                g_nSystemMode = MODE_STAT_DRYRUN;
                VERBOSE_VERBOSE("Set Sysmode DryRun!\n");
            }
            else if(g_pShmemTEStatus->run_prog_mod == PROG_RUNMODE_STEP)
            {
                g_pShm_SysStatus->nSystemMode = MODE_STAT_STEP;
                g_nSystemMode = MODE_STAT_STEP;
                VERBOSE_VERBOSE("Set Sysmode StepRun!\n");
                g_fRestartStepMode = ON;
            }
            else
            {
                g_pShm_SysStatus->nSystemMode = MODE_STAT_AUTORUN;
                g_nSystemMode = MODE_STAT_AUTORUN;
                VERBOSE_VERBOSE("Set Sysmode AutoRun!(mode: %d)\n",
                                g_pShmemTEStatus->run_prog_mod);
            }
            
            g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;

            // if TE reaction delay, set error state
            if(RM_reply_packet.nCode != RM_packet.nCode)
            {
                VERBOSE_WARNING("Exceeded TE State Change Time Limit!(Start: %d, TEIdx: %d)\n",
                                nStartIdx, g_pShmemTEStatus->run_prog_idx);

                MSG_SendPulse(g_coidSC, SC_SERV_SERVO, OFF);

                SVC_DefineErrorState(ON, SVC_ERR_JOBEXE_TE_REACT);
                return SVC_ERR_JOBEXE_TE_REACT;
            }
        }
        else
        {
            VERBOSE_ERROR("Not Ready for Restart Condition!\n");
            MSG_SendPulse(g_coidSC, SC_SERV_SERVO, OFF);

            SVC_DefineErrorState(ON, SVC_ERR_NOT_READY_RESTART);
            return SVC_ERR_NOT_READY_RESTART;
        }
    }    

    // AutoRun mode
    g_fJobExecRun = ON;
    
    // Clear Job Load Done Flag for receive new request
    g_fJobLoadDoneCheck = OFF;

    return (nRet == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_StopJobProg()
//      - Service Name: RSVC_SERV_JOBEXESTOP

int SVC_StopJobProg(void)
{
    int iCnt = 0;
    
    // request stop event to sibling process
    MSG_SendPulse(g_coidSC, SC_SERV_JOBSTOP_EVENT, 0);  // SC
    DANDY_SLEEP(10);
    MSG_SendPulse(g_coidTE, TESERV_STOP, 0);            // TE
  
	VERBOSE_VERBOSE("Stop Service Executed!\n");
    
    // Wait for Sibling Process Service Result..
    g_fJobStopRequest = ON;
    DANDY_SLEEP(500);
    
    // Set system mode
    if(g_pShm_SysStatus != NULL)
    {
        // Confirm Exec Mode
        g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
        
        // Set Manual mode
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
        g_nSystemMode = MODE_STAT_MANUAL;

        g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;
    }

    // wait for TE reaction
    if(g_pShmemTEStatus != NULL)
    {
        while(g_pShmemTEStatus->run_mode != RUNMODE_NONE)
        {
            iCnt++;
            DANDY_SLEEP(2);

            if(iCnt >= 1000 || g_pShmemTEStatus->run_mode == RUNMODE_NONE)
            {
                // if TE reaction delay, set error state
                if(iCnt >= 1000)
                {
                    VERBOSE_ERROR("Exceeded TE State Change Time Limit!\n");
                    SVC_DefineErrorState(ON, SVC_ERR_JOBEXE_TE_REACT);
                }
                iCnt = 0;
                break;
            }
        }
    }

    // wait for SC Estop process done state
    while(g_pShm_SysStatus->fEstopDone == OFF)
    {
        DANDY_SLEEP(10);

        if(g_pShm_SysStatus->fEstopDone == ON)
        {
            VERBOSE_VERBOSE("Stop Process Done!\n");
            break;
        }
    }

    // servo off process
    if(g_fJobExecRun == ON)
    {
        // Request Servo Off to SC
        MSG_SendPulse(g_coidSC, SC_SERV_SERVO, OFF);
        
        // Wait for Servo Off State
        DANDY_SLEEP(100);

        if(g_pShmemSC != NULL)
        {
            while(g_pShmemSC->outputstate.fServoOnOutState == ON)
            {
                iCnt++;
                DANDY_SLEEP(10);

                if(iCnt >= 100 || g_pShmemSC->outputstate.fServoOnOutState == OFF)
                {
                    if(iCnt >= 100)
                    {
                        VERBOSE_WARNING("Exceeded Servo Off Time Limit!\n");
                    }
                    iCnt = 0;
                    break;
                }
            }
        }
    }

    g_fJobExecRun = OFF;

    // Set system mode (for confirmation)
    if(g_pShm_SysStatus != NULL)
    {
        // Confirm Exec Mode
        g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
        
        // Set Manual mode
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
        g_nSystemMode = MODE_STAT_MANUAL;

        g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;
    }

    // reset the data for prevent duplicated job run commnad
    for(iCnt = 0; iCnt < ROBOT_RUN_REQ_COUNT; iCnt++)
    {
        g_rgdwExecReqTimeNew[iCnt]  = 0;
        g_rgdwExecReqTimeOld[iCnt]  = 0;
        g_rgdwExecReqTimeDiff[iCnt] = 0;
        g_nOldRunReqLineIdx[iCnt] = -1;
    }

    return RESULT_OK;
}
