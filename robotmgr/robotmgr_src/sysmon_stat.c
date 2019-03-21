/////////////////////////////////////////////////////////////////////////////
//
//  sysmon_stat.c: System Status Monitoring
//                                            2013.06.18  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"
#include "service.h"

///////////////////////////////////////

#define VERBOSE_NAME    "RM_VERBOSE_"

int g_nSiblingProcState_RM = -1;
int g_nSiblingProcState_TE = -1;
int g_nSiblingProcState_SC = -1;

int g_fJobModeOn = OFF;
int g_fErrorStop = OFF;

static int _loc_SYSMON_CheckJobExecTerminating(void);

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_SetSystemModeString()
//

static int _loc_SYSMON_SetSystemModeString(void)
{
    if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "ESTOP");

        return RESULT_OK;
    }

    if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "ERROR");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_INIT)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "INIT");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_MANUAL)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "MANUAL");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "AUTORUN");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "DRYRUN");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_TERMINATE)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "TERMINATE");
    }
    else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_STEP)
    {
        CRT_strcpy(g_szSystemMode, SYSMODE_NAME_LEN, "STEP");
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_SetExecStatString()
//          - Write EXEC_MODE Name String

static int _loc_SYSMON_SetExecStatString(void)
{
    /* Write EXEC_MODE Name String */
    if(g_pShm_SysStatus->nExecStat == EXEC_STAT_IDLE)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "IDLE");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_PREPARATION)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "PREPARATION");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_WARM_EXEC)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "WARM_EXEC");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_WARM_EXCEPT)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "WARM_EXCEPT");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "EXECUTING");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXCEPT)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "EXCEPT");
    }
    else if(g_pShm_SysStatus->nExecStat == EXEC_STAT_TERMINATING)
    {
        CRT_strcpy(g_szExecStat, EXECSTATE_NAME_LEN, "TERMINATING");
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_SetWorkTypeString()
//

static int _loc_SYSMON_SetWorkTypeString(void)
{
    if(g_pShm_SysStatus->nWorkType == WORK_TYPE_NONE)
    {
        CRT_strcpy(g_szWorkType, WORKTYPE_NAME_LEN, "WTYPE_NONE");
    }
    if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB)
    {
        CRT_strcpy(g_szWorkType, WORKTYPE_NAME_LEN, "WTYPE_JOB");
    }
    if(g_pShm_SysStatus->nWorkType == WORK_TYPE_HOME)
    {
        CRT_strcpy(g_szWorkType, WORKTYPE_NAME_LEN, "WTYPE_HOME");
    }
    if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOG)
    {
        CRT_strcpy(g_szWorkType, WORKTYPE_NAME_LEN, "WTYPE_JOG");
    }
    if(g_pShm_SysStatus->nWorkType == WORK_TYPE_WIRECUT)
    {
        CRT_strcpy(g_szWorkType, WORKTYPE_NAME_LEN, "WTYPE_WCUT");
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckSystemMode()
//

static int _loc_SYSMON_CheckSystemMode(void)
{
    if(g_pShm_SysStatus->fEStopState == TRUE)
    {
        g_pShm_SysStatus->nSystemMode = MODE_STAT_ESTOP;
        g_nSystemMode = MODE_STAT_ESTOP;

        if(g_fJobExecRun == ON)
        {
            g_fJobExecRun = OFF;
        }

        if(g_fJobExecStepMode == ON)
        {
            g_fJobExecStepMode = OFF;
        }

        return 0;
    }

    if(g_pShm_SysStatus->fErrorState == TRUE)
    {
        g_pShm_SysStatus->nSystemMode = MODE_STAT_ERROR;
        g_nSystemMode = MODE_STAT_ERROR;

        if(g_fJobExecRun == ON)
        {
            g_fJobExecRun = OFF;
        }

        if(g_fJobExecStepMode == ON)
        {
            g_fJobExecStepMode = OFF;
        }

        return 0;
    }

    if(((g_pShm_SysStatus->fErrorState != TRUE &&
         g_pShm_SysStatus->fEStopState != TRUE) &&
        (g_nSystemMode != MODE_STAT_ENTRY &&
         g_nSystemMode != MODE_STAT_INIT &&
         g_nSystemMode != MODE_STAT_TERMINATE&&
         g_nSystemMode != MODE_STAT_DRYRUN &&
         g_nSystemMode != MODE_STAT_STEP &&
         g_nSystemMode != MODE_STAT_AUTORUN)) ||
         g_nSystemMode == MODE_STAT_MANUAL)
    {
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
    }
    else if(g_nSystemMode == MODE_STAT_ENTRY)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_ENTRY;
    }
    else if(g_nSystemMode == MODE_STAT_INIT)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_INIT;
    }
    else if(g_nSystemMode == MODE_STAT_TERMINATE)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_TERMINATE;
    }
    else if(g_nSystemMode == MODE_STAT_DRYRUN)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_DRYRUN;
    }
    else if(g_nSystemMode == MODE_STAT_AUTORUN)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_AUTORUN;
    }
    else if(g_nSystemMode == MODE_STAT_STEP)
    {
         g_pShm_SysStatus->nSystemMode = MODE_STAT_STEP;
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckExecStatAndWorkType()
//

static int  _loc_SYSMON_CheckExecStatAndWorkType(void)
{
    /* Determine EXEC_MODE */
    if(g_pShmemTEStatus != NULL)
    {
        g_nTERunNextIdx = g_pShmemTEStatus->run_next_idx;

         /* Job Work Type - No Estop, No Error */
        if(g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ESTOP &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR)
        {
            // TE Run State, Set Executing
            if((g_pShmemTEStatus->run_mode == RUNMODE_PROG ||
                g_pShmemTEStatus->run_mode == RUNMODE_RESTART) &&
                g_fJobStopRequest != ON &&
                g_pShm_SysStatus->nExecStat != EXEC_STAT_EXECUTING &&
                g_fJobExecRun == ON)
            {
                g_pShm_SysStatus->nExecStat = EXEC_STAT_EXECUTING;
                VERBOSE_VERBOSE("Set Executing Mode!\n");
            }
            
            // AutoRun, DryRun State, Set terminating
            if((g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
                g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN  ||
                g_pShm_SysStatus->nSystemMode == MODE_STAT_MANUAL) &&
                g_fJobStopRequest == ON &&
                g_pShm_SysStatus->nExecStat != EXEC_STAT_TERMINATING)
            {
                DANDY_SLEEP(500);   // prevent from too fast mode change (in case exec time short)
                g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;

                if(g_pShmemTEStatus != NULL)
                {
                    VERBOSE_VERBOSE("Set Terminating Mode!(Stop Req)\n");
#if 0
                    VERBOSE_VERBOSE("Set Terminating Mode!(Stop Req)"
                                    "(JobRun: %d, StopReq: %d, Idx: %d, NextIdx: %d)\n",
                                    g_fJobExecRun, g_fJobStopRequest,
                                    g_pShmemTEStatus->run_prog_idx, g_nTERunNextIdx);
#endif
                }

                g_fJobStopRequest = OFF;
            }

            // StepRun & Restart State, Set terminating
            if(g_pShm_SysStatus->nSystemMode == MODE_STAT_STEP &&
              (g_pShmemTEStatus->run_mode != RUNMODE_PROG &&
               g_pShmemTEStatus->run_mode != RUNMODE_RESTART) &&
               g_fRestartStepMode == ON &&
               g_pShm_SysStatus->nExecStat != EXEC_STAT_TERMINATING)
            {
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
                        g_nJobExecLineIdx = g_nTERunNextIdx;
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

                _loc_SYSMON_CheckJobExecTerminating();
            }

            // Job Compile State, Set preparation
            if(g_pShm_SysStatus->nExecStat != EXEC_STAT_EXECUTING &&
               g_fJobExecRun == OFF && 
               g_fJobLoadDoneCheck == ON &&
               g_fJobExecStepMode != ON &&
               g_pShm_SysStatus->nExecStat   != EXEC_STAT_PREPARATION &&
               g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR)
            {
                g_pShm_SysStatus->nExecStat = EXEC_STAT_PREPARATION;
            }
        }
        
        /* Home or WireCut Work Type - No Estop, No Error */
        if((g_pShm_SysStatus->nWorkType == WORK_TYPE_HOME    ||
            g_pShm_SysStatus->nWorkType == WORK_TYPE_WIRECUT)&&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ESTOP  &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR)
        {
            if(g_fJobExecRun == ON && g_pShmemTEStatus->run_mode == RUNMODE_PROG)
            {
                g_pShm_SysStatus->nExecStat = EXEC_STAT_EXCEPT;
            }

            if(g_pShmemTEStatus->run_mode != RUNMODE_PROG &&
               g_fJobExecRun == ON)
            {
                g_pShm_SysStatus->nExecStat = EXEC_STAT_WARM_EXCEPT;
            }

            if(g_fJobExecRun == ON && g_pShmemTEStatus->run_mode == RUNMODE_NONE)
            {
                g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
                g_nSystemMode = MODE_STAT_MANUAL;
                g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
            }
        }
        
        /* Jog Work Type - No Estop, No Error */
        if(g_pShmemTEStatus->run_mode == RUNMODE_JOG &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ESTOP &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR)
        {
            g_pShm_SysStatus->nWorkType = WORK_TYPE_JOG;
            g_pShm_SysStatus->nExecStat = EXEC_STAT_EXCEPT;
            g_fJobModeOn = ON;
        }
        else if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB     &&
                g_pShm_SysStatus->nWorkType != WORK_TYPE_HOME    &&
                g_pShm_SysStatus->nWorkType != WORK_TYPE_WIRECUT &&
                g_fJobModeOn == ON &&
                g_pShmemTEStatus->run_mode == RUNMODE_NONE)
        {
            g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
            g_fJobModeOn = OFF;
        }

        /* Estop */
        if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP)
        {
            if((g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING ||
                g_pShm_SysStatus->nExecStat == EXEC_STAT_EXCEPT    ||
                g_pShm_SysStatus->nExecStat == EXEC_STAT_PREPARATION) &&    // for fast execution
                g_pShm_SysStatus->nExecStat != EXEC_STAT_TERMINATING)
            {
                SYSMON_CheckJobExecLineIndex();
                THREAD_Sleep(1000);  // prevent from too fast mode change
                g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
                VERBOSE_VERBOSE("Set Terminating Mode!(Estop State)\n");
            }
        }

        /* Error */
        if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR)
        {
            if((g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING ||
                g_pShm_SysStatus->nExecStat == EXEC_STAT_EXCEPT    ||
                g_pShm_SysStatus->nExecStat == EXEC_STAT_PREPARATION) &&    // for fast execution
                g_pShm_SysStatus->nExecStat != EXEC_STAT_TERMINATING)
            {
                SYSMON_CheckJobExecLineIndex();
                THREAD_Sleep(1000);  // prevent from too fast mode change
                g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
                VERBOSE_VERBOSE("Set Terminating Mode!(Error State)\n");
            }
        }

        /* Idle */
        if(g_pShm_SysStatus->nWorkType   == WORK_TYPE_NONE &&
          (g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN))
        {
            g_pShm_SysStatus->nExecStat = EXEC_STAT_IDLE;
        }
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckSiblingProcessState(char* szProcessName)
//

static int _loc_SYSMON_CheckSiblingProcessState(char* szProcessName)
{
    /* RM */
    if(strcmp(szProcessName, ROBOMAN_ABB_NAME) == 0)
    {
        if(g_pShm_SysStatus->nSystemMode == MODE_STAT_INIT ||
           g_pShm_SysStatus->nSystemMode == MODE_STAT_ENTRY)
        {
            CRT_strcpy(g_szRMState, SIBLINGSTATE_NAME_LEN, "[RM]Entry");
        }

        if((g_fSVCExecThRun == RUN || g_fServiceStatus == RUN) &&
          (g_pShm_SysStatus->fErrorState != TRUE &&
           g_pShm_SysStatus->fEStopState != TRUE))
        {
            CRT_strcpy(g_szRMState, SIBLINGSTATE_NAME_LEN, "[RM]Ready");
            g_nSiblingProcState_RM = RUN;
        }
        else if(g_pShm_SysStatus->fErrorState == TRUE)
        {
            CRT_strcpy(g_szRMState, SIBLINGSTATE_NAME_LEN, "[RM]Error");
            g_nSiblingProcState_RM = RUN;
        }

        if(g_pShm_SysStatus->fEStopState == TRUE)
        {
            CRT_strcpy(g_szRMState, SIBLINGSTATE_NAME_LEN, "[RM]Estop");
            g_nSiblingProcState_RM = RUN;
        }

        if(g_fSVCExecThRun == STOP || g_fServiceStatus == STOP ||
           g_fConsoleExitAct == TRUE)
        {
            CRT_strcpy(g_szRMState, SIBLINGSTATE_NAME_LEN, "[RM]Exit");
            g_nSiblingProcState_RM = STOP;
        }
    }
    /* TE */
    else if(strcmp(szProcessName, TASKEXEC_ABB_NAME) == 0)
    {
        if(g_pShm_SysStatus->fInitProcTE == FALSE &&
         ((g_coidTE == INVALID_COID && g_pShmemTEStatus == NULL) &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_TERMINATE))
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]NoAct");
        }

        if((g_coidTE != INVALID_COID && g_pShmemTEStatus != NULL) &&
           (g_pShm_SysStatus->fInitProcTE == FALSE &&
           (g_pShm_SysStatus->nSystemMode == MODE_STAT_INIT ||
            g_pShm_SysStatus->nSystemMode == MODE_STAT_ENTRY)))
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]Entry");
        }
        else if(g_pShm_SysStatus->fInitProcTE == TRUE)
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]Ready");
            g_nSiblingProcState_TE = RUN;
        }
        else if(((g_pShm_SysStatus->fInitProcTE == FALSE &&
                  g_pShm_SysStatus->fExitProcTE == FALSE) &&
                 (g_pShm_SysStatus->nSystemMode == MODE_STAT_MANUAL ||
                  g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
                  g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)) ||
                 (g_pShm_SysStatus->fInitProcTE == TRUE &&
                  g_pShmemTEStatus->run_error.code != ERR_NONE))
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]Error");
            g_nSiblingProcState_TE = RUN;
        }

        if(g_pShm_SysStatus->fEStopState == TRUE)
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]EStop");
            g_nSiblingProcState_TE = RUN;
        }

        if(g_pShm_SysStatus->fExitProcTE == TRUE || g_fConsoleExitAct == TRUE)
        {
            CRT_strcpy(g_szTEState, SIBLINGSTATE_NAME_LEN, "[TE]Exit");
            g_nSiblingProcState_TE = STOP;
        }
    }
    /* SC */
    else if(strcmp(szProcessName, SERVOCON_ABB_NAME) == 0)
    {
        if(g_pShm_SysStatus->fInitProcSC == FALSE &&
         ((g_coidSC == INVALID_COID && g_pShmemSC == NULL) &&
           g_pShm_SysStatus->nSystemMode != MODE_STAT_TERMINATE))
        {
            CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]NoAct");
        }

        if((g_coidSC != INVALID_COID && g_pShmemSC != NULL) &&
           (g_pShm_SysStatus->fInitProcSC == FALSE &&
           (g_pShm_SysStatus->nSystemMode == MODE_STAT_INIT ||
            g_pShm_SysStatus->nSystemMode == MODE_STAT_ENTRY)))
        {
            CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]Entry");
        }
        else if(g_pShm_SysStatus->fInitProcSC == TRUE)
        {
            CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]Ready");
            g_nSiblingProcState_SC = RUN;
        }

        if(g_pShmemSC != NULL)
        {
            if(((g_pShm_SysStatus->fInitProcSC == FALSE &&
                      g_pShm_SysStatus->fExitProcSC == FALSE) &&
                     (g_pShm_SysStatus->nSystemMode == MODE_STAT_MANUAL ||
                      g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
                      g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)) ||
                      (g_pShm_SysStatus->fInitProcSC == TRUE &&
                       g_pShmemSC->sysstate.fErrorState == TRUE))
            {
                CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]Error");
                g_nSiblingProcState_SC = RUN;
            }
        }

        if(g_pShm_SysStatus->fEStopState == TRUE)
        {
            CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]EStop");
            g_nSiblingProcState_SC = RUN;
        }

        if(g_pShm_SysStatus->fExitProcSC == TRUE || g_fConsoleExitAct == TRUE)
        {
            CRT_strcpy(g_szSCState, SIBLINGSTATE_NAME_LEN, "[SC]Exit");
            g_nSiblingProcState_SC = STOP;
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSMON_CheckJobExecTerminating(void)
//
static int _loc_SYSMON_CheckJobExecTerminating(void)
{
    int nRet = RESULT_OK;

    // Wait for Status Change..
    //DANDY_SLEEP(100);

    if(g_pShm_SysStatus->nWorkType   == WORK_TYPE_JOB &&
       g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN)
    {
        // Wait for Status Change..
        DANDY_SLEEP(500);

        if(g_fJobExecRun == ON)
        {
            nRet = SVC_StopJobProg();
        }
    }
    
    if(g_pShm_SysStatus->nWorkType   == WORK_TYPE_JOB &&
       g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)
    {
        g_fJobStopRequest = ON;

        // Wait for Status Change..
        DANDY_SLEEP(500);

        // Confirm Exec Mode
        g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
        
        // Set Manual mode
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
        g_nSystemMode = MODE_STAT_MANUAL;

        g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;

        g_fJobExecRun = OFF;
        g_fJobStopRequest = OFF;
    }

    if(g_pShm_SysStatus->nWorkType   == WORK_TYPE_JOB &&
       g_pShm_SysStatus->nSystemMode == MODE_STAT_STEP &&
       g_fRestartStepMode == ON)
    {
        g_fJobStopRequest = ON;

        // Wait for Status Change..
        DANDY_SLEEP(500);

        // Confirm Exec Mode
        g_pShm_SysStatus->nExecStat = EXEC_STAT_TERMINATING;
        
        // Set Manual mode
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
        g_nSystemMode = MODE_STAT_MANUAL;

        g_pShm_SysStatus->nWorkType = WORK_TYPE_JOB;

        g_fJobExecRun = OFF;
        g_fJobStopRequest = OFF;
        g_fRestartStepMode = OFF;
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSMON_CheckJobExecLineIndex(void)
//
int SYSMON_CheckJobExecLineIndex(void)
{
    if(g_pShmemTEStatus != NULL)
    {
        /* AutoRun & DryRun */
        if((g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
            g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN) &&
            g_fJobExecStepMode == OFF &&
            g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING)
        {
            // TE line index operates Last Line Index + 1, so set index limit
            if(g_pShmemTEStatus->run_prog_idx < g_nCmdLoadCount - 1)
            {
                g_nJobExecLineIdx = g_pShmemTEStatus->run_prog_idx;
            }
            else if(g_pShmemTEStatus->run_prog_idx >= g_nCmdLoadCount - 1)
            {
                g_nJobExecLineIdx = g_nCmdLoadCount - 1;
            }
        }

        if(g_fJobExecStepMode == OFF &&
          (g_pShm_SysStatus->nExecStat == EXEC_STAT_TERMINATING ||
           g_fJobStopRequest == ON))
        {
            // if Job exec line index is bigger than max index count, set index to zero
            if(g_nTERunNextIdx == END_OF_JOBRUNIDX && g_fJobExecRun == ON)
            {
                g_nJobExecLineIdx = 0;
                g_pShm_SysStatus->nJobRunIndex = g_nJobExecLineIdx;
            }
        }

        if((g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
            g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN) &&
            g_fJobExecStepMode == OFF &&
           (g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP ||
            g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR))
        {
            if(g_pShmemTEStatus->run_prog_idx < g_nCmdLoadCount - 1)
            {
                g_nJobExecLineIdx = g_pShmemTEStatus->run_prog_idx;
            }
        }

        // Make shared memory index refresh
        g_pShm_SysStatus->nJobRunIndex = g_nJobExecLineIdx;
    }
    else
    {
        return RESULT_ERROR;
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SYSMON_SysStatusCheckThread()
//

THREAD_ENTRY_TYPE SYSMON_SysStatusCheckThread(void* pParam)
{
    int nOldExecStat = 0;
    int fExecStatChanged;

    while(g_fSysModeStatusThRun == RUN)
	{
        if(g_pShm_SysStatus != NULL)
        {
            if(nOldExecStat != g_pShm_SysStatus->nExecStat)
            {
                fExecStatChanged = ON;
            }
            else
            {
                fExecStatChanged = OFF;
            }

            nOldExecStat = g_pShm_SysStatus->nExecStat;

            // check system mode state
            _loc_SYSMON_CheckSystemMode();

            // set system mode string
            _loc_SYSMON_SetSystemModeString();

            // check executioin state & work type
            _loc_SYSMON_CheckExecStatAndWorkType();
            
            // set excution state string
            _loc_SYSMON_SetExecStatString();

            // set work type string
            _loc_SYSMON_SetWorkTypeString();

            // check job exec line index
            SYSMON_CheckJobExecLineIndex();

            // in case of auto exec done, servo off
            if((g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN ||
                g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN) &&
               (g_fJobExecRun == ON && g_pShmemTEStatus->run_mode == RUNMODE_NONE &&
                g_nTERunNextIdx == END_OF_JOBRUNIDX))
            {
                if(g_pShmemTEStatus != NULL)
                {
                    VERBOSE_WARNING("Entered Job Stop State!!(ProgIdx: %d, NextIdx: %d)\n",
                                    g_pShmemTEStatus->run_prog_idx, g_nTERunNextIdx);
                }

                _loc_SYSMON_CheckJobExecTerminating();
            }

            // set sibling process state (RM)
            _loc_SYSMON_CheckSiblingProcessState(ROBOMAN_ABB_NAME);

            // set sibling process state (TE)
            _loc_SYSMON_CheckSiblingProcessState(TASKEXEC_ABB_NAME);

            // set sibling process state (SC)
            _loc_SYSMON_CheckSiblingProcessState(SERVOCON_ABB_NAME);
        }

        DANDY_SLEEP(1);
    }
    
    g_nSiblingProcState_RM = STOP;
    g_nSiblingProcState_TE = STOP;
    g_nSiblingProcState_SC = STOP;

    g_fSysModeStatusThExit = TRUE;

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: SYSMON_SysIOCheckThread()
//
#define ESTOP_MODE                      0
#define STOP_MODE                       1

THREAD_ENTRY_TYPE SYSMON_SysIOCheckThread(void* pParam)
{
    int nCurrentSysMode = 0;
    int nOldSysMode     = 0;
    int fSysModeChanged;
    int fServoOffErr    = OFF;
    
    while(g_fSysIOStatusThRun == RUN)
	{
        if(g_pShmemSC != NULL)
        {
            if(nOldSysMode != g_pShm_SysStatus->nSystemMode)
            {
                fSysModeChanged = ON;
            }
            else
            {
                fSysModeChanged = OFF;
            }

            nOldSysMode  = g_pShm_SysStatus->nSystemMode;

            if(g_pShmemSC->sysstate.fEStopState == ON)
            {
                g_pShm_SysStatus->fEStopState = TRUE;
                nCurrentSysMode = g_pShm_SysStatus->nSystemMode;
            }
            else if(g_pShmemSC->sysstate.fEStopState == OFF)
            {
                g_pShm_SysStatus->fEStopState = FALSE;
            }

            if(g_nSiblingProcState_RM == RUN &&
               g_nSiblingProcState_TE == RUN &&
               g_nSiblingProcState_SC == RUN)
            {
                g_pShmemSC->outputcmd.fLampControllerReadyCmd = ON;
            }
            else
            {
                g_pShmemSC->outputcmd.fLampControllerReadyCmd = OFF;
            }

            /* Lamp Control */
            if(g_pShm_SysStatus->nExecStat == EXEC_STAT_EXECUTING &&
               g_pShm_SysStatus->nWorkType == WORK_TYPE_JOB)
            {
                g_pShmemSC->outputcmd.fLampUnderOperatingCmd = ON;
                g_pShmemSC->outputcmd.fCartLampRunCmd = ON;
                g_pShmemSC->outputcmd.fCartJobStartConfirmCmd = ON;
            }
            else
            {
                g_pShmemSC->outputcmd.fLampUnderOperatingCmd = OFF;
                g_pShmemSC->outputcmd.fCartLampRunCmd = OFF;
                g_pShmemSC->outputcmd.fCartJobStartConfirmCmd = OFF;
            }

            if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR)
            {
                g_pShmemSC->outputcmd.fLampErrorCmd = ON;
            }
            else
            {
                g_pShmemSC->outputcmd.fLampErrorCmd = OFF;
            }

            if(g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR ||
               g_pShm_SysStatus->nSystemMode == MODE_STAT_ESTOP)
            {
                g_pShmemSC->outputcmd.fCartLampAlarmCmd = ON;
            }
            else if(g_pShm_SysStatus->nSystemMode != MODE_STAT_ERROR &&
                    g_pShm_SysStatus->nSystemMode != MODE_STAT_ESTOP)
            {
                g_pShmemSC->outputcmd.fCartLampAlarmCmd = OFF;
            }

            // during job executing, error occurred
            if(g_pShm_SysStatus->nWorkType   == WORK_TYPE_JOB &&
              (g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR ||
              (g_pShm_SysStatus->fErrorState == ON ||
              (g_pShmemSC != NULL       && g_pShmemSC->sysstate.fErrorState == ON) ||
              (g_pShmemTEStatus != NULL && g_pShmemTEStatus->run_error.code != ERR_NONE))))
            {
                if(g_pShmemSC != NULL &&
                   g_pShmemSC->outputstate.fServoOnOutState == ON &&
                   g_fErrorStop == OFF)
                {
                    VERBOSE_ERROR("Job Run Error, Error Stop!\n");

                    // Request Error Stop to SC
                    MSG_SendPulse(g_coidSC, SC_SERV_ERRORSTOP, 0);
                    
                    g_fErrorStop = ON;
                }
            }

            // during home & jog moving, error occurred
            if((g_pShm_SysStatus->nWorkType  == WORK_TYPE_JOG      ||
                g_pShm_SysStatus->nWorkType  == WORK_TYPE_HOME     ||
                g_pShm_SysStatus->nWorkType  == WORK_TYPE_WIRECUT) &&
                g_pShm_SysStatus->nSystemMode == MODE_STAT_ERROR)
            {
                if(g_pShmemSC != NULL &&
                   g_pShmemSC->outputstate.fServoOnOutState == ON &&
                   g_fErrorStop == OFF)
                {
                    VERBOSE_ERROR("Except Run Error, Error Stop!\n");

                    // Request Servo Off to SC
                    MSG_SendPulse(g_coidSC, SC_SERV_ERRORSTOP, 0);

                    g_fErrorStop = ON;
                }
            }

            // in execting mode, set servo off error
            if((g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN  ||
                g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)  &&
                g_pShm_SysStatus->nExecStat  == EXEC_STAT_EXECUTING &&
               (g_pShmemSC != NULL &&
                g_pShmemSC->outputstate.fServoOnOutState == OFF) &&
                fServoOffErr == OFF)
            {
                SVC_DefineErrorState(ON, SVC_ERR_SERVO_OFF);
                
                if(g_pShm_SysStatus->nSystemMode == MODE_STAT_AUTORUN)
                {
                    VERBOSE_ERROR("Servo Off State! Auto Run Error!\n");
                }
                else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_DRYRUN)
                {
                    VERBOSE_ERROR("Servo Off State! Dry Run Error!\n");
                }

                g_pShm_SysStatus->nSystemMode = MODE_STAT_ERROR;
                g_nSystemMode = MODE_STAT_ERROR;

                fServoOffErr = ON;
            }
            else if(g_pShm_SysStatus->nSystemMode == MODE_STAT_STEP &&
                   (g_fJobExecStepMode == ON && g_fJobExecStepFinalMode == OFF) &&
                   (g_pShmemSC != NULL &&
                    g_pShmemSC->outputstate.fServoOnOutState == OFF) &&
                    fServoOffErr == OFF)
            {
                SVC_DefineErrorState(ON, SVC_ERR_SERVO_OFF);
                VERBOSE_ERROR("Servo Off State! Step Run Error!\n");

                g_pShm_SysStatus->nSystemMode = MODE_STAT_ERROR;
                g_nSystemMode = MODE_STAT_ERROR;

                fServoOffErr = ON;
            }

            fServoOffErr = OFF;
        }
        else if(g_pShmemSC == NULL)
        {
            DANDY_SLEEP(50);
        }

        DANDY_SLEEP(1);
    }

    g_fSysIOStatusThExit = TRUE;

    return 0;
}
