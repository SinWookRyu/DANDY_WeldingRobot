/////////////////////////////////////////////////////////////////////////////
//
//  svc_handling.c: Service Receive & Process
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#include "service.h"

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int  g_fServiceStatus = 0;
char g_szServContent[SERV_NAME_LEN];
int  g_fCallModeJobLoad = FALSE;

THREAD_HANDLE                  hSVCCheck_Thread;

/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SVC_SetTrajSamplgingTime()
//
static int _loc_SVC_SetTrajSamplgingTime(int nTrajUpdateTime)
{
    // Request Sampling Time Setting Service
    if(g_coidTE != INVALID_COID)
    {
        MSG_SendPulse(g_coidTE, TESERV_SAMPTIME, nTrajUpdateTime);
        VERBOSE_MESSAGE("Set Traj Update Time: %d ms\n", nTrajUpdateTime);
    }
    else
    {
        VERBOSE_ERROR("Can't Set Traj Update Time (Set to Default)!\n");
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SVC_SetServoSamplgingTime()
//
static int _loc_SVC_SetServoSamplgingTime(int nIntPeriod, int nIntIdx)
{
    ///////////////////////////////////////////
    //
    // interpolation time: period * 10 ^ index
    //
    if(g_coidSC != INVALID_COID)
    {
        THREAD_Sleep(10);

        SC_msg.code = SC_SET_INTERP_PERIOD;
        SC_msg.value = nIntPeriod;          // servo interpolation period
        SC_msg.data.serv_opt.nOpt = 1;      // quite option
        SC_msg.size = sizeof(SC_msg.data.serv_opt);
        g_retSCmsg = MSG_Send(g_coidSC,
                              &SC_msg,
                              sizeof(SC_msg),  //sizeof(SC_msg),
                              &SC_reply,
                              sizeof(SC_reply));   //sizeof(SC_reply));

        THREAD_Sleep(10);

        SC_msg.code = SC_SET_INTERP_INDEX;
        SC_msg.value = nIntIdx;             // servo interpolation index
        SC_msg.data.serv_opt.nOpt = 1;      // quite option
        SC_msg.size = sizeof(SC_msg.data.serv_opt);
        g_retSCmsg = MSG_Send(g_coidSC,
                              &SC_msg,
                              sizeof(SC_msg),     //RMGR_PACKET_HEAD_LEN,
                              &SC_reply,
                              sizeof(SC_reply));  //RMGR_REPLY_PACKET_HEAD_LEN);

        VERBOSE_MESSAGE("Set Set Interpolation Time: %d ms\n", nIntPeriod);
    }
    else
    {
        VERBOSE_ERROR("Can't Set Interpolation Time (Set to Default)!\n");
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: _loc_SVC_ExternSVCExecThread()
//

static THREAD_ENTRY_TYPE _loc_SVC_ExternSVCExecThread(void* pParam)
{
    static int rgRcvid;
    static int nDataSize;
    static int nRet = RESULT_OK;
    void* pReplyData;
    
    CRT_strcpy(g_szServContent, SERV_NAME_LEN, "NO SERVICE REQUEST");

    while(g_fSVCExecThRun == RUN && g_fConsoleExitAct == FALSE)
	{
        //RM_packet.nDataSize = 0;
        rgRcvid = MSG_Receive(g_chidRM,
                              &RM_packet,
                              sizeof(RM_packet),
                              &info_msg);

        if(g_fSVCExecThRun == STOP || g_fConsoleExitAct == TRUE)
        {
            break;
        }

        if(RM_packet.nCode != RSVC_SERV_SYSMON_TO_TP &&
           RM_packet.nCode != RSVC_SERV_JOBLINE_DSP)
        //if(RM_packet.nCode != RSVC_SERV_JOBLINE_DSP)
        {
            VERBOSE_VERBOSE("Received service(code: %d, value: %d)"
                            "from pid: %d.\n",
                            RM_packet.nCode,
                            RM_packet.nValue,
                            info_msg.pid);
        }

        if (rgRcvid == 0)               // pulse
        {
            RM_packet.nDataSize = 0;
        }
        nDataSize = 0;
        pReplyData = NULL;

        switch(RM_packet.nCode)
        {
        case RMGR_SERV_SYSEXIT:             /////// svc code : 0
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SYSEXIT");
			if(RM_packet.nValue == 0)       // svc value: 0
            {
                g_fServiceStatus = STOP;
                g_fSVCExecThRun = STOP;

			    VERBOSE_VERBOSE("RM Service Quit\n"); 
                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_ExitSCService();
                    nRet = SVC_ExitTEService();
                }
            }
            else if(RM_packet.nValue == 1)  // svc value: 1
            {
                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_ExitTEService();
                }
            }
            else if(RM_packet.nValue == 2)  // svc value: 2
            {
                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_ExitSCService();
                }
            }
            //VERBOSE_MESSAGE("Exit Request Done!\n");
            break;

        case RMGR_SERV_SYSVERSION:          /////// svc code : 1
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SYSVERSION");
            nRet = SVC_SendVersionInform();
            break;

        case RMGR_SERV_SYSINIT:             /////// svc code : 2
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SYSINIT");
            if(RM_packet.nValue == 0)       // svc value: 0
            {
                if(g_retInitRM == -1)
                {
                    SVC_RetryInitProcessRM();
                }
                else
                {
                    VERBOSE_WARNING("RM already INIT state!\n");
                }

                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_InitTEService();
                    nRet = SVC_InitSCService();

                    _loc_SVC_SetTrajSamplgingTime(g_nTrajUpdateTime);
                }
            }
            else if(RM_packet.nValue == 1)  // svc value: 1
            {
                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_InitTEService();
                }
            }
            else if(RM_packet.nValue == 2)  // svc value: 2
            {
                if(g_Arg.bSingleRun == FALSE)
                {
                    nRet = SVC_InitSCService();
                }
            }

            if(g_Arg.bSingleRun == FALSE)
            {
                SHM_OpenSharedMemory();

                if(g_pShm_SysStatus->fInitProcTE == TRUE)
                {
                    g_pShm_SysStatus->fExitProcTE = FALSE;
                }

                if(g_pShm_SysStatus->fInitProcSC == TRUE)
                {
                    g_pShm_SysStatus->fExitProcSC = FALSE;
                }
            }

            break;
        
        case RMGR_SERV_SYSSTATE:            /////// svc code : 3
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SYSSTATE");
            nRet = SVC_SetSystemStatePacketDefine();
            nRet = SVC_ShowSystemStateDisplay();
            break;

        case RMGR_SERV_SYSINFO:             /////// svc code : 4
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SYSINFO");
            nRet = SVC_ShowSystemInfoDisplay(RM_packet.nValue);
            nRet = SVC_SetSystemInfoPacketDefine(RM_packet.nValue);
            break;

        case RMGR_SERV_TIMERTEST:           /////// svc code : 5
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_TIMERTEST");
            nRet = SVC_TimerTest();
            break;

        case RMGR_SERV_CONFRELOAD:          /////// svc code : 6
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_CONFRELOAD");
            nRet = SYSC_LoadSystemConfigParameter();
            if(nRet == RESULT_OK)
            {
                nRet = SYSC_LoadConfigToShmem(g_pShm_SysConfig);
                SYSC_LoadParamToShmem(g_pShm_SysParam);
                SYSC_LoadParamToTE_RestartShmem();
                SYSC_LoadParamToTE_TaskShmem();

                if(nRet == RESULT_OK)
                {
                    SVC_ShowSystemInfoDisplay(RM_packet.nValue);
                    DANDY_SLEEP(100);
                    if(g_Arg.bSingleRun == FALSE)
                    {
                        nRet = SVC_InitTEService();
                        nRet = SVC_InitSCService();
                        _loc_SVC_SetTrajSamplgingTime(g_nTrajUpdateTime);
                    }
                }
                else if(nRet == RESULT_ERROR)
                {
                    //SVC_DefineErrorState(ON, SVC_ERR_CONF_RELOAD);
                    VERBOSE_ERROR("System Config Shared Memory Reload Fail!!\n");
                }
            }
            else if(nRet == RESULT_ERROR)
            {
                //SVC_DefineErrorState(ON, SVC_ERR_CONF_RELOAD);
                VERBOSE_ERROR("System Config Parameter Reload Fail!!\n");
            }
            break;

        case RMGR_SERV_ERRHISTORY:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_ERRHISTORY");
            if(RM_packet.nValue == 0)
            {
                SVC_SetErrorHistoryPacketDefine();
                nRet = SVC_ShowErrorHistoryDisplay();
            }
            else if(RM_packet.nValue == 1)
            {
                nRet = SVC_Clear_ErrorHistory();
            }
            break;

        case RMGR_SERV_GETSTATISTIC:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_GETSTATISTIC");
            nRet = SVC_GetStatisticsData(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_GET_STATISTICS_DATA);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_JOBLOAD:             /////// svc code : 10
        case RSVC_SERV_GENMAPFILE:
            if (rgRcvid == 0)               // pulse
            {
                g_fCallModeJobLoad = TRUE;
                nRet = SVC_LoadJobData(g_pShm_SysStatus->szTargJobFileName, JOBASM_AF_DANDY1996);
            }
            else if(rgRcvid > 0)            // message
            {
                g_fCallModeJobLoad = FALSE;
                nRet = SVC_LoadJobData(RM_packet.Data.job_load.szJobFileName, RM_packet.nValue);
            }
            break;

        case RSVC_SERV_JOBSHMDUMP:          /////// svc code : 12
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBSHMDUMP");
            nRet = SVC_DumpJobShmData(RM_packet.nValue);
            break;

        case RSVC_SERV_JOBEXECAUTO:          /////// svc code : 13
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBEXECAUTO");
            if(g_Arg.bSingleRun  == FALSE)
            {
                nRet = SVC_ExecuteJobProgAuto(RM_packet.nValue);
            }
            else
            {
                nRet = RESULT_OK;
            }
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOBEXECAUTO);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_JOBEXECDRY:           /////// svc code : 14
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBEXECDRY");
            if(g_Arg.bSingleRun  == FALSE)
            {
                nRet = SVC_ExecuteJobProgDry(RM_packet.nValue);
            }
            else
            {
                nRet = RESULT_OK;
            }
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOBEXECDRY);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_JOBEXESTOP:          /////// svc code : 15
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBEXESTOP");
            if(g_Arg.bSingleRun  == FALSE)
            {
                nRet = SVC_StopJobProg();
            }
            else
            {
                nRet = RESULT_OK;
            }
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOBSTOP);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_JOBLINE_DSP:         /////// svc code : 16
            nRet = SVC_JobProgLineDisplay(RM_packet.nValue);
            break;

        case RSVC_SERV_SYSMON_TO_TP:        /////// svc code : 17
            nRet = SVC_SystemMonDataSendToTP();
            break;

        case RSVC_SERV_JOBEXECSTEP:         /////// svc code : 18
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBEXECSTEP");
            if(g_Arg.bSingleRun  == FALSE)
            {
                if(g_pShm_SysStatus != NULL && g_fJobExecStepMode == OFF)
                {
                    g_fStepExecSVCAct = ON;
                    g_nStepModeLineIndex = RM_packet.nValue;
                }
            }
            break;
        
        case RSVC_SERV_JOBFILESAVE:         /////// svc code : 19
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_JOBFILESAVE");
            nRet = JOB_SaveJobToFile(RM_packet.nValue,
                                     RM_packet.Data.job_load.szJobFileName);
            if(nRet != RESULT_OK)
            {
                nRet = RESULT_ERROR;
            }
            break;

        case RCON_SERV_ERROR_RESET:         /////// svc code : 20
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_ERROR_RESET");
            nRet = SVC_Error_Reset();
            break;
        
        case RCON_SERV_HOME_MOVE:           /////// svc code : 21
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_HOME_MOVE");
            g_fHomeExecSVCAct = ON;
            g_nHomeExecIndex = RM_packet.nValue;
            break;

        case RCON_WIRECUT_JOB_CALL:         /////// svc code : 22
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "WIRECUT_JOB_CALL");
            g_fWireCutSVCAct = ON;
            break;

        case RCON_VOLT_REALTIME_OFFSET:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "VOLT_OFFSET");
            nRet = SVC_RCON_SetVoltageRealTimeOffset(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                //SVC_DefineErrorState(ON, SVC_ERR_WIRECUT_JOB_CALL);
                nRet = RESULT_ERROR;
            }
            break;

        case RCON_CURR_REALTIME_OFFSET:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "CURR_OFFSET");
            nRet = SVC_RCON_SetCurrentRealTimeOffset(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                //SVC_DefineErrorState(ON, SVC_ERR_WIRECUT_JOB_CALL);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_HOME_EDIT:         /////// svc code : 30
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_HOME_EDIT");
            nRet = SVC_SetHomePositionValue(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_HOMEPOS_EDIT);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_USERCRD_EDIT:      /////// svc code : 31
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_USERCRD_EDIT");
            nRet = SVC_SetUserCoordinateValue(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_USRCRD_EDIT);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_PARAM_EDIT:        /////// svc code : 32
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_PARAM_EDIT");
            if(RM_packet.nValue == 0)
            {
                nRet = SVC_SetToolCenterdPointValue();
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_TCP_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 1)
            {
                nRet = SVC_SetWordCoordinateOffsetValue();
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_WORLD_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 2)
            {
                nRet = SVC_SetWeldTuneInputValue(0);
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_WELDIN_TUNE_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 3)
            {
                nRet = SVC_SetWeldTuneOutputValue(0);
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_WELDOUT_TUNE_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 4)
            {
                nRet = SVC_SetRestartParameterValue();
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_RESTART_PARAM_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 5)
            {
                nRet = SVC_SetArcSensorParamValue();
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_ARCSENS_PARAM_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            else if(RM_packet.nValue == 6)
            {
                nRet = SVC_SetCartOffsetValue();
                if(nRet != RESULT_OK)
                {
                    SVC_DefineErrorState(ON, SVC_ERR_ARCSENS_PARAM_EDIT);
                    nRet = RESULT_ERROR;
                }
            }
            break;

        case RSVC_SERV_SHOWHOME_TP:       /////// svc code : 33
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SHOWHOME_TP");
            nRet = SVC_SendHomePositionValue(RM_packet.nValue);
            break;

        case RSVC_SERV_SHOWUSRCRD_TP:     /////// svc code : 34
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SHOWUSRCRD_TP");
            nRet = SVC_SendUserCoordinateValue(RM_packet.nValue);
            break;

        case RSVC_SERV_SHOWPARAM_TP:      /////// svc code : 35
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SHOWPARAM_TP");
            nRet = SVC_SendUserParameterValue(RM_packet.nValue);
            break;

        case RSVC_SERV_SET_GAPCOND:       /////// svc code : 36
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_SET_GAPCOND");
            nRet = SVC_SetGapCondition();
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_GAP_COND_SET);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_CONSTVARINFO:       /////// svc code : 37
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CONSTVARINFO");
            nRet = SVC_SendConstJobVarInfo(RM_packet.nValue);
            break;

        case RSVC_JOB_WELDVARINFO:       /////// svc code : 38
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_WELDVARINFO");
            nRet = SVC_SendWeldJobVarInfo(RM_packet.nValue);
            break;

        case RSVC_JOB_CONSTVAREDIT:      /////// svc code : 39
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CONSTVAREDIT");
            nRet = SVC_EditConstJobVar(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_CONST_VAR_EDIT);
                nRet = RESULT_ERROR;
            }
            break;
        
        case RSVC_JOB_BIN_SNDHEADER:     /////// svc code : 40
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_BIN_SNDHEADER");
            nRet = SVC_JobBinSendHeader(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_SNDHEADER);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_BIN_SNDBODY:       /////// svc code : 41
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_BIN_SNDBODY");
            nRet = SVC_JobBinSendData(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_SNDBODY);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_BIN_RCVHEADER:     /////// svc code : 42
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_BIN_RCVHEADER");
            nRet = SVC_JobBinReceiveHeader(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_RCVHEADER);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_BIN_RCVBODY:       /////// svc code : 43
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_BIN_RCVBODY");
            nRet = SVC_JobBinReceiveData(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_RCVBODY);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_RESTART:           /////// svc code : 44
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_RESTART");
            nRet = SVC_JobRestart();
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_EXECRESTART);
                nRet = RESULT_ERROR;
            }
            break;
        
        case RSVC_JOB_COMP_BIN_SNDHEADER:  /////// svc code : 45
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_COMP_SNDHEADER");
            nRet = SVC_JobCompBinSendHeader(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_SNDHEADER);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_COMP_BIN_SNDBODY:   /////// svc code : 46
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_COMP_SNDBODY");
            nRet = SVC_JobCompBinSendData(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_SNDBODY);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_COMP_BIN_RCVHEADER: /////// svc code : 47
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_COMP_RCVHEADER");
            nRet = SVC_JobCompBinReceiveHeader(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_RCVHEADER);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_COMP_BIN_RCVBODY:  /////// svc code : 48
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_COMP_RCVBODY");
            nRet = SVC_JobCompBinReceiveData(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_JOB_RCVBODY);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SERV_WELDTUNE_IN_EDIT:  /////// svc code : 49
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_WELDIN_EDIT");
            nRet = SVC_SetWeldTuneInputValue(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_WELDIN_TUNE_EDIT);
                nRet = RESULT_ERROR;
            }
            break;
                                       
        case RSVC_SERV_WELDTUNE_OUT_EDIT: /////// svc code : 50
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SERV_WELDOUT_EDIT");
            nRet = SVC_SetWeldTuneOutputValue(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                ;
                //SVC_DefineErrorState(ON, SVC_ERR_WELDOUT_TUNE_EDIT);
                //nRet = RESULT_ERROR;
            }
            break;
                                       
        case RSVC_SERV_SHOWWELDTUNE_IN:   /////// svc code : 51
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SHOW_WELDIN_PARAM");
            nRet = SVC_SendWeldTuneInputParameterValue(RM_packet.nValue);
            break;
                                       
        case RSVC_SERV_SHOWWELDTUNE_OUT:  /////// svc code : 52
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SHOW_WELDOUT_PARAM");
            nRet = SVC_SendWeldTuneOutputParameterValue(RM_packet.nValue);
            break;

        case RSVC_SRV_WELDTUNE_IN_APPLY:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "WLDIN_PARAM_APPLY");
            nRet = SVC_ApplyWeldTuneInputParameterValue(RM_packet.nValue);
            break;

        case RSVC_SRV_WELDTUNE_OUT_APPLY:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "WLDOUT_PARAM_APPLY");
            nRet = SVC_ApplyWeldTuneOutputParameterValue(RM_packet.nValue);
            break;

        case RSVC_JOBFILE_ZIP_UNCOMPRESS:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOBFILE_UNZIP");
            nRet = SVC_JobFileZipUncompress();
            break;

        case RSVC_SERV_SET_SKIPCOND:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SET_SKIPCOND");
            if(RM_packet.nValue != SHOW_SKIP_COND)
            {
                nRet = SVC_SetSkipCondition(RM_packet.nValue);
                g_fSkipBvarModifySVCAct = ON;
            }
            else
            {
                nRet = SVC_ShowSkipCondition();
            }
            break;

        case RSVC_CONSTVAR_FILE_HANDLE:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "CONSTVAR_FILE");
            nRet = SVC_ConstVarFileHandle(RM_packet.nValue);
            break;

        case RSVC_SERV_SET_GAPSKIPCOND:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "SET_GAPSKIPCOND");
            if(RM_packet.nValue == SET_GAPSKIP_COND)
            {
                nRet = SVC_SetGapSkipCondition();
                g_fGapSkipBvarModifySVCAct = ON;
            }
            else if(RM_packet.nValue == SHOW_GAPSKIP_COND)
            {
                nRet = SVC_ShowGapSkipCondition();
            }
            break;

        case RSVC_SERV_REBOOT_SYSTEM:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "REBOOT_SYSTEM");
            SVC_ShutdownSystem(RM_packet.nValue);
            break;

        case RSVC_SERV_CONSTVAR_INIT:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "CONSTVAR_INIT");
            nRet = SVC_InitConstJobVar(RM_packet.nValue);
            if(nRet != RESULT_OK)
            {
                SVC_DefineErrorState(ON, SVC_ERR_CONST_VAR_EDIT);
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_SRV_WELD_MEASURE_WRITE:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "WELD_MEASURE_VAL");
            nRet = SVC_SetWeldMeasureParamValue(RM_packet.nValue);
            break;

        case RSVC_SRV_CAL_WELDCOEFF:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "WELDCOEFF_CAL");
            nRet = SVC_CalculateAndSaveWeldCoefficient(RM_packet.nValue);
            break;

        case RSVC_JOB_CMDJUMP:            /////// svc code : 90
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CMDJUMP");
            nRet = SVC_JobCmdJump();
            if(nRet != RESULT_OK)
            {
                g_pShm_SysStatus->nErrCode = nRet;
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_CMDCALL:            /////// svc code : 91
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CMDCALL");
            nRet = SVC_JobCmdCall();
            if(nRet != RESULT_OK)
            {
                g_pShm_SysStatus->nErrCode = nRet;
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_CMDRETURN:          /////// svc code : 92
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CMDRETURN");
            nRet = SVC_JobCmdReturn();
            if(nRet != RESULT_OK)
            {
                g_pShm_SysStatus->nErrCode = nRet;
                nRet = RESULT_ERROR;
            }
            break;

        case RSVC_JOB_CMDCONTINUE:
            CRT_strcpy(g_szServContent, SERV_NAME_LEN, "JOB_CMDCONTINUE");
            nRet = SVC_JobCmdContinue();
            break;

        case RSVC_DSP_STATISTICSDATA:
            SVC_VGADisplayStatisticsData(RM_packet.nValue);
            break;

        case RSVC_SET_SERVOCONTROL_TIME:
            _loc_SVC_SetServoSamplgingTime(g_nServoInterpolationTime,
                                           -3);
            break;

        default:
            VERBOSE_WARNING("Not defined service code : code=<%d>\n", 
                            RM_packet.nCode);
        }

        nDataSize = RMGR_REPLY_PACKET_LEN;
        pReplyData = &RM_reply_packet;
        
        // message reply
        if (rgRcvid == RCVID_PULSE) // pulse
        {
            ;
        }
        else if (rgRcvid > 0)       // massage
        {
            if(nRet != RESULT_OK && g_pShm_SysStatus != NULL)
            {
                if(RM_packet.nCode != RSVC_SERV_WELDTUNE_OUT_EDIT &&
                   RM_packet.nCode != RSVC_SRV_CAL_WELDCOEFF)
                {
                    nRet = g_pShm_SysStatus->nErrCode;
                }
                else if(RM_packet.nCode == RSVC_SERV_WELDTUNE_OUT_EDIT ||
                        RM_packet.nCode == RSVC_SRV_CAL_WELDCOEFF)
                {
                    nRet = nRet | ERR_MOD_PARAM_VALIDCHECK | ERR_OWNER_FROM_RM;
                    VERBOSE_WARNING("Code: %d, Msg Return Value: %x\n",
                                    RM_packet.nCode, nRet);
                }
            }

            MSG_Reply(rgRcvid, nRet, pReplyData, nDataSize);
        }
        else
        {
            VERBOSE_ERROR("Failed to receive data...\n");
        }

        DANDY_SLEEP(1);
    }

    g_fSVCExecThExit = TRUE;

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SVC_ShowKeyMenu()
//

static int _loc_SVC_ShowKeyMenu(void)
{
    VERBOSE_VERBOSE("<- Key-in Usage: Ctrl+HotKey ->\n"); 
    VERBOSE_VERBOSE("\v  'Q': RM Exit\t 'L': Show Menu\t "
                    "'A': Sys State\t 'W': Sys Info\n"); 
    VERBOSE_VERBOSE("\v  'E': TE INIT\t 'D': TE Exit\t "
                    "'R': SC INIT\t 'F': SC Exit\n"); 
    VERBOSE_VERBOSE("\v  'T': TimerRun\t 'P': Err Reset\t "
                    "'G': Confload\t 'H': Err History\n\n");

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SVC_ExecService(BOOL fKeyIn)
//

int SVC_ExecService(BOOL fKeyIn)
{
    int nKeyInput;
    int nRet = RESULT_OK;

    g_fServiceStatus = RUN;
    nKeyInput = -1;

    //  launch external service check thread
    hSVCCheck_Thread = THREAD_Create(_loc_SVC_ExternSVCExecThread,
                                    NULL,
                                    0,                 // auto stack size
                                    //THREAD_PRIO_BELOW_NORMAL,// normal priority
                                    THREAD_PRIO_REALTIME,
                                    THREAD_DETACHED,   // detach thread
                                    THREAD_POLICY_RR); // round-robin

    if (hSVCCheck_Thread == INVALID_THREAD)
    {
        VERBOSE_ERROR("cannot create service execution for external request "
                      "thread...\n");

        exit(1);
    }
    else
    {
        // Set System Mode: MANUAL state
        g_nSystemMode = MODE_STAT_MANUAL;
        g_pShm_SysStatus->nSystemMode = MODE_STAT_MANUAL;
    }

    VERBOSE_MESSAGE("Ready to start Service Request Receive!!\n");

    if(fKeyIn == TRUE)
    {
        _loc_SVC_ShowKeyMenu();

        _loc_SVC_SetTrajSamplgingTime(g_nTrajUpdateTime);

        // Reset Gap Skip Condition Var.
        g_nLeftVertGapSkipCond     = FUNC_GetBVarValue(g_nLeftVertSkipBvar);
        g_nRightVertGapSkipCond    = FUNC_GetBVarValue(g_nRightVertSkipBvar);
        g_nLeftCollarGapSkipCond   = FUNC_GetBVarValue(g_nLeftCollarSkipBvar);
        g_nRightCollarGapSkipCond  = FUNC_GetBVarValue(g_nRightCollarSkipBvar);
        g_nHorizontalGapSkipCond   = FUNC_GetBVarValue(g_nHorizontalSkipBvar);
        g_nLeftBracketGapSkipCond  = FUNC_GetBVarValue(g_nLeftBracketSkipBvar);
        g_nRightBracketGapSkipCond = FUNC_GetBVarValue(g_nRightBracketSkipBvar);
        
        // Key Input Process
	    while(g_fServiceStatus == RUN && g_fConsoleExitAct == FALSE)
	    {
            if(g_fSVCExecThRun == STOP || g_fConsoleExitAct == TRUE)
            {
                break;
            }
            if(CIO_kbhit() && g_fConsoleExitAct == FALSE)
            {
	    	    nKeyInput = CIO_getch();
            }

            switch(nKeyInput)
	    	{
            case ASCII_CTRL_Q:
                g_fServiceStatus = STOP;
                MSG_SendPulse(g_coidRM, RMGR_SERV_SYSEXIT, 0);

                VERBOSE_VERBOSE("RM Service Quit by key\n"); 
                break; 

            case ASCII_CTRL_D:
                if(g_Arg.bSingleRun == FALSE)
                {
                    SVC_ExitTEService();
                }
                break; 

            case ASCII_CTRL_E:
                if(g_Arg.bSingleRun == FALSE)
                {
                    SVC_InitTEService();
                }
	    	    break; 

            case ASCII_CTRL_F:
                if(g_Arg.bSingleRun == FALSE)
                {
	    		    SVC_ExitSCService();
                }
	    	    break; 

            case ASCII_CTRL_R:
                if(g_Arg.bSingleRun == FALSE)
                {
                    SVC_InitSCService();
                }
                break; 

            case ASCII_CTRL_V:
                VERBOSE_VERBOSE("Version Information: %s - %s\n",
                                                                SYS_RM_VERSION,
                                                                SYS_RM_BUILD);
                break;

            case ASCII_CTRL_T:
                SVC_TimerTest();
                break;

            case ASCII_CTRL_H:
                SVC_ShowErrorHistoryDisplay();
                break;

            case ASCII_CTRL_A:
                SVC_SetSystemStatePacketDefine();
                SVC_ShowSystemStateDisplay();
                break;

            case ASCII_CTRL_W:
                // Show All Data: argument is zero
                SVC_ShowSystemInfoDisplay(0);
                break;

            case ASCII_CTRL_L:
                _loc_SVC_ShowKeyMenu();
                break;

            case ASCII_CTRL_G:
                nRet = SYSC_LoadSystemConfigParameter();
                if(nRet == 0)
                {
                    nRet = SYSC_LoadConfigToShmem(g_pShm_SysConfig);
                    SYSC_LoadParamToShmem(g_pShm_SysParam);

                    if(nRet == 0)
                    {
                        SVC_ShowSystemInfoDisplay(RM_packet.nValue);
                        DANDY_SLEEP(100);
                        SVC_InitTEService();
                        SVC_InitSCService();
                        _loc_SVC_SetTrajSamplgingTime(g_nTrajUpdateTime);
                    }
                    else if(nRet == -1)
                        VERBOSE_ERROR("System Config Shared Memory Reload Fail!!\n");
                }
                else if(nRet == -1)
                {
                    VERBOSE_ERROR("System Config Parameter Reload Fail!!\n");
                }
                break;

            case ASCII_CTRL_P:
                SVC_Error_Reset();
                break;

            case ASCII_CTRL_J:
                //_loc_SVC_RCON_Jog();
                break;

            default:
                if(nKeyInput != -1)
                {
	    		    VERBOSE_VERBOSE("Unsupported Service! Key(%d)\n",nKeyInput);
                }
	    	}
          
            DANDY_SLEEP(5);
            nKeyInput = -1;
	    }
    }
    else
    {
        while(g_fServiceStatus == RUN)
	    {
           DANDY_SLEEP(100);
        }
    }

    while(g_fSVCExecThExit == FALSE)
    {
        DANDY_SLEEP(100);
    }

    return 0;
}
