/////////////////////////////////////////////////////////////////////////////
//
//  svc_system.c: System Control & Process Sync Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#include "service.h"

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int  g_retTEmsg = 0;
int  g_retSCmsg = 0;

/////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////
//
//  Function: SVC_ExitTEService()
//      - Service Name: RMGR_SERV_SYSEXIT
int SVC_ExitTEService(void)
{
    ///////////////////////////////////
    //
    // Restart Parameters Save to File
    //

    FUNC_SaveRestartParamToFile();

    if(g_coidTE != INVALID_COID)
    {
        g_fSysAliveThRun = PAUSE;
        TE_msg.code = TESERV_EXIT;
        TE_msg.value = 0;
        TE_msg.size = 0;

        g_retTEmsg = MSG_Send(g_coidTE,
                              &TE_msg,
                              RMGR_PACKET_HEAD_LEN,
                              &TE_reply,
                              RMGR_REPLY_PACKET_HEAD_LEN);
	    VERBOSE_VERBOSE("TE Quit\n"); 

        DANDY_SLEEP(100);
        
        // Close connection
        MSG_CloseSiblingConnection(g_coidTE, g_coidTEalive);
    }
    else
    {
        SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_TE);
        VERBOSE_ERROR("Fail to MSG Sending, TE Connection is not valid\n"); 
    }
    
    // Error Flag Check by Return Value
    if(g_retTEmsg == RESULT_ERROR && g_coidTE != INVALID_COID)
    {
        SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_TE);
        VERBOSE_WARNING("TE EXIT Sended! (return: %d)\n", g_retTEmsg);
    }
    else if(g_retTEmsg == RESULT_OK && g_coidTE != INVALID_COID)
    {
        VERBOSE_VERBOSE("TE EXIT Sended! (return: %d)\n", g_retTEmsg);
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_ExitSCService()
//      - Service Name: RMGR_SERV_SYSEXIT
int SVC_ExitSCService(void)
{
	if(g_coidSC != INVALID_COID)
    {
        g_fSysAliveThRun = PAUSE;
        SC_msg.code = SC_SERV_EXIT;
        SC_msg.value = 0;
        SC_msg.size = 0;

        g_retSCmsg = MSG_Send(g_coidSC,
                              &SC_msg,
                              RMGR_PACKET_HEAD_LEN,
                              &SC_reply,
                              RMGR_REPLY_PACKET_HEAD_LEN);
	    VERBOSE_VERBOSE("SC Quit\n"); 

        DANDY_SLEEP(100);

        // Close connection
        MSG_CloseSiblingConnection(g_coidSC, g_coidSCalive);
    }
    else
    {
        SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_SC);
        VERBOSE_ERROR("Fail to MSG Sending, SC Connection is not valid\n"); 
    }

    // Error Flag Check by Return Value
    if(g_retSCmsg == RESULT_ERROR && g_coidSC != INVALID_COID)
    {
        SVC_DefineErrorState(ON, SYS_ERR_FINALIZE_SC);
        VERBOSE_WARNING("SC EXIT Sended! (return: %d)\n", g_retSCmsg);
    }
    else if(g_retSCmsg == RESULT_OK && g_coidSC != INVALID_COID)
    {
        VERBOSE_VERBOSE("SC EXIT Sended! (return: %d)\n", g_retSCmsg);
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_InitTEService()
//          -Service Name: RMGR_SERV_SYSINIT
int SVC_InitTEService(void)
{
    //MSG_SendPulse(g_coidTE, TESERV_RUN_PAUSE, 1);   // Value: Pause
    //DANDY_SLEEP(100);
    g_fErrorReset = OFF;

    if(g_coidTE != INVALID_COID)
    {
        // If Prog. Argument Manual Init is TRUE, Try to Connect Channel
        if(g_Arg.bManualInit == TRUE)
        {
            g_coidTE = MSG_ConnectChannelServer(TE_CHANNEL_ID);
        }

SEND_TEINIT:
        // Define Message Contents
        TE_msg.code = TESERV_INIT;
        TE_msg.value = 0;
        TE_msg.size = 0;

        // Send Massage
        g_retTEmsg = MSG_Send(g_coidTE,
                              &TE_msg,
                              RMGR_PACKET_HEAD_LEN,
                              &TE_reply,
                              RMGR_REPLY_PACKET_HEAD_LEN);
#if defined(__DUBUG)
        VERBOSE_VERBOSE("[TE sended]code: %d value: %d, size: %d\n",
                                                        TE_msg.code,
                                                        TE_msg.value,
                                                        sizeof(TE_msg));
#endif
        //DANDY_SLEEP(100);
        //MSG_SendPulse(g_coidTE, TESERV_RUN_PAUSE, 0);   // Value: Restart
    }
    // If Message Sending is Failed by Invalid Connection, Retry to Connect
    if(g_coidTE == INVALID_COID || g_retTEmsg == RESULT_ERROR)
    {
        VERBOSE_WARNING("Fail to MSG Send, Retry Connection to TE!\n"); 

        // Retry Connection
        g_coidTE = MSG_AttachNamedConnection(TE_CHANNEL_NAME);
        g_coidTEalive = MSG_AttachNamedConnection(TE_CHANNEL_NAME);

        if(g_coidTE != INVALID_COID && g_coidTEalive != INVALID_COID)
        {
            // Retry to Send Message
            goto SEND_TEINIT;

            VERBOSE_VERBOSE("%s connected : coid=%d, %d\n",
                                                           TE_CHANNEL_NAME,
                                                           g_coidTE,
                                                           g_coidTEalive);
        }
        else
        {
            // Define Init Error State
            SVC_DefineErrorState(ON, SYS_ERR_INIT_TE);
            VERBOSE_ERROR("Connection Fail! To %s channel\n",TE_CHANNEL_NAME);
        }
    }

    // Error Flag Check by Return Value
    if(g_retTEmsg == RESULT_ERROR && g_coidTE != INVALID_COID)
    {
        SVC_DefineErrorState(ON, SYS_ERR_INIT_TE);
	    VERBOSE_WARNING("TE INIT Sended! (return: %d)\n", g_retTEmsg);
        return RESULT_ERROR;
    }
    else if(g_retTEmsg == RESULT_OK && g_coidTE != INVALID_COID)
    {
	    VERBOSE_VERBOSE("TE INIT Sended! (return: %d)\n", g_retTEmsg); 
        //return RESULT_ERROR;
    }

    if(g_coidTE == INVALID_COID)
    {
        return RESULT_ERROR;
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_InitSCService()
//          -Service Name: RMGR_SERV_SYSINIT
int SVC_InitSCService(void)
{
    g_fErrorReset = OFF;

	if(g_coidSC != INVALID_COID)
    {
        // If Prog. Argument Manual Init is TRUE, Try to Connect Channel
        if(g_Arg.bManualInit == TRUE)
        {
            g_coidSC = MSG_ConnectChannelServer(SC_CHANNEL_ID);
        }

SEND_SCINIT:
        // Define Message Contents
        SC_msg.code = SC_SERV_INIT;
        SC_msg.value = 0;
        SC_msg.size = 0;

        // Send Message
        g_retSCmsg = MSG_Send(g_coidSC,
                              &SC_msg,
                              RMGR_PACKET_HEAD_LEN,
                              &SC_reply,
                              RMGR_REPLY_PACKET_HEAD_LEN);
#if defined(__DUBUG)
        VERBOSE_VERBOSE("[SC sended]code: %d value: %d, size: %d\n",
                                                        SC_msg.code,
                                                        SC_msg.value,
                                                        sizeof(SC_msg));
#endif
    }
    // If Message Sending is Failed by Invalid Connection, Retry to Connect
    if(g_coidSC == INVALID_COID || g_retSCmsg == RESULT_ERROR)
    {
        VERBOSE_WARNING("Fail to MSG Send, Retry Connection to SC!\n");

        // Retry Connection
        g_coidSC = MSG_AttachNamedConnection(SC_CHANNEL_NAME);
        g_coidSCalive = MSG_AttachNamedConnection(SC_CHANNEL_NAME);

        if(g_coidSC != INVALID_COID && g_coidSCalive != INVALID_COID)
        {
            // Retry to Send Message
            goto SEND_SCINIT;

            VERBOSE_VERBOSE("%s connected : coid=%d, %d\n",
                                                           SC_CHANNEL_NAME,
                                                           g_coidSC,
                                                           g_coidSCalive);
        }
        else
        {
            // Define Init Error State
            SVC_DefineErrorState(ON, SYS_ERR_INIT_SC);
            VERBOSE_ERROR("Connection Fail! To %s channel\n",SC_CHANNEL_NAME);
        }
    }
    
    // Error Flag Check by Return Value
    if(g_retSCmsg == RESULT_ERROR && g_coidSC != INVALID_COID)
    {
        SVC_DefineErrorState(ON, SYS_ERR_INIT_SC);
	    VERBOSE_WARNING("SC INIT Sended! (return: %d)\n", g_retSCmsg); 
        return RESULT_ERROR;
    }
    else if(g_retSCmsg == RESULT_OK && g_coidSC != INVALID_COID)
    {
	    VERBOSE_VERBOSE("SC INIT Sended! (return: %d)\n", g_retSCmsg);
        //return RESULT_ERROR;
    }

    if(g_coidSC == INVALID_COID)
    {
        return RESULT_ERROR;
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_RetryInitProcessRM()
//      - Service Name: RMGR_SERV_SYSINIT

int SVC_RetryInitProcessRM(void)
{
    // Load System Parameters
    if(g_fInitRet[INIT_STEP0_LOAD_PARAM] == -1)
    {
        VERBOSE_VERBOSE("Retry RM Shared Memory Create!\n");
        SHM_CreateSharedMemory();
    }
    
    // Create Shared Memory
    if(g_fInitRet[INIT_STEP1_CREATE_SHM] == -1)
    {
        VERBOSE_VERBOSE("Retry Parameter Loading!\n");
        SYSC_LoadSystemConfigParameter();
    }

    // Connect to Other Proc Channel
    if(g_fInitRet[INIT_STEP2_CONNECT_CH] == -1)
    {
        VERBOSE_VERBOSE("Retry Connect Channel Server!\n");
        // Retry to Connect Automatically by Alive Check Thread in system_mon.c
    }

    // Init RM Services
    if(g_fInitRet[INIT_STEP3_INIT_SERV] == -1)
    {
        VERBOSE_VERBOSE("Retry RM Service Initialize!\n");
        SVC_InitRMService();
    }
            
    // Open Shared Memory
    if(g_fInitRet[INIT_STEP4_OPEN_SHM] == -1)
    {
        VERBOSE_VERBOSE("Retry Open Shared Memory!\n");
        SHM_OpenSharedMemory();
    }
            
    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SVC_ShutdownSystem()
//        - shutdown system command in QNX Environment

void SVC_ShutdownSystem(int nOpt)
{
    if(nOpt == SVC_OPT_REBOOT)
    {
        VERBOSE_WARNING("Now, Reboot System!\n");
    }
    else if(nOpt == SVC_OPT_SHUTDOWN)
    {
        VERBOSE_WARNING("Now, Shutdown System!\n");
    }

#if defined(_WIN32)
    //ExitWindowsEx(EWX_POWEROFF | EWX_FORCEIFHUNG,
    //              SHTDN_REASON_MAJOR_SOFTWARE | SHTDN_REASON_MINOR_MAINTENANCE);
    
#else
    // Wait for App Exit State
    system("sleep 3");

    if(nOpt == SVC_OPT_REBOOT)
    {
        system("shutdown -S reboot");
    }
    else if(nOpt == SVC_OPT_SHUTDOWN)
    {
        system("shutdown -b -v");
    }
#endif

    system("sleep 1");

    MSG_SendPulse(g_coidRM, RMGR_SERV_SYSEXIT, 0);

    system("sleep 5");
}

