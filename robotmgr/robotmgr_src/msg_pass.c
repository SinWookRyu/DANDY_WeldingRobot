/////////////////////////////////////////////////////////////////////////////
//
//  msg_pass.c: RobotManager message passing
//                                            2013.04.11  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"


///////////////////////////////////////
//Global_variable

int g_coidTE        = INVALID_COID;
int g_coidSC        = INVALID_COID;
int g_coidTEalive   = INVALID_COID;
int g_coidSCalive   = INVALID_COID;
int g_chidRM        = INVALID_CHID;
int g_coidRM        = INVALID_COID;

ARGUMENT_OPTION     g_Arg;


/////////////////////////////////////////////////////////////////////////////
//
//  Function: MSG_ConnectChannelServer(int nCoid)
//      - Connect to channel (TE, SC)

int MSG_ConnectChannelServer(int nCoid)
{
    int iRetry;
    char rgchChName[CHANNEL_NAME_LEN]; 
    static unsigned s_nWaitConnmsec;

    // Set Wait Time: default 5sec and possible to set by program argument
    s_nWaitConnmsec = g_nWaitConnSec * 1000;

    // No. of Connection Retry: default 2 times(defined at CONNECT_RETRY_NO)
    iRetry = 1;

    // Try to Connect Process
CONNECT_CHANNEL:

    // Try to Connect to TE channel
    if(nCoid == TE_CHANNEL_ID)
    {
        nCoid = MSG_WaitForNamedConnection(TE_CHANNEL_NAME, s_nWaitConnmsec);
        CRT_strcpy(rgchChName, CHANNEL_NAME_LEN, TE_CHANNEL_NAME);
        g_coidTE = nCoid;
    }
    // Try to Connect to SC channel
    if(nCoid == SC_CHANNEL_ID)
    {
        nCoid = MSG_WaitForNamedConnection(SC_CHANNEL_NAME, s_nWaitConnmsec);
        CRT_strcpy(rgchChName, CHANNEL_NAME_LEN, SC_CHANNEL_NAME);
        g_coidSC = nCoid;
    }
    // Try to Connect to RM channel
    if(nCoid == RM_CHANNEL_ID)
    {
        nCoid = MSG_WaitForNamedConnection(SYS_RM_CHANNEL_NAME, s_nWaitConnmsec);
        CRT_strcpy(rgchChName, CHANNEL_NAME_LEN, SYS_RM_CHANNEL_NAME);
        g_coidRM = nCoid;
    }

    if(nCoid == INVALID_COID)
    {
        if(g_Arg.bManualInit == FALSE)
        {
            // Retry to Channel Connect
            if(iRetry <= CONNECT_RETRY_NO && iRetry != -1)
            {
                VERBOSE_ERROR("cannot connect to %s channel(wait time: %d)\n",
                                                    rgchChName, g_nWaitConnSec);
                VERBOSE_ERROR("Retry Connection (Retry Cnt: %d)\n", iRetry);
                iRetry++;

                goto CONNECT_CHANNEL;
            }
            // If No. of Retry Reach to CONNECT_RETRY_NO, Define Connection Fail
            else if(iRetry > CONNECT_RETRY_NO)
            {
                VERBOSE_ERROR("Connection Fail! To %s channel\n",rgchChName);
                iRetry = -1;
            }
        }
        else
        {
            // In Case of Manual Init, No Retry
            VERBOSE_ERROR("Connection Fail! To %s channel\n",rgchChName);
        }
    }
    // Success to Connecting Channel
    else if(nCoid != INVALID_COID)
    {
        VERBOSE_VERBOSE("%s connected : coid=%d\n",rgchChName, nCoid);
    }

    return (nCoid == INVALID_COID) ?
        RESULT_ERROR : RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: MSG_CreateRMChannel()
//     - Create RM channel

int MSG_CreateRMChannel(void)
{
    // Create RM Channel
    g_chidRM = MSG_CreateNamedChannel(SYS_RM_CHANNEL_NAME);

    // If Create Fail
    if (g_chidRM == INVALID_CHID)
    {
        VERBOSE_ERROR("Failed to create the channel: '%s'\n",
                      SYS_RM_CHANNEL_NAME);
        
        // Alive Check Thread is Paused
        g_fSysAliveThRun = PAUSE;

        // Send to TE Terminate Signal
        if(g_coidTE != INVALID_COID)
        {
            MSG_SendPulse(g_coidTE, TESERV_EXIT, 0); 
		    VERBOSE_ERROR("Fail to create channel, TE Quit\n"); 
        }

        // Send to SC Terminate Signal
        if(g_coidSC != INVALID_COID)
        {
            MSG_SendPulse(g_coidSC, SC_SERV_EXIT, 0); 
            VERBOSE_ERROR("Fail to create channel, SC Quit\n"); 
        }
    }
    // If Create Success
	else
	{
        VERBOSE_VERBOSE("Succeeded to create the channel : '%s'\n",
                        SYS_RM_CHANNEL_NAME);
	}

    return (g_chidRM == INVALID_CHID) ?
        RESULT_ERROR : RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: MSG_CloseSiblingConnection(int nCoid, int nCoidAlive)
//      - Close Sibling Channel Connection
//        - nCoid: g_coidSC, g_coidTE
//        - nCoidAlive: g_coidSCalive, g_coidTEalive

int MSG_CloseSiblingConnection(int nCoid, int nCoidAlive)
{
    int nResult_Co, nResult_CoAlive;

    // Init Variables
    nResult_Co = -1;
    nResult_CoAlive = -1;

    // Alive Check Thread is Paused
    g_fSysAliveThRun = PAUSE;
    
    // In Case Of Main Connection
    // Detach Connection
    if(nCoid != INVALID_COID)
        nResult_Co = MSG_DetachConnection(nCoid);
    
    // If Detach Connection Fail
    if(nResult_Co == RESULT_ERROR &&
      (g_pShm_SysStatus->fExitProcTE != TRUE &&
       g_pShm_SysStatus->fExitProcSC != TRUE))
    {
    	VERBOSE_ERROR("Failed to detach the connection to %d.\n", nCoid);
    }
    // If Detach Connection Success
    else if(nResult_Co != RESULT_ERROR)
    {
    	VERBOSE_VERBOSE("Succeeded to detach the connection to %d.\n", nCoid);
        if(nCoid == g_coidSC)
        {
            g_coidSC = INVALID_COID; 
        }
        if(nCoid == g_coidTE)
        {
            g_coidTE = INVALID_COID; 
        }
    }

    // In Case Of Alive Connection
    // Detach Connection
    if(nCoidAlive != INVALID_COID)
        nResult_CoAlive = MSG_DetachConnection(nCoidAlive);

    // If Detach Connection Fail
    if(nResult_CoAlive == RESULT_ERROR &&
      (g_pShm_SysStatus->fExitProcTE != TRUE &&
       g_pShm_SysStatus->fExitProcSC != TRUE))
    {
    	VERBOSE_ERROR("Failed to detach the alive connection to %d.\n",
                                                                 nCoidAlive);
    }
    // If Detach Connection Success
    else if(nResult_CoAlive != RESULT_ERROR)
    {
    	VERBOSE_VERBOSE("Succeeded to detach the alive connection to %d.\n",
                                                                    nCoidAlive);
        if(nCoidAlive == g_coidSCalive)
        {
            g_coidSCalive = INVALID_COID; 
        }
        if(nCoidAlive == g_coidTEalive)
        {
            g_coidTEalive = INVALID_COID; 
        }
    }

    // Alive Check Thread is Restarted
    g_fSysAliveThRun = RUN;

    return (nResult_Co == RESULT_ERROR || nResult_CoAlive == RESULT_ERROR) ?
        RESULT_ERROR: RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: MSG_CloseRMInternalConnection()
//      - Close Internal RM Channel Connection

int MSG_CloseRMInternalConnection(void)
{
    int nResult;

    // Init Variables
    nResult = -1;

    if(g_coidRM != INVALID_COID)
        nResult = MSG_DetachConnection(g_coidRM);
    
    // If Detach Connection Fail
    if(nResult == RESULT_ERROR)
    {
    	VERBOSE_ERROR("Failed to detach the connection to %d.\n", g_coidRM);
    }
    // If Detach Connection Success
    else if(nResult != RESULT_ERROR)
    {
    	VERBOSE_VERBOSE("Succeeded to detach the connection to %d.\n", g_coidRM);
        g_coidRM = INVALID_COID; 
    }

    return (nResult == RESULT_ERROR) ?  RESULT_ERROR: RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: MSG_CloseRMChannel()
//      - Close RM channel

int MSG_CloseRMChannel(void)
{
    int nResult_Ch;

    // Init Variable
    nResult_Ch = -1;

    // Finalize RM Channel
    if(g_chidRM != INVALID_CHID) 
	{
        // Destroy Channel
        nResult_Ch = MSG_DestroyChannel(g_chidRM);

        // If Destroy Channel Fail
	    if(nResult_Ch == RESULT_ERROR)
	    {
		    VERBOSE_ERROR("Failed to destroy the channel : '%s'\n",
                          SYS_RM_CHANNEL_NAME);
	    }
        // If Destroy Channel Succeess
	    else
	    {
		    VERBOSE_VERBOSE("Succeeded to destroy the channel : '%s'\n",
                            SYS_RM_CHANNEL_NAME);
            g_chidRM = INVALID_CHID; 
	    }
    }
    // If Destroy Channel Fail, Because Of Invalid Connection
    else
    {
        VERBOSE_WARNING("No channel to destroy. (%s)\n", SYS_RM_CHANNEL_NAME);
    }
    
    return (nResult_Ch == RESULT_ERROR)?
        RESULT_ERROR : RESULT_OK;
}
