/////////////////////////////////////////////////////////////////////////////
//
//  sys_svc.c: System & Network Related Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#include "service.h"

///////////////////////////////////////


///////////////////////////////////////
//Global_variable
int g_nSlaveCntNetwork = 0;
int g_fSetEcatParam = FALSE;


/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// SERV_GetNetworkState()
//
int SERV_GetNetworkState(void)
{
    int nSlaveCntFile = 0;

    // get slave count in network
    g_nSlaveCntNetwork = ECATNET_GetSlaveCountFromNetwork();
    //VERBOSE_MESSAGE("Configured No. of Slave: %d\n", g_nSlaveCntNetwork);
    
    // get slave count in config. file
    nSlaveCntFile = ECATNET_GetSlaveCountFromFile();
    //VERBOSE_MESSAGE("file slave : %d\n", nSlaveCntFile);

    return(g_nSlaveCntNetwork != nSlaveCntFile || g_nSlaveCntNetwork <= 0 || nSlaveCntFile <= 0)
        ? RESULT_ERROR : RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetControlMode()
//
int SERV_SetControlMode(int nMode, int nOpt)
{
    int iAxis, nRet = 0;

    g_fSetEcatParam = TRUE;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet = ECATSERV_ControlModeSet(iAxis, nMode, nOpt);
    }

    g_fSetEcatParam = FALSE;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_ReadControlMode()
//
int SERV_ReadControlMode(void)
{
    int iAxis, nRet = 0;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet = ECATSERV_ControlModeRead(iAxis);
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetInterpolationTimePeriod()
//
int SERV_SetInterpolationTimePeriod(int nTimePeriod, int nOpt)
{
    int iAxis, nRet = 0;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet = ECATSERV_SetInterpolationTimePeriod(iAxis, nTimePeriod, nOpt);
    }

    VERBOSE_MESSAGE("Set Interplatioin Time Period: %d\n", nTimePeriod);

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetInterpolationTimeIndex()
//
int SERV_SetInterpolationTimeIndex(int nTimeIndex, int nOpt)
{
    int iAxis, nRet = 0;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet = ECATSERV_SetInterpolationTimeIndex(iAxis, nTimeIndex, nOpt);
    }

    VERBOSE_MESSAGE("Set Interplatioin Time Index: %d\n", nTimeIndex);

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetSyncErrorCountLimit()
//
int  SERV_SetSyncErrorCountLimit(int nCountLimit)
{
    int iAxis, nRet = 0;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet = ECATSERV_SetSyncErrorCountLimit(iAxis, nCountLimit);
    }

    return nRet;
}

////////////////////////////////////////////////////////////////////////////////
//
// SERV_GetServoState()
//
int SERV_GetServoState(void)
{
    int iAxis, i;
    int nRet;
    int nState[8];
    int nServoOnResult = 0;
    int mask;
    int nErrRegisterState = 0;
    int nErrCodeState = 0;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        mask = 0x0001;

        // read status word
        nRet = ECATSERV_ReadStatus(iAxis, ALL);

        if(nRet != 0)
        {
            VERBOSE_ERROR("Read Status Error: %d\n", g_nReadStatusValue[iAxis]);
            return RESULT_ERROR;
        }

        for(i = 0; i < 8; i++)
        {
            nState[i] = g_nReadStatusValue[iAxis] & mask;
            mask = mask << 1;
#if 0   //tmp for test
            VERBOSE_VERBOSE("[Axis %d] State Value: %x, nState: %x[%d]\n",
                            iAxis, g_nReadStatusValue[iAxis], nState[i], i);
#endif
        }

        nErrRegisterState = nErrRegisterState + g_nErrorRegister[iAxis];
        nErrCodeState = nErrCodeState + g_nErrCodeServo[iAxis];

        // Servo state (bit 2: xx110111)
        if(nState[0] == 0x01 && nState[1] == 0x02 && nState[2] == 0x04 &&
           nState[3] == 0x00 && nState[4] == 0x10)
        {
            g_nServoState[iAxis] = ON;
        }
        else
        {
            g_nServoState[iAxis] = OFF;
        }

        nServoOnResult = nServoOnResult + g_nServoState[iAxis];
    }

#if 0
    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        VERBOSE_VERBOSE("[Axis %d] Result: %d\n", iAxis, g_nServoState[iAxis]);
    VERBOSE_VERBOSE("All Result: %d\n", nServoOnResult);
#endif
#if 0
    if(g_pShmem_sc != NULL)
    {
        if(nServoOnResult == g_nAxisCount)
        {
            g_pShmem_sc->outputstate.fServoOnOutState = ON;
        }
        else if(nServoOnResult != g_nAxisCount)
        {
            g_pShmem_sc->outputstate.fServoOnOutState = OFF;
        }
    }
#endif

    // Check fault state (bit 3: xxxx1xxx & Err Flag)
    if(nState[3] == 0x08  || nErrRegisterState >= 1 ||
       nErrCodeState >= 1 || g_nErrCodeEcat >= 1 ||
       g_nEmergencyCodeServo[g_nErrAxis] != 0 || g_nErrCodeEcat != 0)
    {
        if(g_pShmem_sc != NULL)
        {
            g_pShmem_sc->sysstate.fErrorState = TRUE;
            if(g_nErrCodeEcat != 0)
            {
                g_pShmem_sc->sysstate.nErrorCode = g_nErrCodeEcat;
            }
        }
    }
    else
    {
#if 0
        if(g_pShmem_sc != NULL)
        {
            g_pShmem_sc->sysstate.fErrorState = FALSE;
        }
#endif
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_RestartMaster()
//
int SERV_RestartMaster(void)
{
    SERV_ClearServoAlarm();

    ECATNET_ReleaseMaster();

    ECATNET_InitializeMaster();

    //ECATNET_GetSlaveState();

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetVersion()
//
// -pMsg: message packet data
// 
int SERV_SetVersion(SC_MSG* pMsg)
{
    CRT_strcpy(pMsg->data.vers.sc_build,
               sizeof(pMsg->data.vers.sc_build), 
               SC_BUILD);

    CRT_strcpy(pMsg->data.vers.sc_vers,
               sizeof(pMsg->data.vers.sc_vers),
               SC_VERSION);            
        
    pMsg->size = sizeof(pMsg->data.vers);

    return RESULT_OK;
}
