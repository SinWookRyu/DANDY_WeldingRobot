#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "servocon_main.h"
#include "dandy_platform.h"
#include "dandy_thread.h"
#include "dandy_echo.h"

int g_nErrCodeEcat;                     // error code of EtherCAT
int g_fMasterInitEventActive = OFF;     // Master Init Event Flag
int g_fEcatStartUpDone = 0;
int g_fEcatStateChangeDone = 0;

////////////////////////////////////////////////////////////////////////////////
//
// Functions
//
#if defined(__QNXNTO__)
ECAT_CHAR* ECATNET_GetEcatModeString(ECAT_BYTE byState);
ECAT_CHAR* ECATLIB_GetErrorDescription(ECAT_RESULT nErrorCode);
void  ECATLIB_SleepMS(ECAT_TIME_MS timeMS);
void  ECATLIB_wchar2char(ECAT_CHAR* psz, const ECAT_WCHAR* pwsz);

#include <unistd.h>
#include <pthread.h>

#include "ecatmkpa.h"
#include "libmkpaiodev.h"
#include "mkpaauxiliary.h"
#include "mkpasyncfeedback.h"
#include "config.h"
#endif

int ECATNET_GetSlaveCountFromNetwork(void);
int ECATNET_GetSlaveCountFromFile(void);
int ECATNET_GetSlaveState(void);


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_GetEcatModeString()
//
ECAT_CHAR* ECATNET_GetEcatModeString(ECAT_BYTE byState)
{
#if defined(__QNXNTO__)
    switch (byState)
    {
    case EcatStateNotSet:   return "Undefined";
    case EcatStateI:        return "Init";
    case EcatStateP:        return "Pre-Operational";
    case EcatStateB:        return "Bootstrap";
    case EcatStateS:        return "Safe-Operational";
    case EcatStateO:        return "Operational";
    default:                return "Invalid state";
    }
#endif
    return "WIN";
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ECATNET_GetAdapterName()
//
#if defined (__QNXNTO__)
static ECAT_RESULT _loc_ECATNET_GetAdapterName(ECAT_WORD wIndex, const ECAT_WCHAR** ppwszName)
{
	ECAT_WORD wCount;
	ECAT_WCHAR** ppwszAdapters;

	if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetAdapterList(NULL, &ppwszAdapters, &wCount)))
	{
		VERBOSE_WARNING("Cannot get adapter list.\n");
	}

	if (wCount <= wIndex)
	{
		VERBOSE_WARNING("Cannot find the adapter %d\n", wIndex);
		return ECAT_E_FAIL;
	}

	// show adapter list
	VERBOSE_VERBOSE("Adapter count : %d\n", (int) wCount);

	*ppwszName = ppwszAdapters[wIndex];

	return ECAT_S_OK;
}
#endif


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ECATNET_LoadConfiguration()
//
#if defined (__QNXNTO__)
static ECAT_RESULT _loc_ECATNET_LoadConfiguration(ECAT_HANDLE hMaster, ECAT_CHAR* pszFileName)
{
	ECAT_CHAR szBuffer[ECAT_LOAD_CONFIG_BUF_SIZE];
	size_t nLength;
	ECAT_BOOL bDone;
	ECAT_DWORD dwCookie;
	FILE* fpStream;

	// open file stream
	fpStream = fopen(pszFileName, "rb");

	if (fpStream == NULL)
	{
		VERBOSE_WARNING("Cannot open the \'%s\' configuration file.\n", pszFileName);
        fclose(fpStream);
		return ECAT_E_FAIL;
	}

	for (;;)
	{
		nLength = fread(szBuffer, sizeof(ECAT_CHAR), ECAT_LOAD_CONFIG_BUF_SIZE, fpStream);
		bDone = (nLength <= 0) ?  ECAT_TRUE : ECAT_FALSE;
		g_nErrCodeEcat = EcatIODevLoadConfigFromString(hMaster,
				                                       szBuffer,
				                                      (ECAT_DWORD)nLength,
				                                       bDone,
	             			                           &dwCookie);
		if (bDone || ECAT_FAILED(g_nErrCodeEcat))
		{
			break;
		}
	}

	fclose(fpStream);

	if (ECAT_FAILED(g_nErrCodeEcat))
	{
		VERBOSE_ERROR("An error occurred while loading the \"%s\" XML file.\n", pszFileName);
		return ECAT_E_FAIL;
	}
	else
	{
		VERBOSE_VERBOSE("Loaded master configuration file : \"%s\"\n", pszFileName);
	}

	return ECAT_S_OK;
}
#endif


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_ReleaseMaster()
//
int ECATNET_ReleaseMaster(void)
{    
#if defined (__QNXNTO__)
    //ECAT_RESULT hr;
    ECAT_BYTE byState;
    ECAT_BYTE byReqState;
    int nWait;

	if (g_hMaster != NULL)
	{
        g_fMasterInitEventActive = ON;
        
        ECATLIB_SleepMS(200);

        // change state (operational -> safe operational)
        byReqState = EcatStateS;    //EcatStateI;

        if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevRequestMasterState(g_hMaster, byReqState)))
        {
            VERBOSE_WARNING("Cannot request master state: %04X(%s)\n",
                            g_nErrCodeEcat,
                            ECATLIB_GetErrorDescription(g_nErrCodeEcat));
        }
        else
        {
        	VERBOSE_VERBOSE("Request master state: %s...\n",
                            ECATNET_GetEcatModeString(byReqState));
            ECATLIB_SleepMS(500);
        }

        nWait = 10;
        do
        {
            ECATLIB_SleepMS(ECAT_AUTORECOVERY_TIMEOUT_MS * 5);  // 0.5sec

            // check master state
            if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetMasterState(g_hMaster, &byState)))
            {
                VERBOSE_WARNING("Cannot get master state: %04X(%s)\n",
                		        g_nErrCodeEcat,
                		        ECATLIB_GetErrorDescription(g_nErrCodeEcat));

                g_fMasterInitEventActive = OFF;
                return ECAT_E_FAIL;
                //goto STOP_ECAT_MASTER;
            }
            if (byState == byReqState)
            {
            	break;
            }
        }
        while (--nWait > 0);

        if(nWait <= 0)
        {
            VERBOSE_VERBOSE("Current master state : %s\n", ECATNET_GetEcatModeString(byState));
            VERBOSE_WARNING("Can't reach requested state \n");

            g_fMasterInitEventActive = OFF;
            return ECAT_E_FAIL;
        }
        else
        {
            VERBOSE_VERBOSE("Changed master state : %s\n",
            		        ECATNET_GetEcatModeString(byReqState));
        }
#if 0
        if (ECAT_FAILED(g_nErrCodeEcat = EcatSyncStopSyncThread(g_hMaster))){
            VERBOSE_VERBOSE("Can't stop sync thread: %04X(%s)",g_nErrCodeEcat,KPAGetErrorDescription(g_nErrCodeEcat));
        }
#endif
        // Stop cyclic PI Update Cycle
        g_fSetEcatParam = TRUE;

        if(g_pShmem_sc != NULL)
        {
        	g_pShmem_sc->sysstate.fEcatInitState = FALSE;
        }

        THREAD_Sleep(200);

        // stop cyclic operation
        if(ECAT_FAILED(g_nErrCodeEcat = EcatIODevStopCyclicOperation(g_hMaster)))
        {
        	VERBOSE_WARNING("Cannot stop cyclic operation.\n");
        }
        else
        {
        	VERBOSE_VERBOSE("Stop cyclic operation.\n");
        }
        
        // disconnect master
        if(ECAT_FAILED(g_nErrCodeEcat = EcatIODevDisconnectMaster(g_hMaster)))
        {
        	VERBOSE_WARNING("Cannot disconnect master.\n");
        }
        else
        {
        	VERBOSE_VERBOSE("Disconnected master.\n");
        }
        
        // free(release) master
        if(ECAT_FAILED(g_nErrCodeEcat = EcatIODevFreeMaster(g_hMaster)))
        {
        	VERBOSE_WARNING("Cannot release master\n");
        }
        else
        {
        	VERBOSE_VERBOSE("Released master\n");
        }
        
        VERBOSE_MESSAGE("Master release done.!!!\n");

        g_fMasterInitEventActive = OFF;
        g_hMaster = NULL;
        g_fConfigLoad = FALSE;
        g_fSetEcatParam = FALSE;
	}
	else
	{
		VERBOSE_WARNING("Master already released.!!!\n");
	}

#else
    if(g_pShmem_sc != NULL)
    {
        g_pShmem_sc->sysstate.fEcatInitState = FALSE;
    }

    VERBOSE_VERBOSE("Released EtherCAT Master.\n");
#endif

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ECATNET_ChangeMasterState()
//
int _loc_ECATNET_ChangeMasterState(ECAT_BYTE byReqState)
{
#if defined (__QNXNTO__)
	ECAT_BYTE byState;
	int nWait;

	// (8) request master state (init or safe operational -> operational)
    //byReqState = EcatStateO;   // operational state
    VERBOSE_VERBOSE("Request master state : %s\n", ECATNET_GetEcatModeString(byReqState));
	if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevRequestMasterState(g_hMaster, byReqState)))
	{
		VERBOSE_WARNING("Cannot request master state.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}

	// wait for master state 5 seconds
    //nWait = 1000 / ECAT_AUTORECOVERY_TIMEOUT_MS;  // 10 count
	nWait = 10;
    do
    {
        ECATLIB_SleepMS(ECAT_AUTORECOVERY_TIMEOUT_MS * 5);  // 0.5sec

        // check master state
        if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetMasterState(g_hMaster, &byState)))
        {
            VERBOSE_WARNING("Cannot get master state: %04X(%s)\n",
            		        g_nErrCodeEcat,
            		        ECATLIB_GetErrorDescription(g_nErrCodeEcat));
            ECATNET_ReleaseMaster();
            return ECAT_E_FAIL;
        }

        if (byState == byReqState)
        {
        	break;
        }
    }
    while (--nWait > 0);

    if(nWait <= 0)
    {
        VERBOSE_WARNING("Current master state : %s\n", ECATNET_GetEcatModeString(byState));
        VERBOSE_WARNING("Can't reach requested state \n");

        return ECAT_E_FAIL;
    }
    else
    {
        VERBOSE_VERBOSE("Changed master state : %s\n",
        		        ECATNET_GetEcatModeString(byReqState));
    }

    return RESULT_OK;
#else
    return RESULT_OK;
#endif
}

ECAT_RESULT SyncCtrlHandler(
    IN const ECAT_LPVOID  pvArg)
{
	VERBOSE_VERBOSE("sync callback\n");

	return ECAT_S_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ECATNET_StartUp()
//

static int _loc_ECATNET_StartUp(void)
{
#if defined (__QNXNTO__)
	ECAT_WORD wAdapterIndex1;
	const ECAT_WCHAR* pwszAdapter1;
#if defined (REDUNDANCY_MODE)
    ECAT_WORD wAdapterIndex2;
    const ECAT_WCHAR* pwszAdapter2;
#endif
	char pstrParams[1024];
	ECAT_DWORD dwVersionMS, dwVersionLS;
    ECAT_WORD  wInputsSize, wOutputsSize;

	// (0) get master version
	if(ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetMasterVersion(&dwVersionMS, &dwVersionLS)))
	{
		VERBOSE_WARNING("Cannot get master version.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
	else
	{
		VERBOSE_VERBOSE("Master Version : %d.%d.%d.%d\n",
				        ECAT_HIWORD(dwVersionMS),
				        ECAT_LOWORD(dwVersionMS),
				        ECAT_HIWORD(dwVersionLS),
				        ECAT_LOWORD(dwVersionLS));
	}

	// (1) get adapter name
	wAdapterIndex1 = ECAT_MASTER_REDUNDANCY_CHANNEL_MAIN;
	if(ECAT_FAILED(_loc_ECATNET_GetAdapterName(wAdapterIndex1, &pwszAdapter1)))
	{
		VERBOSE_WARNING("Cannot get Main adapter name.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
    else
    {
  	    ECATLIB_wchar2char(pstrParams, pwszAdapter1);
        VERBOSE_VERBOSE("Master Main adapter name : %s\n", pstrParams);
    }
#if defined (REDUNDANCY_MODE)
    wAdapterIndex2 = ECAT_MASTER_REDUNDANCY_CHANNEL_SECONDARY;
	if(ECAT_FAILED(_loc_ECATNET_GetAdapterName(wAdapterIndex2, &pwszAdapter2)))
	{
		VERBOSE_WARNING("Cannot get Secondary adapter name.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
    else
    {
  	    ECATLIB_wchar2char(pstrParams, pwszAdapter2);
        VERBOSE_VERBOSE("Master Secondary adapter name : %s\n", pstrParams);
    }
#endif
	// (2) create master
    if(g_hMaster == NULL)
    {
        g_nErrCodeEcat = EcatIODevCreateMaster(&g_hMaster);

        if(ECAT_FAILED(g_nErrCodeEcat))
        {
            VERBOSE_WARNING("Cannot create master.\n");
            ECATNET_ReleaseMaster();
            return ECAT_E_FAIL;
        }
        else
	    {
	    	VERBOSE_VERBOSE("Created master.\n");
	    }
    }
#if 0
    VERBOSE_VERBOSE("Start sync thread\n");
    if (ECAT_FAILED(g_nErrCodeEcat = EcatSyncStartSyncThread(g_hMaster,SyncCtrlHandler,(ECAT_LPVOID)g_hMaster))){
	   VERBOSE_VERBOSE("Can't start sync thread: %04X(%s)",g_nErrCodeEcat,KPAGetErrorDescription(g_nErrCodeEcat));
	   ECATNET_ReleaseMaster();
    }
#endif
	// (3) Connect master
#if defined (REDUNDANCY_MODE)
	g_nErrCodeEcat = EcatIODevConnectMaster(g_hMaster,
                                            ECAT_MASTER_REDUNDANCY_TYPE_REDUNDANCY,
			                                pwszAdapter1,
			                                pwszAdapter2,
			                                ECAT_NULL);
#else
    g_nErrCodeEcat = EcatIODevConnectMaster(g_hMaster,
			                                ECAT_MASTER_REDUNDANCY_TYPE_NORMAL,
			                                pwszAdapter1,
                                            ECAT_NULL,
			                                ECAT_NULL);
#endif

	if(ECAT_FAILED(g_nErrCodeEcat) && (g_nErrCodeEcat != ECAT_E_MASTER_ALREADY_CONNECTED))
	{
		VERBOSE_WARNING("Cannot connect master.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
	else
	{
		VERBOSE_VERBOSE("Connected master.\n");
	}
    
	// (4) Load master configuration
    if(g_fConfigLoad == FALSE)
    {
        g_nErrCodeEcat = _loc_ECATNET_LoadConfiguration(g_hMaster, g_pszEcatConfigDir);
        
        if(ECAT_FAILED(g_nErrCodeEcat))
        {
            VERBOSE_WARNING("Cannot load configuration file.\n");
            ECATNET_ReleaseMaster();
            g_fConfigLoad = FALSE;
            g_pShmem_sc->sysstate.fErrorState = TRUE;
            g_pShmem_sc->sysstate.nErrorCode = ECAT_ERR_LOAD_XML_CONFIG;

            return ECAT_E_FAIL;
        }
        else
        {
            VERBOSE_VERBOSE("Master config. file was loaded successfully.\n");
            g_fConfigLoad = TRUE;
        }
    }
    
#if 0
    VERBOSE_VERBOSE("\n Set external cycle time...\n");
	if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevSetExternalCycleTime(g_hMaster, 1000 * 1000ULL)))
    {
		printf("Can't set external cycle time: %04X(%s)",
               g_nErrCodeEcat,
               KPAGetErrorDescription(g_nErrCodeEcat));
		//goto finish;
	}
#endif
#if 1
    // (5) Make Process Image
    if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetProcessImageSize(g_hMaster,
                                                                  &wInputsSize,
                                                                  &wOutputsSize)))
    {
        VERBOSE_WARNING("Cannot get process image size.\n");
		ECATNET_ReleaseMaster();
        return ECAT_E_FAIL;
    }
    VERBOSE_VERBOSE(" Process image size : Inputs %d byte(s), Outputs %d byte(s)\n",
				    wInputsSize, wOutputsSize);
#endif
	// (6) start cyclic operation
#if 1
	g_nErrCodeEcat = EcatIODevStartCyclicOperation(g_hMaster,
			                                       ECAT_MASTER_CYCLE_TIMEOUT_US,
			                                       ECAT_MASTER_SUBCYCLE_TIMEOUT_US,
			                                       ECAT_MASTER_PRIORITY);

	if (ECAT_FAILED(g_nErrCodeEcat) && (g_nErrCodeEcat != ECAT_E_MASTER_ALREADY_STARTED))
	{
		VERBOSE_WARNING("Cannot start cyclic operation.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
	else
	{
		VERBOSE_VERBOSE("Started cyclic operation.\n");
	}
#endif

	// (7) set auto recovery timeout
	if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevSetAutoRecoveryTimeout(g_hMaster, ECAT_AUTORECOVERY_TIMEOUT_MS)))
	{
		VERBOSE_WARNING("Cannot set auto recovery timeout.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}
	else
	{
		VERBOSE_VERBOSE("Set auto recovery timeout.\n");
	}

#if 0
	// (8) request master state (init or safe operational -> operational)
    byReqState = EcatStateO;   // operational state
    VERBOSE_VERBOSE("Request master state : %s\n", ECATNET_GetEcatModeString(byReqState));
	if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevRequestMasterState(g_hMaster, byReqState)))
	{
		VERBOSE_WARNING("Cannot request master state.\n");
		ECATNET_ReleaseMaster();
		return ECAT_E_FAIL;
	}

	// wait for master state 5 seconds
    nWait = 1000 / ECAT_AUTORECOVERY_TIMEOUT_MS;  // 10 count
    do
    {
        ECATLIB_SleepMS(ECAT_AUTORECOVERY_TIMEOUT_MS * 5);  // 0.5sec

        // check master state
        if (ECAT_FAILED(g_nErrCodeEcat = EcatIODevGetMasterState(g_hMaster, &byState)))
        {
            VERBOSE_WARNING("Cannot get master state: %04X(%s)\n",
            		        g_nErrCodeEcat,
            		        ECATLIB_GetErrorDescription(g_nErrCodeEcat));
            ECATNET_ReleaseMaster();
            return ECAT_E_FAIL;
        }

        if (byState == byReqState)
        {
        	break;
        }
    }
    while (--nWait > 0);

    if(nWait <= 0)
    {
        VERBOSE_WARNING("Current master state : %s\n", ECATNET_GetEcatModeString(byState));
        VERBOSE_WARNING("Can't reach requested state \n");

        return ECAT_E_FAIL;
    }
    else
    {
        VERBOSE_VERBOSE("Changed master state : %s\n",
        		        ECATNET_GetEcatModeString(byReqState));
    }
#endif
    return RESULT_OK;
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_InitializeMaster()
//
int ECATNET_InitializeMaster(void)
{
#if defined (__QNXNTO__)
    g_fMasterInitEventActive = ON;

	// EtherCAT Master init.
	g_fEcatStartUpDone = _loc_ECATNET_StartUp();

    if(g_fEcatStartUpDone == ECAT_E_FAIL)
    {
        if(g_pShmem_sc != NULL)
        {
        	g_pShmem_sc->sysstate.fEcatInitState = FALSE;  // write 'EcatInit' flag to SC_SHM
            g_pShmem_sc->sysstate.fErrorState = TRUE;
            g_pShmem_sc->sysstate.nErrorCode = ECAT_ERR_INIT_ECAT_FAIL;
        }

    	VERBOSE_ERROR("Failed to initialize EhterCAT Master.\n");
        return RESULT_ERROR;
    }
    else if(g_fEcatStartUpDone == RESULT_OK)
    {
        if(g_pShmem_sc != NULL)
    	{
    		g_pShmem_sc->sysstate.fEcatInitState = TRUE;  // write 'EcatInit' flag to SC_SHM
            if(g_pShmem_sc->sysstate.nErrorCode == ECAT_ERR_INIT_ECAT_FAIL ||
               g_pShmem_sc->sysstate.nErrorCode == ECAT_ERR_LOAD_XML_CONFIG)
            {
                g_pShmem_sc->sysstate.fErrorState = FALSE;
                g_pShmem_sc->sysstate.nErrorCode = SYS_ERR_OK;
            }
    	}
        
        SERV_GetNetworkState();
        VERBOSE_VERBOSE("Configured No. of Slave: %d\n", g_nSlaveCntNetwork);

        VERBOSE_MESSAGE("Master Initialize Done!\n");
    }

    THREAD_Sleep(200);
    
    if(g_fEcatStartUpDone == RESULT_OK)
    {
        g_fEcatStateChangeDone = _loc_ECATNET_ChangeMasterState(EcatStateO);
        if(g_fEcatStateChangeDone == ECAT_E_FAIL)
        {
            if(g_pShmem_sc != NULL)
            {
            	g_pShmem_sc->sysstate.fEcatInitState = FALSE;  // write 'EcatInit' flag to SC_SHM
                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = ECAT_ERR_CHANGE_OP_MODE;
            }

        	VERBOSE_ERROR("Failed to Change EhterCAT Master State.\n");
        }
        else if(g_fEcatStateChangeDone == RESULT_OK)
        {
            if(g_pShmem_sc != NULL)
        	{
        		g_pShmem_sc->sysstate.fEcatInitState = TRUE;  // write 'EcatInit' flag to SC_SHM
                if(g_pShmem_sc->sysstate.nErrorCode == ECAT_ERR_INIT_ECAT_FAIL ||
                   g_pShmem_sc->sysstate.nErrorCode == ECAT_ERR_LOAD_XML_CONFIG ||
                   g_pShmem_sc->sysstate.nErrorCode == ECAT_ERR_CHANGE_OP_MODE)
                {
                    g_pShmem_sc->sysstate.fErrorState = FALSE;
                    g_pShmem_sc->sysstate.nErrorCode = SYS_ERR_OK;
                }
        	}
            
            VERBOSE_MESSAGE("Change Master State Done!\n");
        }
    }
    
    g_fMasterInitEventActive = OFF;
	
    return RESULT_OK;
#else
    g_pShmem_sc->sysstate.fEcatInitState = TRUE;
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_GetSlaveCountFromNetwork()
//
// -return: slave count in network
//
int ECATNET_GetSlaveCountFromNetwork(void)
{
#if defined (__QNXNTO__)
    int nRet = 0;
    unsigned short SlaveCount = 0;

    nRet = EcatIODevSSGetSlaveCount(g_hMaster, &SlaveCount);

    if (nRet != 0)
    {
        return -1;        
    }

    g_nSlaveCount = SlaveCount;

    return SlaveCount;
#else
    return g_nSlaveCount;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_GetSlaveCountFromFile()
//
// -return: slave count in config. file
//
int ECATNET_GetSlaveCountFromFile(void)
{
#if defined (__QNXNTO__)
    int nRet;
    unsigned short SlaveCount = 0;

    nRet = EcatIODevGetSlaveCount(g_hMaster, &SlaveCount);
    
    if (nRet != 0)
    {
        return -1;
    }
    
    g_nSlaveCount = SlaveCount;

    return SlaveCount;
#else
    return g_nSlaveCount;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATNET_GetSlaveState()
//
int ECATNET_GetSlaveState(void)
{
#if defined (__QNXNTO__)
    //ECAT_BYTE byState;
    int byState = 0;
    int iSlave;

    for(iSlave = 0; iSlave < g_nSlaveCount; iSlave++)
    {
        byState = EcatIODevRequestSlaveState(g_hMaster, iSlave, byState);
        //VERBOSE_VERBOSE("Current %d slave state : %d\n", iSlave, byState);
    }

#endif
    return RESULT_OK;
}

#if 0
ECAT_RESULT ECATMKPA_CALL EcatIODevGetSlaveDiagnosticStates(
    IN ECAT_HANDLE      hMaster,
    OUT ECAT_DWORD*     pdwMasterDiagState,
    IN ECAT_WORD        wSlaveCount,
    IN ECAT_WORD*       pwSlaveId,
    OUT ECAT_DWORD*     pdwSlaveDiagState,
    OUT ECAT_SII_CONST_CONTENT_SHORT* pSIIContent);
#endif
