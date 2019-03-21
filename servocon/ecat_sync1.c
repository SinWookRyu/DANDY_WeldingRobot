#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>

#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/trace.h>
#include <sys/dispatch.h>
#include <unistd.h>
#endif

#include "ecattypes.h"
#include "libmkpaiodev.h"
#include "mkpasyncfeedback.h"
#include "mkpaauxiliary.h"
#include "config.h"


#include "servocon_main.h"
#include "dandy_platform.h"
#include "dandy_thread.h"
#include "dandy_echo.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// SYNCRONISATION
/////////////////////////////////////////////////////////////////////////////////////////////
#if defined(__QNXNTO__)
name_attach_t *sync_channel = ECAT_NULL;

ECAT_RESULT EcatSyncInit(void)
{
	// create named channel
	if(sync_channel == ECAT_NULL){
		if ((sync_channel = name_attach(NULL, KPA_SYNC_CHANNEL_POINT, 0)) == NULL) {
			printf("Can't create communication channel.\n");
		    return ECAT_E_FAIL;
		}
	}	
	return ECAT_S_OK;
}

ECAT_RESULT EcatSyncWait(void)
{
	int rcvid;
	struct _pulse   pulse;
	
	if(sync_channel == ECAT_NULL)
		return ECAT_E_FAIL;
	
	// wait for message from channel
	rcvid = MsgReceive (sync_channel->chid, &pulse, sizeof (pulse), NULL);
	if(rcvid != 0){
		return ECAT_E_FAIL;
	}
	
	return ECAT_S_OK;
}

void EcatSyncDestroy(void)
{
	if(sync_channel != ECAT_NULL){
		 name_detach(sync_channel, 0);
		 sync_channel = ECAT_NULL;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////
// SYNCRONISATION THREAD
/////////////////////////////////////////////////////////////////////////////////////////////
 

ECAT_RESULT EcatSyncStartSyncThread(ECAT_HANDLE hMaster, EcatSyncHandler pfnCall, ECAT_LPVOID  pvArg);
ECAT_RESULT EcatSyncStopSyncThread(ECAT_HANDLE hMaster);



static ECAT_BOOL g_bSyncExit = ECAT_FALSE;
static int g_nSyncThreadId = -1;
static void* SyncThread(ECAT_LPVOID lpArg);

typedef struct tagSyncThreadData{
	EcatSyncHandler pfnSyncCall;
	ECAT_LPVOID  pvSyncArg;
}SyncThreadData;


ECAT_RESULT EcatSyncStartSyncThread(ECAT_HANDLE hMaster, EcatSyncHandler pfnCall, ECAT_LPVOID  pvArg)
{
	ECAT_RESULT erResult;
	SyncThreadData* pData = malloc(sizeof(SyncThreadData));
	if(pData == NULL){
		return ECAT_E_FAIL;
	}
	g_bSyncExit = ECAT_FALSE; 
	pData->pfnSyncCall = pfnCall;
	pData->pvSyncArg = pvArg;
	
	if (ECAT_FAILED(EcatSyncInit())){
    	printf("can't initialize sync\n");
    	return ECAT_E_FAIL;
    }
		
	// start process task
    printf("\nStart synchronization process task...");
    if (ECAT_FAILED(erResult = EcatIODevStartTask(hMaster, ECAT_TRUE,KPA_SYNC_PROCESS_TASK_LIBRARY))){
    	//printf("\nCan't start process task: 0x%04X (%s)\n",
    	//        			erResult, KPAGetErrorDescription(erResult));
    	free(pData);
    	return ECAT_E_FAIL;
    }
    printf("complete\n");
	
	if (pthread_create(&g_nSyncThreadId,
	    	                NULL,
	    	                SyncThread,
	    	                pData) != 0){
		free(pData);
		return ECAT_E_FAIL;
	}
	return ECAT_S_OK;
}

ECAT_RESULT EcatSyncStopSyncThread(ECAT_HANDLE hMaster)
{
	ECAT_RESULT erResult;
	g_bSyncExit = ECAT_TRUE;
	if(g_nSyncThreadId != -1){
		pthread_cancel(g_nSyncThreadId);
	}
	
	printf("\nStop process task...");
    if (ECAT_FAILED(erResult = EcatIODevStartTask(hMaster, ECAT_FALSE,KPA_SYNC_PROCESS_TASK_LIBRARY))){
    	//printf("\nCan't stop process task: 0x%04X (%s)\n",
    	//    	        			erResult, KPAGetErrorDescription(erResult));
    }
    printf("complete\n");
    EcatSyncDestroy();
    return ECAT_S_OK;
}

void* SyncThread(ECAT_LPVOID lpArg)
{
	SyncThreadData SyncData = *((SyncThreadData*)lpArg);
	free(lpArg);
	
	// infinity loop
	while(!g_bSyncExit){	
		ECAT_RESULT res;
		    	
    	// wait syncronisation
    	if(ECAT_FAILED(res = EcatSyncWait())){
    		break;
    	} 	
    	SyncData.pfnSyncCall(SyncData.pvSyncArg);

	}
	    	
	return 0;
}
#endif


















