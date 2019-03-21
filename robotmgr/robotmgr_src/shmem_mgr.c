/////////////////////////////////////////////////////////////////////////////
//
//  shmem_mgr.c: shared memory management
//                                            2013.04.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES

#include "ipc_jobshm.h"
#include "robotmgr_main.h"
#include "dandy_shmem.h"
#include <math.h>

///////////////////////////////////////
/* RM */
int g_hShm_SysStatus               = DANDY_INVALID_SHMEM_HANDLE;// SysStatus handle
SHM_RM_SYSSTATUS* g_pShm_SysStatus = NULL;              // SysStatus mapped memory
int g_hShm_SysParam                = DANDY_INVALID_SHMEM_HANDLE;// SysParam handle
SHM_RM_SYSPARAM*  g_pShm_SysParam  = NULL;              // SysParam mapped memory
int g_hShm_SysConfig               = DANDY_INVALID_SHMEM_HANDLE;// SysConfig handle
SHM_RM_SYSCONFIG* g_pShm_SysConfig = NULL;              // SysConfig mapped memory
int g_hShmemJobRM                  = DANDY_INVALID_SHMEM_HANDLE;// Job handle
SHM_DANDY_JOB* g_pShmemJobRM       = NULL;              // Job mapped memory

/* TE */
int g_hShmemTEStatus               = DANDY_INVALID_SHMEM_HANDLE;// TEhandle
SHM_TE_STATUS* g_pShmemTEStatus    = NULL;              // TE mapped memory
int g_hShmemTE_Restart             = DANDY_INVALID_SHMEM_HANDLE;// TEhandle
SHM_RESTART* g_pShmemTE_Restart    = NULL;              // TE mapped memory
int g_hShmemTE_Task_t              = DANDY_INVALID_SHMEM_HANDLE;// TEhandle
shm_mmi_task_t* g_pShmemTE_Task_t  = NULL;              // TE mapped memory

/* SC */
int g_hShmemSC                     = DANDY_INVALID_SHMEM_HANDLE;// SC handle
SHM_SC_SYSTEM* g_pShmemSC          = NULL;              // SC mapped memory

unsigned            g_nLoadSizeRestartSHM;
int                 g_retOpenShmemTE;
int                 g_retOpenShmemSC;

CONFIG_CTRL         g_nControlParam;
CONFIG_ECAT         g_nECATParam;

CONFIG_ROBOT*       g_pRobot;
CONFIG_AXIS*        g_pAxis;
CONFIG_MOTOR*       g_pMotor;
SHM_RM_SYSCONFIG*   g_pShm_SysConfig;
CONFIG_WELDER*      g_pWelder;

int SYSC_LoadParamToTE_RestartShmem(void);

int g_nTERunNextIdx = INIT_JOBRUNIDX;


/////////////////////////////////////////////////////////////////////////////
//
//  _loc_SHM_BuildJobShmem(int nRobotIdx)
//

static int _loc_SHM_BuildJobShmem(int nRobotIdx)
{
    unsigned long dwShmemSize;

    int hShmemHandle;
    SHM_DANDY_JOB* pShmem;

    ///////////////////////////////////
    //
    //  calc the shared memory size
    //
    
    dwShmemSize = GET_SHM_JOB_SIZE_EX(g_rgdwCmdSize[nRobotIdx],
                                      g_rgdwTVarSize[nRobotIdx],
                                      g_rgdwPVarSize[nRobotIdx],
                                      g_rgdwBVarSize[nRobotIdx],
                                      g_rgdwIVarSize[nRobotIdx],
                                      g_rgdwRVarSize[nRobotIdx],
                                      g_rgdwWeaveSize[nRobotIdx],
                                      g_rgdwSWFSize[nRobotIdx],
                                      g_rgdwMWFSize[nRobotIdx],
                                      g_rgdwEWFSize[nRobotIdx]);
    DANDY_ASSERT(dwShmemSize >= sizeof(SHM_DANDY_JOB));

    //VERBOSE_VERBOSE("the job shared memory size to create for RM : %lu bytes\n", dwShmemSize);

    ///////////////////////////////////
    //
    //  create the job shared memory
    //

    hShmemHandle = SHM_Create(SHM_DANDY_JOB_NAME, dwShmemSize);

    if (hShmemHandle == INVALID_SHMEM_HANDLE)
    {
       VERBOSE_ERROR("Cannot create Job shared memory\n");
        return RESULT_ERROR;
    }
    else
    {
		VERBOSE_VERBOSE("Create Job shared memory Done!\n"); 
	}

    ///////////////////////////////////
    //
    //  map the job shared memory
    //

    pShmem = (SHM_DANDY_JOB*) SHM_Map(hShmemHandle, dwShmemSize);

    if (pShmem == NULL)
    {
        VERBOSE_ERROR("Cannot map Job shared memory.\n"); 
        SHM_Destroy(hShmemHandle, SHM_DANDY_JOB_NAME);

        return RESULT_ERROR;
    }

    ///////////////////////////////////
    //
    //  assign the size of the shared memory
    //

    //VERBOSE_VERBOSE("Success creating the RM job shared memory\n");
    VERBOSE_VERBOSE("Mapped Job shared memory. Size:%ld bytes\n",
                        dwShmemSize); 

    // reset the memory
    memset(pShmem, 0, dwShmemSize);

    // assign the capacity sizes
    pShmem->dwCmdSize = g_rgdwCmdSize[nRobotIdx];
    pShmem->dwTVaSize = g_rgdwTVarSize[nRobotIdx];
    pShmem->dwPVaSize = g_rgdwPVarSize[nRobotIdx];
    pShmem->dwBVaSize = g_rgdwBVarSize[nRobotIdx];
    pShmem->dwIVaSize = g_rgdwIVarSize[nRobotIdx];
    pShmem->dwRVaSize = g_rgdwRVarSize[nRobotIdx];
    pShmem->dwWvfSize = g_rgdwWeaveSize[nRobotIdx];
    pShmem->dwSwfSize = g_rgdwSWFSize[nRobotIdx];
    pShmem->dwMwfSize = g_rgdwMWFSize[nRobotIdx];
    pShmem->dwEwfSize = g_rgdwEWFSize[nRobotIdx];

    // assign the total size
    pShmem->dwLength = dwShmemSize;

    ///////////////////////////////////
    //
    //  confirm the shared memory size
    //

    DANDY_ASSERT(pShmem->dwLength == GET_SHM_JOB_SIZE(pShmem));

    ///////////////////////////////////
    //
    //  assign the handle and the pointer
    //

    g_hShmemJobRM = hShmemHandle;
    g_pShmemJobRM = pShmem;

    VERBOSE_VERBOSE("\v\nCmdSize: %ld, TVaSize: %ld, PVaSize: %ld\n"
                    "BVaSize: %ld, IVaSize: %ld, RVaSize: %ld\n"
                    "WvfSize: %ld, SwfSize: %ld, MwfSize: %ld, EwfSize: %ld\n",
                    g_rgdwCmdSize[nRobotIdx],
                    g_rgdwTVarSize[nRobotIdx],
                    g_rgdwPVarSize[nRobotIdx],
                    g_rgdwBVarSize[nRobotIdx],
                    g_rgdwIVarSize[nRobotIdx],
                    g_rgdwRVarSize[nRobotIdx],
                    g_rgdwWeaveSize[nRobotIdx],
                    g_rgdwSWFSize[nRobotIdx],
                    g_rgdwMWFSize[nRobotIdx],
                    g_rgdwEWFSize[nRobotIdx]);

    return RESULT_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_SHM_DestroyJobShmem()
//

int _loc_SHM_DestroyJobShmem(void)
{
    unsigned long dwShmemSize;

    int hShmemHandle = g_hShmemJobRM;
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;

    ///////////////////////////////////
    //
    //  unmap the job shared memory
    //

    if (pShmem != NULL)
    {
        g_pShmemJobRM = NULL;

        dwShmemSize = GET_SHM_JOB_SIZE(pShmem);

        DANDY_ASSERT(dwShmemSize >= sizeof(SHM_DANDY_JOB));
        DANDY_ASSERT(pShmem->dwLength == dwShmemSize);

        SHM_Unmap(pShmem, dwShmemSize);

        VERBOSE_VERBOSE("Unmapped Job shared memory. Size:%ld bytes\n",
                        dwShmemSize); 
    }

    ///////////////////////////////////
    //
    //  destroy the job shared memory
    //

    if (hShmemHandle != INVALID_SHMEM_HANDLE)
    {
        g_hShmemJobRM = INVALID_SHMEM_HANDLE;
        SHM_Destroy(hShmemHandle, SHM_DANDY_JOB_NAME);

        VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHM_DANDY_JOB_NAME); 
    }
        
    ///////////////////////////////////
    //
    //
    //

    DANDY_ASSERT(g_hShmemJobRM == INVALID_SHMEM_HANDLE);
    DANDY_ASSERT(g_pShmemJobRM == NULL);

    //if (pShmem != NULL)
    //    VERBOSE_VERBOSE("the job shared memory for RM destroyed...\n");
    //else
    //    VERBOSE_VERBOSE("the job shared memory for RM checked....\n");

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  JOB_LoadJobToShmem()
//

BOOL JOB_LoadJobToShmem(const char* pszModuleName, const DANDY_JOB_MEM* pJobMem)
{
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;
    DANDY_JOB_MEM mem;
    
    ///////////////////////////////////

    DANDY_ASSERT(pJobMem != NULL);

    if (pShmem == NULL)
    {
        VERBOSE_ERROR("load failure : job shared memory is not created...\n");
        return RESULT_ERROR;
    }

    ///////////////////////////////////
    //
    //  make shared memory info to load the job
    //

    // cmd
    mem.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
    mem.size.nCmd = pShmem->dwCmdSize;

    // pos
    mem.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
    mem.size.nPos = pShmem->dwTVaSize;

    // wvf
    mem.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
    mem.size.nWvf = pShmem->dwWvfSize;

    // swf
    mem.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
    mem.size.nSwf = pShmem->dwSwfSize;

    // mwf
    mem.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
    mem.size.nMwf = pShmem->dwMwfSize;

    // ewf
    mem.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);
    mem.size.nEwf = pShmem->dwEwfSize;

    ///////////////////////////////////
    //
    //  load the job from metadata to shared memory
    //

    if (JOBMEM_Copy(pJobMem, &mem) == FALSE)
    {
        VERBOSE_ERROR("cannot load the job to the shared memory...\n");

        return RESULT_ERROR;
    }

    VERBOSE_MESSAGE("Success to load the job to shared memory\n");

    ///////////////////////////////////
    //
    //  assigned real loaded sizes
    //

    pShmem->dwCmdLoadCount = pJobMem->size.nCmd;
    pShmem->dwTVaLoadCount = pJobMem->size.nPos;
    pShmem->dwWvfLoadCount = pJobMem->size.nWvf;
    pShmem->dwSwfLoadCount = pJobMem->size.nSwf;
    pShmem->dwMwfLoadCount = pJobMem->size.nMwf;
    pShmem->dwEwfLoadCount = pJobMem->size.nEwf;
    
    // set module name
    CRT_strncpy(pShmem->szModuleName, sizeof(pShmem->szModuleName),
                pszModuleName, DANDY_JOB_MODULE_NAME_SIZE);
    pShmem->szModuleName[DANDY_JOB_MODULE_NAME_SIZE-1] = 0;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  JOB_DumpJobShmem()
//

int JOB_DumpJobShmem(int nOpt)
{
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;

    int nCmdCount;
    DANDY_JOB_MEM mem;

    ///////////////////////////////////

    if (pShmem == NULL)
    {
        VERBOSE_ERROR("the job shared memory is not created...\n");
        return RESULT_ERROR;
    }

    DSP_DispDivider();

    ///////////////////////////////////
    //
    //  display info
    //

    nCmdCount = pShmem->dwCmdLoadCount;

    VERBOSE_MESSAGE("module name   : '%s'\n", pShmem->szModuleName);
    VERBOSE_MESSAGE("command count : %d\n", nCmdCount);

    ///////////////////////////////////
    //
    //  build memory info
    //

    // cmd
    mem.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
    mem.size.nCmd = pShmem->dwCmdLoadCount;

    // pos
    mem.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
    mem.size.nPos = pShmem->dwTVaLoadCount;

    // wvf
    mem.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
    mem.size.nWvf = pShmem->dwWvfLoadCount;

    // swf
    mem.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
    mem.size.nSwf = pShmem->dwSwfLoadCount;

    // mwf
    mem.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
    mem.size.nMwf = pShmem->dwMwfLoadCount;

    // ewf
    mem.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);
    mem.size.nEwf = pShmem->dwEwfLoadCount;
    
    DANDY_SLEEP(100);
    
    ///////////////////////////////////
    //
    //  command dump
    //

    if(nOpt == 0 || nOpt == 1)
    {
        JOBDIS_DumpCmd(GET_STDOUT_HANDLE(), &mem, NULL, JOBDIS_DF_NONE);
    }

    if(nOpt == 0 || nOpt == 2)
    {
        JOBDIS_DumpPos(GET_STDOUT_HANDLE(), GET_SHM_JOB_TVA_BUFFER(pShmem), pShmem->dwTVaLoadCount, JOBDIS_DF_NONE);
    }

    if(nOpt == 0 || nOpt == 3)
    {
        JOBDIS_DumpWvf(GET_STDOUT_HANDLE(), GET_SHM_JOB_WVF_BUFFER(pShmem), pShmem->dwWvfLoadCount, JOBDIS_DF_NONE);
    }

    if(nOpt == 0 || nOpt == 4)
    {
        JOBDIS_DumpSwf(GET_STDOUT_HANDLE(), GET_SHM_JOB_SWF_BUFFER(pShmem), pShmem->dwSwfLoadCount, JOBDIS_DF_NONE);
    }

    if(nOpt == 0 || nOpt == 5)
    {
        JOBDIS_DumpMwf(GET_STDOUT_HANDLE(), GET_SHM_JOB_MWF_BUFFER(pShmem), pShmem->dwMwfLoadCount, JOBDIS_DF_NONE);
    }

    if(nOpt == 0 || nOpt == 6)
    {
        JOBDIS_DumpEwf(GET_STDOUT_HANDLE(), GET_SHM_JOB_EWF_BUFFER(pShmem), pShmem->dwEwfLoadCount, JOBDIS_DF_NONE);
    }

    ///////////////////////////////////

    DSP_DispDivider();

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_BuildSysStatusShmem()
//

static int _loc_SHM_BuildSysStatusShmem(void)
{
    VERBOSE_MESSAGE("-------- Shared memory Creating ----------\n\n");

    ///////////////////////////////////
    //
    //  create the shared memory
    //

    g_hShm_SysStatus = SHM_Create(SHM_RM_SYSSTATUS_NAME,
                                    sizeof(SHM_RM_SYSSTATUS));

    if (g_hShm_SysStatus == DANDY_INVALID_SHMEM_HANDLE)
    {
        VERBOSE_ERROR("Cannot create System Status shared memory\n");
        return RESULT_ERROR;
    }
    else
	{
		VERBOSE_VERBOSE("Create System Status shared memory Done!\n"); 
	}

    ///////////////////////////////////
    //
    //  map the shared memory handle
    //

    g_pShm_SysStatus = (SHM_RM_SYSSTATUS*) SHM_Map(g_hShm_SysStatus,
                        sizeof(SHM_RM_SYSSTATUS));
    memset(g_pShm_SysStatus, 0, sizeof(SHM_RM_SYSSTATUS));
    
    if(g_pShm_SysStatus == NULL) 
	{ 
		VERBOSE_VERBOSE("Cannot map System Status shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{
		g_pShm_SysStatus->nSize = sizeof(SHM_RM_SYSSTATUS); 
		VERBOSE_VERBOSE("Mapped System Status shared memory. Size:%d bytes\n",
                        g_pShm_SysStatus->nSize); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroySysStatusShmem()
//

static int _loc_SHM_DestroySysStatusShmem(void)
{
    if(g_pShm_SysStatus != NULL)
	{
		SHM_Unmap((void*)g_pShm_SysStatus, sizeof(SHM_RM_SYSSTATUS)); 
		g_pShm_SysStatus = NULL; 

		VERBOSE_VERBOSE("Unmapped System Status shared memory. Size:%dbytes\n",
                        sizeof(SHM_RM_SYSSTATUS)); 
	}
	if(g_hShm_SysStatus != RESULT_ERROR)
	{
		SHM_Destroy(g_hShm_SysStatus, SHM_RM_SYSSTATUS_NAME); 
		g_hShm_SysStatus = DANDY_INVALID_SHMEM_HANDLE;

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHM_RM_SYSSTATUS_NAME); 
	} 

    return RESULT_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadParamToShmem()
//      - copy parameters to shared memory

int SYSC_LoadParamToShmem(SHM_RM_SYSPARAM* pSystemParam)
{
    ///////////////////////////////////
    // WELDER PARAM
    memcpy(pSystemParam->weld_tune,
           g_rgWeldTuneParam,
           sizeof(WELD_PARAM_TUNE) * MAX_WELDER_COUNT);

    pSystemParam->nWeldTuneInParamApplyIndex   = g_nWeldTuneInParamApplyIndex;
    pSystemParam->nWeldTuneOutParamApplyIndex  = g_nWeldTuneOutParamApplyIndex;

    pSystemParam->nEstopGasOffDelayTime        = g_nEstopGasOffDelayTime;
    pSystemParam->nEstopTouchReadyOffDelayTime = g_nEstopTouchReadyOffDelayTime;

    pSystemParam->dbAinMaxVolt = g_dbAinMaxVolt;
    pSystemParam->dbADCMaxBit  = g_dbADCMaxBit;

    memcpy(pSystemParam->szSensDir,
           g_pszSensDir,
           PATH_NAME_BUFFER_SIZE);

    ///////////////////////////////////
    // RESTART PARAM
    memcpy(pSystemParam->TE_restart,
           g_rgRestartParam,
           sizeof(TE_RESTART_PARAM));

    ///////////////////////////////////
    // RESTART PARAM
    memcpy(pSystemParam->arcsensor,
           g_rgArcSensorParam,
           sizeof(ARCSENSOR_PARAM));

    return RESULT_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_BuildSysParamShmem()
//

static int _loc_SHM_BuildSysParamShmem(void)
{
    ///////////////////////////////////
    //
    //  create the shared memory
    //

    g_hShm_SysParam = SHM_Create(SHM_RM_SYSPARAM_NAME,
                                    sizeof(SHM_RM_SYSPARAM));

    if (g_hShm_SysParam == DANDY_INVALID_SHMEM_HANDLE)
    {
        VERBOSE_ERROR("Cannot create System Parameter shared memory\n");
        return RESULT_ERROR;
    }
    else
	{
		VERBOSE_VERBOSE("Create System Parameter shared memory Done!\n"); 
	}

    ///////////////////////////////////
    //
    //  map the shared memory handle
    //

    g_pShm_SysParam = (SHM_RM_SYSPARAM*) SHM_Map(g_hShm_SysParam,
                        sizeof(SHM_RM_SYSPARAM));
    memset(g_pShm_SysParam, 0, sizeof(SHM_RM_SYSPARAM));
    
    if(g_pShm_SysParam == NULL) 
	{ 
		VERBOSE_VERBOSE("Cannot map System Parameter shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{
		g_pShm_SysParam->nSize = sizeof(SHM_RM_SYSPARAM); 
		VERBOSE_VERBOSE("Mapped System Parameter shared memory. Size:%d bytes\n",
                        g_pShm_SysParam->nSize); 
	}

    ///////////////////////////////////
    //
    //  Load Config Data to Shared Memory
    //

    if (SYSC_LoadParamToShmem(g_pShm_SysParam) == RESULT_ERROR)
    {
        memset(g_pShm_SysParam, 0, sizeof(SHM_RM_SYSPARAM));

        SHM_Unmap(g_pShm_SysParam, sizeof(SHM_RM_SYSPARAM));
        SHM_Destroy(g_hShm_SysParam, SHM_RM_SYSPARAM_NAME);

        return RESULT_ERROR;
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroySysParamShmem()
//

static int _loc_SHM_DestroySysParamShmem(void)
{
    if(g_pShm_SysParam != NULL)
	{
		SHM_Unmap((void*)g_pShm_SysParam, sizeof(SHM_RM_SYSPARAM)); 
		g_pShm_SysParam = NULL; 

		VERBOSE_VERBOSE("Unmapped System Parameter shared memory. Size:%dbytes\n",
                        sizeof(SHM_RM_SYSPARAM)); 
	}
	if(g_hShm_SysParam != RESULT_ERROR)
	{
		SHM_Destroy(g_hShm_SysParam, SHM_RM_SYSPARAM_NAME); 
		g_hShm_SysParam = DANDY_INVALID_SHMEM_HANDLE;

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHM_RM_SYSPARAM_NAME); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_LoadTEStatusShmem()
//     - TE Shared Memory Get Var to Global

static int _loc_SHM_LoadTEStatusShmem(void)
{
    g_nTERunNextIdx = g_pShmemTEStatus->run_next_idx;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_OpenTEStatusShmem()
//     - TE Shared Memory Init

static int _loc_SHM_OpenTEStatusShmem(void)
{
    ///////////////////////////////////
    //
    //  Open TE shared memory
    //
	g_hShmemTEStatus = SHM_Open(SHMNAME_TE_TEST); 
	if(g_hShmemTEStatus == RESULT_ERROR)
	{
        if((g_Arg.bManualInit = FALSE))
        {
		    VERBOSE_ERROR("Failed to open the TE Status shared memory handle.\n"); 
        }
            return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Open Done! the TE Status shared memory handle. Handle:%d\n",
                        g_hShmemTEStatus); 
	}

	g_pShmemTEStatus = (SHM_TE_STATUS*) SHM_Map(g_hShmemTEStatus,
                                                sizeof(SHM_TE_STATUS)); 
	
    ///////////////////////////////////
    //
    //  Map TE shared memory
    //
	if(g_pShmemTEStatus == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map the TE Status shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{		
		VERBOSE_VERBOSE("Mapped the TE Status shared memory. Size:%dbytes\n",
                        g_pShmemTEStatus->size); 
	}

    ///////////////////////////////////
    //
    //  Size check TE shared memory
    //
	if(g_pShmemTEStatus->size != sizeof(SHM_TE_STATUS))
	{
		VERBOSE_ERROR("Mismatched the TE Status shared memory size."
                      "Map:%dbytes Chk:%dbytes \n",
                      sizeof(SHM_TE_STATUS), g_pShmemTEStatus->size); 
		return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Matched TE Status shared memory size."
                        "Map:%dbytes Chk:%dbytes \n",
                        sizeof(SHM_TE_STATUS), g_pShmemTEStatus->size); 

        _loc_SHM_LoadTEStatusShmem();
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroyTEShmem()
//

static int _loc_SHM_DestroyTEStatusShmem(void)
{
    if(g_pShmemTEStatus != NULL)
	{
		SHM_Unmap((void*)g_pShmemTEStatus, sizeof(SHM_TE_STATUS)); 
		g_pShmemTEStatus = NULL; 
		
		VERBOSE_VERBOSE("Unmapped the TE Status shared memory. Size:%dbytes\n",
                        sizeof(SHM_TE_STATUS)); 
	}
	if(g_hShmemTEStatus != RESULT_ERROR)
	{
        SHM_Close(g_hShmemTEStatus); 
		g_hShmemTEStatus = DANDY_INVALID_SHMEM_HANDLE; 

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHMNAME_TE_TEST); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadParamToTE_RestartShmem()
//      - copy parameters to TE Restart shared memory

int SYSC_LoadParamToTE_RestartShmem(void)
{
    if(g_pShmemTE_Restart == NULL || g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    g_pShmemTE_Restart->moving_type    = g_pShm_SysParam->TE_restart[0].moving_type;
    g_pShmemTE_Restart->d_overlap_horz = g_pShm_SysParam->TE_restart[0].d_overlap_horz;
    g_pShmemTE_Restart->d_overlap_vert = g_pShm_SysParam->TE_restart[0].d_overlap_vert;
    g_pShmemTE_Restart->path_speed     = g_pShm_SysParam->TE_restart[0].path_speed;
    g_pShmemTE_Restart->hori_start_vol = g_pShm_SysParam->TE_restart[0].hori_start_vol;
    g_pShmemTE_Restart->hori_start_cur = g_pShm_SysParam->TE_restart[0].hori_start_cur;
    g_pShmemTE_Restart->hori_main_vol  = g_pShm_SysParam->TE_restart[0].hori_main_vol;
    g_pShmemTE_Restart->hori_main_cur  = g_pShm_SysParam->TE_restart[0].hori_main_cur;
    g_pShmemTE_Restart->vert_start_vol = g_pShm_SysParam->TE_restart[0].vert_start_vol;
    g_pShmemTE_Restart->vert_start_cur = g_pShm_SysParam->TE_restart[0].vert_start_cur;
    g_pShmemTE_Restart->vert_main_vol  = g_pShm_SysParam->TE_restart[0].vert_main_vol;
    g_pShmemTE_Restart->vert_main_cur  = g_pShm_SysParam->TE_restart[0].vert_main_cur;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_OpenTERestartShmem()
//     - TE Restart Shared Memory Init

static int _loc_SHM_OpenTERestartShmem(void)
{
    int nRet = RESULT_OK;

    ///////////////////////////////////
    //
    //  Open TE Restart shared memory
    //
	g_hShmemTE_Restart = SHM_Open(SHMNAME_RESTART); 
	if(g_hShmemTE_Restart == RESULT_ERROR)
	{
        if((g_Arg.bManualInit = FALSE))
        {
		    VERBOSE_ERROR("Failed to open the TE Restart shared memory handle.\n"); 
        }
        
        return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Open Done! the TE Restart shared memory handle. Handle:%d\n",
                        g_hShmemTE_Restart); 
	}

	g_pShmemTE_Restart = (SHM_RESTART*) SHM_Map(g_hShmemTE_Restart,
                                                sizeof(SHM_RESTART)); 
	
    ///////////////////////////////////
    //
    //  Map TE Restart shared memory
    //
	if(g_pShmemTE_Restart == NULL) 
	{
		VERBOSE_ERROR("Cannot map the TE Restart shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{		
		VERBOSE_VERBOSE("Mapped the TE Restart shared memory. Size:%d bytes\n",
                        g_pShmemTE_Restart->size_total); 
	}
    
    g_nLoadSizeRestartSHM = g_pShmemTE_Restart->size_total;

    SHM_Unmap((void*)g_pShmemTE_Restart, sizeof(SHM_RESTART));

    g_pShmemTE_Restart = (SHM_RESTART*) SHM_Map(g_hShmemTE_Restart,
                                                g_nLoadSizeRestartSHM); 

    /* File data load */
    nRet = FUNC_ReadRestartParamFromFile();
    
    if(nRet == RESULT_ERROR)
    {
        return RESULT_ERROR;
    }

    nRet = SYSC_LoadParamToTE_RestartShmem();

    if(nRet == RESULT_ERROR)
    {
        return RESULT_ERROR;
    }
    
#if 0
    ///////////////////////////////////
    //
    //  Size check TE Restart shared memory
    //
	if(g_pShmemTE_Restart->size_total != sizeof(SHM_RESTART))
	{
		VERBOSE_ERROR("Mismatched the TE Restart shared memory size."
                      "Map:%dbytes Chk:%dbytes \n",
                      sizeof(SHM_RESTART), g_pShmemTE_Restart->size_total); 
		return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Matched TE Restart shared memory size."
                        "Map:%dbytes Chk:%dbytes \n",
                        sizeof(SHM_RESTART), g_pShmemTE_Restart->size_total); 
	}
#endif

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroyTERestartShmem()
//

static int _loc_SHM_DestroyTERestartShmem(void)
{
    if(g_pShmemTE_Restart != NULL)
	{
		//SHM_Unmap((void*)g_pShmemTE_Restart, sizeof(SHM_RESTART)); 
        SHM_Unmap((void*)g_pShmemTE_Restart, g_nLoadSizeRestartSHM); 
		g_pShmemTE_Restart = NULL; 
		
		VERBOSE_VERBOSE("Unmapped the TE Restart shared memory. Size:%dbytes\n",
                        g_nLoadSizeRestartSHM);             
                        //sizeof(SHM_RESTART)); 
	}
	if(g_hShmemTE_Restart != RESULT_ERROR)
	{
        SHM_Close(g_hShmemTE_Restart); 
		g_hShmemTE_Restart = DANDY_INVALID_SHMEM_HANDLE; 

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHMNAME_RESTART); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadParamToTE_RestartShmem()
//      - copy parameters to TE Task shared memory (Arc Sensor Parameters)

int SYSC_LoadParamToTE_TaskShmem(void)
{
    if(g_pShmemTE_Task_t == NULL || g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    g_pShmemTE_Task_t->wm.c_a =
        g_pShm_SysParam->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurr_a;
    g_pShmemTE_Task_t->wm.c_b =
        g_pShm_SysParam->weld_tune[g_nWeldTuneInParamApplyIndex].input.dbCurr_b;

    g_pShmemTE_Task_t->wm.ain_max = g_pShm_SysParam->dbAinMaxVolt;
    g_pShmemTE_Task_t->wm.adc_max = g_pShm_SysParam->dbADCMaxBit;

    g_pShmemTE_Task_t->sdata_save_cond.status =
                              g_pShm_SysParam->arcsensor[0].fSaveArcSensorData;
    g_pShmemTE_Task_t->sdata_save_cond.start =
                              g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo;
    g_pShmemTE_Task_t->sdata_save_cond.number =
                              g_pShm_SysParam->arcsensor[0].nSaveNodeCount;
    
    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_OpenTETaskShmem()
//     - TE ArcSensor Task Shared Memory Init

static int _loc_SHM_OpenTETaskShmem(void)
{
    int nRet = RESULT_OK;

    ///////////////////////////////////
    //
    //  Open TE ArcSensor Task shared memory
    //
	g_hShmemTE_Task_t = SHM_Open(SHMNAME_SENSOR_RM); 
	if(g_hShmemTE_Task_t == RESULT_ERROR)
	{
        if((g_Arg.bManualInit = FALSE))
        {
		    VERBOSE_ERROR("Failed to open the TE ArcSensor Task shared memory handle.\n"); 
        }
        
        return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Open Done! the TE ArcSensor Task shared memory handle. Handle:%d\n",
                        g_hShmemTE_Task_t); 
	}

	g_pShmemTE_Task_t = (shm_mmi_task_t*) SHM_Map(g_hShmemTE_Task_t,
                                                  sizeof(shm_mmi_task_t)); 
	
    ///////////////////////////////////
    //
    //  Map TE ArcSensor Task shared memory
    //
	if(g_pShmemTE_Task_t == NULL) 
	{
		VERBOSE_ERROR("Cannot map the TE ArcSensor Task shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{		
		VERBOSE_VERBOSE("Mapped the TE ArcSensor Task shared memory. Size:%dbytes\n",
                        g_pShmemTE_Task_t->size); 
	}

    ///////////////////////////////////
    //
    //  Size check TE Restart shared memory
    //
	if(g_pShmemTE_Task_t->size != sizeof(shm_mmi_task_t))
	{
		VERBOSE_ERROR("Mismatched the TE ArcSensor Task shared memory size."
                      "Map:%dbytes Chk:%dbytes \n",
                      sizeof(shm_mmi_task_t), g_pShmemTE_Task_t->size); 
		return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Matched TE ArcSensor Task shared memory size."
                        "Map:%dbytes Chk:%dbytes \n",
                        sizeof(shm_mmi_task_t), g_pShmemTE_Task_t->size); 
	}

    nRet = SYSC_LoadParamToTE_TaskShmem();

    if(nRet == RESULT_ERROR)
    {
        return RESULT_ERROR;
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroyTETaskShmem()
//

static int _loc_SHM_DestroyTETaskShmem(void)
{
    if(g_pShmemTE_Task_t != NULL)
	{
		SHM_Unmap((void*)g_pShmemTE_Task_t, sizeof(shm_mmi_task_t)); 
		g_pShmemTE_Task_t = NULL; 
		
		VERBOSE_VERBOSE("Unmapped the TE ArcSensor Task shared memory. Size:%dbytes\n",
                        sizeof(shm_mmi_task_t)); 
	}
	if(g_hShmemTE_Task_t != RESULT_ERROR)
	{
        SHM_Close(g_hShmemTE_Task_t); 
		g_hShmemTE_Task_t = DANDY_INVALID_SHMEM_HANDLE; 

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                        SHMNAME_SENSOR_RM); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_OpenSCShmem()
//    - SC Shared Memory Init

int _loc_SHM_OpenSCShmem(void)
{
    ///////////////////////////////////
    //
    //  Open SC shared memory
    //
	g_hShmemSC = SHM_Open(SC_SHM_NAME); 
	if(g_hShmemSC == RESULT_ERROR)
	{
        if((g_Arg.bManualInit = FALSE))
        {
		    VERBOSE_ERROR("Failed to open the SC shared memory handle.\n"); 
        }
            return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Open Done! the SC shared memory handle. Handle:%d\n",
                        g_hShmemSC); 
	}

	g_pShmemSC = (SHM_SC_SYSTEM*) SHM_Map(g_hShmemSC,
                                             sizeof(SHM_SC_SYSTEM)); 
	
    ///////////////////////////////////
    //
    //  Map SC shared memory
    //
	if(g_pShmemSC == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map the SC shared memory.\n"); 
		return RESULT_ERROR; 
	}
	else
	{		
		VERBOSE_VERBOSE("Mapped the SC shared memory. Size:%dbytes\n",
                        g_pShmemSC->nsize);
	}

    ///////////////////////////////////
    //
    //  Size check SC shared memory
    //
	if(g_pShmemSC->nsize != sizeof(SHM_SC_SYSTEM))
	{
		VERBOSE_ERROR("Mismatched the SC shared memory size."
                      "Map:%d bytes Chk:%d bytes \n",
                      sizeof(SHM_SC_SYSTEM), g_pShmemSC->nsize);
		return RESULT_ERROR; 
	}
	else
	{
		VERBOSE_VERBOSE("Matched SC shared memory size."
                        "Map:%d bytes Chk:%d bytes \n",
                        sizeof(SHM_SC_SYSTEM), g_pShmemSC->nsize);
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroySCShmem()
//

static int _loc_SHM_DestroySCShmem(void)
{
    if(g_pShmemSC != NULL)
	{
		SHM_Unmap((void*)g_pShmemSC, sizeof(SHM_SC_SYSTEM)); 
		g_pShmemSC = NULL; 
		
		VERBOSE_VERBOSE("Unmapped the SC shared memory. Size:%dbytes\n",
                        sizeof(SHM_SC_SYSTEM)); 
	}
	if(g_hShmemSC != RESULT_ERROR)
	{
		//SHM_Destroy(g_hShmemSC, SC_SHM_NAME); 
        SHM_Close(g_hShmemSC);
		g_hShmemSC = DANDY_INVALID_SHMEM_HANDLE; 

		VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n", SC_SHM_NAME); 
	}

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigToShmem()
//      - copy configurations to shared memory

int SYSC_LoadConfigToShmem(SHM_RM_SYSCONFIG* pSystemConfig)
{
    int iRobot, iAxis, iMotor, iIndex;
    int iWelder;
    int nAxesCount, nAxisIndex;

    //DANDY_ASSERT(pSystemConfig != NULL);

    ///////////////////////////////////
    // check validation
    //if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD)
    
    if (SYSC_ArrangeConfigGlobal() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange global config\n");
        return RESULT_ERROR;
    }

    if (SYSC_ArrangeConfigRobot() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange robot config\n");
        return RESULT_ERROR;
    }

    if (SYSC_ArrangeConfigAxis() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange axis config\n");
        return RESULT_ERROR;
    }

    if (SYSC_ArrangeConfigMotor() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange motor config\n");
        return RESULT_ERROR;
    }

    if (SYSC_ArrangeConfigWelder() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange welder config\n");
        return RESULT_ERROR;
    }

#if 0
    if (SYSC_ArrangeConfigSensor() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Cannot arrange sensor config\n");
        return RESULT_ERROR;
    }
#endif

    ///////////////////////////////////
    // check validation

    if (SYSC_CheckConfigGlobal() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid global config\n");
        return RESULT_ERROR;
    }

    if (SYSC_CheckConfigRobot() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid robot config\n");
        return RESULT_ERROR;
    }

    if (SYSC_CheckConfigAxis() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid axis config\n");
        return RESULT_ERROR;
    }

    if (SYSC_CheckConfigMotor() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid motor config\n");
        return RESULT_ERROR;
    }

    if (SYSC_CheckConfigWelder() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid welder axis config\n");
        return RESULT_ERROR;
    }
#if 0
    if (SYSC_CheckConfigSensor() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Invalid sensor axis config\n");
        return RESULT_ERROR;
    }
#endif
    ///////////////////////////////////
    // GLOBAL

    g_nControlParam.nQNXTimerTick   = g_nQNXTimerTick;  DANDY_ASSERT(g_nQNXTimerTick > 0);
    g_nControlParam.nQNXTimerRes    = g_nQNXTimerRes;   DANDY_ASSERT(g_nQNXTimerRes > 0);
    g_nControlParam.nIoScanTime     = g_nIoTime;       DANDY_ASSERT(g_nIoTime > 0);
    g_nControlParam.nTrajUpdateTime = g_nTrajUpdateTime; DANDY_ASSERT(g_nTrajUpdateTime > 0);
    g_nControlParam.nServoOnBrakeDelayTime = g_nServoOnBrakeDelayTime;
    g_nControlParam.nServoOffBrakeDelayTime = g_nServoOffBrakeDelayTime;
    g_nControlParam.nServoInterpolationTime = g_nServoInterpolationTime;
    DANDY_ASSERT(g_nServoInterpolationTime >= 0 && g_nServoInterpolationTime <= MAX_INTERPOLATION_TIME);

    pSystemConfig->ctrl = g_nControlParam;

    //CRT_strcpy(g_nECATParam.szPathName, sizeof(g_pszEcatConfigDir)+1, g_pszEcatConfigDir);
    CRT_strcpy(g_nECATParam.szPathName, CONFIG_PATH_NAME_LEN, g_pszEcatConfigDir);
    g_nECATParam.nSlaveCount      = g_nSlaveCount;      DANDY_ASSERT(g_nSlaveCount > 0);
    g_nECATParam.nWriteOffsetSize = g_nWriteOffsetSize; DANDY_ASSERT(g_nWriteOffsetSize >= 0);
    g_nECATParam.nReadOffsetSize  = g_nReadOffsetSize;  DANDY_ASSERT(g_nReadOffsetSize >= 0);

    pSystemConfig->ecat = g_nECATParam;

    ///////////////////////////////////
    // SYSIO

    //SYSC_LoadSysIoStyle(&pSystemConfig->sysio);

    ///////////////////////////////////
    // ROBOT

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        g_pRobot = pSystemConfig->robot + iRobot;

        g_pRobot->fUsed = g_rgfRobotUsed[iRobot];
        CRT_strcpy(g_pRobot->szRobotName, ROBOT_NAME_LEN, g_rgszRobotName[iRobot]);
        g_pRobot->nRobotType = g_rgnRobotType[iRobot];
        
        // maximum speed
        for (iAxis = 0; iAxis < g_rgnRobotAxisCount[iRobot]; iAxis++)
        {
            g_pRobot->dbMaxJointSpeed[iAxis]  =
                               g_rgRobotMotion[iRobot].dbMaxJointSpeed[iAxis] * 0.001;  //0.001: ms
        }
        g_pRobot->dbMaxLinearSpeed = g_rgRobotMotion[iRobot].dbMaxLinearSpeed * 0.001;  //0.001: ms
        g_pRobot->dbMaxOrientSpeed = g_rgRobotMotion[iRobot].dbMaxOrientSpeed * 0.001;  //0.001: ms
        
        // jerk
        g_pRobot->dbJerk = g_rgRobotMotion[iRobot].dbJerk;

        // accel/decel pattern determined by jerk(0: trapezoidal)
        g_pRobot->dbAccel = g_rgRobotMotion[iRobot].dbAccel;
        g_pRobot->dbDecel = g_rgRobotMotion[iRobot].dbDecel;
        g_pRobot->dbDecel_Error = g_rgRobotMotion[iRobot].dbDecel_Error;
        g_pRobot->dbDecel_Estop = g_rgRobotMotion[iRobot].dbDecel_Estop;
        g_pRobot->dbDecel_Touch = g_rgRobotMotion[iRobot].dbDecel_Touch;

        g_pAxis = g_pRobot->axis + 0;
        nAxesCount = 0;
        nAxisIndex = 0;

        g_pRobot->nAxesCount = g_rgnRobotAxisCount[iRobot];
        
        ///////////////////////////////////
        // Axis

        for (iAxis = 0; iAxis < g_rgnRobotAxisCount[iRobot]; iAxis++)
        {
            nAxesCount = g_pRobot->nAxesCount;
            DANDY_ASSERT(nAxesCount >= 0 && nAxesCount <= MAX_AXIS_COUNT);
            
            //g_pAxis = pSystemConfig->robot->axis + iAxis;
            g_pAxis = pSystemConfig->robot->axis;
            
            CRT_strcpy(g_pAxis[iAxis].szName, AXIS_NAME_LEN, g_rgszAxisName[iAxis]);
            g_pAxis[iAxis].nAxisType  = g_rgnAxisType[iAxis];
            g_pAxis[iAxis].nAxisIndex = g_rgnAxisIndex[iAxis];

            g_pAxis[iAxis].fHwLim_min = g_rgfHwLimitUsed[iAxis][0];
            g_pAxis[iAxis].fHwLim_max = g_rgfHwLimitUsed[iAxis][1];
            g_pAxis[iAxis].pos_hwlim_min = g_rgdbHwLimit[iAxis][0];
            g_pAxis[iAxis].pos_hwlim_max = g_rgdbHwLimit[iAxis][1];

            g_pAxis[iAxis].fSwLim_min = g_rgfSwLimitUsed[iAxis][0];
            g_pAxis[iAxis].fSwLim_max = g_rgfSwLimitUsed[iAxis][1];
            g_pAxis[iAxis].pos_swlim_min = g_rgdbSwLimit[iAxis][0];
            g_pAxis[iAxis].pos_swlim_max = g_rgdbSwLimit[iAxis][1];

            g_pAxis[iAxis].nMotorCount = g_nMotorCount[iAxis];

            ///////////////////////////////////
            // MOTOR
    
            //for (iMotor = iAxis; iMotor < (iAxis+g_nMotorCount[iAxis]); iMotor++)
            for (iMotor = 0; iMotor < g_nMotorCount[iAxis]; iMotor++)
            {
                nAxisIndex = nAxisIndex + iMotor;

                //g_pMotor = pSystemConfig->motor + iMotor;
                g_pMotor = pSystemConfig->motor;
                g_pRobot->axis[iAxis].nMotorIndex[iMotor] = nAxisIndex;
                
                CRT_strcpy(g_pMotor[nAxisIndex].szName, MOTOR_NAME_LEN, g_rgszMotorName[nAxisIndex]);
                g_pMotor[nAxisIndex].nMotorType  = g_rgnMotorType[nAxisIndex];
                g_pMotor[nAxisIndex].nMotorIndex = g_rgnMotorIndex[nAxisIndex];

                g_pMotor[nAxisIndex].jrk = g_rgMotorConfig[nAxisIndex].jrk;

                g_pMotor[nAxisIndex].acc = g_rgMotorConfig[nAxisIndex].acc;
                g_pMotor[nAxisIndex].dec = g_rgMotorConfig[nAxisIndex].dec;

                g_pMotor[nAxisIndex].dec_error = g_rgMotorConfig[nAxisIndex].dec_error;
                g_pMotor[nAxisIndex].dec_estop = g_rgMotorConfig[nAxisIndex].dec_estop;

                g_pMotor[nAxisIndex].vellim_max = g_rgMotorConfig[nAxisIndex].vellim_max;
                g_pMotor[nAxisIndex].nEncRes = g_rgMotorConfig[nAxisIndex].nEncRes;

                g_pAxis[iAxis].ori[iMotor] = g_rgnEcnoderHomeVal[nAxisIndex];
                g_pAxis[iAxis].red[iMotor] = g_rgdbGearRatio[nAxisIndex];
                g_pAxis[iAxis].dir[iMotor] = g_rgnAxisDirection[nAxisIndex];

                //pSystemConfig->robot->axis[iAxis].motor[iMotor] = &g_pMotor[nAxisIndex];
                pSystemConfig->motor[nAxisIndex] = g_pMotor[nAxisIndex];
            }

            nAxisIndex++;
        }   // end of axis
        
        // other
        for (iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
        {
            g_pRobot->dh[iAxis].l  = g_rgRobotDHParam[iRobot][iAxis].l;
            g_pRobot->dh[iAxis].al = g_rgRobotDHParam[iRobot][iAxis].al;
            g_pRobot->dh[iAxis].d  = g_rgRobotDHParam[iRobot][iAxis].d;
            g_pRobot->dh[iAxis].th = g_rgRobotDHParam[iRobot][iAxis].th;
        }

        // world coord references (Unit Convert: deg -> rad)
        g_pRobot->world.x       = g_rgCoordInfo[iRobot].world.x;
        g_pRobot->world.y       = g_rgCoordInfo[iRobot].world.y;
        g_pRobot->world.z       = g_rgCoordInfo[iRobot].world.z;
        g_pRobot->world.rol     = g_rgCoordInfo[iRobot].world.rol * (M_PI/180);
        g_pRobot->world.pit     = g_rgCoordInfo[iRobot].world.pit * (M_PI/180);
        g_pRobot->world.yaw     = g_rgCoordInfo[iRobot].world.yaw * (M_PI/180);

        // TCP references (Unit Convert: deg -> rad)
        g_pRobot->coordTool.x   = g_rgCoordInfo[iRobot].coordTool.x;
        g_pRobot->coordTool.y   = g_rgCoordInfo[iRobot].coordTool.y;
        g_pRobot->coordTool.z   = g_rgCoordInfo[iRobot].coordTool.z;
        g_pRobot->coordTool.rol = g_rgCoordInfo[iRobot].coordTool.rol * (M_PI/180);
        g_pRobot->coordTool.pit = g_rgCoordInfo[iRobot].coordTool.pit * (M_PI/180);
        g_pRobot->coordTool.yaw = g_rgCoordInfo[iRobot].coordTool.yaw * (M_PI/180);
        
        // Home Position references (Unit Convert: deg -> rad)
        for(iIndex = 0; iIndex < MAX_HOME_COUNT; iIndex++)
        {
            for (iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
            {
                g_pRobot->rgdbHomePosVal[iIndex][iAxis] = 
                                   g_rgdbHomePosVal[iIndex][iAxis] * (M_PI/180);
            }
        }

        // world coord references (Unit Convert: deg -> rad)
        for(iIndex = 0; iIndex < MAX_USER_COORD_COUNT; iIndex++)
        {
            g_pRobot->user[iIndex].x   = g_rgCoordInfo[iRobot].user[iIndex].x;
            g_pRobot->user[iIndex].y   = g_rgCoordInfo[iRobot].user[iIndex].y;
            g_pRobot->user[iIndex].z   = g_rgCoordInfo[iRobot].user[iIndex].z;
            g_pRobot->user[iIndex].rol = g_rgCoordInfo[iRobot].user[iIndex].rol * (M_PI/180);
            g_pRobot->user[iIndex].pit = g_rgCoordInfo[iRobot].user[iIndex].pit * (M_PI/180);
            g_pRobot->user[iIndex].yaw = g_rgCoordInfo[iRobot].user[iIndex].yaw * (M_PI/180);
        }

        // corresponding welders
        g_pRobot->nWelderCount = g_rgnRobotWelderCount[iRobot];

        for (iWelder = 0; iWelder < g_rgnRobotWelderCount[iRobot]; iWelder++)
        {
            g_pRobot->nWelderIndex[iWelder] = g_rgnRobotWelderList[iRobot][iWelder];
            pSystemConfig->welder[g_pRobot->nWelderIndex[iWelder]] = 
                                  g_rgWelderConfig[g_pRobot->nWelderIndex[iWelder]];
        }

        // corresponding welding functions
        g_pRobot->weld_func.fGapRefVarUsed = g_Weld_Function->fGapRefVarUsed;
        g_pRobot->weld_func.nGapRefBvar    = g_Weld_Function->nGapRefBvar;
        g_pRobot->weld_func.nLeftWeldBvar  = g_Weld_Function->nLeftWeldBvar;
        g_pRobot->weld_func.nRightWeldBvar = g_Weld_Function->nRightWeldBvar;

        g_pRobot->weld_func.dbCWeavTouchUpDis  = g_Weld_Function->dbCWeavTouchUpDis;
        g_pRobot->weld_func.dbCWeavHorMargin   = g_Weld_Function->dbCWeavHorMargin;
        g_pRobot->weld_func.dbCWeavWeldLegDis  = g_Weld_Function->dbCWeavWeldLegDis;

        // corresponding sensors
#if 0
        pRobot->nSensorCount = g_rgnRobotSensorCount[iRobot];

        for (iSensor = 0; iSensor < g_rgnRobotSensorCount[iRobot]; iSensor++)
        {
            pRobot->rgnSensors[iSensor] = g_rgnRobotSensorList[iRobot][iSensor];
        }
#endif
    }   // end of robot

    ///////////////////////////////////
    // WELDER
    memcpy(pSystemConfig->welder,
           g_rgWelderConfig,
           sizeof(CONFIG_WELDER) * MAX_WELDER_COUNT);
    
    //////////////////////////////////
    // Sensor
#if 0
    memcpy(pSystemConfig->sensors,
           g_rgSensorConfig,
           sizeof(CONFIG_SENSOR) * MAX_SENSOR_COUNT);
#endif
    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_BuildSysConfigShmem()
//

STATIC  int s_hShm_SysConfig = -1;

static int _loc_SHM_BuildSysConfigShmem(void)
{
    int hMem;
    //SHM_RM_SYSCONFIG* pMem;

    //DANDY_ASSERT(s_hShm_SysConfig == -1);
    //DANDY_ASSERT(g_pShm_SysConfig == NULL);

    ///////////////////////////////////
    //
    //  create the shared memory
    //

    hMem = SHM_Create(SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG));

    if (hMem == DANDY_INVALID_SHMEM_HANDLE)
    {
        VERBOSE_ERROR("Cannot create System Config shared memory\n");

        return RESULT_ERROR;
    }
    else
	{
		VERBOSE_VERBOSE("Create System Config shared memory Done!\n"); 
	}

    ///////////////////////////////////
    //
    //  map the shared memory handle
    //

    g_pShm_SysConfig = (SHM_RM_SYSCONFIG*) SHM_Map(hMem, sizeof(SHM_RM_SYSCONFIG));

    memset(g_pShm_SysConfig, 0, sizeof(SHM_RM_SYSCONFIG));

    if(g_pShm_SysConfig == NULL)
    {
        VERBOSE_ERROR("Cannot map System Config shared memory\n");
		return RESULT_ERROR; 
	}
	else
	{
        g_pShm_SysConfig->dwLength = sizeof(SHM_RM_SYSCONFIG);
        VERBOSE_VERBOSE("Mapped System Config shared memory. Size:%ld bytes\n",
                        g_pShm_SysConfig->dwLength); 
    }

    ///////////////////////////////////
    //
    //  Load Config Data to Shared Memory
    //

    if (SYSC_LoadConfigToShmem(g_pShm_SysConfig) == RESULT_ERROR)
    {
        memset(g_pShm_SysConfig, 0, sizeof(SHM_RM_SYSCONFIG));

        SHM_Unmap(g_pShm_SysConfig, sizeof(SHM_RM_SYSCONFIG));
        SHM_Destroy(hMem, SHM_RM_SYSCONFIG_NAME);

        return RESULT_ERROR;
    }

    s_hShm_SysConfig = hMem;
    //g_pShm_SysConfig = pMem;

    VERBOSE_VERBOSE("System Config Data Loading to shared memory was Done!\n");

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SHM_DestroySysConfigShmem()
//

static int _loc_SHM_DestroySysConfigShmem(void)
{
    int hMem;
    SHM_RM_SYSCONFIG* pMem;

    hMem = s_hShm_SysConfig;
    pMem = g_pShm_SysConfig;

    g_pShm_SysConfig = NULL;
    s_hShm_SysConfig = RESULT_ERROR;

    if (pMem != NULL)
    {
        DANDY_ASSERT(hMem != RESULT_ERROR);

        SHM_Unmap(pMem, sizeof(SHM_RM_SYSCONFIG));
        VERBOSE_VERBOSE("Unmapped Robot Config shared memory. Size:%dbytes\n",
                        sizeof(SHM_RM_SYSCONFIG)); 
        SHM_Destroy(hMem, SHM_RM_SYSCONFIG_NAME);
        VERBOSE_VERBOSE("Destroyed the shared memory. Name:%s\n",
                                                        SHM_RM_SYSCONFIG_NAME);
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SHM_CreateSharedMemory()
//

int SHM_CreateSharedMemory(void)
{
    // create the RM shared memory
    if (_loc_SHM_BuildSysStatusShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Status shared memory create fail!\n");
        return EXIT_FAILURE;
    }
    if (_loc_SHM_BuildSysConfigShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Config shared memory create fail!\n");
        return EXIT_FAILURE;
    }
#if 1
    if (_loc_SHM_BuildSysParamShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Parameter shared memory create fail!\n");
        return EXIT_FAILURE;
    }
#endif
    if (_loc_SHM_BuildJobShmem(ROBOT_0_INDEX) == RESULT_ERROR)
    {
        VERBOSE_ERROR("Job shared memory create fail!\n");
        return EXIT_FAILURE;
    }

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SHM_OpenSharedMemory()
//

int SHM_OpenSharedMemory(void)
{
    int nRet = RESULT_OK;

    // open the TE Restart shared memory
    nRet = _loc_SHM_OpenTERestartShmem();
    if(nRet == RESULT_ERROR)
    {
        g_retOpenShmemTE = RESULT_ERROR;
    }

    // open the TE ArcSensor Task shared memory
    nRet = _loc_SHM_OpenTETaskShmem();
    if(nRet == RESULT_ERROR)
    {
        g_retOpenShmemTE = RESULT_ERROR;
    }

    // open the TE Status shared memory
    nRet = _loc_SHM_OpenTEStatusShmem();

    if(nRet == RESULT_ERROR)
    {
        g_retOpenShmemTE = RESULT_ERROR;
    }

    // open the SC shared memory
    g_retOpenShmemSC = _loc_SHM_OpenSCShmem();

    return(g_retOpenShmemTE == RESULT_ERROR ||
           g_retOpenShmemSC == RESULT_ERROR) ?
        RESULT_ERROR : RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SHM_DestroyShmem()
//

int SHM_DestroyShmem(void)
{
    // destroy the System Status shared memory
    if (_loc_SHM_DestroySysStatusShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Status shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }
#if 1
    // destroy the System Parameter shared memory
    if (_loc_SHM_DestroySysParamShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Parameter shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }
#endif
    // destroy the System Config shared memory
    if (_loc_SHM_DestroySysConfigShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("System Config shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }

    // destroy the Job shared memory
    if (_loc_SHM_DestroyJobShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("Job shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }
    
    // destroy the TE Restart shared memory
    if (_loc_SHM_DestroyTERestartShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("TE Restart shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }

    // destroy the TE ArcSensor Task shared memory
    if (_loc_SHM_DestroyTETaskShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("TE ArcSensor Task shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }

    // destroy the TE Status shared memory
    if (_loc_SHM_DestroyTEStatusShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("TE shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }

    // destroy the SC shared memory
    if (_loc_SHM_DestroySCShmem() == RESULT_ERROR)
    {
        VERBOSE_ERROR("SC shared memory destroy fail!\n");
        return EXIT_FAILURE;
    }

    return RESULT_OK;
}
