/////////////////////////////////////////////////////////////////////////////
//
//  svc_job_mon.c: Job Monitoring Related Service
//                                            2014.03.20  Ryu SinWook

///////////////////////////////////////
#include "service.h"


///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int  g_fBinSndHeaderDefine = OFF;
int  g_fBinRcvHeaderDefine = OFF;
unsigned long g_dwTotalBinLoadSize = 0;
unsigned long g_dwTotalBinRcvSize = 0;
unsigned long g_dwTotalBinCompLoadSize = 0;
unsigned long g_dwTotalBinCompRcvSize = 0;

int           g_nRawBinLoadSize = 0;
int           g_nRawBinRcvSize = 0;

int           g_nSendBodyDivCount = 0;
int           g_nRcvBodyDivCount = 0;
int           g_dwCmdRcvCount = 0;
int           g_dwTVaRcvCount = 0;
int           g_dwWvfRcvCount = 0;
int           g_dwSwfRcvCount = 0;
int           g_dwMwfRcvCount = 0;
int           g_dwEwfRcvCount = 0;
int           g_nSndIndex = 0;
int           g_nRcvIndex = 0;

DANDY_JOB_MEM_SIZE      g_BulkMemSize;

DANDY_JOB_MEM*          g_pSndJobMem = NULL;
DANDY_JOB_MEM*          g_pRcvJobMem = NULL;
void*                   g_pCompressed;
int                     g_nSndBodyModulus;
int                     g_nRcvJobExecLineIdx = 0;

/////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////
//
//  Function: SVC_JobProgLineDisplay()
//      - Service Name: RSVC_SERV_JOBLINE_DSP

int SVC_JobProgLineDisplay(int nDisplayLineIndex)
{
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;
    DANDY_JOB_CMD* pCmdStart;
    char rgszCurrJobCmdOneLine[MAX_CMD_CHAR_SIZE] = "";
    char* pszRep;

    pCmdStart = GET_SHM_JOB_CMD_BUFFER(pShmem);

    g_nCmdLoadCount = pShmem->dwCmdLoadCount;

    if(nDisplayLineIndex > g_nCmdLoadCount)
    {
        VERBOSE_ERROR("Invalid Cmd Index Count(0 ~ %d, %d)!\n",
                          g_nCmdLoadCount, nDisplayLineIndex);
        return RESULT_ERROR;
    }

    JOBDIS_DisassemCmd(&pCmdStart[nDisplayLineIndex],
                       nDisplayLineIndex,
                       NULL,
                       rgszCurrJobCmdOneLine,
                       MAX_CMD_CHAR_SIZE,
                       JOBDIS_DF_NONE);
    
    /* Skip Tab Space */
    pszRep = rgszCurrJobCmdOneLine;
    while ((pszRep = strchr(pszRep, '\t')) != NULL)
    {
        *pszRep = ' ';
        pszRep++;   // find from next character
    }

    VERBOSE_VERBOSE("CmdLine: %d, Cmd: %s\n",
                    nDisplayLineIndex,
                    rgszCurrJobCmdOneLine);

    RM_reply_packet.Data.job_dsp.nCmdLoadCount = pShmem->dwCmdLoadCount;
    memcpy(RM_reply_packet.Data.job_dsp.szCommand,
           rgszCurrJobCmdOneLine,
           MAX_CMD_CHAR_SIZE);

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.job_dsp);
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_EditConstJobVar()
//      - Service Name: RSVC_JOB_CONSTVAREDIT

int SVC_EditConstJobVar(int nVarType)
{
    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    int             nVarIndex;
    int             iAxis;

    /* P Var */
    DANDY_JOB_POS*  pPVarStart;
    int             nPCoordIdx;
    double          dbPVarVal[ROB_AXIS_COUNT];
    
    /* B Var */
    BYTE*           pBVarStart;
    BYTE            byBVarVal;

    /* I Var */
    INT *           pIVarStart;
    INT             nIVarVal;

    /* R Var */
    double*         pRVarStart;
    double          dbRVarVal;
#if 0
    if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB)
    {
        VERBOSE_ERROR("No Job Loaded!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOB_EDIT);
        return SVC_ERR_JOB_EDIT;
    }
#endif
    pPVarStart = GET_SHM_JOB_PVA_BUFFER(pShmem);
    pBVarStart = GET_SHM_JOB_BVA_BUFFER(pShmem);
    pIVarStart = GET_SHM_JOB_IVA_BUFFER(pShmem);
    pRVarStart = GET_SHM_JOB_RVA_BUFFER(pShmem);
    
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    if(nVarType == DANDY_JOB_VAL_TYPE_B_VAR)
    {
        nVarIndex = RM_packet.Data.const_var_val.nVarIndex;         // Request Idx
        byBVarVal = RM_packet.Data.const_var_val.value.ByteVal;
        VERBOSE_VERBOSE("\v[BVar] Index: %d, Val: %d -> %d\n",
                        nVarIndex, pBVarStart[nVarIndex], (int) byBVarVal);
        pBVarStart[nVarIndex] = byBVarVal;

        RM_reply_packet.Data.const_var_val.value.ByteVal = pBVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_I_VAR)
    {
        nVarIndex = RM_packet.Data.const_var_val.nVarIndex;         // Request Idx
        nIVarVal = RM_packet.Data.const_var_val.value.IntVal;
        VERBOSE_VERBOSE("\v[IVar] Index: %d, Val: %d -> %d\n",
                        nVarIndex, pIVarStart[nVarIndex], nIVarVal);
        pIVarStart[nVarIndex] = nIVarVal;

        RM_reply_packet.Data.const_var_val.value.IntVal = pIVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_R_VAR)
    {
        nVarIndex = RM_packet.Data.const_var_val.nVarIndex;         // Request Idx
        dbRVarVal = RM_packet.Data.const_var_val.value.RealVal;
        VERBOSE_VERBOSE("\v[RVar] Index: %d, Val: %.2lf -> %.2lf\n",
                        nVarIndex, pRVarStart[nVarIndex], dbRVarVal);
        pRVarStart[nVarIndex] = dbRVarVal;

        RM_reply_packet.Data.const_var_val.value.RealVal = pRVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_P_VAR)
    {
        nVarIndex = RM_packet.Data.pvar_val.nVarIndex;         // Request Idx
        nPCoordIdx = RM_packet.Data.pvar_val.nCoordIdx;
        
        VERBOSE_VERBOSE("\v[PVar] Index: %d, Coord: %d, Val: "
                        "%.2lf  %.2lf  %.2lf  %.2lf  %.2lf  %.2lf\n",
                        nVarIndex, pPVarStart[nVarIndex].nConfig,
                        pPVarStart[nVarIndex].pos[0], pPVarStart[nVarIndex].pos[1],
                        pPVarStart[nVarIndex].pos[2], pPVarStart[nVarIndex].pos[3],
                        pPVarStart[nVarIndex].pos[4], pPVarStart[nVarIndex].pos[5]);
        
        pPVarStart[nVarIndex].nConfig = nPCoordIdx;

        for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
        {
            dbPVarVal[iAxis] = RM_packet.Data.pvar_val.dbPosVal[iAxis];
            pPVarStart[nVarIndex].pos[iAxis] = dbPVarVal[iAxis];
            RM_reply_packet.Data.pvar_val.dbPosVal[iAxis] = dbPVarVal[iAxis];
        }
        RM_reply_packet.Data.pvar_val.nVarIndex = nVarIndex;   // Confirmed Idx

        VERBOSE_VERBOSE("\v-> Coord: %d, Val: "
                        "%.2lf  %.2lf  %.2lf  %.2lf  %.2lf  %.2lf\n",
                        nPCoordIdx,
                        dbPVarVal[0], dbPVarVal[1], dbPVarVal[2],
                        dbPVarVal[3], dbPVarVal[4], dbPVarVal[5]);
        
        RM_reply_packet.Data.pvar_val.nCoordIdx = nPCoordIdx;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.pvar_val);
    }
    else
    {
        VERBOSE_ERROR("Invalid Variable Type(%d)!\n", nVarType);
        return RESULT_ERROR;
    }
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SendConstJobVarInfo()
//      - Service Name: RSVC_JOB_CONSTVARINFO

int SVC_SendConstJobVarInfo(int nVarType)
{
    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    int             nVarIndex;
    int             iAxis;

    /* P Var */
    DANDY_JOB_POS*  pPVarStart;
    int             nPCoordIdx;
    double          dbPVarVal[ROB_AXIS_COUNT];
    
    /* B Var */
    BYTE*           pBVarStart;
    BYTE            byBVarVal;

    /* I Var */
    INT *           pIVarStart;
    INT             nIVarVal;

    /* R Var */
    double*         pRVarStart;
    double          dbRVarVal;

    /* T Var */
    DANDY_JOB_POS*  pTVarStart;
    double          dbTVarVal[ROB_AXIS_COUNT];
    int             nTCoordIdx;
#if 0
    if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB)
    {
        VERBOSE_ERROR("No Job Loaded!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOB_MON);
        return SVC_ERR_JOB_MON;
    }
#endif
    pPVarStart = GET_SHM_JOB_PVA_BUFFER(pShmem);
    pBVarStart = GET_SHM_JOB_BVA_BUFFER(pShmem);
    pIVarStart = GET_SHM_JOB_IVA_BUFFER(pShmem);
    pRVarStart = GET_SHM_JOB_RVA_BUFFER(pShmem);
    pTVarStart = GET_SHM_JOB_TVA_BUFFER(pShmem);
    
    nVarIndex = RM_packet.Data.var_mon_req.nVarIndex;   // Request Idx
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    if(nVarType == DANDY_JOB_VAL_TYPE_B_VAR)
    {
        byBVarVal = pBVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.value.ByteVal = byBVarVal;
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        VERBOSE_VERBOSE("\v[BVar] Index: %d, Val: %d\n", nVarIndex, (int) byBVarVal);
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_I_VAR)
    {
        nIVarVal = pIVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.value.IntVal = nIVarVal;
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        VERBOSE_VERBOSE("\v[IVar] Index: %d, Val: %d\n", nVarIndex, nIVarVal);
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_R_VAR)
    {
        dbRVarVal = pRVarStart[nVarIndex];
        RM_reply_packet.Data.const_var_val.value.RealVal = dbRVarVal;
        RM_reply_packet.Data.const_var_val.nVarIndex = nVarIndex;   // Confirmed Idx
        
        VERBOSE_VERBOSE("\v[RVar] Index: %d, Val: %.2lf\n", nVarIndex, dbRVarVal);
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.const_var_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_P_VAR)
    {
        nPCoordIdx = pPVarStart[nVarIndex].nConfig;
        RM_reply_packet.Data.pvar_val.nCoordIdx = nPCoordIdx;

        for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
        {
            dbPVarVal[iAxis] = pPVarStart[nVarIndex].pos[iAxis];
            RM_reply_packet.Data.pvar_val.dbPosVal[iAxis] = dbPVarVal[iAxis];
        }
        RM_reply_packet.Data.pvar_val.nVarIndex = nVarIndex;   // Confirmed Idx

        VERBOSE_VERBOSE("\v[PVar] Index: %d, Coord: %d, Val: "
                        "%.2lf  %.2lf  %.2lf  %.2lf  %.2lf  %.2lf\n",
                        nVarIndex, nPCoordIdx,
                        dbPVarVal[0], dbPVarVal[1], dbPVarVal[2],
                        dbPVarVal[3], dbPVarVal[4], dbPVarVal[5]);
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.pvar_val);
    }
    else if(nVarType == DANDY_JOB_VAL_TYPE_T_VAR)
    {
        if(nVarIndex < 0 || nVarIndex > (int) pShmem->dwTVaLoadCount)
        {
            VERBOSE_ERROR("Invalid TVar Index Count(0 ~ %ld, %d)!\n",
                          pShmem->dwTVaLoadCount, nVarIndex);
            return RESULT_ERROR;
        }

        nTCoordIdx = pTVarStart[nVarIndex].nConfig;
        RM_reply_packet.Data.tvar_val.nCoordIdx = nTCoordIdx;

        for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
        {
            dbTVarVal[iAxis] = pTVarStart[nVarIndex].pos[iAxis];
            RM_reply_packet.Data.tvar_val.dbPosVal[iAxis] = dbTVarVal[iAxis];
        }
        RM_reply_packet.Data.tvar_val.nVarIndex = nVarIndex;   // Confirmed Idx

        VERBOSE_VERBOSE("\v[TVar] Index: %d, Coord: 0x%x, Val: "
                        "%.2lf  %.2lf  %.2lf  %.2lf  %.2lf  %.2lf\n",
                        nVarIndex, nTCoordIdx,
                        dbTVarVal[0], dbTVarVal[1], dbTVarVal[2],
                        dbTVarVal[3], dbTVarVal[4], dbTVarVal[5]);
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.tvar_val);
    }
    else
    {
        VERBOSE_ERROR("Invalid Variable Type(%d)!\n", nVarType);
        return RESULT_ERROR;
    }
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_InitConstJobVar()
//      - Service Name: RSVC_SERV_CONSTVAR_INIT
#define     INIT_REQ_PVAR       0
#define     INIT_REQ_BVAR       1
#define     INIT_REQ_IVAR       2
#define     INIT_REQ_RVAR       3

int SVC_InitConstJobVar(int nVarType)
{
    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    int             iIdx;
    int             iAxis;

    /* P Var */
    DANDY_JOB_POS*  pPVarStart;
    
    /* B Var */
    BYTE*           pBVarStart;

    /* I Var */
    INT *           pIVarStart;

    /* R Var */
    double*         pRVarStart;

    pPVarStart = GET_SHM_JOB_PVA_BUFFER(pShmem);
    pBVarStart = GET_SHM_JOB_BVA_BUFFER(pShmem);
    pIVarStart = GET_SHM_JOB_IVA_BUFFER(pShmem);
    pRVarStart = GET_SHM_JOB_RVA_BUFFER(pShmem);
    
    if(nVarType == INIT_REQ_PVAR)
    {
        for(iIdx = 0; iIdx < (int) pShmem->dwPVaSize; iIdx++)
        {
            pPVarStart[iIdx].nConfig = 0;
            
            for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
            {
                pPVarStart[iIdx].pos[iAxis] = 0;
            }
        }

        VERBOSE_VERBOSE("\v[PVar] Index: %d ~ %ld Initialize Done!\n",
                        0, pShmem->dwPVaSize);
    }
    else if(nVarType == INIT_REQ_BVAR)
    {
        for(iIdx = 0; iIdx < (int) pShmem->dwBVaSize; iIdx++)
        {
            pBVarStart[iIdx] = 0;
        }

        VERBOSE_VERBOSE("\v[BVar] Index: %d ~ %ld Initialize Done!\n",
                        0, pShmem->dwBVaSize);
    }
    else if(nVarType == INIT_REQ_IVAR)
    {
        for(iIdx = 0; iIdx < (int) pShmem->dwIVaSize; iIdx++)
        {
            pIVarStart[iIdx] = 0;
        }

        VERBOSE_VERBOSE("\v[IVar] Index: %d ~ %ld Initialize Done!\n",
                        0, pShmem->dwIVaSize);
    }
    else if(nVarType == INIT_REQ_RVAR)
    {
        for(iIdx = 0; iIdx < (int) pShmem->dwRVaSize; iIdx++)
        {
            pRVarStart[iIdx] = 0;
        }

        VERBOSE_VERBOSE("\v[RVar] Index: %d ~ %ld Initialize Done!\n",
                        0, pShmem->dwRVaSize);
    }
    else
    {
        VERBOSE_ERROR("Invalid Variable Type(%d)!\n", nVarType);
        return RESULT_ERROR;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
        
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SendWeldJobVarInfo()
//      - Service Name: RSVC_JOB_WELDVARINFO

int SVC_SendWeldJobVarInfo(int nVarType)
{
    SHM_DANDY_JOB*   pShmem = g_pShmemJobRM;
    int              nVarIndex;
    int              iIdx;

    /* Wvf Var */
    DANDY_JOB_WEAV*  pWvfVarStart;
    double           dbWvfVarVal[DANDY_JOB_WVF_ELEMENT_COUNT];
    
    /* Swf Var */
    DANDY_JOB_SWF*   pSwfVarStart;
    double           dbSwfVarVal[DANDY_JOB_SWF_ELEMENT_COUNT];

    /* Mwf Var */
    DANDY_JOB_MWF*   pMwfVarStart;
    double           dbMwfVarVal[DANDY_JOB_MWF_ELEMENT_COUNT];

    /* Ewf Var */
    DANDY_JOB_EWF*   pEwfVarStart;
    double           dbEwfVarVal[DANDY_JOB_EWF_ELEMENT_COUNT];

#if 0
    if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB)
    {
        VERBOSE_ERROR("No Job Loaded!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOB_MON);
        return SVC_ERR_JOB_MON;
    }
#endif

    // get shared memory buffer
    pWvfVarStart  = GET_SHM_JOB_WVF_BUFFER(pShmem);
    pSwfVarStart  = GET_SHM_JOB_SWF_BUFFER(pShmem);
    pMwfVarStart  = GET_SHM_JOB_MWF_BUFFER(pShmem);
    pEwfVarStart  = GET_SHM_JOB_EWF_BUFFER(pShmem);
    
    nVarIndex = RM_packet.Data.var_mon_req.nVarIndex;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    // weaving condition
    if(nVarType == DANDY_JOB_VAL_TYPE_WVF_VAR)
    {
        VERBOSE_VERBOSE("\v[Wfv Var] Index: %d\n", nVarIndex);
        
        if(nVarIndex < 0 || nVarIndex > (int) pShmem->dwWvfLoadCount)
        {
            VERBOSE_ERROR("Invalid Wvf Index Count(0 ~ %ld, %d)!\n",
                          pShmem->dwWvfLoadCount, nVarIndex);
            
            // reply zero value
            for(iIdx = 0; iIdx < DANDY_JOB_WVF_ELEMENT_COUNT; iIdx++)
            {
                dbWvfVarVal[iIdx] = 0;
                RM_reply_packet.Data.wvf_val.rgdbWeavCond[iIdx] = dbWvfVarVal[iIdx];
            }

            return RESULT_ERROR;
        }

        for(iIdx = 0; iIdx < DANDY_JOB_WVF_ELEMENT_COUNT; iIdx++)
        {
            dbWvfVarVal[iIdx] = pWvfVarStart[nVarIndex].rgdbWeavCond[iIdx];
            RM_reply_packet.Data.wvf_val.rgdbWeavCond[iIdx] = dbWvfVarVal[iIdx];
            VERBOSE_VERBOSE("\v%.2lf\t", dbWvfVarVal[iIdx]);
        }
        VERBOSE_VERBOSE("\v\n");
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.wvf_val);
    }
    // welding start condition
    else if(nVarType == DANDY_JOB_VAL_TYPE_SWF_VAR)
    {
        VERBOSE_VERBOSE("\v[Swf Var] Index: %d\n", nVarIndex);
        
        if(nVarIndex < 0 || nVarIndex > (int) pShmem->dwSwfLoadCount)
        {
            VERBOSE_ERROR("Invalid Swf Index Count(0 ~ %ld: %d)!\n",
                          pShmem->dwSwfLoadCount, nVarIndex);

            // reply zero value
            for(iIdx = 0; iIdx < DANDY_JOB_SWF_ELEMENT_COUNT; iIdx++)
            {
                dbSwfVarVal[iIdx] = 0;
                RM_reply_packet.Data.swf_val.swf.rgdbWeldCond[iIdx] = dbSwfVarVal[iIdx];
            }

            return RESULT_ERROR;
        }

        for(iIdx = 0; iIdx < DANDY_JOB_SWF_ELEMENT_COUNT; iIdx++)
        {
            dbSwfVarVal[iIdx] = pSwfVarStart[nVarIndex].swf.rgdbWeldCond[iIdx];
            RM_reply_packet.Data.swf_val.swf.rgdbWeldCond[iIdx] = dbSwfVarVal[iIdx];
            VERBOSE_VERBOSE("\v%.2lf\t", dbSwfVarVal[iIdx]);
        }
        VERBOSE_VERBOSE("\v\n");
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.swf_val);
    }
    // welding main condition
    else if(nVarType == DANDY_JOB_VAL_TYPE_MWF_VAR)
    {
        VERBOSE_VERBOSE("\v[Mwf Var] Index: %d\n", nVarIndex);

        if(nVarIndex < 0 || nVarIndex > (int) pShmem->dwMwfLoadCount)
        {
            VERBOSE_ERROR("Invalid Mwf Index Count(0 ~ %ld: %d)!\n",
                          pShmem->dwMwfLoadCount, nVarIndex);

            // reply zero value
            for(iIdx = 0; iIdx < DANDY_JOB_MWF_ELEMENT_COUNT; iIdx++)
            {
                dbMwfVarVal[iIdx] = 0;
                RM_reply_packet.Data.mwf_val.mwf.rgdbWeldCond[iIdx] = dbMwfVarVal[iIdx];
            }

            return RESULT_ERROR;
        }

        for(iIdx = 0; iIdx < DANDY_JOB_MWF_ELEMENT_COUNT; iIdx++)
        {
            dbMwfVarVal[iIdx] = pMwfVarStart[nVarIndex].mwf.rgdbWeldCond[iIdx];
            RM_reply_packet.Data.mwf_val.mwf.rgdbWeldCond[iIdx] = dbMwfVarVal[iIdx];
            VERBOSE_VERBOSE("\v%.2lf\t", dbMwfVarVal[iIdx]);
        }
        VERBOSE_VERBOSE("\v\n");
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.mwf_val);
    }
    // welding end condition 
    else if(nVarType == DANDY_JOB_VAL_TYPE_EWF_VAR)
    {
        VERBOSE_VERBOSE("\v[Ewf Var] Index: %d\n", nVarIndex);

        if(nVarIndex < 0 || nVarIndex > (int) pShmem->dwEwfLoadCount)
        {
            VERBOSE_ERROR("Invalid Ewf Index Count(0 ~ %ld: %d)!\n",
                          pShmem->dwEwfLoadCount, nVarIndex);

            // reply zero value
            for(iIdx = 0; iIdx < DANDY_JOB_EWF_ELEMENT_COUNT; iIdx++)
            {
                dbEwfVarVal[iIdx] = 0;
                RM_reply_packet.Data.ewf_val.ewf.rgdbWeldCond[iIdx] = dbEwfVarVal[iIdx];
            }

            return RESULT_ERROR;
        }

        for(iIdx = 0; iIdx < DANDY_JOB_EWF_ELEMENT_COUNT; iIdx++)
        {
            dbEwfVarVal[iIdx] = pEwfVarStart[nVarIndex].ewf.rgdbWeldCond[iIdx];
            RM_reply_packet.Data.ewf_val.ewf.rgdbWeldCond[iIdx] = dbEwfVarVal[iIdx];
            VERBOSE_VERBOSE("\v%.2lf\t", dbEwfVarVal[iIdx]);
        }
        VERBOSE_VERBOSE("\v\n");
        
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.ewf_val);
    }
    else
    {
        VERBOSE_ERROR("Invalid Variable Type(%d)!\n", nVarType);
        return RESULT_ERROR;
    }
    
    return RESULT_OK;
}




/////////////////////////////////////////////////////////////////////////////
//
//  _loc_JOBMEM_Alloc()
//

DANDY_JOB_MEM* _loc_JOBMEM_Alloc(const DANDY_JOB_MEM_SIZE* pSize)
{
    DANDY_JOB_MEM*  pMem;
    int             nTotalSize;

    ///////////////////////////////////
    //
    //  calc size
    //
    nTotalSize = JOBMEM_GetTotalSize(pSize);
    

    ///////////////////////////////////
    //
    //  alloc memory
    pMem = (DANDY_JOB_MEM*) DEBUG_MALLOC(nTotalSize);

    if (pMem == NULL)
    {
        return NULL;
    }

    return pMem;
}

///////////////////////////////////////
//
//  Function: SVC_JobBinSendHeader()
//      - Service Name: RSVC_JOB_BIN_SNDHEADER

int SVC_JobBinSendHeader(int nOpt)
{
    DANDY_JOB_MEM       loadedJobShm;
    SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;
    int                 nTotalSize = 0;

#if 0
    if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB)
    {
        VERBOSE_ERROR("No Job Loaded!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
#endif

    ///////////////////////////////////
    //
    //  convert SHM to DANDY_JOB_MEM format
    //

    loadedJobShm.size.nCmd = pShmem->dwCmdLoadCount;
    loadedJobShm.size.nPos = pShmem->dwTVaLoadCount;
    loadedJobShm.size.nWvf = pShmem->dwWvfLoadCount;
    loadedJobShm.size.nSwf = pShmem->dwSwfLoadCount;
    loadedJobShm.size.nMwf = pShmem->dwMwfLoadCount;
    loadedJobShm.size.nEwf = pShmem->dwEwfLoadCount;

    loadedJobShm.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
    loadedJobShm.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
    loadedJobShm.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
    loadedJobShm.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
    loadedJobShm.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
    loadedJobShm.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);


    ///////////////////////////////////
    //
    //  create bulk memory
    //

    // create the job memory
    g_pSndJobMem = JOBMEM_Create(&loadedJobShm.size);

    if (g_pSndJobMem == NULL)
    {
        VERBOSE_ERROR("cannot create the job memory for Send\n");
        return RESULT_ERROR;
    }
    else
    {
        VERBOSE_MESSAGE("Create the job memory for Send Done!\n");
    }
    
    ///////////////////////////////////
    //
    //  copy SHM to bulk memory
    //
    if(JOBMEM_Copy(&loadedJobShm, g_pSndJobMem) == FALSE)
    {
        VERBOSE_ERROR("cannot copy shared memory job to the bulk memory...\n");

        return RESULT_ERROR;
    }
    nTotalSize = JOBMEM_GetTotalSize(&loadedJobShm.size) - sizeof(DANDY_JOB_MEM);
    
    ///////////////////////////////////
    //
    // Check Total Size & Divided Count
    //

    g_nSndBodyModulus = nTotalSize % RMGR_REPLY_PACKET_DATA_LEN;
    g_nSendBodyDivCount = nTotalSize / RMGR_REPLY_PACKET_DATA_LEN;
    if(g_nSndBodyModulus != 0)
    {
        g_nSendBodyDivCount = g_nSendBodyDivCount + 1;
    }
    g_dwTotalBinLoadSize = nTotalSize;

    VERBOSE_MESSAGE("Copy the job memory Done!(Size: %ld, Count: %d, PacketSize: %d)\n",
                    g_dwTotalBinLoadSize, g_nSendBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);

    ///////////////////////////////////
    //
    // Define Reply Msg
    //

    RM_reply_packet.Data.snd_header.nBodyDivIndex = g_nSendBodyDivCount;
    RM_reply_packet.Data.snd_header.dwRawBinLoadSize = g_dwTotalBinLoadSize;

    RM_reply_packet.Data.snd_header.dwCmdLoadCount = pShmem->dwCmdLoadCount;
    RM_reply_packet.Data.snd_header.dwTVaLoadCount = pShmem->dwTVaLoadCount;
    RM_reply_packet.Data.snd_header.dwWvfLoadCount = pShmem->dwWvfLoadCount;
    RM_reply_packet.Data.snd_header.dwSwfLoadCount = pShmem->dwSwfLoadCount;
    RM_reply_packet.Data.snd_header.dwMwfLoadCount = pShmem->dwMwfLoadCount;
    RM_reply_packet.Data.snd_header.dwEwfLoadCount = pShmem->dwEwfLoadCount;
    
    RM_reply_packet.Data.snd_header.nJobExecLineIdx = g_nJobExecLineIdx;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.snd_header);


    ///////////////////////////////////
    //
    //  if option is activated, free bulk job memory
    //

    if(nOpt == 1)
    {
        JOBMEM_Delete(g_pSndJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");
    }

    g_fBinSndHeaderDefine = ON;
    g_nSndIndex = 0;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobBinSendData()
//      - Service Name: RSVC_JOB_BIN_SNDBODY

int SVC_JobBinSendData(int nIndex)
{
    ///////////////////////////////////
    //
    // Check Header Data Sended or Not!
    //

    if(g_fBinSndHeaderDefine == OFF)
    {
        VERBOSE_ERROR("Binary Send Header Not Defined!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex > g_nSendBodyDivCount)
    {
        VERBOSE_ERROR("Request Index is bigger than Send Body Div Index!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex != g_nSndIndex)
    {
        VERBOSE_ERROR("Request Index is Not Continuous Value(Ideal Value: %d)!\n",
                      g_nSndIndex);
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }

    g_nSndIndex = nIndex + 1;

    ///////////////////////////////////
    //
    //  copy SHM to bulk memory
    //

    if(nIndex != (g_nSendBodyDivCount - 1))
    {
        memcpy(RM_reply_packet.Data.data,
               g_pSndJobMem->data + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
               RMGR_REPLY_PACKET_DATA_LEN);
    }
    else if(nIndex == (g_nSendBodyDivCount - 1))
    {
        memcpy(RM_reply_packet.Data.data,
               g_pSndJobMem->data + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
               g_nSndBodyModulus);
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    if(nIndex != (g_nSendBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.data);
        VERBOSE_MESSAGE("[Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }
    else if(nIndex == (g_nSendBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = g_nSndBodyModulus;
        VERBOSE_MESSAGE("[End of Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }

    ///////////////////////////////////
    //
    //  if last index, free bulk job memory
    //

    if(nIndex == (g_nSendBodyDivCount - 1))
    {
        JOBMEM_Delete(g_pSndJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");

        g_fBinSndHeaderDefine = OFF;
        g_nSendBodyDivCount = 0;
        g_dwTotalBinLoadSize = 0;
        g_nSndIndex = 0;
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobBinReceiveHeader()
//      - Service Name: RSVC_JOB_BIN_RCVHEADER

int SVC_JobBinReceiveHeader(int nOpt)
{
    DANDY_JOB_MEM_SIZE BulkMemSize;
    //int nTotalSize;

    ///////////////////////////////////
    //
    // Get Size to Global Variables
    //

    g_nRcvBodyDivCount = RM_packet.Data.rcv_header.nBodyDivIndex;
    g_dwTotalBinRcvSize = RM_packet.Data.rcv_header.dwRawBinLoadSize;

    g_dwCmdRcvCount = RM_packet.Data.rcv_header.dwCmdLoadCount;
    g_dwTVaRcvCount = RM_packet.Data.rcv_header.dwTVaLoadCount;
    g_dwWvfRcvCount = RM_packet.Data.rcv_header.dwWvfLoadCount;
    g_dwSwfRcvCount = RM_packet.Data.rcv_header.dwSwfLoadCount;
    g_dwMwfRcvCount = RM_packet.Data.rcv_header.dwMwfLoadCount;
    g_dwEwfRcvCount = RM_packet.Data.rcv_header.dwEwfLoadCount;

    g_nRcvJobExecLineIdx = RM_packet.Data.rcv_header.nJobExecLineIdx;

    VERBOSE_MESSAGE("Receive the job memory Header Done!(Size: %ld, Count: %d, PacketSize: %d)\n",
                    g_dwTotalBinRcvSize, g_nRcvBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);

    BulkMemSize.nCmd = g_dwCmdRcvCount;
    BulkMemSize.nPos = g_dwTVaRcvCount;
    BulkMemSize.nWvf = g_dwWvfRcvCount;
    BulkMemSize.nSwf = g_dwSwfRcvCount;
    BulkMemSize.nMwf = g_dwMwfRcvCount;
    BulkMemSize.nEwf = g_dwEwfRcvCount;

    ///////////////////////////////////
    //
    //  create bulk memory
    //

    // create the job memory
    g_pRcvJobMem = JOBMEM_Create(&BulkMemSize);
    
    if (g_pRcvJobMem == NULL)
    {
        VERBOSE_ERROR("cannot create the job memory for Receive\n");
        return RESULT_ERROR;
    }
    else
    {
        VERBOSE_MESSAGE("Create the job memory for Receive Done!\n");
        VERBOSE_MESSAGE("(Cmd: %d, Pos: %d, Wvf: %d, Swf: %d, Mwf: %d, Ewf: %d)\n",
                        g_pRcvJobMem->size.nCmd, g_pRcvJobMem->size.nPos,
                        g_pRcvJobMem->size.nWvf, g_pRcvJobMem->size.nSwf,
                        g_pRcvJobMem->size.nMwf, g_pRcvJobMem->size.nEwf);
    }
    
    ///////////////////////////////////
    //
    // Define Reply Msg
    //

    RM_reply_packet.Data.rcv_header.nBodyDivIndex = g_nRcvBodyDivCount;
    RM_reply_packet.Data.rcv_header.dwRawBinLoadSize = g_dwTotalBinRcvSize;

    RM_reply_packet.Data.rcv_header.dwCmdLoadCount = g_dwCmdRcvCount;
    RM_reply_packet.Data.rcv_header.dwTVaLoadCount = g_dwTVaRcvCount;
    RM_reply_packet.Data.rcv_header.dwWvfLoadCount = g_dwWvfRcvCount;
    RM_reply_packet.Data.rcv_header.dwSwfLoadCount = g_dwSwfRcvCount;
    RM_reply_packet.Data.rcv_header.dwMwfLoadCount = g_dwMwfRcvCount;
    RM_reply_packet.Data.rcv_header.dwEwfLoadCount = g_dwEwfRcvCount;
    
    RM_reply_packet.Data.rcv_header.nJobExecLineIdx = g_nRcvJobExecLineIdx;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.rcv_header);

    ///////////////////////////////////
    //
    //  if option is activated, free bulk job memory
    //

    if(nOpt == 1)
    {
        JOBMEM_Delete(g_pRcvJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");
    }

    g_fBinRcvHeaderDefine = ON;
    g_nRcvIndex = 0;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobBinReceiveData()
//      - Service Name: RSVC_JOB_BIN_RCVBODY

int SVC_JobBinReceiveData(int nIndex)
{
    DANDY_JOB_MEM       editedJobShm;
    SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;
    int                 nTotalSize;

    ///////////////////////////////////
    //
    // Check Header Data Received or Not!
    //

    if(g_fBinRcvHeaderDefine == OFF)
    {
        VERBOSE_ERROR("Binary Receive Header Not Defined!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex > g_nRcvBodyDivCount)
    {
        VERBOSE_ERROR("Request Index is bigger than Receive Body Div Index!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex != g_nRcvIndex)
    {
        VERBOSE_ERROR("Request Index is Not Continuous Value(Ideal Value: %d)!\n",
                      g_nRcvIndex);
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }

    g_nRcvIndex = nIndex + 1;

    ///////////////////////////////////
    //
    //  copy received data to bulk memory
    //

    memcpy(g_pRcvJobMem->data + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
           RM_packet.Data.data,
           RM_packet.nDataSize);

    if(nIndex != (g_nRcvBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.data);
        VERBOSE_MESSAGE("[Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }
    else if(nIndex == (g_nRcvBodyDivCount - 1))
    {
        VERBOSE_MESSAGE("[End of Body]Packet Data\n");
    }

    ///////////////////////////////////
    //
    //  if last index, copy & free bulk job memory
    //

    if(nIndex == (g_nRcvBodyDivCount - 1))
    {
        ///////////////////////////////////
        //
        //  copy bulk memory to SHM
        //
        editedJobShm.size.nCmd = pShmem->dwCmdSize;
        editedJobShm.size.nPos = pShmem->dwTVaSize;
        editedJobShm.size.nWvf = pShmem->dwWvfSize;
        editedJobShm.size.nSwf = pShmem->dwSwfSize;
        editedJobShm.size.nMwf = pShmem->dwMwfSize;
        editedJobShm.size.nEwf = pShmem->dwEwfSize;

        editedJobShm.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
        editedJobShm.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
        editedJobShm.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
        editedJobShm.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
        editedJobShm.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
        editedJobShm.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);

        if(JOBMEM_Copy(g_pRcvJobMem, &editedJobShm) == FALSE)
        {
            VERBOSE_ERROR("cannot copy received memory job to the shared memory...\n");

            g_fBinRcvHeaderDefine = OFF;
            g_nRcvBodyDivCount = 0;
            g_dwTotalBinRcvSize = 0;
            g_nRcvIndex = 0;

            return RESULT_ERROR;
        }

        nTotalSize = JOBMEM_GetTotalSize(&g_pRcvJobMem->size) - sizeof(DANDY_JOB_MEM);

        pShmem->dwCmdLoadCount = g_pRcvJobMem->size.nCmd;
        pShmem->dwTVaLoadCount = g_pRcvJobMem->size.nPos;
        pShmem->dwWvfLoadCount = g_pRcvJobMem->size.nWvf;
        pShmem->dwSwfLoadCount = g_pRcvJobMem->size.nSwf;
        pShmem->dwMwfLoadCount = g_pRcvJobMem->size.nMwf;
        pShmem->dwEwfLoadCount = g_pRcvJobMem->size.nEwf;

        if(nTotalSize != g_dwTotalBinRcvSize)
        {
            VERBOSE_ERROR("Total Size Wrong! (MemSize: %d, RcvSize: %ld)\n",
                          nTotalSize, g_dwTotalBinRcvSize);
        }
        else
        {
            VERBOSE_MESSAGE("Copy the job memory Done!\n"
                            "(MemSize: %d, RcvSize: %ld, Count: %d, PacketSize: %d)\n",
                            nTotalSize, g_dwTotalBinRcvSize,
                            g_nRcvBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);
        }

        ///////////////////////////////////
        //
        //  free bulk job memory
        //

        JOBMEM_Delete(g_pRcvJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");

        g_fBinRcvHeaderDefine = OFF;
        g_nRcvBodyDivCount = 0;
        g_dwTotalBinRcvSize = 0;
        g_nRcvIndex = 0;

        // if no error, apply job execution line
        g_nJobExecLineIdx = g_nRcvJobExecLineIdx;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobCompBinSendHeader()
//      - Service Name: RSVC_JOB_COMP_BIN_SNDHEADER

int SVC_JobCompBinSendHeader(int nOpt)
{
    DANDY_JOB_MEM       loadedJobShm;
    SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;
    int                 nTotalSize = 0;
    int                 nCompBinSize = 0;

#if 0
    if(g_pShm_SysStatus->nWorkType != WORK_TYPE_JOB)
    {
        VERBOSE_ERROR("No Job Loaded!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
#endif

    ///////////////////////////////////
    //
    //  convert SHM to DANDY_JOB_MEM format
    //

    loadedJobShm.size.nCmd = pShmem->dwCmdLoadCount;
    loadedJobShm.size.nPos = pShmem->dwTVaLoadCount;
    loadedJobShm.size.nWvf = pShmem->dwWvfLoadCount;
    loadedJobShm.size.nSwf = pShmem->dwSwfLoadCount;
    loadedJobShm.size.nMwf = pShmem->dwMwfLoadCount;
    loadedJobShm.size.nEwf = pShmem->dwEwfLoadCount;

    loadedJobShm.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
    loadedJobShm.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
    loadedJobShm.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
    loadedJobShm.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
    loadedJobShm.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
    loadedJobShm.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);

    ///////////////////////////////////
    //
    //  Compress bulk memory
    //
    g_pCompressed = JOBMEM_Compress(&loadedJobShm, &nCompBinSize);

    //sizeof(DANDY_JOB_MEM) means memory header size
    g_nRawBinLoadSize = JOBMEM_GetTotalSize(&loadedJobShm.size) - sizeof(DANDY_JOB_MEM);
    nTotalSize = nCompBinSize;

    VERBOSE_MESSAGE("Compress the job memory Done!(Raw: %d, Compressed: %d)\n",
                    g_nRawBinLoadSize, nCompBinSize);
    
    ///////////////////////////////////
    //
    // Check Total Size & Divided Count
    //

    g_nSndBodyModulus = nTotalSize % RMGR_REPLY_PACKET_DATA_LEN;
    g_nSendBodyDivCount = nTotalSize / RMGR_REPLY_PACKET_DATA_LEN;
    if(g_nSndBodyModulus != 0)
    {
        g_nSendBodyDivCount = g_nSendBodyDivCount + 1;
    }
    g_dwTotalBinCompLoadSize = nTotalSize;

    VERBOSE_MESSAGE("Copy the job memory Done!(Size: %ld, Count: %d, PacketSize: %d)\n",
                    g_dwTotalBinCompLoadSize, g_nSendBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);

    ///////////////////////////////////
    //
    // Define Reply Msg
    //

    RM_reply_packet.Data.snd_header.nBodyDivIndex     = g_nSendBodyDivCount;
    RM_reply_packet.Data.snd_header.dwRawBinLoadSize  = g_nRawBinLoadSize;
    RM_reply_packet.Data.snd_header.dwCompBinLoadSize = g_dwTotalBinCompLoadSize;

    RM_reply_packet.Data.snd_header.dwCmdLoadCount = pShmem->dwCmdLoadCount;
    RM_reply_packet.Data.snd_header.dwTVaLoadCount = pShmem->dwTVaLoadCount;
    RM_reply_packet.Data.snd_header.dwWvfLoadCount = pShmem->dwWvfLoadCount;
    RM_reply_packet.Data.snd_header.dwSwfLoadCount = pShmem->dwSwfLoadCount;
    RM_reply_packet.Data.snd_header.dwMwfLoadCount = pShmem->dwMwfLoadCount;
    RM_reply_packet.Data.snd_header.dwEwfLoadCount = pShmem->dwEwfLoadCount;
    
    RM_reply_packet.Data.snd_header.nJobExecLineIdx = g_nJobExecLineIdx;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.snd_header);


    ///////////////////////////////////
    //
    //  if option is activated, free bulk job memory
    //

    if(nOpt == 1)
    {
        VERBOSE_MESSAGE("Memory Free Done!\n");
        DEBUG_FREE(g_pCompressed);
    }

    g_fBinSndHeaderDefine = ON;
    g_nSndIndex = 0;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobCompBinSendData()
//      - Service Name: RSVC_JOB_COMP_BIN_SNDBODY

int SVC_JobCompBinSendData(int nIndex)
{
    ///////////////////////////////////
    //
    // Check Header Data Sended or Not!
    //

    if(g_fBinSndHeaderDefine == OFF)
    {
        VERBOSE_ERROR("Binary Send Header Not Defined!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex > g_nSendBodyDivCount)
    {
        VERBOSE_ERROR("Request Index is bigger than Send Body Div Index!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex != g_nSndIndex)
    {
        VERBOSE_ERROR("Request Index is Not Continuous Value(Ideal Value: %d)!\n",
                      g_nSndIndex);
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }

    g_nSndIndex = nIndex + 1;

    ///////////////////////////////////
    //
    //  copy SHM to bulk memory
    //

    if(nIndex != (g_nSendBodyDivCount - 1))
    {
        memcpy(RM_reply_packet.Data.data,
               (unsigned char*)g_pCompressed + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
               RMGR_REPLY_PACKET_DATA_LEN);
    }
    else if(nIndex == (g_nSendBodyDivCount - 1))
    {
        memcpy(RM_reply_packet.Data.data,
               (unsigned char*)g_pCompressed + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
               g_nSndBodyModulus);
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    if(nIndex != (g_nSendBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.data);
        VERBOSE_MESSAGE("[Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }
    else if(nIndex == (g_nSendBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = g_nSndBodyModulus;
        VERBOSE_MESSAGE("[End of Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }

    ///////////////////////////////////
    //
    //  if last index, free bulk job memory
    //

    if(nIndex == (g_nSendBodyDivCount - 1))
    {
        VERBOSE_MESSAGE("Memory Free Done!\n");
        DEBUG_FREE(g_pCompressed);

        g_fBinSndHeaderDefine = OFF;
        g_nSendBodyDivCount = 0;
        g_dwTotalBinCompLoadSize = 0;
        g_nRawBinLoadSize = 0;
        g_nSndIndex = 0;
    }

    return RESULT_OK;
}

///////////////////////////////////////
//
//  Function: SVC_JobCompBinReceiveHeader()
//      - Service Name: RSVC_JOB_COMP_BIN_RCVHEADER

int SVC_JobCompBinReceiveHeader(int nOpt)
{
    //SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;

    ///////////////////////////////////
    //
    // Get Size to Global Variables
    //

    g_nRcvBodyDivCount      = RM_packet.Data.rcv_header.nBodyDivIndex;
    g_dwTotalBinCompRcvSize = RM_packet.Data.rcv_header.dwCompBinLoadSize;
    g_nRawBinRcvSize        = RM_packet.Data.rcv_header.dwRawBinLoadSize;

    g_dwCmdRcvCount = RM_packet.Data.rcv_header.dwCmdLoadCount;
    g_dwTVaRcvCount = RM_packet.Data.rcv_header.dwTVaLoadCount;
    g_dwWvfRcvCount = RM_packet.Data.rcv_header.dwWvfLoadCount;
    g_dwSwfRcvCount = RM_packet.Data.rcv_header.dwSwfLoadCount;
    g_dwMwfRcvCount = RM_packet.Data.rcv_header.dwMwfLoadCount;
    g_dwEwfRcvCount = RM_packet.Data.rcv_header.dwEwfLoadCount;

    g_nRcvJobExecLineIdx = RM_packet.Data.rcv_header.nJobExecLineIdx;

    VERBOSE_MESSAGE("Receive the job memory Header Done!(Size: %ld, Count: %d, PacketSize: %d)\n",
                    g_dwTotalBinCompRcvSize, g_nRcvBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);

    g_pCompressed = (unsigned char*) DEBUG_MALLOC(g_dwTotalBinCompRcvSize);

    g_BulkMemSize.nCmd = g_dwCmdRcvCount;
    g_BulkMemSize.nPos = g_dwTVaRcvCount;
    g_BulkMemSize.nWvf = g_dwWvfRcvCount;
    g_BulkMemSize.nSwf = g_dwSwfRcvCount;
    g_BulkMemSize.nMwf = g_dwMwfRcvCount;
    g_BulkMemSize.nEwf = g_dwEwfRcvCount;

    ///////////////////////////////////
    //
    //  create bulk memory
    //

    // create the job memory
    g_pRcvJobMem = JOBMEM_Create(&g_BulkMemSize);
    
    if (g_pRcvJobMem == NULL)
    {
        VERBOSE_ERROR("cannot create the job memory for Receive\n");
        return RESULT_ERROR;
    }
    else
    {
        VERBOSE_MESSAGE("Create the job memory for Receive Done!\n");
        VERBOSE_MESSAGE("(Cmd: %d, Pos: %d, Wvf: %d, Swf: %d, Mwf: %d, Ewf: %d)\n",
                        g_pRcvJobMem->size.nCmd, g_pRcvJobMem->size.nPos,
                        g_pRcvJobMem->size.nWvf, g_pRcvJobMem->size.nSwf,
                        g_pRcvJobMem->size.nMwf, g_pRcvJobMem->size.nEwf);
    }
    
    ///////////////////////////////////
    //
    // Define Reply Msg
    //

    RM_reply_packet.Data.rcv_header.nBodyDivIndex     = g_nRcvBodyDivCount;
    RM_reply_packet.Data.rcv_header.dwCompBinLoadSize = g_dwTotalBinCompRcvSize;
    RM_reply_packet.Data.rcv_header.dwRawBinLoadSize  = g_nRawBinRcvSize;

    RM_reply_packet.Data.rcv_header.dwCmdLoadCount = g_dwCmdRcvCount;
    RM_reply_packet.Data.rcv_header.dwTVaLoadCount = g_dwTVaRcvCount;
    RM_reply_packet.Data.rcv_header.dwWvfLoadCount = g_dwWvfRcvCount;
    RM_reply_packet.Data.rcv_header.dwSwfLoadCount = g_dwSwfRcvCount;
    RM_reply_packet.Data.rcv_header.dwMwfLoadCount = g_dwMwfRcvCount;
    RM_reply_packet.Data.rcv_header.dwEwfLoadCount = g_dwEwfRcvCount;
    
    RM_reply_packet.Data.rcv_header.nJobExecLineIdx = g_nRcvJobExecLineIdx;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.rcv_header);

    ///////////////////////////////////
    //
    //  if option is activated, free bulk job memory
    //

    if(nOpt == 1)
    {
        JOBMEM_Delete(g_pRcvJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");
        DEBUG_FREE(g_pCompressed);
    }

    g_fBinRcvHeaderDefine = ON;
    g_nRcvIndex = 0;

    return RESULT_OK;
}

///////////////////////////////////////
//
//  Function: SVC_JobCompBinReceiveData()
//      - Service Name: RSVC_JOB_COMP_BIN_RCVBODY

int SVC_JobCompBinReceiveData(int nIndex)
{
    DANDY_JOB_MEM       editedJobShm;
    SHM_DANDY_JOB*      pShmem = g_pShmemJobRM;
    int                 nTotalSize;

    ///////////////////////////////////
    //
    // Check Header Data Received or Not!
    //

    if(g_fBinRcvHeaderDefine == OFF)
    {
        VERBOSE_ERROR("Binary Receive Header Not Defined!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex > g_nRcvBodyDivCount)
    {
        VERBOSE_ERROR("Request Index is bigger than Receive Body Div Index!\n");
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }
    if(nIndex != g_nRcvIndex)
    {
        VERBOSE_ERROR("Request Index is Not Continuous Value(Ideal Value: %d)!\n",
                      g_nRcvIndex);
        SVC_DefineErrorState(ON, SVC_ERR_JOBDATA_COMM);
        return SVC_ERR_JOBDATA_COMM;
    }

    g_nRcvIndex = nIndex + 1;

    ///////////////////////////////////
    //
    //  copy received data to bulk memory
    //

    //memcpy(g_pRcvJobMem->data + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
    memcpy((unsigned char*)g_pCompressed + (nIndex * RMGR_REPLY_PACKET_DATA_LEN),
           RM_packet.Data.data,
           RM_packet.nDataSize);

    if(nIndex != (g_nRcvBodyDivCount - 1))
    {
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.data);
        VERBOSE_MESSAGE("[Body]Packet Data Size: %d\n", RM_reply_packet.nDataSize);
    }
    else if(nIndex == (g_nRcvBodyDivCount - 1))
    {
        VERBOSE_MESSAGE("[End of Body]Packet Data\n");
    }

    ///////////////////////////////////
    //
    //  if last index, copy & free bulk job memory
    //

    if(nIndex == (g_nRcvBodyDivCount - 1))
    {
        ///////////////////////////////////
        //
        //  uncompress bulk memory
        //

        if(JOBMEM_Uncompress(g_pCompressed,
                             g_dwTotalBinCompRcvSize,
                             &g_BulkMemSize,
                             g_pRcvJobMem) != JOBASM_OK)
        {
            VERBOSE_ERROR("Uncompress the job memory Error!(Raw: %d, Compressed: %ld)\n",
                          g_nRawBinRcvSize, g_dwTotalBinCompRcvSize);
        }
        else
        {
            VERBOSE_MESSAGE("Uncompress the job memory Done!(Raw: %d, Compressed: %ld)\n",
                            g_nRawBinRcvSize, g_dwTotalBinCompRcvSize);
        }

        ///////////////////////////////////
        //
        //  copy bulk memory to SHM
        //
        editedJobShm.size.nCmd = pShmem->dwCmdSize;
        editedJobShm.size.nPos = pShmem->dwTVaSize;
        editedJobShm.size.nWvf = pShmem->dwWvfSize;
        editedJobShm.size.nSwf = pShmem->dwSwfSize;
        editedJobShm.size.nMwf = pShmem->dwMwfSize;
        editedJobShm.size.nEwf = pShmem->dwEwfSize;

        editedJobShm.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
        editedJobShm.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
        editedJobShm.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
        editedJobShm.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
        editedJobShm.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
        editedJobShm.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);

        if(JOBMEM_Copy(g_pRcvJobMem, &editedJobShm) == FALSE)
        {
            VERBOSE_ERROR("cannot copy received memory job to the shared memory...\n");

            g_fBinRcvHeaderDefine = OFF;
            g_nRcvBodyDivCount = 0;
            g_dwTotalBinCompRcvSize = 0;
            g_nRawBinRcvSize = 0;
            g_nRcvIndex = 0;

            return RESULT_ERROR;
        }

        //sizeof(DANDY_JOB_MEM) means memory header size
        nTotalSize = JOBMEM_GetTotalSize(&g_pRcvJobMem->size) - sizeof(DANDY_JOB_MEM);

        pShmem->dwCmdLoadCount = g_pRcvJobMem->size.nCmd;
        pShmem->dwTVaLoadCount = g_pRcvJobMem->size.nPos;
        pShmem->dwWvfLoadCount = g_pRcvJobMem->size.nWvf;
        pShmem->dwSwfLoadCount = g_pRcvJobMem->size.nSwf;
        pShmem->dwMwfLoadCount = g_pRcvJobMem->size.nMwf;
        pShmem->dwEwfLoadCount = g_pRcvJobMem->size.nEwf;

        if(nTotalSize != g_nRawBinRcvSize)
        {
            VERBOSE_ERROR("Total Size Wrong! (MemSize: %d, RcvSize: %d)\n",
                          nTotalSize, g_nRawBinRcvSize);
        }
        else
        {
            VERBOSE_MESSAGE("Copy the job memory Done!\n"
                            "(MemSize: %d, RcvSize: %d, Count: %d, PacketSize: %d)\n",
                            nTotalSize, g_nRawBinRcvSize,
                            g_nRcvBodyDivCount, RMGR_REPLY_PACKET_DATA_LEN);
        }

        ///////////////////////////////////
        //
        //  free bulk job memory
        //

        JOBMEM_Delete(g_pRcvJobMem);
        VERBOSE_MESSAGE("Memory Free Done!\n");
        DEBUG_FREE(g_pCompressed);

        g_fBinRcvHeaderDefine = OFF;
        g_nRcvBodyDivCount = 0;
        g_dwTotalBinCompRcvSize = 0;
        g_nRawBinRcvSize = 0;
        g_nRcvIndex = 0;

        // if no error, apply job execution line
        g_nJobExecLineIdx = g_nRcvJobExecLineIdx;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return RESULT_OK;
}
