#include <string.h>     // for memset(), memcpy()
#include <assert.h>

#include "dandy_debug.h"

#include "dandy_jobasm.h"
#include "dandy_jobasm_priv.h"
#include "dandy_jobasm_asmblr.h"

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_GetBinarySize()
//

static void _loc_GetBinarySize(const JOBASM_ASSEMBLER_INFO* pAssembler,
                               DANDY_JOB_MEM_SIZE* pSize)
{
    DANDY_ASSERT(pAssembler != NULL);
    DANDY_ASSERT(pSize != NULL);

    pSize->nCmd = pAssembler->binary.bufferCmd.nUsedCount;
    pSize->nPos = pAssembler->binary.bufferPos.nUsedCount;
    pSize->nWvf = pAssembler->binary.bufferWvf.nUsedCount;

    pSize->nSwf = pAssembler->binary.bufferSwf.nUsedCount;
    pSize->nMwf = pAssembler->binary.bufferMwf.nUsedCount;
    pSize->nEwf = pAssembler->binary.bufferEwf.nUsedCount;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBASM_GetBinarySize()
//

void JOBASM_GetBinarySize(JOBASM_ASSEMBLER hAssembler,
                          DANDY_JOB_MEM_SIZE* pExpSize /*=NULL*/,
                          DANDY_JOB_MEM_SIZE* pRealSize /*=NULL*/)
{
    const JOBASM_ASSEMBLER_INFO* pAssembler = _JOBASM_GetAssembler(hAssembler);

    ///////////////////////////////////

    if (pAssembler == NULL)
    {
        JOB_ERROR("invalid assembler : h=%p\n", hAssembler);
    }

    ///////////////////////////////////
    //
    //  
    //

    if (pAssembler == NULL && pExpSize != NULL)
    {
        memset(pExpSize, 0, sizeof(DANDY_JOB_MEM_SIZE));
    }
    else if (pExpSize != NULL)
    {
        *pExpSize = pAssembler->binsize;
    }

    ///////////////////////////////////
    //
    //  real required buffer size
    //

    if (pAssembler == NULL && pRealSize != NULL)
    {
        memset(pRealSize, 0, sizeof(DANDY_JOB_MEM_SIZE));
    }
    else if (pRealSize != NULL)
    {
        _loc_GetBinarySize(pAssembler, pRealSize);
    }
}

/////////////////////////////////////////////////////////////////////////////
//
//  _JOBASM_ReferJobMem()
//

BOOL _JOBASM_ReferJobMem(const JOBASM_ASSEMBLER_INFO* pAssembler, DANDY_JOB_MEM* pJobMem)
{
    int iGroup;

    ///////////////////////////////////

    DANDY_ASSERT(pAssembler != NULL);
    DANDY_ASSERT(pJobMem != NULL);

    ///////////////////////////////////
    //
    //
    //

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        pJobMem->size.sizes[iGroup] = pAssembler->binary.rgBuffers[iGroup].nUsedCount;
        pJobMem->rgpBuffer[iGroup]  = pAssembler->binary.rgBuffers[iGroup].pDataArray;
    }

    return DANDY_TRUE;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBASM_ReferJobMem()
//

BOOL JOBASM_ReferJobMem(JOBASM_ASSEMBLER hAssembler, DANDY_JOB_MEM* pJobMem)
{
    const JOBASM_ASSEMBLER_INFO* pAssembler = _JOBASM_GetAssembler(hAssembler);

    ///////////////////////////////////

    if (pAssembler == NULL)
    {
        JOB_ERROR("invalid assembler : h=%p\n", hAssembler);
        return DANDY_FALSE;
    }

    ///////////////////////////////////
    //
    //  reference
    //

    return _JOBASM_ReferJobMem(pAssembler, pJobMem);
}
