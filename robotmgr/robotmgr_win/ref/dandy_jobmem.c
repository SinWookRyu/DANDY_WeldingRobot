#include <stdlib.h>
#include <string.h> // for memset()
#include <assert.h>

#include "dandy_debug.h"
#include "dandy_jobasm.h"
#include "dandy_jobasm_priv.h"

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

///////////////////////////////////////

#define JOB_MEM_MAGIC_NUM   0x4a4f424d  // 'JOBM'

//////////////////////////////////////

DANDY_STATIC_ASSERT(DANDY_JOB_MEM_GROUP_COUNT == 6);

static const struct
{
    int nMemType;
    int nMemTypeSize;
    const char* pszMemTypeName;
} s_rgJobMemTypeInfo[DANDY_JOB_MEM_GROUP_COUNT] =
{
    {DANDY_JOB_VAL_TYPE_NONE,    sizeof(DANDY_JOB_CMD), "JOB_CMD"},
    {DANDY_JOB_VAL_TYPE_T_VAR,   sizeof(DANDY_JOB_POS), "JOB_POS"},
    {DANDY_JOB_VAL_TYPE_WVF_VAR, sizeof(DANDY_JOB_WVF), "JOB_WVF"},
    {DANDY_JOB_VAL_TYPE_SWF_VAR, sizeof(DANDY_JOB_SWF), "JOB_SWF"},
    {DANDY_JOB_VAL_TYPE_MWF_VAR, sizeof(DANDY_JOB_MWF), "JOB_MWF"},
    {DANDY_JOB_VAL_TYPE_EWF_VAR, sizeof(DANDY_JOB_EWF), "JOB_EWF"},
};

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_GetTypeSize()
//

int JOBMEM_GetType(int nGroup)
{
    DANDY_ASSERT(nGroup >= 0 && nGroup < DANDY_JOB_MEM_GROUP_COUNT);

    return (nGroup >= 0 && nGroup < DANDY_JOB_MEM_GROUP_COUNT)
           ? s_rgJobMemTypeInfo[nGroup].nMemType
           : DANDY_JOB_VAL_TYPE_NONE;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_GetTypeSize()
//

int JOBMEM_GetTypeSize(int nGroup)
{
    DANDY_ASSERT(nGroup >= 0 && nGroup < DANDY_JOB_MEM_GROUP_COUNT);

    return (nGroup >= 0 && nGroup < DANDY_JOB_MEM_GROUP_COUNT)
           ? s_rgJobMemTypeInfo[nGroup].nMemTypeSize
           : 0;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_GetTotalSize()
//

int JOBMEM_GetTotalSize(const DANDY_JOB_MEM_SIZE* pSize)
{
    int nTotalSize;

    ///////////////////////////////////

#if 0
    int iGroup;

    nTotalSize = 0;

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        nTotalSize += JOBMEM_GetTypeSize(iGroup) * pSize->sizes[iGroup];
    }

    ///////////////////////////////////
#else
    DANDY_STATIC_ASSERT(DANDY_JOB_MEM_GROUP_COUNT == 6);

    nTotalSize = sizeof(DANDY_JOB_MEM)
               + sizeof(DANDY_JOB_CMD) * pSize->nCmd
               + sizeof(DANDY_JOB_POS) * pSize->nPos
               + sizeof(DANDY_JOB_WVF) * pSize->nWvf
               + sizeof(DANDY_JOB_SWF) * pSize->nSwf
               + sizeof(DANDY_JOB_MWF) * pSize->nMwf
               + sizeof(DANDY_JOB_EWF) * pSize->nEwf;
#endif

    ///////////////////////////////////

    return nTotalSize;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_CombineSize()
//

int JOBMEM_CombineSize(const DANDY_JOB_MEM_SIZE* pSize1,
                       const DANDY_JOB_MEM_SIZE* pSize2,
                       DANDY_JOB_MEM_SIZE* pSizeCombined)
{
    int iGroup;

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        pSizeCombined->sizes[iGroup] = DANDY_MIN(pSize1->sizes[iGroup], pSize2->sizes[iGroup]);
    }

    return pSizeCombined->nCmd;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_Create()
//

DANDY_JOB_MEM* JOBMEM_Create(const DANDY_JOB_MEM_SIZE* pSize)
{
    DANDY_JOB_MEM* pMem;
    int nTotalSize;

    unsigned char* p;

    ///////////////////////////////////
    //
    //  calc size
    //

    nTotalSize = JOBMEM_GetTotalSize(pSize);
    DANDY_ASSERT(nTotalSize >= sizeof(DANDY_JOB_MEM));

    ///////////////////////////////////
    //
    //  alloc memory

    pMem = (DANDY_JOB_MEM*) DEBUG_MALLOC(nTotalSize);

    if (pMem == NULL)
    {
        return NULL;
    }

#if defined(_DEBUG)
    memset(pMem, 0, nTotalSize);
#endif

    ///////////////////////////////////
    //
    //  assign mem info
    //

    // copy size
    pMem->dwMagicNum = JOB_MEM_MAGIC_NUM;
    pMem->size = *pSize;

    ///////////////////////////////////
    //
    // assign buffer pointers
    //

    p = pMem->data;

    // cmd buffer
    pMem->buffer.pCmdBuffer = (DANDY_JOB_CMD*) p;
    p += sizeof(DANDY_JOB_CMD) * pSize->nCmd;

    // pos buffer
    pMem->buffer.pPosBuffer = (DANDY_JOB_POS*) p;
    p += sizeof(DANDY_JOB_POS) * pSize->nPos;

    // weaving condition buffer
    pMem->buffer.pWvfBuffer = (DANDY_JOB_WVF*) p;
    p += sizeof(DANDY_JOB_WVF) * pSize->nWvf;

    // start welding condition buffer
    pMem->buffer.pSwfBuffer = (DANDY_JOB_SWF*) p;
    p += sizeof(DANDY_JOB_SWF) * pSize->nSwf;

    // main welding condition buffer
    pMem->buffer.pMwfBuffer = (DANDY_JOB_MWF*) p;
    p += sizeof(DANDY_JOB_MWF) * pSize->nMwf;

    // end welding condition buffer
    pMem->buffer.pEwfBuffer = (DANDY_JOB_EWF*) p;
    p += sizeof(DANDY_JOB_EWF) * pSize->nEwf;

    ///////////////////////////////////
    //
    //
    //

    return pMem;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_Delete()
//

void JOBMEM_Delete(DANDY_JOB_MEM* pJobMem)
{
    DANDY_ASSERT(pJobMem != NULL);
    DANDY_ASSERT(pJobMem->dwMagicNum == JOB_MEM_MAGIC_NUM);

    if (pJobMem != NULL && pJobMem->dwMagicNum == JOB_MEM_MAGIC_NUM)
    {
#if defined(_DEBUG)
        // ensure the memory validation
        int nTotalSize = JOBMEM_GetTotalSize(&pJobMem->size);
        DANDY_ASSERT(nTotalSize >= sizeof(DANDY_JOB_MEM));
        memset(pJobMem, 0, nTotalSize);
#else
        pJobMem->dwMagicNum = 0;
#endif

        // free
        DEBUG_FREE(pJobMem);
    }
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBMEM_Copy()
//

BOOL JOBMEM_Copy(const DANDY_JOB_MEM* pSrcMem, const DANDY_JOB_MEM* pDstMem)
{
    int iGroup;

    ///////////////////////////////////

    DANDY_ASSERT(pSrcMem != NULL);
    DANDY_ASSERT(pDstMem != NULL);

    ///////////////////////////////////
    //
    //  check memory buffer size
    //

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        DANDY_ASSERT(pSrcMem->size.sizes[iGroup] == 0 || pSrcMem->rgpBuffer[iGroup] != NULL);
        DANDY_ASSERT(pDstMem->size.sizes[iGroup] == 0 || pDstMem->rgpBuffer[iGroup] != NULL);

        DANDY_ASSERT(pSrcMem->size.sizes[iGroup] >= 0);
        DANDY_ASSERT(pDstMem->size.sizes[iGroup] >= 0);
        //DANDY_ASSERT(pDstMem->size.sizes[iGroup] >= pSrcMem->size.sizes[iGroup]);

        // dest memory size is enough?
        if (pDstMem->size.sizes[iGroup] < pSrcMem->size.sizes[iGroup])
        {
            JOB_ERROR("dest(%d, '%s') memory is not enough : %d < %d\n",
                      iGroup,
                      JOBASM_GetTypeName(JOBMEM_GetType(iGroup)),
                      pDstMem->size.sizes[iGroup],
                      pSrcMem->size.sizes[iGroup]);

            return DANDY_FALSE;
        }
    }

    ///////////////////////////////////
    //
    //  copy memories
    //

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        // copy memory
        memcpy(pDstMem->rgpBuffer[iGroup],
               pSrcMem->rgpBuffer[iGroup],
               s_rgJobMemTypeInfo[iGroup].nMemTypeSize * pSrcMem->size.sizes[iGroup]);
    }

    ///////////////////////////////////
    //
    //
    //

    return DANDY_TRUE;
}
