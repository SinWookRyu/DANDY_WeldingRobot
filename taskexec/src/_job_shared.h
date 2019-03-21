#ifndef __JOB_SHARED_H__
#define __JOB_SHARED_H__

#include "sys_conf.h"
//#include "ipc_robotmgr.h"


#define JOB_MODULE_NAME_SIZE    32

/////////////////////////////////////////////////////////////////////////////
//
// shared memory for job share
//


///////////////////////////////////////
//
//  SHM_DANDY_JOB
//

#pragma pack(push, 1)
typedef struct _SHM_DANDY_JOB
{
    unsigned long   dwLength;

    // module info
    char    szModuleName[JOB_MODULE_NAME_SIZE];

    // capacity of the shmem
    unsigned long   dwCmdSize;      // number of commands can be loaded
    unsigned long   dwTVaSize;
    unsigned long   dwPVaSize;
    unsigned long   dwIVaSize;
    unsigned long   dwRVaSize;

    unsigned long   dwWvfSize;      // weaving condition file
    unsigned long   dwSwfSize;      // start welding condition file
    unsigned long   dwMwfSize;      // main welding condition file
    unsigned long   dwEwfSize;      // end welding condition file

    // real loaded data in shmem
    unsigned long   dwCmdLoadCount;
    unsigned long   dwTVaLoadCount;
    unsigned long   dwWvfLoadCount;
    unsigned long   dwSwfLoadCount;
    unsigned long   dwMwfLoadCount;
    unsigned long   dwEwfLoadCount;

    // binary job from below:
    //      DANDY_JOB_CMD   [CmdSize]
    //      DANDY_JOB_POS   [TVaSize]
    //      DANDY_JOB_POS   [PVaSize]
    //      INT             [IVaSize]
    //      FLOAT           [RVaSize]
    //      DANDY_JOB_WVF   [WvfSize]
    //      DANDY_JOB_SWF   [SwfSize]
    //      DANDY_JOB_MWF   [MwfSize]
    //      DANDY_JOB_EWF   [EwfSize]
    unsigned char   data[8];
} SHM_DANDY_JOB;
#pragma pack(pop)


///////////////////////////////////////
//
//  job item capacity
//  this sizes should be changed properly
//

#define SHMEM_JOB_CMD_SIZE      8192
#define SHMEM_JOB_TVA_SIZE      256
#define SHMEM_JOB_PVA_SIZE      1024
#define SHMEM_JOB_IVA_SIZE      1024
#define SHMEM_JOB_RVA_SIZE      1024
#define SHMEM_JOB_WVF_SIZE      32
#define SHMEM_JOB_SWF_SIZE      128
#define SHMEM_JOB_MWF_SIZE      128
#define SHMEM_JOB_EWF_SIZE      128


///////////////////////////////////////
//
//  GET_SHM_JOB_SIZE()
//  GET_SHM_JOB_SIZE_EX()
//

// get job shared memory size from specified capacity
#define GET_SHM_JOB_SIZE_EX(__CmdSize,  \
                            __TVaSize,  \
                            __PVaSize,  \
                            __IVaSize,  \
                            __RVaSize,  \
                            __WvfSize,  \
                            __SwfSize,  \
                            __MwfSize,  \
                            __EwfSize)  \
        (  sizeof(SHM_DANDY_JOB)                \
         + (__CmdSize) * sizeof(DANDY_JOB_CMD)  \
         + (__TVaSize) * sizeof(DANDY_JOB_POS)  \
         + (__PVaSize) * sizeof(DANDY_JOB_POS)  \
         + (__IVaSize) * sizeof(INT)            \
         + (__RVaSize) * sizeof(FLOAT)          \
         + (__WvfSize) * sizeof(DANDY_JOB_WVF)  \
         + (__SwfSize) * sizeof(DANDY_JOB_SWF)  \
         + (__MwfSize) * sizeof(DANDY_JOB_MWF)  \
         + (__EwfSize) * sizeof(DANDY_JOB_EWF)  \
         )

// get job shared memory size from itself size
#define GET_SHM_JOB_SIZE(__pShmJob)                 \
        GET_SHM_JOB_SIZE_EX((__pShmJob)->dwCmdSize, \
                            (__pShmJob)->dwTVaSize, \
                            (__pShmJob)->dwPVaSize, \
                            (__pShmJob)->dwIVaSize, \
                            (__pShmJob)->dwRVaSize, \
                            (__pShmJob)->dwWvfSize, \
                            (__pShmJob)->dwSwfSize, \
                            (__pShmJob)->dwMwfSize, \
                            (__pShmJob)->dwEwfSize  \
                            )

///////////////////////////////////////
//
//  GET_SHM_JOB_BUFFER_START() = job buffer start pointer
//  GET_SHM_JOB_BUFFER_END()   = job buffer end pointer
//

#define GET_SHM_JOB_BUFFER_START(__pShmJob) \
        ((void*)         ((__pShmJob)->data)
#define GET_SHM_JOB_BUFFER_END(__pShmJob)   \
        ((void*)         ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize    \
                                            + sizeof(INT)           * (__pShmJob)->dwIVaSize)   \
                                            + sizeof(FLOAT)         * (__pShmJob)->dwRVaSize)   \
                                            + sizeof(DANDY_JOB_WVF) * (__pShmJob)->dwWvfSize    \
                                            + sizeof(DANDY_JOB_SWF) * (__pShmJob)->dwSwfSize    \
                                            + sizeof(DANDY_JOB_MWF) * (__pShmJob)->dwMwfSize    \
                                            + sizeof(DANDY_JOB_EWF) * (__pShmJob)->dwEwfSize))

///////////////////////////////////////
//
//  GET_SHM_JOB_CMD_BUFFER() = job command buffer
//  GET_SHM_JOB_TVA_BUFFER() = job embedded position buffer
//  GET_SHM_JOB_PVA_BUFFER() = job position variable buffer
//  GET_SHM_JOB_IVA_BUFFER() = job integer number variable buffer
//  GET_SHM_JOB_RVA_BUFFER() = job real number variable buffer
//  GET_SHM_JOB_WVF_BUFFER() = job weaving condition file buffer
//  GET_SHM_JOB_SWF_BUFFER() = job start welding condition file buffer
//  GET_SHM_JOB_MWF_BUFFER() = job main welding condition file buffer
//  GET_SHM_JOB_EWF_BUFFER() = job end welding condition file buffer
//

// job command buffer
#define GET_SHM_JOB_CMD_BUFFER(__pShmJob)   \
        ((DANDY_JOB_CMD*) ((__pShmJob)->data + 0))

// job embedded position buffer
#define GET_SHM_JOB_TVA_BUFFER(__pShmJob)   \
        ((DANDY_JOB_POS*) ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize))

// job position variable buffer, P[]
#define GET_SHM_JOB_PVA_BUFFER(__pShmJob)   \
        ((DANDY_JOB_POS*) ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize))
// job integer number variable buffer, I[]
#define GET_SHM_JOB_IVA_BUFFER(__pShmJob)   \
        ((INT*)           ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize))

// job real number variable buffer, R[]
#define GET_SHM_JOB_RVA_BUFFER(__pShmJob)   \
        ((FLOAT*)         ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize   \
                                             + sizeof(INT)           * (__pShmJob)->dwIVaSize))

// job weaving condition file buffer
#define GET_SHM_JOB_WVF_BUFFER(__pShmJob)   \
        ((DANDY_JOB_WVF*) ((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize   \
                                             + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize   \
                                             + sizeof(INT)           * (__pShmJob)->dwIVaSize   \
                                             + sizeof(FLOAT)         * (__pShmJob)->dwRVaSize))

// job start welding condition file buffer
#define GET_SHM_JOB_SWF_BUFFER(__pShmJob)   \
        ((DANDY_JOB_SWF*)((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize    \
                                            + sizeof(INT)           * (__pShmJob)->dwIVaSize    \
                                            + sizeof(FLOAT)         * (__pShmJob)->dwRVaSize    \
                                            + sizeof(DANDY_JOB_WVF) * (__pShmJob)->dwWvfSize))

// job main welding condition file buffer
#define GET_SHM_JOB_MWF_BUFFER(__pShmJob)   \
        ((DANDY_JOB_MWF*)((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize    \
                                            + sizeof(INT)           * (__pShmJob)->dwIVaSize    \
                                            + sizeof(FLOAT)         * (__pShmJob)->dwRVaSize    \
                                            + sizeof(DANDY_JOB_WVF) * (__pShmJob)->dwWvfSize    \
                                            + sizeof(DANDY_JOB_SWF) * (__pShmJob)->dwSwfSize))

// job end welding condition file buffer
#define GET_SHM_JOB_EWF_BUFFER(__pShmJob)   \
        ((DANDY_JOB_EWF*)((__pShmJob)->data + sizeof(DANDY_JOB_CMD) * (__pShmJob)->dwCmdSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwTVaSize    \
                                            + sizeof(DANDY_JOB_POS) * (__pShmJob)->dwPVaSize    \
                                            + sizeof(INT)           * (__pShmJob)->dwIVaSize    \
                                            + sizeof(FLOAT)         * (__pShmJob)->dwRVaSize    \
                                            + sizeof(DANDY_JOB_WVF) * (__pShmJob)->dwWvfSize    \
                                            + sizeof(DANDY_JOB_SWF) * (__pShmJob)->dwSwfSize    \
                                            + sizeof(DANDY_JOB_MWF) * (__pShmJob)->dwMwfSize))

///////////////////////////////////////

#endif  // __JOB_SHARED_H__
