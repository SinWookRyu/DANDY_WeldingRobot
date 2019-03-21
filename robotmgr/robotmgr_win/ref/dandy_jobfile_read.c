#include <assert.h>

#include "dandy_cvt.h"      // for path name handling

#include "dandy_jobasm.h"
#include "dandy_jobasm_priv.h"

#include "dandy_job.h"
#include "dandy_job_priv.h"

///////////////////////////////////////
//
//  configurations
//

#define LOAD_DUMP_HEAD  1       // 1 = dump header info
#define LOAD_DUMP_BODY  1       // 1 = dump body disassemble

// disassemble flags
#define LOAD_DUMP_DISASM_FLAGS  JOBDIS_DF_NONE

///////////////////////////////////////

#if LOAD_DUMP_BODY
#include "dandy_jobasm.h"
#endif

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_ReadJobFile_Header()
//

static BOOL _loc_ReadJobFile_Header(FILE* fpJob)
{
    DANDY_JOB_FILE_HEADER h;
    size_t sizeRead;
    int iGroup;

    ///////////////////////////////////

    DANDY_ASSERT(fpJob != NULL);

    ///////////////////////////////////
    //
    //  read header
    //

    sizeRead = fread(&h, 1, sizeof(h), fpJob);

    if (sizeRead != sizeof(h))
    {
        JOB_ERROR("read size mismatch : %u != %u\n",
                  sizeRead,
                  sizeof(h));

        return DANDY_FALSE;
    }

    JOB_VERBOSE(".job file header %u bytes read\n", sizeRead);

    ///////////////////////////////////
    //
    //  dump header
    //

#if defined(_DEBUG) && LOAD_DUMP_HEAD
    JOB_VERBOSE("message  : '%s'\n", h.szMessage);
    JOB_VERBOSE("magic    : 0x%08X\n", h.dwMagicNumber);
    JOB_VERBOSE("version  : %lu\n", h.dwVersion);
    JOB_VERBOSE("groups   : %d\n", h.nGroupCount);
    JOB_VERBOSE("cmd size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_CMD]);
    JOB_VERBOSE("pos size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_POS]);
    JOB_VERBOSE("wvf size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_WVF]);
    JOB_VERBOSE("swf size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_SWF]);
    JOB_VERBOSE("mwf size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_MWF]);
    JOB_VERBOSE("ewf size : %u\n", h.rgnTypeSize[DANDY_JOB_MEM_GROUP_EWF]);
#endif

    ///////////////////////////////////
    //
    //  check the header validation
    //

    // endian check
    if (h.dwMagicNumber == DANDY_JOB_FILE_MAGIC_NUMBER2)
    {
        JOB_ERROR("endian mismatch...\n");
        return DANDY_FALSE;
    }

    // magic number
    if (h.dwMagicNumber != DANDY_JOB_FILE_MAGIC_NUMBER)
    {
        JOB_ERROR("magic number mismatch...\n");
        return DANDY_FALSE;
    }

    // version
    if (h.dwVersion != DANDY_JOB_FILE_VERSION)
    {
        JOB_ERROR("version mismatch : %lu != %d\n",
                  h.dwVersion,
                  DANDY_JOB_FILE_VERSION);

        return DANDY_FALSE;
    }

    // group count
    if (h.nGroupCount != DANDY_JOB_MEM_GROUP_COUNT)
    {
        JOB_ERROR("group count mismatch : %d != %d\n",
                  h.nGroupCount,
                  DANDY_JOB_MEM_GROUP_COUNT);

        return DANDY_FALSE;
    }

    ///////////////////////////////////
    //
    //  check type sizes
    //

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        if (h.rgnTypeSize[iGroup] != JOBMEM_GetTypeSize(iGroup))
        {
            JOB_ERROR("type[%d] size mismatch : %u != %u\n",
                      iGroup,
                      h.rgnTypeSize[iGroup],
                      JOBMEM_GetTypeSize(iGroup));

            return DANDY_FALSE;
        }
    }

    return DANDY_TRUE;
}

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_ReadJobFile_Body()
//

static DANDY_JOB_MEM* _loc_ReadJobFile_Body(FILE* fpJob,
                                            const DANDY_JOB_MEM* pJobMem /*=NULL*/,
                                            DANDY_JOB_MEM_SIZE* pJobSize /*=NULL*/)
{
    DANDY_JOB_MEM_SIZE sizeJob;
    size_t sizeRead;
    size_t sizeTotal;

    int iGroup;

    ///////////////////////////////////

    DANDY_ASSERT(fpJob != NULL);

    ///////////////////////////////////
    //
    //  readout data size
    //

    sizeRead = fread(&sizeJob, 1, sizeof(sizeJob), fpJob);

    if (sizeRead != sizeof(sizeJob))
    {
        JOB_ERROR("metasize read size mismtach : %u != %d\n",
                  sizeRead,
                  sizeof(sizeJob));

        return NULL;
    }

    ///////////////////////////////////
    //
    //  check job memory
    //

    if (pJobMem == NULL)
    {
        // create the job memory
        pJobMem = JOBMEM_Create(&sizeJob);

        if (pJobMem == NULL)
        {
            JOB_ERROR("cannot create the job memory\n");
            return NULL;
        }
    }
    else
    {
        // check buffer size
        for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
        {
            if (pJobMem->size.sizes[iGroup] < sizeJob.sizes[iGroup])
            {
                JOB_ERROR("buffer[%d] size not enough : %d < %d\n",
                          iGroup,
                          pJobMem->size.sizes[iGroup],
                        sizeJob.sizes[iGroup]);

                return NULL;
            }
        }
    }

    ///////////////////////////////////
    //
    //  read the data
    //

    sizeTotal = 0;

    for (iGroup = 0; iGroup < DANDY_JOB_MEM_GROUP_COUNT; iGroup++)
    {
        sizeRead = fread(pJobMem->rgpBuffer[iGroup],
                         JOBMEM_GetTypeSize(iGroup),
                         sizeJob.sizes[iGroup],
                         fpJob);

        if (sizeRead != (size_t) sizeJob.sizes[iGroup])
        {
            JOB_ERROR("cannot read buffer[%d] : %u != %d\n",
                      iGroup,
                      sizeRead,
                      sizeJob.sizes[iGroup]);

            return NULL;
        }

        sizeTotal += sizeRead;
    }

    ///////////////////////////////////
    //
    //  read the data
    //

    JOB_VERBOSE(".job file body %u bytes read\n", sizeTotal);

    if (pJobSize != NULL)
    {
        *pJobSize = sizeJob;
    }

    ///////////////////////////////////

    return (DANDY_JOB_MEM*) pJobMem;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBFILE_ReadJobFile()
//

DANDY_JOB_MEM* JOBFILE_ReadJobFile(FILE* fpJob,
                                   const DANDY_JOB_MEM* pJobMem /*=NULL*/,
                                   DANDY_JOB_MEM_SIZE* pJobSize /*=NULL*/)
{
    DANDY_JOB_MEM_SIZE sizeJob;

    ///////////////////////////////////

    DANDY_ASSERT(fpJob != NULL);

    ///////////////////////////////////

    if (fpJob == NULL)
    {
        JOB_ERROR("invalid FILE*=%p\n", fpJob);
        return NULL;
    }

    ///////////////////////////////////
    //
    //  header reading
    //

    if (_loc_ReadJobFile_Header(fpJob) == DANDY_FALSE)
    {
        JOB_ERROR("job file header reading error\n");
        return NULL;
    }

    ///////////////////////////////////
    //
    //  data reading
    //

    pJobMem = _loc_ReadJobFile_Body(fpJob, pJobMem, &sizeJob);
    
    if (pJobMem == NULL)
    {
        JOB_ERROR("job file read failure...\n");
        return NULL;
    }

    ///////////////////////////////////
    //
    //  dump the job data to standard-out
    //

#if defined(_DEBUG) && LOAD_DUMP_BODY
    JOBDIS_fDumpCmd(stdout, pJobMem, &sizeJob, LOAD_DUMP_DISASM_FLAGS);
    JOBDIS_fDumpPos(stdout, pJobMem->buffer.pPosBuffer, sizeJob.nPos, LOAD_DUMP_DISASM_FLAGS);
    JOBDIS_fDumpWvf(stdout, pJobMem->buffer.pWvfBuffer, sizeJob.nWvf, LOAD_DUMP_DISASM_FLAGS);
    JOBDIS_fDumpSwf(stdout, pJobMem->buffer.pSwfBuffer, sizeJob.nSwf, LOAD_DUMP_DISASM_FLAGS);
    JOBDIS_fDumpMwf(stdout, pJobMem->buffer.pMwfBuffer, sizeJob.nMwf, LOAD_DUMP_DISASM_FLAGS);
    JOBDIS_fDumpEwf(stdout, pJobMem->buffer.pEwfBuffer, sizeJob.nEwf, LOAD_DUMP_DISASM_FLAGS);
#endif

    ///////////////////////////////////
    //
    //
    //

    if (pJobSize != NULL)
    {
        *pJobSize = sizeJob;
    }

    return (DANDY_JOB_MEM*) pJobMem;
}

/////////////////////////////////////////////////////////////////////////////
//
//  JOBFILE_ReadJobFileName()
//
//  read the .job file with path name
//

DANDY_JOB_MEM* JOBFILE_ReadJobFileName(const char* pszJobPathName,
                                       const DANDY_JOB_MEM* pJobMem /*=NULL*/,
                                       DANDY_JOB_MEM_SIZE* pJobSize /*=NULL*/)
{
    char szNameBuffer[CVT_MAX_PATHNAME_SIZE];
    const char* pszPathName;

    FILE* fpJob;

    ///////////////////////////////////
    //
    //  build .job file name
    //      ==> default .extension name is ".job"
    //  

    CVT_SetDefExtName(pszJobPathName,
                      DANDY_JOB_FILE_EXTENSION,
                      szNameBuffer,
                      dimof(szNameBuffer));

    pszPathName = szNameBuffer;

    JOB_VERBOSE("try to load .job file : '%s'\n",
                pszPathName);

    ///////////////////////////////////
    //
    //  open the .job file pointer
    //

    fpJob = fopen(pszPathName, "rb");

    if (fpJob == NULL)
    {
        JOB_ERROR("cannot open job file to load : '%s'\n",
                  pszPathName);

        return NULL;
    }

    ///////////////////////////////////
    //
    //  save the job contents
    //

    pJobMem = JOBFILE_ReadJobFile(fpJob, pJobMem, pJobSize);
    fclose(fpJob);
    
    if (pJobMem == NULL)
    {
        JOB_ERROR("job file read error\n");
        // pass
    }

    ///////////////////////////////////

    return (DANDY_JOB_MEM*) pJobMem;
}

