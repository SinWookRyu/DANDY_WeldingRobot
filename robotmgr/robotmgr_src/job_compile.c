#include <stdio.h>

#include "dandy_cvt.h"

#include "ipc_jobshm.h"
#include "robotmgr_main.h"
#include "service.h"

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

int     g_nAssembleOpt;
char*   g_rgpszJobTargetFiles[COMPILE_FILE_COUNT][PATH_NAME_BUFFER_SIZE];
char    g_rgszLoadedJobModName[DANDY_JOB_MODULE_NAME_SIZE];
int     g_nJobTargetFileCount = 0;
int     g_fAssemError = FALSE;


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_JOB_GetOutputPathName()
//      - get output path name

const char* _loc_JOB_GetOutputPathName(char szExtName[DANDY_JOB_MODULE_NAME_SIZE])
{
    static char szOutNameBuffer[256];

    const char* pszOutPathName;
    
    ///////////////////////////////////
    //
    // define out file name
    //

    // out file name is specified by argument
    if(g_Arg.bOutFile == TRUE)
    {
        pszOutPathName = g_Arg.pszOutFileName;
    }
    // output file name is not specified by argument
    else
    {
        // use first source file name
        //      as out file with .extension name .job
        CVT_ChangeExtName(*g_rgpszJobTargetFiles[0],
                          //".job",
                          szExtName,
                          szOutNameBuffer,
                          dimof(szOutNameBuffer));

        pszOutPathName = szOutNameBuffer;
    }

    return pszOutPathName;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: JOB_GenerateMapFile()
//      - write .map file

void JOB_GenerateMapFile(JOBASM_ASSEMBLER hAssembler)
{
    char szMapNameBuffer[256];
    const char* pszMapPathName;

    ///////////////////////////////////
    //
    //  create the .map file?
    //

    if (g_Arg.bMapFile == FALSE && RM_packet.nCode != RSVC_SERV_GENMAPFILE)
    {
        return;
    }

    // map file name...
    pszMapPathName = g_Arg.pszMapFileName;

    ///////////////////////////////////
    //
    // map file name is not specified?
    //

    if (pszMapPathName == NULL)
    {
        // use first source file name
        //      as out file with .extension name .job
        CVT_ChangeExtName(*g_rgpszJobTargetFiles[0],
                          ".map",
                          szMapNameBuffer,
                          dimof(szMapNameBuffer));

        pszMapPathName = szMapNameBuffer;
    }

    ///////////////////////////////////
    //
    //  create the .map file
    //

    VERBOSE_VERBOSE("creating .map file : '%s'\n",
                    pszMapPathName);

    if (JOBASM_GenerateMapFileName(hAssembler, pszMapPathName) == FALSE)
    {
        VERBOSE_ERROR(".map file saving error : '%s'\n", pszMapPathName);
        return;
    }

    VERBOSE_VERBOSE(".map file created...\n");
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_JOB_SaveJobBinary()
//      - save the assembled job to .job file

void _loc_JOB_SaveJobBinary(const DANDY_JOB_MEM* pJobMem)
{
    const char* pszOutPathName;

    ///////////////////////////////////
    //
    //  get output file name...
    //

    //pszOutPathName = _loc_JOB_GetOutputPathName(".job");
    pszOutPathName = _loc_JOB_GetOutputPathName(DANDY_JOB_FILE_EXTENSION);

    ///////////////////////////////////
    //
    //  create the .job file
    //

    VERBOSE_VERBOSE("creating %s file : '%s'\n",
                    DANDY_JOB_FILE_EXTENSION, pszOutPathName);

    if (JOBFILE_SaveJobFileName(pszOutPathName, pJobMem, NULL) == FALSE)
    {
        VERBOSE_ERROR("%s file saving error : '%s'\n",
                      DANDY_JOB_FILE_EXTENSION,
                      pszOutPathName);
    }
    else
    {
        VERBOSE_VERBOSE("%s file created...\n", DANDY_JOB_FILE_EXTENSION);
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_JOB_SaveJobShmem()
//      - load the assembled job to the shared memory

void _loc_JOB_SaveJobShmem(const DANDY_JOB_MEM* pJobMem)
{
    const char* pszOutPathName;

    const char* pszModName;
    const char* pszExtName;

    ///////////////////////////////////
    //
    //  get output file name...
    //

    pszOutPathName = _loc_JOB_GetOutputPathName(".job");

    ///////////////////////////////////
    //
    //  build module name
    //

    CVT_SplitPathName(pszOutPathName, &pszModName, &pszExtName);

    ///////////////////////////////////
    //
    //  load the job to shared memory
    //

    JOB_LoadJobToShmem(pszModName, pJobMem);
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: JOB_DoJobAssemble()
//      - assemble .pgm file and than save .job file

char szJobDir[PATH_NAME_BUFFER_SIZE] = "./job/";

int JOB_DoJobAssemble(void)
{
    JOBASM_ASSEMBLER hAssembler;
    int nResult, iCnt;
    const char* pszModName;
    const char* pszExtName;
    DANDY_JOB_MEM job;

    memcpy(szJobDir, g_pszJobDir, strlen(g_pszJobDir)+1);

    ///////////////////////////////////
    //
    //  create the assembler
    //

    hAssembler = JOBASM_CreateAssembler();

    if (hAssembler == NULL)
    {
        VERBOSE_ERROR("cannot create the compiler\n");
        return RESULT_ERROR;
    }

    ///////////////////////////////////
    //
    //  assemble the source files
    //
        //if argument request, multipule files can be compiled
    if(g_Arg.bAssemble == TRUE && g_fArgAseembleDone == FALSE)
    {
        for(iCnt = 0; iCnt < g_Arg.nFileCount; iCnt ++)
        {
            *g_rgpszJobTargetFiles[iCnt] = szJobDir;
            CRT_strcpy(*g_rgpszJobTargetFiles[iCnt],
                       sizeof(szJobDir),  //don't use excessive big size
                       szJobDir);
            CRT_strcat(*g_rgpszJobTargetFiles[iCnt],
                       sizeof(szJobDir) + sizeof(g_Arg.rgpszFiles[iCnt]),
                       g_Arg.rgpszFiles[iCnt]);
        }

        g_nJobTargetFileCount = g_Arg.nFileCount;
    }
        // if service request, only one file can be compiled
    else
    {
        *g_rgpszJobTargetFiles[0] = szJobDir;
        CRT_strcpy(*g_rgpszJobTargetFiles[0],
                   sizeof(szJobDir),  //don't use excessive big size
                   szJobDir);

        if(g_fCallModeJobLoad == FALSE)
        {
            CRT_strcat(*g_rgpszJobTargetFiles[0],
                       PATH_NAME_BUFFER_SIZE + DANDY_JOB_MODULE_NAME_SIZE,
                       //sizeof(szJobDir) + sizeof(Job_msg_data.szJobFileName),
                       Job_msg_data.szJobFileName);
        }
        else
        {
            CRT_strcat(*g_rgpszJobTargetFiles[0],
                       PATH_NAME_BUFFER_SIZE + DANDY_JOB_MODULE_NAME_SIZE,
                       //sizeof(szJobDir) + sizeof(g_pShm_SysStatus->szTargJobFileName),
                       g_pShm_SysStatus->szTargJobFileName);
        }

        for(iCnt = 1; iCnt < g_Arg.nFileCount; iCnt ++)
        {
            memset(g_rgpszJobTargetFiles[iCnt], 0, sizeof(g_rgpszJobTargetFiles[iCnt]));
        }

        g_nJobTargetFileCount = 1;
    }
    
    nResult = JOBASM_AssembleFile(hAssembler,
                                 (const char *const*) g_rgpszJobTargetFiles,
                                  g_nJobTargetFileCount,
                                  g_nAssembleOpt);
    
    ///////////////////////////////////
    //
    // generate map file if option specified...
    //

    JOB_GenerateMapFile(hAssembler);

    ///////////////////////////////////
    //
    //  check the assemble resultant...
    //

    if (nResult != JOBASM_OK)
    {
        char szErrMessage[512];
        
        // get error message
        JOBASM_GetErrMessage(hAssembler,
                             szErrMessage,
                             dimof(szErrMessage));

        // display error message
        VERBOSE_ERROR("Result: %d, * %s\n", nResult, szErrMessage);

        // destroy the assembler...
        JOBASM_DestroyAssembler(hAssembler);

        // define error code
        g_fAssemError = TRUE;
        SVC_DefineErrorState(ON, nResult);  //assemble error code

        return RESULT_ERROR;
    }

    VERBOSE_MESSAGE("Assembling completed successfully...\n");
    g_fAssemError = FALSE;

    // Identify Currently Loaded Job File Name(excluding extension)
    CVT_SplitPathName(*g_rgpszJobTargetFiles[0], &pszModName, &pszExtName);
    CRT_strncpy(g_rgszLoadedJobModName, DANDY_JOB_MODULE_NAME_SIZE,
                pszModName, DANDY_JOB_MODULE_NAME_SIZE);

    ///////////////////////////////////
    //
    //  refer the assembled data
    //      and then save the data
    //
        
    if (JOBASM_ReferJobMem(hAssembler, &job) == FALSE)
    {
        VERBOSE_ERROR("cannot get the assembled job memory\n");
    }
    else
    {
        ///////////////////////////////
        //
        //  save as a .job file if option specified...
        //

        _loc_JOB_SaveJobBinary(&job);
        _loc_JOB_SaveJobShmem(&job);

        g_fAseembleDone = TRUE;
    }

    ///////////////////////////////////
    //
    //  destroy the assembler
    //

    JOBASM_DestroyAssembler(hAssembler);

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: JOB_DoJobDisassemble()
//      - read .job file and disassmble


int JOB_DoJobDisassemble(void)
{
    DANDY_JOB_MEM*   pJobMem;

    const char* pszPathName;
    const char* pszTargetPathName;

    int iFiles;
    char szDelJobFileCmd[PATH_NAME_BUFFER_SIZE];

    if(g_Arg.bDisassem == TRUE)
        g_nJobTargetFileCount = g_Arg.nFileCount;
    else
        g_nJobTargetFileCount = 1;

    for (iFiles = 0; iFiles < g_nJobTargetFileCount; iFiles++)
    {
        ///////////////////////////////
        if(g_Arg.bDisassem == TRUE ||
          (g_Arg.bAssemble == TRUE && g_fArgAseembleDone == TRUE))
        {
            pszPathName = g_Arg.rgpszFiles[iFiles];
        }
        else
        {
            pszPathName = *g_rgpszJobTargetFiles[iFiles];
        }

        VERBOSE_VERBOSE("\n*'%s' file read and disassemble...\n",
                        pszPathName);
#if defined(_DUBUG)
        DSP_DispDivider();
#endif
        ///////////////////////////////
        //
        //  read .job file
        //

        DANDY_SLEEP(100);

        pszTargetPathName = _loc_JOB_GetOutputPathName(".job");

        pJobMem = JOBFILE_ReadJobFileName(pszTargetPathName, NULL, NULL);

        if (pJobMem == NULL)
        {
            VERBOSE_WARNING("fail to read .job file : '%s'\n", pszPathName);
            continue;
        }

        DANDY_SLEEP(200);

#if defined(___DEBUG)
        VERBOSE_VERBOSE("\n*'%s' memory dump\n", pszPathName);
        DSP_DispDivider();
#endif
#if 0           // dump .job binary file
        ///////////////////////////////
        //
        //  dump the .job file
        //

        JOBDIS_DumpCmd(GET_STDOUT_HANDLE(), pJobMem, NULL, JOBDIS_DF_NONE);
        JOBDIS_DumpPos(GET_STDOUT_HANDLE(), pJobMem->buffer.pPosBuffer, pJobMem->size.nPos, JOBDIS_DF_NONE);
        JOBDIS_DumpWvf(GET_STDOUT_HANDLE(), pJobMem->buffer.pWvfBuffer, pJobMem->size.nWvf, JOBDIS_DF_NONE);
        JOBDIS_DumpSwf(GET_STDOUT_HANDLE(), pJobMem->buffer.pSwfBuffer, pJobMem->size.nSwf, JOBDIS_DF_NONE);
        JOBDIS_DumpMwf(GET_STDOUT_HANDLE(), pJobMem->buffer.pMwfBuffer, pJobMem->size.nMwf, JOBDIS_DF_NONE);
        JOBDIS_DumpEwf(GET_STDOUT_HANDLE(), pJobMem->buffer.pEwfBuffer, pJobMem->size.nEwf, JOBDIS_DF_NONE);
#endif
        ///////////////////////////////
        //
        //  delete the metadata & file
        //
        JOBMEM_Delete(pJobMem);

        if(g_fAseembleDone == TRUE && g_Arg.bOutFile != TRUE)
        {
            CRT_sprintf(szDelJobFileCmd, sizeof(szDelJobFileCmd),
                        "rm -f %s", pszTargetPathName);
            VERBOSE_VERBOSE(".job File Delete Done!(cmd: %s)\n", szDelJobFileCmd);
#if defined(_WIN32)
            system(szDelJobFileCmd);
#else
            unlink(szDelJobFileCmd);
#endif
        }
    }

    VERBOSE_MESSAGE("%d file(s) are disassmbled...\n", g_nJobTargetFileCount);
    
    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: JOB_SaveJobToFile()
//      - read .job file and disassmble

char szJobSaveDir[PATH_NAME_BUFFER_SIZE] = "./job/";

int JOB_SaveJobToFile(int nOpt, const char* pszRcvSaveFileName)
{
    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    DANDY_JOB_MEM   job;
    char pszSaveTargetFileName[(PATH_NAME_BUFFER_SIZE + JOB_MODULE_NAME_SIZE)];

    memcpy(szJobSaveDir, g_pszJobDir, strlen(g_pszJobDir)+1);

    if(nOpt == JOBDIS_DF_DANDY1996)
    {
        nOpt = JOBDIS_DF_DANDY1996;
        VERBOSE_MESSAGE("Save Option: DANDY1996\n");
    }
    else if(nOpt == JOBDIS_DF_CAS)
    {
        nOpt = JOBDIS_DF_CAS;
        VERBOSE_MESSAGE("Save Option: CAS\n");
    }
    else
    {
        nOpt = JOBDIS_DF_NONE;
        VERBOSE_MESSAGE("Save Option: DANDY-II\n");
    }
    
    // cmd
    job.buffer.pCmdBuffer = GET_SHM_JOB_CMD_BUFFER(pShmem);
    job.size.nCmd = pShmem->dwCmdLoadCount;

    // pos
    job.buffer.pPosBuffer = GET_SHM_JOB_TVA_BUFFER(pShmem);
    job.size.nPos = pShmem->dwTVaLoadCount;

    // wvf
    job.buffer.pWvfBuffer = GET_SHM_JOB_WVF_BUFFER(pShmem);
    job.size.nWvf = pShmem->dwWvfLoadCount;

    // swf
    job.buffer.pSwfBuffer = GET_SHM_JOB_SWF_BUFFER(pShmem);
    job.size.nSwf = pShmem->dwSwfLoadCount;

    // mwf
    job.buffer.pMwfBuffer = GET_SHM_JOB_MWF_BUFFER(pShmem);
    job.size.nMwf = pShmem->dwMwfLoadCount;

    // ewf
    job.buffer.pEwfBuffer = GET_SHM_JOB_EWF_BUFFER(pShmem);
    job.size.nEwf = pShmem->dwEwfLoadCount;

    if(pszRcvSaveFileName != NULL)
    {
        CRT_strcpy(pszSaveTargetFileName,
                   sizeof(szJobSaveDir),  //don't use excessive big size
                   szJobSaveDir);
        CRT_strcat(pszSaveTargetFileName,
                   PATH_NAME_BUFFER_SIZE + DANDY_JOB_MODULE_NAME_SIZE,
                   pszRcvSaveFileName);
    }
    else
    {
        memcpy(pszSaveTargetFileName, *g_rgpszJobTargetFiles[0], JOB_MODULE_NAME_SIZE);
    }

#if 0
    if(pszSaveTargetFileName == NULL)
    {
        VERBOSE_ERROR("job file saving error : No Files!\n");
        
        SVC_DefineErrorState(ON, SVC_ERR_JOB_SAVE);
        return SVC_ERR_JOB_SAVE;
    }
#endif

    if(JOBDIS_SaveFile(pszSaveTargetFileName, &job, NULL, nOpt) == RESULT_OK)
    {
        VERBOSE_MESSAGE("job file saving done! : '%s'\n", pszSaveTargetFileName);

        SVC_LoadJobData(pszRcvSaveFileName, JOBASM_AF_DANDY1996);
    }
    else
    {
        VERBOSE_ERROR("job file saving error : '%s'\n", pszSaveTargetFileName);
        
        SVC_DefineErrorState(ON, SVC_ERR_JOB_SAVE);
        return SVC_ERR_JOB_SAVE;
    }

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RMGR_JOB_LOAD_DATA);

    return RESULT_OK;
}
