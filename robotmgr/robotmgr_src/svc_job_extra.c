/////////////////////////////////////////////////////////////////////////////
//
//  svc_job_extra.c: Job Related Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#include "service.h"

///////////////////////////////////////
//Global_variable

RMGR_JOBCALL_STACK_DATA g_rgCmdCallStack[MAX_CALL_COUNT];
int  g_nCmdCallTop;

RMGR_JOBRETURN_DATA     g_ReturnTarget[MAX_CALL_COUNT];

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_JobFileZipUncompress()
//      - Service Name: RSVC_JOBFILE_ZIP_UNCOMPRESS
//  zip option
//      -d exdir: Extract files into exdir
//      -o      : Overwrite existing files without prompting. 
int SVC_JobFileZipUncompress(void)
{
    char szJobDir[PATH_NAME_BUFFER_SIZE] = "./job/";
    char szJobZipFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szJobFileUnzipCmd[PATH_NAME_BUFFER_SIZE] = "unzip";
    char szZipOption[PATH_NAME_BUFFER_SIZE] = " -o -d ";
    char szSpace[PATH_NAME_BUFFER_SIZE] = " ";
    int  nExitStatus;

    memcpy(szJobDir, g_pszJobDir, strlen(g_pszJobDir)+1);
    memcpy(szJobZipFileName,
           RM_packet.Data.job_unzip.szJobZipFileName,
           strlen(RM_packet.Data.job_unzip.szJobZipFileName)+1);

    CRT_strcat(szJobFileUnzipCmd, PATH_NAME_BUFFER_SIZE, szZipOption);
    CRT_strcat(szJobFileUnzipCmd, PATH_NAME_BUFFER_SIZE, szJobDir);

    CRT_strcat(szJobFileUnzipCmd, PATH_NAME_BUFFER_SIZE, szSpace);
    CRT_strcat(szJobFileUnzipCmd, PATH_NAME_BUFFER_SIZE, szJobDir);
    CRT_strcat(szJobFileUnzipCmd, PATH_NAME_BUFFER_SIZE, szJobZipFileName);

    VERBOSE_MESSAGE("%s\n", szJobFileUnzipCmd);
    nExitStatus = system(szJobFileUnzipCmd);

    if(nExitStatus != RESULT_OK)
    {
        VERBOSE_ERROR("Unzip Fail! (FileName: %s, Exit Status: %d)\n",
                      szJobZipFileName, nExitStatus);
        SVC_DefineErrorState(ON, SVC_ERR_ZIPFILE_UNCOMPRESS);

        return RESULT_ERROR;
    }

    memcpy(RM_reply_packet.Data.job_unzip.szJobZipFileName,
           szJobZipFileName,
           strlen(szJobZipFileName)+1);

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.job_unzip);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: _loc_ViewCmdCallStack()
//

static void _loc_ViewCmdCallStack(void)
{
    const RMGR_JOBCALL_STACK_DATA*  pStackPointer;
    int iStack;

    ///////////////////////////////////
    //
    //  ensure the stack point
    //

    DANDY_ASSERT(g_nCmdCallTop <= MAX_CALL_COUNT);

    ///////////////////////////////////
    //
    //  the calling stack unused?
    //

    if (g_nCmdCallTop == 0)
    {
        VERBOSE_WARNING("No call stack data\n");
        return;
    }

    ///////////////////////////////////
    //
    //  dump the calling stack
    //

    pStackPointer = g_rgCmdCallStack;

    for (iStack = 0; iStack < g_nCmdCallTop; iStack++, pStackPointer++)
    {
        VERBOSE_VERBOSE("\v\tStack[%d] Caller='%s'(%d)  Callee='%s'(%d)\n",
                        iStack,                             // stack point
                        pStackPointer->szCallerFileName,    // caller's job name
                        pStackPointer->nCallerAddr,         // calling command address
                        pStackPointer->szCalleeFileName,    // callee's job name
                        pStackPointer->nCalleeAddr);        // callee's command address
    }

    VERBOSE_VERBOSE("\v\t*--- total %d stack used ---*\n", g_nCmdCallTop);
}


///////////////////////////////////////
//
//  Function: SVC_JobCmdJump()
//      - Service Name: RSVC_JOB_CMDJUMP

int SVC_JobCmdJump(void)
{
    int nResult;
    
    ///////////////////////////////////
    //
    //  new job assembling and then load the binary job
    //

    VERBOSE_VERBOSE("New callee job '%s' assembling and loaded...\n",
                    RM_packet.Data.call_cmd.szCalleeFileName);

    nResult = SVC_LoadJobData(RM_packet.Data.call_cmd.szCalleeFileName, JOBASM_AF_DANDY1996);

    if(nResult == RESULT_ERROR)
    {
        //SVC_DefineErrorState(ON, SVC_ERR_JOB_COMPILE);
        //return SVC_ERR_JOB_COMPILE;
        return RESULT_ERROR;
    }

    VERBOSE_MESSAGE("Jump Cmd Success '%s' (%d) ==> '%s' (%d)\n",
                     RM_packet.Data.call_cmd.szCallerFileName,
                     RM_packet.Data.call_cmd.nCallerAddr,
                     RM_packet.Data.call_cmd.szCalleeFileName,
                     RM_packet.Data.call_cmd.nCalleeAddr);

    ///////////////////////////////////
    //
    //  define reply data
    //

    CRT_strncpy(RM_reply_packet.Data.call_cmd.szCallerFileName,
                JOB_MODULE_NAME_SIZE,
                RM_packet.Data.call_cmd.szCallerFileName,
                JOB_MODULE_NAME_SIZE);
    RM_reply_packet.Data.call_cmd.nCallerAddr = RM_packet.Data.call_cmd.nCallerAddr;

    CRT_strncpy(RM_reply_packet.Data.call_cmd.szCalleeFileName,
                JOB_MODULE_NAME_SIZE,
                RM_packet.Data.call_cmd.szCalleeFileName,
                JOB_MODULE_NAME_SIZE);
    RM_reply_packet.Data.call_cmd.nCalleeAddr = RM_packet.Data.call_cmd.nCalleeAddr;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.call_cmd);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobCmdCall()
//      - Service Name: RSVC_JOB_CMDCALL

int SVC_JobCmdCall(void)
{
    RMGR_JOBCALL_STACK_DATA*    pStackPointer;
    int nResult;

    ///////////////////////////////////
    //
    //  ensure current state
    //

    DANDY_ASSERT(RM_packet.Data.call_cmd.szCallerFileName != NULL);
    DANDY_ASSERT(RM_packet.Data.call_cmd.szCalleeFileName != NULL);

    DANDY_ASSERT(g_nCmdCallTop <= MAX_CALL_COUNT);

    ///////////////////////////////////
    //
    //  check stack full
    //

    if (g_nCmdCallTop == MAX_CALL_COUNT)
    {
        VERBOSE_ERROR("Call Stack Full! : %d\n", g_nCmdCallTop);
        SVC_DefineErrorState(ON, SVC_ERR_JOBCMD_PROC);
        return SVC_ERR_JOBCMD_PROC;
    }

    ///////////////////////////////////
    //
    //  get stack pointer
    //

    pStackPointer = g_rgCmdCallStack + g_nCmdCallTop;

    ///////////////////////////////////
    //
    //  save current caller job
    //

    // save current job name
    CRT_strncpy(pStackPointer->szCallerFileName,
                JOB_MODULE_NAME_SIZE,
                RM_packet.Data.call_cmd.szCallerFileName,
                JOB_MODULE_NAME_SIZE);
    pStackPointer->szCallerFileName[JOB_MODULE_NAME_SIZE-1] = 0;

    // save current calling address
    pStackPointer->nCallerAddr = RM_packet.Data.call_cmd.nCallerAddr;

    // save target job name
    CRT_strncpy(pStackPointer->szCalleeFileName,
                JOB_MODULE_NAME_SIZE,
                RM_packet.Data.call_cmd.szCalleeFileName,
                JOB_MODULE_NAME_SIZE);
    pStackPointer->szCalleeFileName[DANDY_JOB_MODULE_NAME_SIZE-1] = 0;

    // save target address
    pStackPointer->nCalleeAddr = RM_packet.Data.call_cmd.nCalleeAddr;

    ///////////////////////////////////
    //
    //  new job assembling and then load the binary job
    //

    VERBOSE_VERBOSE("New callee job '%s' assembling and loaded...\n",
                    pStackPointer->szCalleeFileName);

    nResult = SVC_LoadJobData(pStackPointer->szCalleeFileName, JOBASM_AF_DANDY1996);

    if(nResult == RESULT_ERROR)
    {
        //SVC_DefineErrorState(ON, SVC_ERR_JOB_COMPILE);
        //return SVC_ERR_JOB_COMPILE;
        return RESULT_ERROR;
    }

    ///////////////////////////////////
    //
    //  increase the call stack
    //

    g_nCmdCallTop++;

    ///////////////////////////////////
    //
    //  display result
    //

    VERBOSE_MESSAGE("Call Cmd Success '%s' ==> '%s'\n",
                     pStackPointer->szCallerFileName,
                     pStackPointer->szCalleeFileName);
    
    _loc_ViewCmdCallStack();

    ///////////////////////////////////
    //
    //  define reply data
    //

    CRT_strncpy(RM_reply_packet.Data.call_cmd.szCallerFileName,
                JOB_MODULE_NAME_SIZE,
                pStackPointer->szCallerFileName,
                JOB_MODULE_NAME_SIZE);
    RM_reply_packet.Data.call_cmd.nCallerAddr = pStackPointer->nCallerAddr;

    CRT_strncpy(RM_reply_packet.Data.call_cmd.szCalleeFileName,
                JOB_MODULE_NAME_SIZE,
                pStackPointer->szCalleeFileName,
                JOB_MODULE_NAME_SIZE);
    RM_reply_packet.Data.call_cmd.nCalleeAddr = pStackPointer->nCalleeAddr;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.call_cmd);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_JobCmdReturn()
//      - Service Name: RSVC_JOB_CMDRETURN

int SVC_JobCmdReturn(void)
{
    const RMGR_JOBCALL_STACK_DATA*  pStackPointer;
    RMGR_JOBRETURN_DATA*            pReturnTarget;
    int   nResult;

    ///////////////////////////////////
    //
    //  ensure current state
    //

    DANDY_ASSERT(g_nCmdCallTop <= MAX_CALL_COUNT);

    ///////////////////////////////////
    //
    //  check stack full
    //

    if (g_nCmdCallTop == 0)
    {
        VERBOSE_ERROR("Call Stack Empty!! (No More Retrun Target Job) : %d\n",
                       g_nCmdCallTop);
        SVC_DefineErrorState(ON, SVC_ERR_JOBCMD_PROC);
        return SVC_ERR_JOBCMD_PROC;
    }

    pReturnTarget = g_ReturnTarget;

    ///////////////////////////////////
    //
    //  get stack pointer
    //

    pStackPointer = g_rgCmdCallStack + g_nCmdCallTop - 1;
    CRT_strcpy(pReturnTarget->szReturnTargetFileName, JOB_MODULE_NAME_SIZE, "");

    // reload the return name
    VERBOSE_VERBOSE("Reload the job '%s', Return address = %d+1 (from '%s'(%d))\n",
                     pStackPointer->szCallerFileName,
                     pStackPointer->nCallerAddr,
                     pStackPointer->szCalleeFileName,
                     pStackPointer->nCalleeAddr);

    // inform the reload job name and return address
    CRT_strncpy(pReturnTarget->szReturnTargetFileName,
                JOB_MODULE_NAME_SIZE,
                pStackPointer->szCallerFileName,
                JOB_MODULE_NAME_SIZE);
    pReturnTarget->szReturnTargetFileName[JOB_MODULE_NAME_SIZE-1] = 0;

    // inform the reload job's retrun address
    pReturnTarget->nReturnTargetAddr = pStackPointer->nCallerAddr + 1; // calling next command

    ///////////////////////////////////
    //
    //  return job assembling and then load the binary job
    //

    VERBOSE_VERBOSE("Return job '%s' assembling and loaded...\n",
                     pReturnTarget->szReturnTargetFileName);

    nResult = SVC_LoadJobData(pReturnTarget->szReturnTargetFileName, JOBASM_AF_DANDY1996);

    if(nResult == RESULT_ERROR)
    {
        //SVC_DefineErrorState(ON, SVC_ERR_JOB_COMPILE);
        //return SVC_ERR_JOB_COMPILE;
        return RESULT_ERROR;
    }

    ///////////////////////////////////
    //
    //  decrease the call stack
    //

    g_nCmdCallTop--;

    ///////////////////////////////////
    //
    //  display result
    //

    VERBOSE_MESSAGE("Return Cmd Success : name='%s', addr=%d\n",
                     pReturnTarget->szReturnTargetFileName,
                     pReturnTarget->nReturnTargetAddr);    
    
    _loc_ViewCmdCallStack();

    ///////////////////////////////////
    //
    //  define reply data
    //

    CRT_strncpy(RM_reply_packet.Data.return_cmd.szReturnTargetFileName,
                JOB_MODULE_NAME_SIZE,
                pReturnTarget->szReturnTargetFileName,
                JOB_MODULE_NAME_SIZE);
    RM_reply_packet.Data.return_cmd.nReturnTargetAddr = pReturnTarget->nReturnTargetAddr;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.return_cmd);
    
    return RESULT_OK;
}
