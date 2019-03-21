/////////////////////////////////////////////////////////////////////////////
//
//  err_handling.c: Error Handling
//                                            2014.04.03  Ryu SinWook

///////////////////////////////////////
#include "service.h"
#include "statistics.h"

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

char g_szErrSysTime[SYSTIME_DATA_LEN];
int  g_fErrorActiveState = OFF;
int  g_fErrorReset = OFF;
int  g_fAfterErrorResetIdxCnt = 0;
int  g_fErrStackFull = FALSE;
int  g_nErrorTimeCheck;
int  g_nErrorTime;
int  g_nOldErrorTime;
CURRENT_TIME    curr_time;

///////////////////////////////////////
//
//  Function: Fn_GetSystemTime(void)
//      - Function to get system time
void Fn_GetSystemTime(void)
{
    struct tm tm;

    TIME_GetLocalTime(&tm);

    curr_time.nYear   = tm.tm_year + 1900;
    curr_time.nMonth  = tm.tm_mon + 1;
    curr_time.nDay    = tm.tm_mday;
    curr_time.nHour   = tm.tm_hour;
    curr_time.nMinute = tm.tm_min;
    curr_time.nSec    = tm.tm_sec;
}


///////////////////////////////////////
//
//  Function: Fn_GetErrorTime(void)
//      - Function to get error time
void Fn_GetErrorTime(void)
{
    struct tm tm;

    TIME_GetLocalTime(&tm);

    CRT_sprintf(g_szErrSysTime, SYSTIME_DATA_LEN,
             "%04d-%02d-%02d,%02d:%02d:%02d",
             tm.tm_year + 1900, // years since 1900
             tm.tm_mon + 1,     // months since January - [0,11]
             tm.tm_mday,        // day of the month - [1,31]
             tm.tm_hour,        // hours since midnight - [0,23]
             tm.tm_min,         // minutes after the hour - [0,59]
             tm.tm_sec);        // seconds after the minute - [0,59]
}


///////////////////////////////////////
//
//  Function: _loc_STACK_Pop()
//      - Function to delete an element from the stack
static int _loc_STACK_Pop(void)
{
    int iCount;

    if(g_nErrHistorySaveMaxCnt >= MAX_ERROR_STACK_SIZE)
    {
        SVC_DefineErrorState(ON, SVC_ERR_ERRHIST_STACK_FULL);
        VERBOSE_ERROR("Error History Stack Full!!(Max: %d, Now: %d)\n",
                      MAX_ERROR_STACK_SIZE,
                      g_nErrHistorySaveMaxCnt);
    }

    for(iCount = 1; iCount <= g_nErrHistorySaveMaxCnt; iCount++)
    {
        CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[iCount - 1],
                   SYSTIME_DATA_LEN,
                   g_ErrCodeStack.szErrStackSysTime[iCount]);
        g_ErrCodeStack.nErrStack[iCount - 1] =  g_ErrCodeStack.nErrStack[iCount];
        g_ErrCodeStack.fErrorActiveState[iCount - 1] = g_ErrCodeStack.fErrorActiveState[iCount];
        CRT_strcpy(g_ErrCodeStack.g_szErrContent[iCount - 1],
                   ERROR_NAME_LEN,
                   g_ErrCodeStack.g_szErrContent[iCount]);
    }

    CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[g_nErrHistorySaveMaxCnt],
               SYSTIME_DATA_LEN,
               "0");
    g_ErrCodeStack.nErrStack[g_nErrHistorySaveMaxCnt]   = SYS_ERR_OK;
    g_ErrCodeStack.fErrorActiveState[g_nErrHistorySaveMaxCnt] = OFF;
    CRT_strcpy(g_ErrCodeStack.g_szErrContent[g_nErrHistorySaveMaxCnt],
               ERROR_NAME_LEN,
               "null");

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: _loc_Fn_STACK_Push()
//      - Function to add an element to the stack

#define MAX_COMPARE_STACK_IDX_CNT       3

static int _loc_Fn_STACK_Push(void)
{
    int iCnt;
    int nErrOwner;
    int nMaxCompareIdx;

    nErrOwner = g_pShm_SysStatus->nErrCode & 0xf000000;

#if 0
    if(g_pShm_SysStatus->fErrorState == ON &&
       g_fAfterErrorResetIdxCnt > MAX_COMPARE_STACK_IDX_CNT)
    {
        return 0;
    }
#endif

    /* get local time */
    Fn_GetErrorTime();
    
    /* 1. check error stack full or not */
    if (g_ErrCodeStack.nTop == g_nErrHistorySaveMaxCnt)
    {
        // set the flag for Stack Full
        g_fErrStackFull = TRUE;
    }

    /* 2. set error count(for refer) */
    if(g_fErrStackFull != TRUE)
    {
        g_ErrCodeStack.nErrCnt = g_ErrCodeStack.nTop + 1;
    }

    /* 3. set variables error index, code, time, act state */
        // push the error time to stack
    CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[g_ErrCodeStack.nTop],
               SYSTIME_DATA_LEN,
               g_szErrSysTime);

    if(g_pShm_SysStatus != NULL)
    {
            // push the error code to stack
        g_ErrCodeStack.nErrStack[g_ErrCodeStack.nTop] =
                                                    g_pShm_SysStatus->nErrCode;
    }

        // push the error description to stack
    SYSMON_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);
    CRT_strcpy(g_ErrCodeStack.g_szErrContent[g_ErrCodeStack.nTop],
               ERROR_NAME_LEN,
               g_szErrContent);

        // push the error active flag to stack
    g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nTop] = g_fErrorActiveState;
    

    /* 4. check error code is valid or duplicated */
    if(g_fErrorReset == ON)
    {
        g_fAfterErrorResetIdxCnt = 0;
    }
    else
    {
        g_fAfterErrorResetIdxCnt = g_fAfterErrorResetIdxCnt + 1;
    }

    if(g_ErrCodeStack.nTop >= MAX_COMPARE_STACK_IDX_CNT)
    {
        nMaxCompareIdx = MAX_COMPARE_STACK_IDX_CNT;
    }
    else
    {
        nMaxCompareIdx = g_ErrCodeStack.nTop;
    }

    //if(g_pShm_SysStatus->fErrorState == ON && g_fAfterErrorResetIdxCnt > MAX_COMPARE_STACK_IDX_CNT)
    if(g_pShm_SysStatus->fErrorState == ON &&
       g_fAfterErrorResetIdxCnt > nMaxCompareIdx &&
       g_ErrCodeStack.nTop > nMaxCompareIdx)
    {
        for(iCnt = 0; iCnt <= nMaxCompareIdx; iCnt++)
        {
            if(g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nTop] == ON &&
              (g_ErrCodeStack.nErrStack[g_ErrCodeStack.nTop] == 
                                g_ErrCodeStack.nErrStack[g_ErrCodeStack.nTop - iCnt] &&
               g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nTop] ==
                                g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nTop - iCnt]))
            {
                CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[g_ErrCodeStack.nTop],
                           SYSTIME_DATA_LEN,
                           "0");
                g_ErrCodeStack.nErrStack[g_ErrCodeStack.nTop] = SYS_ERR_OK;
                g_ErrCodeStack.fErrorActiveState[g_ErrCodeStack.nTop] = OFF;
                g_ErrCodeStack.nTop = g_ErrCodeStack.nTop - 1;
                CRT_strcpy(g_ErrCodeStack.g_szErrContent[g_ErrCodeStack.nTop],
                           ERROR_NAME_LEN,
                           "null");
            }
        }
    }

    /* 5. if stack full, rearrange error stack */
    if(g_fErrStackFull == TRUE)
    {
        _loc_STACK_Pop();
    }

    /* 6. increase error stack top index */
    if(g_fErrStackFull != TRUE)
    {
        g_ErrCodeStack.nTop = g_ErrCodeStack.nTop + 1;
    }

    /* 7. save file */
    SVC_SaveStatisticsDataToFile(ROBOT_0_INDEX, OPT_QUITE);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_Write_ErrorHistory()
//      - write error history to stack
int SVC_Write_ErrorHistory(int nState, int nErrorCode)
{
    int nCompareIdx1;
    int nCompareIdx2;
    
    g_nErrorTime = g_dwRobotStatTime.dwTimePowerOn;

    g_nErrorTimeCheck = (abs) (g_nErrorTime - g_nOldErrorTime);

    if(nState == ON)
    {
        g_fErrorActiveState = ON;
    }
    else
    {
        g_fErrorActiveState = OFF;
    }
    
    if(g_ErrCodeStack.nTop == 0)
    {
        nCompareIdx1 = 0;
        nCompareIdx2 = 0;
    }
    else
    {
        nCompareIdx1 = g_ErrCodeStack.nTop - 1;
        nCompareIdx2 = g_ErrCodeStack.nTop - 2;
    }

    if(g_pShm_SysStatus != NULL)
    {
        if(g_ErrCodeStack.nTop == 0)
        {
            _loc_Fn_STACK_Push();
        }
        else
        {
            // check power on time
            if((g_ErrCodeStack.nErrStack[nCompareIdx2] == nErrorCode ||
                g_ErrCodeStack.nErrStack[nCompareIdx1] == nErrorCode) &&
                g_nErrorTimeCheck <= 1)
            {
                ;
            }
            else
            {
                _loc_Fn_STACK_Push();
            }
        }
    }

    g_nOldErrorTime = g_dwRobotStatTime.dwTimePowerOn;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_Clear_ErrorHistory()
//      - clear error history to stack
int SVC_Clear_ErrorHistory(void)
{
    static int iCount;

    //for(iCount = 0; iCount < g_nErrHistorySaveMaxCnt; iCount++)
    for(iCount = 0; iCount < MAX_ERROR_STACK_SIZE; iCount++)
    {
        g_fErrorReset = OFF;
        CRT_strcpy(g_ErrCodeStack.szErrStackSysTime[iCount],
                   SYSTIME_DATA_LEN,
                   "0");
        g_ErrCodeStack.nErrStack[iCount]   = SYS_ERR_OK;
        g_ErrCodeStack.fErrorActiveState[iCount] = OFF;
        CRT_strcpy(g_ErrCodeStack.g_szErrContent[iCount],
                   ERROR_NAME_LEN,
                   "null");
#if 0
        if(g_pShm_SysStatus != NULL)
        {
            _loc_Fn_STACK_Push();
        }
#endif
    }

    g_ErrCodeStack.nErrCnt = 0;
    g_ErrCodeStack.nTop = 0;

    SVC_SetErrorHistoryPacketDefine();
    SVC_ShowErrorHistoryDisplay();
    SVC_SaveStatisticsDataToFile(ROBOT_0_INDEX, OPT_QUITE);

    VERBOSE_MESSAGE("Error History Cleared!\n");

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_DefineErrorState()
//      - define error state: error flag, error code
#define START_ERRNO_SYSTEM      0X0100
#define START_ERRNO_CONFLOAD    0X0150
#define START_ERRNO_FILE        0X0200
#define START_ERRNO_JOBEXEC     0X0250
#define START_ERRNO_PARAMEDIT   0X0350
#define START_ERRNO_PARAMVALID  0X0400
#define END_OF_ERRNO            0X0450

int SVC_DefineErrorState(int nState, int nErrorCode)
{
    int fWriteHistory = 0;

    if(g_pShm_SysStatus != NULL)
    {
        if(nState == ON)
        {
            // Set Error Code
            //g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode & 0x000ffff;
            
            // Set Error Owner
            g_pShm_SysStatus->nErrCode = nErrorCode | ERR_OWNER_FROM_RM;
            //g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_OWNER_FROM_RM;

            // Set Error Module
            if(nErrorCode >= START_ERRNO_CONFLOAD && nErrorCode <= START_ERRNO_FILE)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_CONF_RELOAD;
            }
            else if(g_fAssemError == TRUE)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_JOB_COMPILE;
            }
            else if(nErrorCode >= START_ERRNO_SYSTEM && nErrorCode <= START_ERRNO_CONFLOAD)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_SYSTEM;
            }
            else if(nErrorCode >= START_ERRNO_FILE && nErrorCode <= START_ERRNO_JOBEXEC)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_FILE;
            }
            else if(nErrorCode >= START_ERRNO_JOBEXEC && nErrorCode <= START_ERRNO_PARAMEDIT)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_JOB_EXEC;
            }
            else if(nErrorCode >= START_ERRNO_PARAMEDIT && nErrorCode <= START_ERRNO_PARAMVALID)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_PARAM_EDIT;
            }
            else if(nErrorCode >= START_ERRNO_PARAMVALID && nErrorCode <= END_OF_ERRNO)
            {
                g_pShm_SysStatus->nErrCode = g_pShm_SysStatus->nErrCode | ERR_MOD_PARAM_VALIDCHECK;
            }


            //g_pShm_SysStatus->nErrCode = nErrorCode;
            g_pShm_SysStatus->fErrorState = TRUE;
            g_fErrorReset = OFF;
            SVC_Write_ErrorHistory(ON, g_pShm_SysStatus->nErrCode);
        }
        else if(nState == OFF)
        {
            if(g_pShm_SysStatus->nErrCode != SYS_ERR_OK)
            {
                fWriteHistory = ON;
            }

            g_pShm_SysStatus->nErrCode = SYS_ERR_OK;
            g_pShm_SysStatus->fErrorState = FALSE;
            g_fErrorReset = ON;
            g_fAfterErrorResetIdxCnt = 0;
            g_fAssemError = FALSE;

            if(fWriteHistory == ON)
            {
                SVC_Write_ErrorHistory(OFF, g_pShm_SysStatus->nErrCode);
            }
        }
    }
    else
    {
        return RESULT_ERROR;
    }

    return RESULT_OK;
}