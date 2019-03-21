#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#if defined (__QNXNTO__)
#include "ecatmkpa.h"
#include "libmkpaiodev.h"
//#include "mkpaauxiliary.h"
#endif 

#include "servocon_main.h"
#include "dandy_platform.h"
#include "dandy_thread.h"
#include "dandy_echo.h"

ECAT_CHAR g_szEcatErrorDescription[ERROR_MESSAGE_BUFFER_SIZE] = {0};   // ethercat error description
int g_nInternalPulse[ROB_AXIS_COUNT];
int g_nModeOperation = CYCLIC_SYNC_POS_MODE;


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_GetErrorDescription()
//
ECAT_CHAR* ECATLIB_GetErrorDescription(ECAT_RESULT nErrorCode)
{
	//static ECAT_CHAR s_szBuffer[ERROR_MESSAGE_BUFFER_SIZE];
#if defined (__QNXNTO__)
    ECAT_DWORD dwSize;
    if (ECAT_FAILED(EcatIODevGetErrorMessage(nErrorCode,
    		                                 g_szEcatErrorDescription,
    		                                 ERROR_MESSAGE_BUFFER_SIZE - 1,
    		                                 &dwSize)))
    {
    	return _T("Unknown");
    }

    g_szEcatErrorDescription[dwSize] = _T('\0');
#endif
    return g_szEcatErrorDescription;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_SleepMS(ECAT_TIME_MS timeMS)
//
void  ECATLIB_SleepMS(ECAT_TIME_MS timeMS)
{
#if defined (__QNXNTO__)
	struct timespec tm;
	tm.tv_sec = (timeMS) / 1000;
	tm.tv_nsec = (timeMS % 1000) * 1000000;
	nanosleep(&tm,NULL);
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_wchar2char()
//
void ECATLIB_wchar2char(ECAT_CHAR* psz, const ECAT_WCHAR* pwsz)
{
#if defined (__QNXNTO__)
    ECAT_CHAR *__psz = psz;
    const ECAT_WCHAR *__pwsz = pwsz;
    while(*__pwsz)
        *(__psz++) = (ECAT_CHAR)*(__pwsz++);
    *(__psz++) = '\0';
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_ECATLIB_ReadWriteSlaveCoEObject()
//

static ECAT_RESULT _loc_ECATLIB_ReadWriteSlaveCoEObject(IN ECAT_HANDLE hMaster,
                                                        IN ECAT_WORD wSlaveId,
                                                        IN ECAT_WORD wIndex,
                                                        IN ECAT_BYTE bySubIndex,
                                                        IN ECAT_BYTE byService,
                                                        IN ECAT_DWORD dwTimeOut,
                                                        IN OUT ECAT_BYTE* pData,
                                                        IN OUT ECAT_DWORD* pdwLength)
{
#if defined (__QNXNTO__)
	ECAT_RESULT erResult;

	erResult = EcatIODevAddCoEObject(hMaster,
                                     wSlaveId,
                                     wIndex,
                                     bySubIndex,
			                         byService, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         pData,
                                     pdwLength,
                                     *pdwLength);

	return erResult;
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_WriteIntegerSlaveCoEObject()
//
int ECATLIB_WriteIntegerSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, ECAT_DWORD dwWriteData)
{
    int nRet = 0;
#if defined (__QNXNTO__)
    ECAT_DWORD dwLength;
    dwLength = sizeof(dwWriteData);
#if defined(_DUBUG)    
    VERBOSE_VERBOSE("Write object(slave: %d, Idx: %x - %x, data: %ld)\n", 
                     nSlave, nWriteIdx, nSubIdx, dwWriteData);
#endif
    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nWriteIdx   /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                1           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                               (ECAT_BYTE*)&dwWriteData,
                                                &dwLength);
#endif
    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_WriteSignedIntegerSlaveCoEObject()
//
int ECATLIB_WriteSignedIntegerSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, int nWriteData, ECAT_DWORD dwLength)
{
    int nRet = 0;
#if defined (__QNXNTO__)
    //ECAT_DWORD dwLength;
    dwLength = sizeof(nWriteData);
#if defined(_DUBUG)    
    VERBOSE_VERBOSE("Write object(slave: %d, Idx: %x - %x, data: %d)\n", 
                     nSlave, nWriteIdx, nSubIdx, nWriteData);
#endif
    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nWriteIdx   /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                1           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                               (ECAT_BYTE*)&nWriteData,
                                                &dwLength);
#endif
    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_WriteStringSlaveCoEObject()
//
int ECATLIB_WriteStringSlaveCoEObject(int nSlave, int nWriteIdx, int nSubIdx, ECAT_BYTE *pszWriteData, ECAT_DWORD dwLength)
{
    int nRet = 0;
#if defined (__QNXNTO__)
#if defined(_DUBUG)
    VERBOSE_VERBOSE("Write object(slave: %d, Idx: %x - %x, data: %s)\n", 
                     nSlave, nWriteIdx, nSubIdx, pszWriteData);
#endif
    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nWriteIdx   /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                1           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                                pszWriteData,
                                                &dwLength);
#endif
    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_ReadSlaveCoEObjectInteger()
//
ECAT_DWORD ECATLIB_ReadSlaveCoEObjectInteger(int nSlave, int nReadIdx, int nSubIdx, ECAT_DWORD dwReadData)
{
    ECAT_DWORD nRet = 0;
#if defined (__QNXNTO__)
    ECAT_DWORD dwLength;
    dwLength = sizeof(dwReadData);
    
    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nReadIdx    /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                2           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                               (ECAT_BYTE*)&dwReadData,
                                                &dwLength);
#if 0
    VERBOSE_VERBOSE("Read object(slave: %d, Idx: %x - %x, data: %ld, length: %d)\n",
                     nSlave, nReadIdx, nSubIdx, dwReadData, dwLength);
#endif

    return dwReadData;
#else
    return nRet;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_ReadSlaveCoEObjectInteger2()
//
int ECATLIB_ReadSlaveCoEObjectInteger2(int nSlave, int nReadIdx, int nSubIdx, int nReadData)
{
    ECAT_DWORD nRet = 0;
#if defined (__QNXNTO__)
    ECAT_DWORD dwLength;
    dwLength = sizeof(nReadData);
    
    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nReadIdx    /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                2           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                               (ECAT_BYTE*)&nReadData,
                                                &dwLength);
#if 0
    VERBOSE_VERBOSE("Read object(slave: %d, Idx: %x - %x, data: %d, length: %d)\n",
                     nSlave, nReadIdx, nSubIdx, nReadData, dwLength);
#endif

    return nReadData;
#else
    return nRet;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_ReadSlaveCoEObjectSignedInteger()
//
int ECATLIB_ReadSlaveCoEObjectSignedInteger(int nSlave, int nReadIdx, int nSubIdx, int nReadData)
{
    int nRet = 0;
#if defined (__QNXNTO__)
    ECAT_DWORD dwLength;
    dwLength = sizeof(nReadData);
    int nTemp; 

    nRet = _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                                nSlave      /* slave index */,
                                                nReadIdx    /* object dictionary index */,
                                                nSubIdx     /* object dictionary subindex */,
                                                2           /* upload (1) / download (2) */,
                                                COE_OPERATION_TIMEOUT_MS /* timeout */,
                                               (ECAT_BYTE*)&nReadData,
                                                &dwLength);
    nTemp = nReadData; 
    if(nTemp >> (8*(dwLength - 1) + 7)) // if minus value
    {
        if(dwLength != 4)
            nReadData = nReadData - (1<<(8*dwLength)); 
        else if(dwLength == 4)
            nReadData = (int) nReadData;
    }
    else                                // if plus value
    {
        nReadData = 0x00000000 | nReadData; 
    }

#if 0
    VERBOSE_VERBOSE("Read object(slave: %d, Idx: %x - %x, data: %d, length: %d, Tmp: %d, TmpCal: %d)\n",
                     nSlave, nReadIdx, nSubIdx, nReadData, (int) dwLength, nTemp, nTemp >> (8*(dwLength - 1) + 7));
#endif

    return nReadData;
#else
    return nRet;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATLIB_ReadSlaveCoEObjectString()
//
ECAT_BYTE* ECATLIB_ReadSlaveCoEObjectString(int nSlave, int nReadIdx, int nSubIdx, ECAT_BYTE *pszReadData, ECAT_DWORD dwLength)
{
#if defined (__QNXNTO__)
    
    _loc_ECATLIB_ReadWriteSlaveCoEObject(g_hMaster,
                                         nSlave         /* slave index */,
                                         nReadIdx       /* object dictionary index */,
                                         nSubIdx        /* object dictionary subindex */,
                                         2              /* upload (1) / download (2) */,
                                         COE_OPERATION_TIMEOUT_MS /* timeout */,
                                         pszReadData,
                                         &dwLength);
#if 0
    VERBOSE_VERBOSE("Read object(slave: %d, Idx: %x - %x, data: %s)\n",
                     nSlave, nReadIdx, nSubIdx, pszReadData);
#endif
    return pszReadData;
#else
    ECAT_BYTE* pszRet = NULL;
    return pszRet;
#endif
}

////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteHomeOffsetVal()
// -nAxis: index of axis(0~5)
//
int ECATSERV_WriteHomeOffsetVal(int nAxis)
{
    int nRet;
    int nHomeOffsetVal;
    int noldHomeOffsetVal;

    int nReadBuff = 1;
    
    g_nInternalPulse[nAxis] = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x6063, 0, nReadBuff);

    nHomeOffsetVal = -1 * g_nInternalPulse[nAxis];

    noldHomeOffsetVal = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x607C, 0, nReadBuff);
#if defined(_DUBUG)
    VERBOSE_VERBOSE("Home Offset Value(Current): %d\n", nRet);
#endif
    // WriteIndex: 607C, SubIndex: 0, Name: Home offset
    //nRet = ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x607C, 0, nHomeOffsetVal, LEN_DINT_BYTE);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;
    ECAT_DWORD  dwHomeOffsetVal;

    dwHomeOffsetVal = (ECAT_DWORD) nHomeOffsetVal;
    dwLength = sizeof(dwHomeOffsetVal);
	erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x607C,
                                     0,
			                         1, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         (ECAT_BYTE*) &dwHomeOffsetVal,
			                         &dwLength,
                                     LEN_DINT_BYTE);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif

    VERBOSE_VERBOSE("Internal Val: %d, HomeOffset Val: %d -> %d\n",
                     g_nInternalPulse[nAxis], noldHomeOffsetVal, nHomeOffsetVal);

    if(g_fAxisDebugMsg == TRUE)
    {
        nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x607C, 0, nReadBuff);

        VERBOSE_VERBOSE("Home Offset Value(New): %d\n", nRet);
    }

#if defined (__QNXNTO__)
    return erResult;
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ABSEncoderReset()
// -nAxis: index of axis(0~5)
//

#define RETRY_CNT_READ_OBJECT   5
#define RETRY_CNT_READ_POSVAL   4
//#define RETRY_WAIT_TIME_MS      30
#define RETRY_WAIT_TIME_MS      200

int ECATSERV_ABSEncoderReset(int nAxis)
{
    int nRet = 1;

    int nCnt = 0;
    int nReadBuff = 0;
    BYTE rgszCommnad[OBJECT_ENCRESET_CHAR_LEN];

    /* 1. Set the request code */
    rgszCommnad[0]  = 0x00;   // reserved
    rgszCommnad[1]  = 0x00;   // reserved

    rgszCommnad[2]  = 0x01;   // CCMD 0: read 1: write
    rgszCommnad[3]  = 0x02;   // CSIZE: CDATA Length in byte

    rgszCommnad[4]  = 0x00;   // CADDRESS: Address
    rgszCommnad[5]  = 0x20;
    rgszCommnad[6]  = 0x00;
    rgszCommnad[7]  = 0x00;

    rgszCommnad[8]  = 0x08;   // CDATA: Writing data
    rgszCommnad[9]  = 0x10;
    rgszCommnad[10] = 0x00;
    rgszCommnad[11] = 0x00;

    nRet = ECATLIB_WriteStringSlaveCoEObject(nAxis, 0x2710, 1, rgszCommnad, OBJECT_ENCRESET_CHAR_LEN);
    //VERBOSE_VERBOSE("Step1 SendData: %d\n", (int) rgszCommnad);
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

READ_OBJECT_STEP1:
    nCnt++;
    nRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x2710, 2, nReadBuff);
    if(g_fAxisDebugMsg == TRUE)
    {
        VERBOSE_VERBOSE("Step1 Set request code Return: %d\n", nRet);
    }
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

    if(nRet != 1 && nCnt < RETRY_CNT_READ_OBJECT)
    {
        goto READ_OBJECT_STEP1;
    }
    if(nRet != 1 && nCnt >= RETRY_CNT_READ_OBJECT)
    {
        goto ABORT_EXEC;
    }
    
    nRet = 0;
    nCnt = 0;

    /* 2. Preparation process */
    rgszCommnad[0]  = 0x00;   // reserved
    rgszCommnad[1]  = 0x00;   // reserved

    rgszCommnad[2]  = 0x01;   // CCMD 0: read 1: write
    rgszCommnad[3]  = 0x02;   // CSIZE: CDATA Length in byte

    rgszCommnad[4]  = 0x01;   // CADDRESS: Address
    rgszCommnad[5]  = 0x20;
    rgszCommnad[6]  = 0x00;
    rgszCommnad[7]  = 0x00;

    rgszCommnad[8]  = 0x02;   // CDATA: Writing data
    rgszCommnad[9]  = 0x00;
    rgszCommnad[10] = 0x00;
    rgszCommnad[11] = 0x00;

    nRet = ECATLIB_WriteStringSlaveCoEObject(nAxis, 0x2710, 1, rgszCommnad, OBJECT_ENCRESET_CHAR_LEN);
    //VERBOSE_VERBOSE("Step2 SendData: %d\n", (int) rgszCommnad);
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

READ_OBJECT_STEP2:
    nCnt++;
    nRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x2710, 2, nReadBuff);
    if(g_fAxisDebugMsg == TRUE)
    {
        VERBOSE_VERBOSE("Step2 Preparation process Return: %d\n", nRet);
    }
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

    if(nRet != 1 && nCnt < RETRY_CNT_READ_OBJECT)
    {
        goto READ_OBJECT_STEP2;
    }
    if(nRet != 1 && nCnt >= RETRY_CNT_READ_OBJECT)
    {
        goto ABORT_EXEC;
    }
    
    nRet = 0;
    nCnt = 0;

    /* 3. Execute requested adjustment */
    rgszCommnad[0]  = 0x00;   // reserved
    rgszCommnad[1]  = 0x00;   // reserved

    rgszCommnad[2]  = 0x01;   // CCMD 0: read 1: write
    rgszCommnad[3]  = 0x02;   // CSIZE: CDATA Length in byte

    rgszCommnad[4]  = 0x01;   // CADDRESS: Address
    rgszCommnad[5]  = 0x20;
    rgszCommnad[6]  = 0x00;
    rgszCommnad[7]  = 0x00;

    rgszCommnad[8]  = 0x01;   // CDATA: Writing data
    rgszCommnad[9]  = 0x00;
    rgszCommnad[10] = 0x00;
    rgszCommnad[11] = 0x00;

    nRet = ECATLIB_WriteStringSlaveCoEObject(nAxis, 0x2710, 1, rgszCommnad, OBJECT_ENCRESET_CHAR_LEN);
    //VERBOSE_VERBOSE("Step3 SendData: %d\n", (int) rgszCommnad);
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

READ_OBJECT_STEP3:
    nCnt++;
    nRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x2710, 2, nReadBuff);
    if(g_fAxisDebugMsg == TRUE)
    {
        VERBOSE_VERBOSE("Step3 Execute Return: %d\n", nRet);
    }
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

    if(nRet != 1 && nCnt < RETRY_CNT_READ_OBJECT)
    {
        goto READ_OBJECT_STEP3;
    }
    if(nRet != 1 && nCnt >= RETRY_CNT_READ_OBJECT)
    {
        goto ABORT_EXEC;
    }
    else
    {
        nCnt = 0;
READ_INTERNAL_POS:
        // Servo State set to switch on disable
        ECATSERV_ServoStateSetSwitchOnDisable(nAxis);

        // User Parameter set to new value
        ECATSERV_ServoUserParamReset(nAxis);

        nCnt++;
        THREAD_Sleep(200);

        nReadBuff = 1;
        g_nInternalPulse[nAxis] = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x6063, 0, nReadBuff);
        VERBOSE_MESSAGE("Internal Pulse: %d, Curr Pulse: %d\n",
                        g_nInternalPulse[nAxis], g_nAct_Pulse[nAxis]);

        if((g_nInternalPulse[nAxis] > g_rgnEncRes[nAxis] ||
            g_nInternalPulse[nAxis] < 0)
           && nCnt < RETRY_CNT_READ_POSVAL)
        {
            goto READ_INTERNAL_POS;
        }

        if((g_nInternalPulse[nAxis] > g_rgnEncRes[nAxis] ||
            g_nInternalPulse[nAxis] < 0)
            && nCnt >= RETRY_CNT_READ_POSVAL)
        {
            VERBOSE_WARNING("Problem: Axis%d Reset position value!\n", nAxis);
            //goto ABORT_EXEC;
        }

        return RESULT_OK;
    }
    
    nRet = 0;
    nCnt = 0;

ABORT_EXEC:
    /* 4. Abort the execution */
    rgszCommnad[0]  = 0x00;   // reserved
    rgszCommnad[1]  = 0x00;   // reserved

    rgszCommnad[2]  = 0x01;   // CCMD 0: read 1: write
    rgszCommnad[3]  = 0x02;   // CSIZE: CDATA Length in byte

    rgszCommnad[4]  = 0x00;   // CADDRESS: Address
    rgszCommnad[5]  = 0x20;
    rgszCommnad[6]  = 0x00;
    rgszCommnad[7]  = 0x00;

    rgszCommnad[8]  = 0x00;   // CDATA: Writing data
    rgszCommnad[9]  = 0x00;
    rgszCommnad[10] = 0x00;
    rgszCommnad[11] = 0x00;

    nRet = ECATLIB_WriteStringSlaveCoEObject(nAxis, 0x2710, 1, rgszCommnad, OBJECT_ENCRESET_CHAR_LEN);
    THREAD_Sleep(RETRY_WAIT_TIME_MS);

    VERBOSE_WARNING("Step4 Axis%d Enc Reset Abort Execution!\n", nAxis);

    return RESULT_ERROR;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ServoUserParamReset()
// -nAxis: index of axis(0~5)
//
#define RETRY_CNT_READ_USER_PARAM_RESET        10
ECAT_DWORD ECATSERV_ServoUserParamReset(int nAxis)
{
    ECAT_DWORD dwRet = 1;
    ECAT_DWORD dwReadBuff = 1;
    int nCnt = 0;

    // WriteIndex: 2700, SubIndex: 0, Name: User Parameter Configuration
    ECATLIB_WriteIntegerSlaveCoEObject(nAxis, 0x2700, 0, 1);

READ_OBJECT:
    dwRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x2700, 0, dwReadBuff);
#if defined(_DUBUG)
    VERBOSE_VERBOSE("User Parameter Set Result: %ld\n", dwRet);
#endif
    THREAD_Sleep(200);
    nCnt++;

    // return 0(zero) is succeeded!
    if(dwRet != 0 && nCnt < RETRY_CNT_READ_USER_PARAM_RESET)
    {
        goto READ_OBJECT;
    }
    else if(dwRet != 0 && nCnt >= RETRY_CNT_READ_USER_PARAM_RESET)
    {
        VERBOSE_ERROR("User Parameter Config Set Fail!\n");
        return RESULT_ERROR;
    }

    if(g_fAxisDebugMsg == TRUE)
    {
        VERBOSE_VERBOSE("User Parameter Config Set Done!\n");
    }

    return dwRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ControlModeSet()
// -nAxis: index of axis(0~5)
// -nMode: index of mode(1~10)
// -nOpt : 1 -> quite or not
//
int ECATSERV_ControlModeSet(int nAxis, int nMode, int nOpt)
{
    ECAT_DWORD dwRet = 1;
    ECAT_DWORD dwReadBuff = 1;
    
    // WriteIndex: 6061, SubIndex: 0, Name: Modes of Operation Display
    dwRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x6061, 0, dwReadBuff);
    if(nOpt != 1)
    {
        VERBOSE_VERBOSE("[Axis %d] Current Control Mode: %ld\n", nAxis, dwRet);
    }

    // WriteIndex: 6060, SubIndex: 0, Name: Modes of Operation
    //ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x6060, 0, nMode, LEN_SINT_BYTE);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;
    ECAT_BYTE   byMode;

    byMode = (ECAT_BYTE) nMode;
    dwLength = sizeof(byMode);
	erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x6060,
                                     0,
			                         1, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         &byMode,
			                         &dwLength,
                                     LEN_SINT_BYTE);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif

    if(nMode == PROFILED_POS_MODE || nMode == INTERPOLATED_POS_MODE)
    {
        ECATSERV_WriteControlWord(nAxis, 0x10, EACH);
        //ECATSERV_WriteControlWord(nAxis, 0x48, EACH);
    }
    
    if(nOpt != 1)
    {
        VERBOSE_VERBOSE("[Axis %d] Set Control Mode: %d\n", nAxis, nMode);
    }

    DANDY_SLEEP(100);
    
    // WriteIndex: 6061, SubIndex: 0, Name: Modes of Operation Display
    dwRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x6061, 0, dwReadBuff);
    VERBOSE_MESSAGE("[Axis %d] Confirmed Control Mode: %ld\n", nAxis, dwRet);

    g_nModeOperation = nMode;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ControlModeRead()
// -nAxis: index of axis(0~5)
//
#define SOFTWAREVER_CHAR_LEN        50

int ECATSERV_ControlModeRead(int nAxis)
{
    ECAT_DWORD dwRet = 1;
    ECAT_DWORD dwReadBuff = 1;
    ECAT_BYTE rgszSoftwareVer[SOFTWAREVER_CHAR_LEN];
    
    // WriteIndex: 0x100a, SubIndex: 0, Name: Manufacturer Software Version
    //rgszSoftwareVer = ECATLIB_ReadSlaveCoEObjectString(nAxis, 0x100a, 0, rgszSoftwareVer, SOFTWAREVER_CHAR_LEN);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;

    dwLength = sizeof(rgszSoftwareVer);

    erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x100a,
                                     0,
			                         2, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         rgszSoftwareVer,
			                         &dwLength,
			                         SOFTWAREVER_CHAR_LEN);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif
    VERBOSE_VERBOSE("[Axis %d] Software Version: %s\n", nAxis, rgszSoftwareVer);

    // WriteIndex: 6061, SubIndex: 0, Name: Modes of Operation Display
    dwRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x6061, 0, dwReadBuff);
    VERBOSE_VERBOSE("[Axis %d] Current Control Mode: %ld\n", nAxis, dwRet);

    // WriteIndex: 6502, SubIndex: 0, Name: Supported Drive Modes
    dwRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x6502, 0, dwReadBuff);
    VERBOSE_VERBOSE("[Axis %d] Drive Control Mode: %x\n", nAxis, (unsigned int) dwRet);

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_SetInterpolationTimePeriod()
// -nAxis: index of axis(0~5)
// -nTimePeriod: 1 ~ 250
// -nOpt : 1 -> quite or not
//
int ECATSERV_SetInterpolationTimePeriod(int nAxis, int nTimePeriod, int nOpt)
{
    int nRet = 1;
    int nReadBuff = 1;

    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);
    DANDY_SLEEP(50);
#if defined(__DEBUG)
    // ReadIndex: 60C2, SubIndex: 1, Name: Interpolation Time Period
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x60C2, 1, nReadBuff);
    VERBOSE_VERBOSE("[Axis %d] Current Interpolation Time Period: %d\n", nAxis, nRet);
#endif
    // WriteIndex: 60C2, SubIndex: 1, Name: Interpolation Time Period
    //ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x60C2, 1, nTimePeriod, LEN_USINT_BYTE);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;
    ECAT_BYTE   byTimePeriod;

    byTimePeriod = (ECAT_BYTE) nTimePeriod;
    dwLength = sizeof(byTimePeriod);
	erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x60C2,
                                     1,
			                         1, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         &byTimePeriod,
			                         &dwLength,
                                     LEN_USINT_BYTE);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif
    //VERBOSE_VERBOSE("[Axis %d] Set Interpolation Time Period: %d\n", nAxis, nTimePeriod);

    // ReadIndex: 60C2, SubIndex: 1, Name: Interpolation Time Period
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x60C2, 1, nReadBuff);

    if(nOpt != 1)
    {
        VERBOSE_MESSAGE("[Axis %d] Confirmed Interpolation Time Period: %d\n", nAxis, nRet);
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_SetInterpolationTimeIndex()
// -nAxis: index of axis(0~5)
// -nTimeIndex: -6 ~ 3
// -nOpt : 1 -> quite or not
//
int ECATSERV_SetInterpolationTimeIndex(int nAxis, int nTimeIndex, int nOpt)
{
    int nRet = 1;
    int nReadBuff = 1;
    
    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);
    DANDY_SLEEP(50);
#if defined(__DEBUG)
    // ReadIndex: 60C2, SubIndex: 2, Name: Interpolation Time Index
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x60C2, 2, nReadBuff);
    VERBOSE_VERBOSE("[Axis %d] Current Interpolation Time Index: %d\n", nAxis, nRet);
#endif
    // WriteIndex: 60C2, SubIndex: 2, Name: Interpolation Time Index
    //ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x60C2, 2, nTimeIndex, LEN_SINT_BYTE);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;
    ECAT_BYTE   byTimeIndex;

    byTimeIndex = (ECAT_BYTE) nTimeIndex;
    dwLength = sizeof(byTimeIndex);
	erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x60C2,
                                     2,
			                         1, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         &byTimeIndex,
			                         &dwLength,
                                     LEN_SINT_BYTE);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif

    //VERBOSE_VERBOSE("[Axis %d] Set Interpolation Time Index: %d\n", nAxis, nTimeIndex);

    // ReadIndex: 60C2, SubIndex: 2, Name: Interpolation Time Index
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x60C2, 2, nReadBuff);
    
    if(nOpt != 1)
    {
        VERBOSE_MESSAGE("[Axis %d] Confirmed Interpolation Time Index: %d\n", nAxis, nRet);
    }

    return RESULT_OK;
}



////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_SetSyncErrorCountLimit()
// -nAxis: index of axis(0~5)
// -nCountLimit: 0 ~ 15
int ECATSERV_SetSyncErrorCountLimit(int nAxis, int nCountLimit)
{
    int nRet = 1;
    int nReadBuff = 1;
    
#if defined(__DEBUG)
    // ReadIndex: 10F1, SubIndex: 2, Name: Sync Error Count Limit
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x10F1, 2, nReadBuff);
    VERBOSE_VERBOSE("[Axis %d] Current Sync Error Count Limit: %d\n", nAxis, nRet);
#endif
    // WriteIndex: 10F1, SubIndex: 2, Name: Sync Error Count Limit
    //ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x10F1, 2, nCountLimit, LEN_UDINT_BYTE);
#if defined (__QNXNTO__)
    ECAT_RESULT erResult;
    ECAT_DWORD  dwLength;
    ECAT_DWORD   dwCountLimit;

    dwCountLimit = (ECAT_DWORD) nCountLimit;
    dwLength = sizeof(dwCountLimit);
	erResult = EcatIODevAddCoEObject(g_hMaster,
                                     nAxis,
                                     0x10F1,
                                     2,
			                         1, // upload (1) / download (2)
			                         COE_OPERATION_TIMEOUT,
			                         (ECAT_BYTE*) &dwCountLimit,
			                         &dwLength,
                                     LEN_UDINT_BYTE);

    if(erResult != 0)
        VERBOSE_ERROR("Write Result: %d\n", erResult);
#endif

    VERBOSE_VERBOSE("[Axis %d] Set Sync Error Count Limit: %d\n", nAxis, nCountLimit);
    
    // User Parameter set to new value
    ECATSERV_ServoUserParamReset(nAxis);
    
    // Store Parameters to EEPROM
    ECATSERV_StoreParameters(nAxis);

    // ReadIndex: 10F1, SubIndex: 2, Name: Sync Error Count Limit
    nRet = ECATLIB_ReadSlaveCoEObjectSignedInteger(nAxis, 0x10F1, 2, nReadBuff);
    VERBOSE_MESSAGE("[Axis %d] Confirmed Sync Error Count Limit: %d\n", nAxis, nRet);

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ServoStateSetSwitchOnDisable()
// -nAxis: index of axis(0~5)
//

int ECATSERV_ServoStateSetSwitchOnDisable(int nAxis)
{
#if defined (__QNXNTO__)
    int nRet;
    int nStatus;

    nRet = ECATSERV_WriteControlWord(nAxis, 0x00, EACH);
    SERV_GetServoState();

    nStatus = g_nReadStatusValue[nAxis] & SRVSTATE_CODE_SWITCHON_DISABLED;
	if (nRet != RESULT_OK)
	{
		VERBOSE_ERROR("Cannot set cmd(0x00) %d-th axis.\n", nAxis);
		return RESULT_ERROR;
	}
	else
    {
#if defined(_DUBUG)
	    VERBOSE_MESSAGE("Axis%d Switch On Disabled!(Status: %x, nStatus: %x)\n",
                        nAxis, g_nReadStatusValue[nAxis], nStatus);
#endif
    }

    return RESULT_OK;
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_StoreParameters()
// -nAxis: index of axis(0~5)
// -Store Parameters to EEPROM
#define RETRY_CNT_READ_STORE_PARAM          5

int ECATSERV_StoreParameters(int nAxis)
{
    int nRet;
    ECAT_DWORD dwReadBuff = 1;
    ECAT_BYTE rgpszSaveCmd[] = {'s', 'a', 'v', 'e'};
    int nCnt = 0;
    
    // Check subindex supported(Subindex: 4)
    nRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x1010, 0, dwReadBuff);
#if defined(_DUBUG)
    VERBOSE_VERBOSE("Read Supported SubIndex: %x\n", nRet);
#endif
    // WriteIndex: 1010, SubIndex: 3, Name: Store Parameters
    ECATLIB_WriteStringSlaveCoEObject(nAxis, 0x1010, 3, rgpszSaveCmd, OBJECT_STORE_CHAR_LEN);
#if defined(_DUBUG)
    VERBOSE_VERBOSE("Cmd: %s(%x)\n", rgpszSaveCmd, (unsigned)*(unsigned*)rgpszSaveCmd);
#endif
READ_OBJECT:
    nRet = ECATLIB_ReadSlaveCoEObjectInteger(nAxis, 0x1010, 3, dwReadBuff);
#if defined(_DUBUG)
    VERBOSE_VERBOSE("Store Parameter Result: %x\n", nRet);
#endif
    THREAD_Sleep(200);
    nCnt++;

    // return 1 is succeeded!
    if(nRet != 1 && nCnt < RETRY_CNT_READ_STORE_PARAM)
    {
        goto READ_OBJECT;
    }
    else if(nRet != 1 && nCnt >= RETRY_CNT_READ_STORE_PARAM)
    {
        VERBOSE_ERROR("Store Parameter Fail!\n");
        return RESULT_ERROR;
    }

    VERBOSE_VERBOSE("Store Parameter Done!\n");

    return nRet;
}
