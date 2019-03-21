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



////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_QuickStop()
//
// -nAxis: index of axis
//
int ECATSERV_QuickStop(void)
{
    int nRet;

    nRet = ECATSERV_WriteControlWord(ALL_AXES, 0x02, ALL);
    THREAD_Sleep(10);
    //ECATSERV_ServoOff(ALL_AXES, ALL);
    if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        SERV_ServoOnCmd(OFF);
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ServoOn()
//
// -nAxis    : index of axis (0 ~ 5)
// -fAllAxis : flag for all axis write or not
#define SERVO_ON_DELAY_TIME1_MS      5   //5  //org: 8
#define SERVO_ON_DELAY_TIME2_MS      40   //10 //org: 50
//#define SERVO_ON_BRAKE_DELAY_TIME    70

int ECATSERV_ServoOn(int nAxis, int fAllAxis)
{
//#if defined (__QNXNTO__)
#if 1
    int nRet[ROB_AXIS_COUNT];
    int iAxis;
    int nCnt = 0;
    int nStartAxis = 0;
    int nEndAxis   = 0;
    int nOpt = 0;

	// check servo on/off state
	if (g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
	{
		VERBOSE_WARNING("Motor is already SERVO-ON status(Axis %d).\n", nAxis);
		return RESULT_OK;
	}

    // check master instance state
	if (g_hMaster == ECAT_NULL)
	{
		VERBOSE_WARNING("Not ready to use Master.\n");
		return RESULT_ERROR;
	}

	// check alarm code
    ECATSERV_GetServoAlarmCode();

	for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
	{
		if (g_nErrCodeServo[iAxis] != 0)
		{
			VERBOSE_WARNING("Cannot servo-on. Check alarm code.\n");
			return RESULT_ERROR;
		}
	}
    
    // read actual position
#if 0
    if (g_pShmem_sc != NULL)
    {
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            // write actual pos. to SC_SHM
            FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
            g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
            g_pShmem_sc->outputcmd.dbTrgPos[iAxis] = g_dbAct_Pos[iAxis];
            g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];
        }
    }
#endif
    FUNC_SyncActualPosToTargetPos();

    if(fAllAxis == ALL)
    {
        nStartAxis = 0;
        nEndAxis   = g_nAxisCount;
        nOpt = ALL;
    }
    else if(fAllAxis == EACH)
    {
        nStartAxis = nAxis;
        nEndAxis   = nAxis + 1;
        nOpt = EACH;
    }

    // write actual position
    ECATSERV_WriteTargetPosition(nAxis, ALL);

    // set control word : 6 (shutdown)    
    ECATSERV_WriteControlWord(nAxis, 0x06, nOpt);
    THREAD_Sleep(g_nServoOnBrakeDelayTime);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
SHUTDOWN:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_ON_DELAY_TIME1_MS);

        if(nRet[iAxis] == RESULT_ERROR ||
           (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2))
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x06, EACH);
        }

        nCnt++;

        if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2 &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF) && nCnt < 50)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Shutdown %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto SHUTDOWN;
        }
        else if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2 &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF) && nCnt >= 50)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Shutdown Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            nCnt = 0;
        }
    }
        
    // set control word : 7 (switch on)
    ECATSERV_WriteControlWord(nAxis, 0x07, nOpt);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
SWITCH_ON:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_ON_DELAY_TIME1_MS);

        if(nRet[iAxis] == RESULT_ERROR ||
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON)
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x07, EACH);
        }

        if(nRet[iAxis] == RESULT_ERROR ||
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON)
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x07, EACH);
        }

        nCnt++;

        if(g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON && nCnt < 50)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Switch On %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto SWITCH_ON;
        }
        else if(g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON && nCnt >= 50)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Switch On Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            nCnt = 0;
        }
    }
    
    // set control word : 15 (enable operation)
    ECATSERV_WriteControlWord(nAxis, 0x0f, nOpt);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
ENABLE_OPERATION:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_ON_DELAY_TIME2_MS);
        
        if(nRet[iAxis] == RESULT_ERROR ||
          (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_R &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR2))
        {
            THREAD_Sleep(SERVO_ON_DELAY_TIME1_MS);
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x0f, EACH);
#if 0   // for interpolated position mode
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, (0x0f | 0x10), EACH);
#endif
        }

        nCnt++;

        if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_R &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR2) && nCnt < 100)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Servo On %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto ENABLE_OPERATION;
        }
        else if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_R &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR2) && nCnt >= 100)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Servo On Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_MESSAGE("Servo On Done %d-th axis(%x).\n",
                                iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }

        g_nServoOnCmdState[iAxis] = nRet[iAxis];
    }

    return g_nServoOnCmdState[ROB_AXIS_COUNT];
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ServoOff()
// -nAxis    : index of axis (0 ~ 5)
// -fAllAxis : flag for all axis write or not
#define SERVO_OFF_DELAY_TIME_MS      5   //org: 1
//#define SERVO_OFF_BRAKE_DELAY_TIME   150

int ECATSERV_ServoOff(int nAxis, int fAllAxis)
{
//#if defined (__QNXNTO__)
#if 1
	int nRet[ROB_AXIS_COUNT];
    int nCnt = 0;
    int nStartAxis = 0;
    int nEndAxis   = 0;
    int iAxis;
    int nOpt = 0;
    
#if 1
	// check servo state
	if (g_pShmem_sc->outputstate.fServoOnOutState == FALSE)
	{
		VERBOSE_WARNING("Motor is already SERVO-OFF status(Axis %d).\n", nAxis);
		return RESULT_OK;
	}
#endif

    if(fAllAxis == ALL)
    {
        nStartAxis = 0;
        nEndAxis   = g_nAxisCount;
        nOpt = ALL;
    }
    else if(fAllAxis == EACH)
    {
        nStartAxis = nAxis;
        nEndAxis   = nAxis + 1;
        nOpt = EACH;
    }

    // set control word : 7 (disable operation)
    ECATSERV_WriteControlWord(nAxis, 0x07, nOpt);
    THREAD_Sleep(g_nServoOffBrakeDelayTime);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
DISABLE_OPERATION:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_OFF_DELAY_TIME_MS);
        if(g_fAxisDebugMsg == TRUE)
        {
            //VERBOSE_VERBOSE("Current Status Word: %d\t", g_nReadStatusValue[iAxis]);
        }

        if(nRet[iAxis] == RESULT_ERROR ||
          (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2))
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x07, EACH);
        }

        nCnt++;

        if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2) && nCnt < 50)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Disable Operation %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto DISABLE_OPERATION;
        }
        else if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2) && nCnt >= 50)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Disable Operation Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            nCnt = 0;
        }
    }

    // set control word : 6 (shutdown)
    ECATSERV_WriteControlWord(nAxis, 0x06, nOpt);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
SHUTDOWN:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_OFF_DELAY_TIME_MS);

        if(nRet[iAxis] == RESULT_ERROR ||
          (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2))
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x06, EACH);
        }
	
        nCnt++;

        if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2) && nCnt < 50)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Shutdown %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto SHUTDOWN;
        }
        else if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR1 &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR2) && nCnt >= 50)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Shutdown Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            nCnt = 0;
        }
    }

    // set control word : 0 (disable voltage)
    ECATSERV_WriteControlWord(nAxis, 0x00, nOpt);

    for(iAxis = nStartAxis; iAxis < nEndAxis; iAxis++)
    {
DISABLE_VOLTAGE:
        ECATSERV_ReadStatus(iAxis, EACH);
        THREAD_Sleep(SERVO_OFF_DELAY_TIME_MS);

        if(nRet[iAxis] == RESULT_ERROR ||
          (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
           g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP))
        {
            nRet[iAxis] = ECATSERV_WriteControlWord(iAxis, 0x00, EACH);
        }

        nCnt++;

        if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
            g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP) && nCnt < 100)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_WARNING("Retry Servo Off %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nRet[iAxis] = RESULT_ERROR;
            goto DISABLE_VOLTAGE;
        }
        else if((g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
                 g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP) && nCnt >= 100)
        {
            nRet[iAxis] = RESULT_ERROR;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_ERROR("Servo Off Fail %d-th axis(%x).\n",
                              iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }
        else
        {
            nRet[iAxis] = RESULT_OK;
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_MESSAGE("Servo Off Done %d-th axis(%x).\n",
                                iAxis, g_nReadStatusValue[iAxis]);    //for test
            }
            nCnt = 0;
        }

        g_nServoOnCmdState[iAxis] = nRet[iAxis];
    }

    return g_nServoOnCmdState[ROB_AXIS_COUNT];
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_AlarmClear()
//
int ECATSERV_AlarmClear(void)
{
    int iAxis;
    int nReadBuff = 1;
    int nCurrCtrlWord[ROB_AXIS_COUNT];

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nCurrCtrlWord[iAxis] = ECATLIB_ReadSlaveCoEObjectSignedInteger(iAxis, 0x6040, 0, nReadBuff);
        if(g_fAxisDebugMsg == TRUE)
        {
            VERBOSE_VERBOSE("[Axis%d] Current Control Word: %4x\n",
                            iAxis, nCurrCtrlWord[iAxis]);
        }

        // WriteIndex: 6040h, SubIndex: 0, Name: ControlWord
#if defined (__QNXNTO__)
#if 1
        ECAT_RESULT erResult;
        ECAT_DWORD  dwLength;
        ECAT_DWORD  dwCmdCtrl;

        dwCmdCtrl = (ECAT_DWORD) (nCurrCtrlWord[iAxis] | 0x80);
        dwLength = sizeof(nCurrCtrlWord[iAxis]);
	    erResult = EcatIODevAddCoEObject(g_hMaster,
                                         iAxis,
                                         0x6040,
                                         0,
	    		                         1, // upload (1) / download (2)
	    		                         COE_OPERATION_TIMEOUT,
	    		                         (ECAT_BYTE*) &dwCmdCtrl,
	    		                         &dwLength,
                                         LEN_DINT_BYTE);

        if(erResult != 0)
        {
            VERBOSE_ERROR("Write Result: %d\n", erResult);
        }
#endif
#endif
    }

#if defined (__QNXNTO__)

	int nRet = 0;

	// set cmd : 128 (0x80)
    nRet = ECATSERV_WriteControlWord(ALL_AXES, 0x80, ALL);

	if (nRet != RESULT_OK)
	{
		VERBOSE_ERROR("Cannot clear servo error.\n");
		return RESULT_ERROR;
	}

    return RESULT_OK;
#else      
    return RESULT_OK;
#endif
}

