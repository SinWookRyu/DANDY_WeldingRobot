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


char* g_pszEcatConfigDir      = DEF_ECAT_CONFIG_FILE_NAME;  // ECAT Config Dir
int g_nAxisCount              = DEF_AXIS_COUNT;             // Axis Count
int g_nSlaveCount             = DEF_SLAVE_COUNT;            // Slave Count

int g_nWriteInitOffsetSize    = DEF_WRITE_INIT_OFFSET_SIZE; // BeckHoff output size
int g_nWriteEcatDataSize      = DEF_WRITE_ECAT_SRVDATA_SIZE;
int g_nWriteOffsetControlWord = DEF_WRITE_OFFSET_CONTROLWORD;
int g_nWirteOffsetPosition    = DEF_WRITE_OFFSET_POSITION;
int g_nWirteOffsetPhysicalOutput = DEF_WRITE_OFFSET_OUTPUT;

int g_nReadInitOffsetSize     = DEF_READ_INIT_OFFSET_SIZE;  // BeckHoff input size
int g_nReadEcatDataSize       = DEF_READ_ECAT_SRVDATA_SIZE;
int g_nReadOffsetStatus       = DEF_READ_OFFSET_STATUS;
int g_nReadOffsetPosition     = DEF_READ_OFFSET_POSITION;
int g_nReadOffsetError        = DEF_READ_OFFSET_ERROR;

#define SIZEOF_TOTAL_INPUT   ((ROB_AXIS_COUNT*(READ_STATUS_SIZE+READ_POSITION_SIZE+READ_ERRORCODE_SIZE)) + \
                              (ROBOT_DI_SLAVE_COUNT * SLAVE_DI_PORT_COUNT * ECAT_READ_DI_PORT_SIZE) + \
                              (ROBOT_AI_SLAVE_COUNT * SLAVE_AI_PORT_COUNT * ECAT_READ_AI_PORT_SIZE))

#define SIZEOF_TOTAL_OUTPUT   ((ROB_AXIS_COUNT*(WRITE_CONTROLWORD_SIZE+WRITE_POSITION_SIZE+WRITE_PHYSIC_OUTPUT_SIZE)) + \
                               (ROBOT_DO_SLAVE_COUNT * SLAVE_DO_PORT_COUNT * ECAT_WRITE_DO_PORT_SIZE) + \
                               (ROBOT_AO_SLAVE_COUNT * SLAVE_AO_PORT_COUNT * ECAT_WRITE_AO_PORT_SIZE))

ECAT_BYTE   g_byInputs[SIZEOF_TOTAL_INPUT];
ECAT_WORD   g_wInputsSize;
ECAT_BYTE   g_byOutputs[SIZEOF_TOTAL_OUTPUT];
ECAT_WORD   g_wOutputsSize;
int         g_nResultStartRead;
int         g_nResultStartWrite;

ECAT_INT16  g_nAoutWriteBuff[ROBOT_AO_SLAVE_COUNT][SLAVE_AO_PORT_COUNT];
ECAT_INT16  g_nAinReadBuff[ROBOT_AI_SLAVE_COUNT][SLAVE_AI_PORT_COUNT];

////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteTargetPosition()
//

int ECATSERV_WriteTargetPosition(int nAxis, int fAllAxis)
{
    int iAxis;

    if(fAllAxis == ALL)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            FUNC_ConvertPosToPulse(iAxis, TARGET_POS_IDX);
        }
    }
    else if(fAllAxis == EACH)
    {
        FUNC_ConvertPosToPulse(nAxis, TARGET_POS_IDX);
    }

#if defined (_WIN32)
    // write actual pos. value
    FUNC_ConvertPulseToPos(nAxis, TARGET_POS_IDX);
    g_pShmem_sc->inputstate.dbActPos[nAxis] = g_pShmem_sc->outputcmd.dbTrgPos[nAxis];    
#endif

	return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteControlWord()
//
// -wValue : value of control command
// -nAxis  : index of axis (0 ~ 5)
// -fAllAxis : flag for all axis write or not

int ECATSERV_WriteControlWord(int nAxis, ECAT_WORD wValue, int fAllAxis)
{
    int iAxis;

    if(fAllAxis == ALL)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            g_nCtrlWord[iAxis] = wValue;
        }
    }
    else if(fAllAxis == EACH)
    {
        g_nCtrlWord[nAxis] = wValue;
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WritePhysicalOutput()
//
// -wValue : value of control command(switch off: 0, swtich on: 1)
// -nAxis  : index of axis (0 ~ 5)
// -fAllAxis : flag for all axis write or not
//

int ECATSERV_WritePhysicalOutput(int nAxis, ECAT_WORD wValue, int fAllAxis)
{
    int iAxis;

    if(fAllAxis == ALL)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(wValue == ON)
            {
	            g_nPhysicalOut[iAxis] = 0x70000;
            }
            else if(wValue == OFF)
            {
                g_nPhysicalOut[iAxis] = 0x00;
            }
        }
    }
    else if(fAllAxis == EACH)
    {
        if(wValue == ON)
        {
	        g_nPhysicalOut[nAxis] = 0x70000;
        }
        else if(wValue == OFF)
        {
            g_nPhysicalOut[nAxis] = 0x00;
        }
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ReadStatus()
//
// -nAxis    : index of axis
// -fAllAxis : flag for all axis read or not
//
#define SIZEOF_STATUS_INDATA     2 * READ_STATUS_SIZE * DEF_AXIS_COUNT

int ECATSERV_ReadStatus(int nAxis, int fAllAxis)
{
#if defined (__QNXNTO__)

#else
    if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        g_nReadStatusValue[nAxis] = SRVSTATE_CODE_SERVO_ON_R;
    }
    else
    {
        g_nReadStatusValue[nAxis] = SRVSTATE_CODE_SERVO_OFF;
    }
#endif
    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ReadPosition()
//
//

int ECATSERV_ReadPosition(void)
{
    int iAxis;

    for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_GetServoAlarmCode()
//
//

int ECATSERV_GetServoAlarmCode(void)
{
    //VERBOSE_VERBOSE("No alarm.\n");
    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteDigitalOut()
//
// -nSlave : n-th Slave (0~SLAVE COUNT)
// -nPort  : n-th Port  (0~7: 2088 Dout 8-bit output)
// -bValue : on(true) / off(false)
//

int ECATSERV_WriteDigitalOut(int nSlave, int nPort, ECAT_BOOL bValue)
{
    g_DoutPortVal[nSlave][nPort] = bValue;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteAnalogOut()
//
// -nSlave : n-th Slave (0~SLAVE COUNT)
// -nPort  : n-th Port  (0~1: 4132 Aout 2ch 16-bit output)
// -rValue : -10~10 Volt
//

int ECATSERV_WriteAnalogOut(int nSlave, int nPort, ECAT_REAL32 rValue)
{
    g_AoutPortVal[nSlave][nPort] = rValue;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_GetInputShadow()
//
int  ECATSERV_GetInputShadow(void)
{
#if defined (__QNXNTO__)
    g_wInputsSize = SIZEOF_TOTAL_INPUT;
    EcatIODevGetInputsShadow(g_hMaster, g_byInputs, g_wInputsSize);

    return RESULT_OK;
#else
    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_StartReadInputWriteOutput()
//
int  ECATSERV_StartReadInputWriteOutput(void)
{
#if defined (__QNXNTO__)
    g_wInputsSize = SIZEOF_TOTAL_INPUT;
    g_wOutputsSize = SIZEOF_TOTAL_OUTPUT;

    //EcatIODevGetOutputsShadow(g_hMaster, g_byOutputs, g_wOutputsSize);

    //EcatIODevGetInputsShadow(g_hMaster, g_byInputs, g_wInputsSize);

    if (ECAT_SUCCEEDED(EcatIODevStartReadInputs(g_hMaster,
                                                g_byInputs,
                                                g_wInputsSize)))
    {
        g_nResultStartRead = RESULT_OK;
    }
    else
    {
        VERBOSE_ERROR("Start Read Inputs Error!\n");
        g_nResultStartRead = RESULT_ERROR;
    }

    if (ECAT_SUCCEEDED(EcatIODevStartWriteOutputs(g_hMaster,
                                                  g_byOutputs,
                                                  g_wOutputsSize)))
    {
        g_nResultStartWrite = RESULT_OK;
    }
    else
    {
        VERBOSE_ERROR("Start Write Outputs Error!\n");
        g_nResultStartWrite = RESULT_ERROR;
    }
        return RESULT_OK;
    #else
        return RESULT_OK;
    #endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_DoneReadInputWriteOutput()
//
int  ECATSERV_DoneReadInputWriteOutput(void)
{
#if defined (__QNXNTO__)
    EcatIODevDoneWriteOutputs(g_hMaster, g_byOutputs, g_wOutputsSize);
#if 1
    EcatIODevDoneReadInputs(g_hMaster);
#endif
        return RESULT_OK;
    #else
        return RESULT_OK;
    #endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_ReadTotalInput()
//
int  ECATSERV_ReadTotalInput(void)
{
#if defined (__QNXNTO__)
    int         iAxis;
    int         nSlave, nPort;
    ECAT_WORD   wAddr = 0;

    if(g_pShmem_sc->sysstate.fEcatInitState == FALSE)
    {
        return RESULT_ERROR;
    }

    if (g_nResultStartRead == RESULT_OK)
    {
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            // status word
            wAddr = iAxis * g_nReadEcatDataSize + g_nReadOffsetStatus;
            if (ECAT_SUCCEEDED(EcatIODevGetVariable(g_byInputs,
                                                   (ECAT_PBYTE) &g_nReadStatusValue[iAxis],
                                                    READ_STATUS_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Get Variable Error[Status Value]!\n");
            }
            
            // actual position
            wAddr = iAxis * g_nReadEcatDataSize + g_nReadOffsetPosition;
            if (ECAT_SUCCEEDED(EcatIODevGetVariable(g_byInputs,
                                                   (ECAT_PBYTE) &g_nAct_Pulse[iAxis],
                                                    READ_POSITION_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Get Variable Error[Actual Position Value]!\n");
            }

            // error code
            wAddr = iAxis * g_nReadEcatDataSize + g_nReadOffsetError;
            if (ECAT_SUCCEEDED(EcatIODevGetVariable(g_byInputs,
                                                   (ECAT_PBYTE) &g_nErrCodeServo[iAxis],
                                                    READ_ERRORCODE_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Get Variable Error[Error Code Value]!\n");
            }

            FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
            
            // write actual position to SHM_SC
            g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
            
            //SC_reply.data.pos.dbCurrPos[iAxis] = g_dbAct_Pos[iAxis];
            if(g_pShmem_Status_te != NULL)
            {
                SC_reply.data.pos.dbCurrPos[iAxis] = g_pShmem_Status_te->xyzrpy_act[iAxis];
            }
		}
#if 1
        // Beckhoff Digital Input
        for(nSlave = 0; nSlave < ROBOT_DI_SLAVE_COUNT; nSlave++)
        {
            for(nPort = 0; nPort < SLAVE_DI_PORT_COUNT; nPort++)
            {
                if(nSlave == 0)
                {
		            wAddr = nPort * ECAT_READ_DI_PORT_SIZE + ECAT_READ_DI_OFFSET;
                }
                else if(nSlave == 1)
                {
		            wAddr = nPort * ECAT_READ_DI_PORT_SIZE + ECAT_READ_DI_OFFSET
                            + ECAT_READ_DI_SLAVE_SIZE;
                }
                else if(nSlave == 2)
                {
		            wAddr = nPort * ECAT_READ_DI_PORT_SIZE + ECAT_READ_DI_OFFSET
                            + 2 * ECAT_READ_DI_SLAVE_SIZE + ECAT_READ_AI_SALVE_SIZE;
                }
                else if(nSlave == 3)
                {
		            wAddr = nPort * ECAT_READ_DI_PORT_SIZE + ECAT_READ_CART_DI_OFFSET;
                }

                if (ECAT_SUCCEEDED(EcatIODevGetVariable(g_byInputs,
                                                       (ECAT_PBYTE) &g_DinPortVal[nSlave][nPort],
                                                        ECAT_READ_DI_PORT_SIZE,
                                                        wAddr)))
                {
                    ;
                }
                else
                {
                    VERBOSE_ERROR("Get Variable Error[DIO Value]!\n");
                }

                if(g_fArcOnSigHigh == ON || g_fArcOnVirtualProc == ON)
                {
                    g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO] = ON;
                }
            }
        }
        
        // Beckhoff Analog Input
        for(nSlave = 0; nSlave < ROBOT_AI_SLAVE_COUNT; nSlave++)
        {
            for(nPort = 0; nPort < SLAVE_AI_PORT_COUNT; nPort++)
            {
                if(nSlave == 0)
                {
		            wAddr = nPort * ECAT_READ_AI_PORT_SIZE + ECAT_READ_AI_OFFSET
                                     + ECAT_READ_AI_STATUS_SIZE;
                }

                if (ECAT_SUCCEEDED(EcatIODevGetVariable(g_byInputs,
                                                       (ECAT_PBYTE) &g_nAinReadBuff[nSlave][nPort],
                                                        ECAT_READ_AI_PORT_SIZE,
                                                        wAddr)))
                {
                    g_AinPortVal[nSlave][nPort] = ((ECAT_REAL32) g_nAinReadBuff[nSlave][nPort] *
                                                    g_dbAinMaxVolt)
                                                  / g_dbADCMaxBit;
                }
                else
                {
                    VERBOSE_ERROR("Get Variable Error[AIO Value]!\n");
                }
            }
        }
#endif        
        //EcatIODevUpdateProcessImage(g_hMaster);
    }
    else
    {
        VERBOSE_ERROR("Start Read Inputs Action Error!\n");
    }

	return RESULT_OK;
#else
    int iAxis;
    int iSlave, iPort;

    // for windows, virtual I/O set
    if(g_pShmem_sc != NULL)
    {
        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] == ON)
        {
            g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO] = ON;
        }

        if(g_fArcOnSigHigh == ON || g_fArcOnVirtualProc == ON)
        {
            g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO] = ON;
        }

        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(g_pShmem_sc->outputstate.fBrakeReleaseOutState[iAxis] == ON ||
               g_pShmem_sc->outputstate.fServoOnOutState == ON)
            {
                g_DinPortVal[BRAKESTATE_DI_SLAVE_NO][iAxis] = ON;
            }
            else
            {
                g_DinPortVal[BRAKESTATE_DI_SLAVE_NO][iAxis] = OFF;
            }

            g_nAct_Pulse[iAxis] = g_nTrg_Pulse[iAxis];

            FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
                    
            // write actual position to SHM_SC
            g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
        }

        //////////////////////////
        // Analog Input
        for(iSlave = 0; iSlave < ROBOT_AI_SLAVE_COUNT; iSlave++)
        {
            for(iPort = 0; iPort < SLAVE_AI_PORT_COUNT; iPort++)
            {
                g_AinPortVal[iSlave][iPort] = g_AoutPortVal[iSlave][iPort];
            }
        }
    }

    return RESULT_OK;
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// ECATSERV_WriteTotalOutput()
//

int  ECATSERV_WriteTotalOutput(void)
{
#if defined (__QNXNTO__)
    int         iAxis;
    int         nSlave, nPort;
    ECAT_WORD   wAddr = 0;

    if(g_pShmem_sc->sysstate.fEcatInitState == FALSE)
    {
        return RESULT_ERROR;
    }

    if (g_nResultStartWrite == RESULT_OK)
    {
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            // get target position from SHM_SC
            if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
            {
                g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];
                FUNC_ConvertPosToPulse(iAxis, TARGET_POS_IDX);
            }

            // control word
            wAddr = iAxis * g_nWriteEcatDataSize + g_nWriteOffsetControlWord;
            if (ECAT_SUCCEEDED(EcatIODevSetVariable(g_byOutputs,
                                                   (ECAT_PBYTE) &g_nCtrlWord[iAxis],
                                                    WRITE_CONTROLWORD_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Set Variable Error[Control Word Value]!\n");
            }
            
            // target position
            wAddr = iAxis * g_nWriteEcatDataSize + g_nWirteOffsetPosition;
            if (ECAT_SUCCEEDED(EcatIODevSetVariable(g_byOutputs,
                                                   (ECAT_PBYTE) &g_nTrg_Pulse[iAxis],
                                                    WRITE_POSITION_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Set Variable Error[Target Position Value]!\n");
            }

            // physical output
            wAddr = iAxis * g_nWriteEcatDataSize + g_nWirteOffsetPhysicalOutput;
            if (ECAT_SUCCEEDED(EcatIODevSetVariable(g_byOutputs,
                                                   (ECAT_PBYTE) &g_nPhysicalOut[iAxis],
                                                    WRITE_PHYSIC_OUTPUT_SIZE,
                                                    wAddr )))
            {
                ;
            }
            else
            {
                VERBOSE_ERROR("Set Variable Error[Physical Output Value]!\n");
            }
		}
#if 1
        // Beckhoff Digital Output
        for(nSlave = 0; nSlave < ROBOT_DO_SLAVE_COUNT; nSlave++)
        {
            for(nPort = 0; nPort < SLAVE_DO_PORT_COUNT; nPort++)
            {
                if(g_fArcOnVirtualProc == ON)
                {
                    g_DoutPortVal[WELD_DO_SLAVE_NO][ARCON_DO_PORT_NO] = OFF;
                }

                if(nSlave == 0)
                {
		            wAddr = nPort * ECAT_WRITE_DO_PORT_SIZE + ECAT_WRITE_DO_OFFSET;
                }
                else if(nSlave == 1)
                {
		            wAddr = nPort * ECAT_WRITE_DO_PORT_SIZE + ECAT_WRITE_DO_OFFSET
                            + ECAT_WRITE_DO_SLAVE_SIZE;
                }
                else if(nSlave == 2)
                {
		            wAddr = nPort * ECAT_WRITE_DO_PORT_SIZE + ECAT_WRITE_CART_DO_OFFSET;
                }
                
                if (ECAT_SUCCEEDED(EcatIODevSetVariable(g_byOutputs,
                                                       (ECAT_PBYTE) &g_DoutPortVal[nSlave][nPort],
                                                        ECAT_WRITE_DO_PORT_SIZE,
                                                        wAddr)))
                {
                    ;
                }
                else
                {
                    VERBOSE_ERROR("Set Variable Error[DIO Value]!\n");
                }
            }
        }
        
        // Beckhoff Analog Output
        for(nSlave = 0; nSlave < ROBOT_AO_SLAVE_COUNT; nSlave++)
        {
            for(nPort = 0; nPort < SLAVE_AO_PORT_COUNT; nPort++)
            {
#if 0
                g_nAoutWriteBuff[nSlave][nPort] = ((ANALOG_RAW_VAL_RES *
                                                    g_AoutPortVal[nSlave][nPort]) /
                                                    ANALOG_MAX_ABS_VOLTAGE);
#endif   
                if(g_fArcOnVirtualProc == ON)
                {
                    g_nAoutWriteBuff[nSlave][nPort] = 0;
                }

                if(nSlave == 0)
                {
		            wAddr = nPort * ECAT_WRITE_AO_PORT_SIZE + ECAT_WRITE_AO_OFFSET;
                }

                if (ECAT_SUCCEEDED(EcatIODevSetVariable(g_byOutputs,
                                                       (ECAT_PBYTE) &g_nAoutWriteBuff[nSlave][nPort],
                                                        ECAT_WRITE_AO_PORT_SIZE,
                                                        wAddr)))
                {
                    ;
                }
                else
                {
                    VERBOSE_ERROR("Set Variable Error[AIO Value]!\n");
                }
            }
        }
#endif        
        //EcatIODevUpdateProcessImage(g_hMaster);
    }
    else
    {
        VERBOSE_ERROR("Start Write Outputs Action Error!\n");
    }

	return RESULT_OK;
#else
    int         iAxis;    

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        // get target position from SHM_SC
        if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
        {
            g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];

            FUNC_ConvertPosToPulse(iAxis, TARGET_POS_IDX);
        }
    }

    return RESULT_OK;
#endif
}

