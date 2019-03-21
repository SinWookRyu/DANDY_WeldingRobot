#define _USE_MATH_DEFINES

#include "servocon_main.h"
#include "service.h"
#include <math.h>


////////////////////////////////////////////////////////////////////////////////
//
// global variable & function
//

// thread
THREAD_HANDLE hRuntimeThread;          // thread to receive timer pulse
THREAD_HANDLE hEcatTraceThread;        // thread to ethercat trace
THREAD_HANDLE hCheckTimeLimitThread;   // thread of receive time checking from RM
THREAD_HANDLE hScanButtonStateThread;  // thread to check button state
THREAD_HANDLE hScanServoStateThread;   // thread to monitoring E-stop state
THREAD_HANDLE hVGADisplayThread;       // thread to VGA Display
THREAD_HANDLE hServiceProcThread;      // thread to service receive & process
THREAD_HANDLE hScanInputStateThread;   // thread to check input state
THREAD_HANDLE hMappingInputThread;     // thread to mapping input state

BOOL g_fThreadRuntimeExitState      = FALSE; // flag of RcvTimerPulse thread exit
BOOL g_fThreadEcatTraceExitState    = FALSE; // flag of EcatTrace thread exit
BOOL g_fScanButtonThreadExitState   = FALSE; // flag of Button thread exit
BOOL g_fEStopStateThreadExitState   = FALSE; // flag of E-StopMon thread exit
BOOL g_fDisplayThreadExitState      = FALSE; // flag of Display thread exit
BOOL g_fServiceProcThreadExitState  = FALSE; // flag of Service thread exit
BOOL g_fScanSDOInputThreadExitState = FALSE; // flag of Input thread exit
BOOL g_fMappingInputThreadExitState = FALSE; // flag of Input mapping thread exit

BOOL g_fScanButtonThreadRunState    = FALSE;
BOOL g_fEStopStateThreadRunState    = FALSE;
BOOL g_fScanSDOInputThreadRunState  = FALSE;
BOOL g_fMappingInputThreadRunState  = FALSE;

int  g_fHWLimitOnState = OFF;           // flag for H/W Limit State
int  g_fAbsEncResetEventActive = OFF;   // flag for Abs Encoder Reset Event State
int  g_nEmergencyCodeServo[ROB_AXIS_COUNT];   // emergency code of motor
int  g_nErrAxis;
int  g_nTimerNanoRes;
ECAT_WORD               g_wEmergencyErrStateCode[ROB_AXIS_COUNT];
ECAT_DWORD              g_dwWrongWC;          // Wrong working counter
ECAT_WORD               g_wFramesPerSecond;   // Frame Count per sec.
ECAT_STATISTICS         g_EcatStatistics;

double g_rgdbWeldVoltInAvrVal[ADC_DATA_AVR_CNT];
double g_rgdbWeldCurrInAvrVal[ADC_DATA_AVR_CNT];
int g_nVoltGetIdx = 0;
int g_nCurrGetIdx = 0;

int g_nArcSensNodeIdx  = 0;
int g_nCntForAvrWeldIn = 0;

double g_dbWeldVoltInVal[ADC_DATA_AVR_CNT];
double g_dbWeldCurrInVal[ADC_DATA_AVR_CNT];
#if 0
double g_dbSaveActPos[ARC_ARRAY_SIZE][ROB_AXIS_COUNT];
double g_dbSaveTrgPos[ARC_ARRAY_SIZE][ROB_AXIS_COUNT];
#endif
void FUNC_ConvertPosToPulse(int nAxis, int nOpt);
void FUNC_ConvertPulseToPos(int nAxis, int nOpt);

static void _loc_FUNC_ScanServoState(void);
static void _loc_FUNC_ControlPannelLampControl(void);
static int  _loc_FUNC_UpdatePI_ALLInOut(void);

void FUNC_MapContInputToWeldInput(void);
void FUNC_MapWeldOutputCmdToContOutput(int nSlave, int nPort);


////////////////////////////////////////////////////////////////////////////////
//
// FUNC_ConvertPosToPulse()
//  - rad -> pulse
//
void FUNC_ConvertPosToPulse(int nAxis, int nOpt)
{
    if(nOpt == ACTUAL_POS_IDX)
        g_nAct_Pulse[nAxis] = (int)(g_rgnEcnoderHomeVal[nAxis] +
                              (g_rgnEncRes[nAxis]/(2 * M_PI)) * g_rgnAxisDirection[nAxis] *
                               g_rgdbGearRatio[nAxis] * g_dbAct_Pos[nAxis]);
    else if(nOpt == TARGET_POS_IDX)
        g_nTrg_Pulse[nAxis] = (int)(g_rgnEcnoderHomeVal[nAxis] +
                              (g_rgnEncRes[nAxis]/(2 * M_PI)) * g_rgnAxisDirection[nAxis] *
                               g_rgdbGearRatio[nAxis] * g_dbTrg_Pos[nAxis]);
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNC_ConvertPulseToPos()
//  - pulse -> rad
//
void FUNC_ConvertPulseToPos(int nAxis, int nOpt)
{
    if(nOpt == ACTUAL_POS_IDX)
        g_dbAct_Pos[nAxis] = (2 * M_PI / g_rgnEncRes[nAxis]) *
                             (g_nAct_Pulse[nAxis] - g_rgnEcnoderHomeVal[nAxis]) /
                             (g_rgnAxisDirection[nAxis] * g_rgdbGearRatio[nAxis]);
    else if(nOpt == TARGET_POS_IDX)
        g_dbTrg_Pos[nAxis] = (2 * M_PI / g_rgnEncRes[nAxis]) *
                             (g_nTrg_Pulse[nAxis] - g_rgnEcnoderHomeVal[nAxis]) /
                             (g_rgnAxisDirection[nAxis] * g_rgdbGearRatio[nAxis]);
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNC_MapContInputToWeldInput()
//  - controller raw data input -> weld current/voltage input
//

//#define NO_SCALE        1
#define     MIN_ADC_IDX_FOR_GET_WELDFEEDBACK    20
#define     MAX_ADC_IDX_FOR_GET_WELDFEEDBACK    120

void FUNC_MapContInputToWeldInput(void)
{
    int iCnt;
    double  dbTmpVoltage = 0;
    double  dbTmpCurrent = 0;

    g_nVoltGetIdx %= ADC_DATA_AVR_CNT;
    g_nCurrGetIdx %= ADC_DATA_AVR_CNT;
    
    if(g_pShmem_Task_te != NULL && g_pShmem_sc != NULL)
    {
        if(g_pShmem_Task_te->ADC_gathering_index < ARC_ARRAY_SIZE )
        {
            if (g_pShmem_Task_te->ADC_gathering_index >= MIN_ADC_IDX_FOR_GET_WELDFEEDBACK &&
                g_pShmem_Task_te->ADC_gathering_index <  MAX_ADC_IDX_FOR_GET_WELDFEEDBACK)
            {
                ++g_nCntForAvrWeldIn;

                if(g_nCntForAvrWeldIn > (MAX_ADC_IDX_FOR_GET_WELDFEEDBACK - MIN_ADC_IDX_FOR_GET_WELDFEEDBACK))
                {
                    g_nCntForAvrWeldIn = MAX_ADC_IDX_FOR_GET_WELDFEEDBACK - MIN_ADC_IDX_FOR_GET_WELDFEEDBACK;
                }

                g_dbWeldVoltInVal[g_nArcSensNodeIdx] += g_pShmem_sc->inputstate.dbWeldVoltInVal;
                g_dbWeldVoltInVal[g_nArcSensNodeIdx] = g_dbWeldVoltInVal[g_nArcSensNodeIdx] / g_nCntForAvrWeldIn;

                g_dbWeldCurrInVal[g_nArcSensNodeIdx] += g_pShmem_sc->inputstate.dbWeldCurrInVal;
                g_dbWeldCurrInVal[g_nArcSensNodeIdx] = g_dbWeldCurrInVal[g_nArcSensNodeIdx] / g_nCntForAvrWeldIn;
            }

            // Arc Sensor ADC data (16 bit data)
#if 1
            g_pShmem_Task_te->ADC_Data[(g_pShmem_Task_te->ADC_gathering_index)++] =
                (word) g_rgdbWeldCurrInAvrVal[g_nCurrGetIdx];
#endif
#if 0
             g_pShmem_Task_te->ADC_Data[(g_pShmem_Task_te->ADC_gathering_index)++] =
                (word) (g_pShmem_Status_te->dist_work * 1000);
#endif
#if 0
            // Arc Sensor Position data
            for(iCnt = 0; iCnt < g_nAxisCount; iCnt++)
            {
                g_dbSaveActPos[g_pShmem_Task_te->ADC_gathering_index][iCnt] =
                                                    g_dbAct_Pos[iCnt] * (180/M_PI);
                g_dbSaveTrgPos[g_pShmem_Task_te->ADC_gathering_index][iCnt] =
                                                    g_dbTrg_Pos[iCnt] * (180/M_PI);
            }
#endif
        }
    }

#if defined (__QNXNTO__)
    g_rgdbWeldVoltInAvrVal[g_nVoltGetIdx] =    //Unit: 16bit data
        g_nAinReadBuff[WELD_AI_SLAVE_NO][g_Ain_portno.nVoltageInPortNo];
    g_nVoltGetIdx++;

    g_rgdbWeldCurrInAvrVal[g_nCurrGetIdx] =   //Unit: 16bit data
        g_nAinReadBuff[WELD_AI_SLAVE_NO][g_Ain_portno.nCurrentInPortNo];
    g_nCurrGetIdx++;
#else
    // for window virtual data
    g_rgdbWeldVoltInAvrVal[g_nVoltGetIdx] =    //Unit: 16bit data
        g_nAoutWriteBuff[WELD_AI_SLAVE_NO][g_Aout_portno.nVoltageOutPortNo] * 2;
    g_nVoltGetIdx++;
    
    g_rgdbWeldCurrInAvrVal[g_nCurrGetIdx] =   //Unit: 16bit data
        g_nAoutWriteBuff[WELD_AI_SLAVE_NO][g_Aout_portno.nCurrentOutPortNo] * 2;
    g_nCurrGetIdx++;
#endif

    for(iCnt = 0; iCnt < ADC_DATA_AVR_CNT; iCnt++)
    {
        dbTmpVoltage += g_WeldTuneParam.input.dbVoltScale * g_rgdbWeldVoltInAvrVal[iCnt];
        dbTmpCurrent += g_WeldTuneParam.input.dbCurrScale * g_rgdbWeldCurrInAvrVal[iCnt];
    }
#if defined (__QNXNTO__)
    if(g_pShmem_sc != NULL)
    {
        g_pShmem_sc->inputstate.dbWeldVoltInVal =
           ((g_WeldTuneParam.input.dbVolt_a * (dbTmpVoltage / ADC_DATA_AVR_CNT)  +
             g_WeldTuneParam.input.dbVolt_b) - g_WeldTuneParam.input.dbVoltOffset);

        g_pShmem_sc->inputstate.dbWeldCurrInVal =
           ((g_WeldTuneParam.input.dbCurr_a * (dbTmpCurrent / ADC_DATA_AVR_CNT) +
             g_WeldTuneParam.input.dbCurr_b) - g_WeldTuneParam.input.dbCurrOffset);
    }
#else
    if(g_pShmem_sc != NULL)
    {
        g_pShmem_sc->inputstate.dbWeldVoltInVal =
                            g_pShmem_sc->outputstate.dbWeldVoltOutVal;

        g_pShmem_sc->inputstate.dbWeldCurrInVal =
                            g_pShmem_sc->outputstate.dbWeldCurrOutVal;
    }
#endif

    if(g_pShmem_Task_te != NULL)
    {
        if(g_pShmem_Task_te->ADC_gathering_request == ON)
        {
            g_pShmem_Task_te->ADC_gathering_index = 0;
            g_pShmem_Task_te->ADC_gathering_request = OFF;
            g_nCntForAvrWeldIn = 0;

            g_nArcSensNodeIdx++;

            if(g_nArcSensNodeIdx >= DEF_NODE_ADD)
            {
                g_nArcSensNodeIdx = 0;
            }
        }

        if(g_pShmem_Status_te->run_cmd_code == DANDY_JOB_CODE_ARCON)
        {
            g_nArcSensNodeIdx = 0;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNC_MapWeldOutputCmdToContOutput()
//  - weld current/voltage output cmd -> controller raw data output
//
void FUNC_MapWeldOutputCmdToContOutput(int nSlave, int nPort)
{
    double dbTmpVoltage_param_c, dbTmpCurrent_param_c;
    double dbDiscVolt = 0, dbDiscCurr = 0;     //discriminant
    double dbRatioToExtent;

    // Extent Board input: 0~5V
    // ANALOG_MAX_CMD_VOLTAGE / ANALOG_MAX_ABS_VOLTAGE
    dbRatioToExtent = 0.5;

    //dbAoutPortCmd: Command Unit(45V, 500A), g_AoutPortVal: Controller Command(5V)
    if(g_pShmem_sc != NULL)
    {
        // Volt out
        if(nPort == g_Aout_portno.nVoltageOutPortNo)
        {
            dbTmpVoltage_param_c =
                g_WeldTuneParam.output.dbVolt_c - 
                (g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] +
                 g_pShmem_sc->outputcmd.dbVoltRealTimeCmdOffset);

            dbDiscVolt = (g_WeldTuneParam.output.dbVolt_b * g_WeldTuneParam.output.dbVolt_b) -
                4* g_WeldTuneParam.output.dbVolt_a * dbTmpVoltage_param_c;

            if(dbDiscVolt < 0.0)
            {
                g_AoutPortVal[nSlave][nPort] = 0;
            }
            else
            {
                g_AoutPortVal[nSlave][nPort] = 
                    (float) (((-1 * g_WeldTuneParam.output.dbVolt_b) + sqrt(fabs(dbDiscVolt))) /
                            (2 * g_WeldTuneParam.output.dbVolt_a) * dbRatioToExtent);
                if(dbDiscVolt < 0.0)
                {
                    g_AoutPortVal[nSlave][nPort] = 0;
                }
                if(g_AoutPortVal[nSlave][nPort] <= 0.0)
                {
                    g_AoutPortVal[nSlave][nPort] = 0.;
                }
                else if(g_AoutPortVal[nSlave][nPort] >= ANALOG_MAX_CMD_VOLTAGE)
                {
                    g_AoutPortVal[nSlave][nPort] = 5.;
                }
            }
            
            g_pShmem_sc->outputstate.dbWeldVoltOutVal =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] +
                g_pShmem_sc->outputcmd.dbVoltRealTimeCmdOffset;

            g_nAoutWriteBuff[nSlave][nPort] =
                (ECAT_INT16) ((g_AoutPortVal[nSlave][nPort] - g_WeldTuneParam.output.dbVoltOffset) /
                               g_WeldTuneParam.output.dbVoltScale);
        }
        // Curr out
        else if(nPort == g_Aout_portno.nCurrentOutPortNo)
        {
            dbTmpCurrent_param_c =
                g_WeldTuneParam.output.dbCurr_c -
                (g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] +
                 g_pShmem_sc->outputcmd.dbCurrRealTimeCmdOffset);

            dbDiscCurr = (g_WeldTuneParam.output.dbCurr_b * g_WeldTuneParam.output.dbCurr_b) -
                4* g_WeldTuneParam.output.dbCurr_a * dbTmpCurrent_param_c;

            if(dbDiscCurr < 0.0)
            {
                g_AoutPortVal[nSlave][nPort] = 0;
            }
            else
            {
                g_AoutPortVal[nSlave][nPort] = 
                    (float) (((-1 * g_WeldTuneParam.output.dbCurr_b) + sqrt(fabs(dbDiscCurr))) /
                            (2 * g_WeldTuneParam.output.dbCurr_a) * dbRatioToExtent);
                
                if(dbDiscCurr < 0.0)
                {
                    g_AoutPortVal[nSlave][nPort] = 0;
                }
                if(g_AoutPortVal[nSlave][nPort] <= 0.0)
                {
                    g_AoutPortVal[nSlave][nPort] = 0.;
                }
                else if(g_AoutPortVal[nSlave][nPort] >= ANALOG_MAX_CMD_VOLTAGE)
                {
                    g_AoutPortVal[nSlave][nPort] = 5.;
                }
            }
            
            g_pShmem_sc->outputstate.dbWeldCurrOutVal =
                    g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] +
                    g_pShmem_sc->outputcmd.dbCurrRealTimeCmdOffset;

            g_nAoutWriteBuff[nSlave][nPort] =
                (ECAT_INT16) ((g_AoutPortVal[nSlave][nPort] - g_WeldTuneParam.output.dbCurrOffset) /
                               g_WeldTuneParam.output.dbCurrScale);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_ReceiveIOCmdFromSHM()
//  - Receive I/O Commmand from shared memory
//

static void _loc_FUNC_ReceiveIOCmdFromSHM(void)
{
    int iSlave, iPort;

    //////////////////////////
    // Digital Output
    for(iSlave = 0; iSlave < ROBOT_DO_SLAVE_COUNT; iSlave++)
    {
        for(iPort = 0; iPort < SLAVE_DO_PORT_COUNT; iPort++)
        {
            // Virtual Arc ON Service OFF
            if(g_fArcOnVirtualProc == OFF)
            {
                g_DoutPortVal[iSlave][iPort] = 
                    g_pShmem_sc->outputcmd.nDoutPortCmd[iSlave * SLAVE_DO_PORT_COUNT + iPort];
            }

            // Virtual Arc ON Service ON
            else if(g_fArcOnVirtualProc == ON)
            {
                if(iSlave * SLAVE_DO_PORT_COUNT + iPort != g_Dout_portno.nArcOn &&
                   iSlave * SLAVE_DO_PORT_COUNT + iPort != g_Dout_portno.nGasOn &&
                   iSlave * SLAVE_DO_PORT_COUNT + iPort != g_Dout_portno.nTouchReady)
                {
                    g_DoutPortVal[iSlave][iPort] = 
                        g_pShmem_sc->outputcmd.nDoutPortCmd[iSlave * SLAVE_DO_PORT_COUNT + iPort];
                }
                
                if(iSlave * SLAVE_DO_PORT_COUNT + iPort == g_Dout_portno.nArcOn)
                {
                    if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] == ON)
                    {
                        g_DoutPortVal[iSlave][iPort] = OFF;
                    }
                }
                if(iSlave * SLAVE_DO_PORT_COUNT + iPort == g_Dout_portno.nGasOn)
                {
                    if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn] == ON)
                    {
                        g_DoutPortVal[iSlave][iPort] = OFF;
                    }
                }
                if(iSlave * SLAVE_DO_PORT_COUNT + iPort == g_Dout_portno.nTouchReady)
                {
                    if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady] == ON)
                    {
                        g_DoutPortVal[iSlave][iPort] = OFF;
                    }
                }
            }
        }
    }

    //////////////////////////
    // Analog Output
     for(iSlave = 0; iSlave < ROBOT_AO_SLAVE_COUNT; iSlave++)
    {
        for(iPort = 0; iPort < SLAVE_AO_PORT_COUNT; iPort++)
        {
            // Change Aout Unit
            if(g_pShmem_sc->sysstate.nAoutMappingType[iPort] == ANALOG_IO_TYPE_RAW)
            {
#if 0
                g_AoutPortVal[iSlave][iPort] = 
                        (float) g_pShmem_sc->outputcmd.dbAoutPortCmd[iSlave * SLAVE_AO_PORT_COUNT + iPort];

                if(g_AoutPortVal[iSlave][iPort] <= 0.0)
                {
                    g_AoutPortVal[iSlave][iPort] = 0;
                }

                g_nAoutWriteBuff[iSlave][iPort] = (ECAT_INT16) ((ANALOG_RAW_VAL_RES *
                                                                 g_AoutPortVal[iSlave][iPort]) /
                                                                 ANALOG_MAX_ABS_VOLTAGE);
#endif
                g_nAoutWriteBuff[iSlave][iPort] = 
                        (ECAT_INT16) g_pShmem_sc->outputcmd.dbAoutPortCmd[iSlave * SLAVE_AO_PORT_COUNT + iPort];

                g_AoutPortVal[iSlave][iPort] = (float)((g_dbAinMaxVolt *
                                                       (float) g_nAoutWriteBuff[iSlave][iPort]) /
                                                        g_dbADCMaxBit);
                if(g_AoutPortVal[iSlave][iPort] <= 0.0)
                {
                    g_AoutPortVal[iSlave][iPort] = 0;
                }
            }
            else if(g_pShmem_sc->sysstate.nAoutMappingType[iPort] == ANALOG_IO_TYPE_WELD &&
                    iPort == g_Aout_portno.nVoltageOutPortNo)
            {
                FUNC_MapWeldOutputCmdToContOutput(iSlave, iPort);
            }
            else if(g_pShmem_sc->sysstate.nAoutMappingType[iPort] == ANALOG_IO_TYPE_WELD &&
                    iPort == g_Aout_portno.nCurrentOutPortNo)
            {
                FUNC_MapWeldOutputCmdToContOutput(iSlave, iPort);
            }
            else if(g_pShmem_sc->sysstate.nAoutMappingType[iPort] == ANALOG_IO_TYPE_INCH &&
                    iPort == g_Aout_portno.nVoltageOutPortNo)
            {
                g_AoutPortVal[iSlave][iPort] = 0;
                g_nAoutWriteBuff[iSlave][iPort] = 0;
            }
            else if(g_pShmem_sc->sysstate.nAoutMappingType[iPort] == ANALOG_IO_TYPE_INCH &&
                    iPort == g_Aout_portno.nCurrentOutPortNo)
            {
                g_AoutPortVal[iSlave][iPort] = 
                        (float) (ANALOG_MAX_ABS_VOLTAGE *
                        (g_pShmem_sc->outputcmd.dbAoutPortCmd[iSlave * SLAVE_AO_PORT_COUNT + iPort] /
                         g_nMaxInchSpeed));

                if(g_AoutPortVal[iSlave][iPort] <= 0.0)
                {
                    g_AoutPortVal[iSlave][iPort] = 0;
                }

                g_nAoutWriteBuff[iSlave][iPort] = (ECAT_INT16) ((g_dbADCMaxBit *
                                                                 g_AoutPortVal[iSlave][iPort]) /
                                                                 g_dbAinMaxVolt);
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_PrintWeldIOStateChange()
//
int g_fOldArcOnSig = 0;
int g_fOldGasOnSig = 0;
int g_fOldTouchReady = 0;
int g_fOldTouchStart = 0;

static void _loc_FUNC_PrintWeldIOStateChange(void)
{
    // Arc ON
    if(g_fOldArcOnSig != g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn])
    {
        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] == OFF &&
           g_pShmem_Status_te != NULL && g_pShmem_Status_te->run_error.code != ERR_NONE)
        {
            VERBOSE_ERROR("[ERROR State] Arc Off!\n");
        }

        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] == ON)
        {
            VERBOSE_MESSAGE("[SHM CMD]Arc On!\n");
        }
        else if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] == OFF &&
                g_pShmem_Status_te != NULL && g_pShmem_Status_te->run_error.code == ERR_NONE)
        {
            VERBOSE_MESSAGE("[SHM CMD]Arc Off!\n");
        }
    }

    g_fOldArcOnSig = g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn];

    // Gas ON
    if(g_fOldGasOnSig != g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn])
    {
        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn] == ON)
        {
            VERBOSE_MESSAGE("[SHM CMD]Gas On!\n");
        }
        else if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn] == OFF)
        {
            VERBOSE_MESSAGE("[SHM CMD]Gas Off!\n");
        }
    }

    g_fOldGasOnSig = g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn];

    // Touch Ready
    if(g_fOldTouchReady != g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady])
    {
        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady] == OFF)
        {
            VERBOSE_MESSAGE("[SHM CMD]Touch Ready On!\n");
        }
        else if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady] == ON)
        {
            VERBOSE_MESSAGE("[SHM CMD]Touch Ready Off!\n");
        }
    }

    g_fOldTouchReady = g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady];

    // Touch Start
    if(g_fOldTouchStart != g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart])
    {
        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart] == OFF &&
           g_pShmem_Status_te != NULL && g_pShmem_Status_te->run_error.code != ERR_NONE)
        {
            VERBOSE_ERROR("[ERROR State] Touch Start Off!\n");
        }

        if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart] == ON)
        {
            VERBOSE_MESSAGE("[SHM CMD]Touch Start On!\n");
        }
        else if(g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart] == OFF &&
                g_pShmem_Status_te != NULL && g_pShmem_Status_te->run_error.code == ERR_NONE)
        {
            VERBOSE_MESSAGE("[SHM CMD]Touch Start Off!\n");
        }
    }

    g_fOldTouchStart = g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart];
}


////////////////////////////////////////////////////////////////////////////////
//
// RuntimeThreadRoutine()
//
THREAD_ENTRY_TYPE RuntimeThreadRoutine(void* pParam)
{
    int rcvidFromTE = 0;
    SC_MSG      msg_packet;
    SC_REPLY    reply_packet;
    int nCnt = 0;

    while(g_fExit == FALSE)
    {
        // receive timer pulse from TE
        if(g_chidSCTime != INVALID_CHID)
        {
            rcvidFromTE = MSG_Receive(g_chidSCTime,
                                      &msg_packet,
                                      sizeof(msg_packet),
                                      NULL);
        }

        memcpy(&reply_packet, &msg_packet, sizeof(SC_MSG));
        
        g_nTimerNanoRes = TIME_GetResolutionNanosec();

        if (g_pShmem_sc->sysstate.fEcatInitState == TRUE)
        {
            nCnt++;

            /* Scan all Input & Set all Output to Process Image */
            if(g_fSetEcatParam == FALSE)
            {
                _loc_FUNC_UpdatePI_ALLInOut();
            }

            /* Scan Servo State */
            _loc_FUNC_ScanServoState();

            /* Control Box Lamp Control */
            if(g_fIoTestModeActive == OFF)
            {
                _loc_FUNC_ControlPannelLampControl();
            }

            /* control I/O device */
            _loc_FUNC_ReceiveIOCmdFromSHM();

            /* Positioin Data File Write for test */
            SERV_PosFileWrite();

            /* Check I/O State */
            _loc_FUNC_PrintWeldIOStateChange();

            /* for Arc Sensor calculation data */
            FUNC_MapContInputToWeldInput();
        }
        else
        {
            THREAD_Sleep(1);
        }

        // reply message
        if (rcvidFromTE == 0)   // pulse
        {
        	reply_packet.size = 0;
        }
        else                    // message
        {
        	MSG_Reply(rcvidFromTE, 0, &reply_packet, sizeof(reply_packet));
        }

        // send pulse to TE
        if(g_Arg.bSingleExec != TRUE)
        {
            if(g_coidTETime != INVALID_COID)
            {
                MSG_SendPulse(g_coidTETime, RUNSERV_TIMER, 0);
            }
        }
    } // end of while

    g_fThreadRuntimeExitState = TRUE;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// EcatTraceThreadRoutine()
//
THREAD_ENTRY_TYPE EcatTraceThreadRoutine(void* pParam)
{
#if defined (__QNXNTO__)
    ECAT_DWORD dwID = 1;
    int iAxis;
    ECAT_UINT64 nErrData[ROB_AXIS_COUNT];
    ECAT_UINT64 nTimeStamp[ROB_AXIS_COUNT];
    ECAT_WORD   wErrStateCode[ROB_AXIS_COUNT];
    ECAT_RESULT erResult = 0;
    ECAT_DWORD  dwRealSize;
    ECAT_WORD   wSeverity, wLocation;

    EcatIODevSetTraceMode(g_hMaster, EcatErrorTraceEnabled, &dwID);
    //EcatIODevSetTraceCategories(g_hMaster, 0xFFFF, 0xFFFF);
    //EcatIODevSetTraceCategories(g_hMaster, TraceSeverity, EcatTraceCatGeneral);
    EcatIODevSetTraceCategories(g_hMaster, ErrorSeverity, EcatTraceCatAll);
    EcatIODevSetMaxTraceCount(g_hMaster, 128);

    while(g_fExit == FALSE)
    {
        while(g_pShmem_sc->sysstate.fEcatInitState == TRUE &&
              g_hMaster != NULL)
        {
            /* ------------------------- */
            /* 1. Get Network Statistics */
            /* ------------------------- */
            if(ECAT_SUCCEEDED(erResult = EcatIODevGetStatistics(g_hMaster,
                                                               &g_EcatStatistics)))
            {
                if(erResult == ECAT_S_OK)
                {
                    g_dwWrongWC = g_EcatStatistics.RTStat.dwWrongWC;
                    g_wFramesPerSecond = g_EcatStatistics.LinkStat.wFramesPerSecond;
                    //VERBOSE_ERROR("Wrong Working Counter: %ld\n", g_dwWrongWC);
                }
            }
            else
            {
                VERBOSE_ERROR("Couldn't get statistics. Error code: %x (%s).\n",
		                      erResult,
		                      ECATLIB_GetErrorDescription(erResult));
            }

            /* ------------------------ */
            /* 2. Get Error Message     */
            /* ------------------------ */
            if(ECAT_SUCCEEDED(erResult = EcatIODevGetLastError(g_hMaster,
                                                               &wSeverity,
                                                               &wLocation,
                                                               g_szEcatErrorDescription,
                                                               sizeof(g_szEcatErrorDescription) - 1,
                                                               &dwRealSize)))
		    {
		        if (erResult != ECAT_S_NOITEMS)
		        {
		            if (dwRealSize < sizeof(g_szEcatErrorDescription))
                    {
		                g_szEcatErrorDescription[dwRealSize] = '\0';
                    }
		            VERBOSE_ERROR("Sever: %d, Loc: %d, Err: %s\n",
		                          wSeverity, wLocation, g_szEcatErrorDescription);
		        }
		    }
            else
		    {
#if 1
		        VERBOSE_ERROR("Couldn't get last error. Error code: %x (%s).\n",
		                      erResult,
		                      ECATLIB_GetErrorDescription(erResult));
#endif
		    }
            
            /* ------------------------ */
            /* 3. Get Emergency Message */
            /* ------------------------ */
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                erResult = EcatIODevGetSlaveNextEmgMessage(g_hMaster,
                                                           iAxis,   // SlaveID
                                                           1,       // Delete: 1, No Delete: 0
                                                           &wErrStateCode[iAxis],
                                                           &nErrData[iAxis],
                                                           &nTimeStamp[iAxis]);
		        if((erResult & 0x8000) == 0)
                {
		            if (erResult != ECAT_S_NOITEMS)
		            {
                        if(wErrStateCode[iAxis] == 0xff00)
                        {
                            nErrData[iAxis] = (int) nErrData[iAxis] >> 8;   // Emergency Code Offset
                            g_nEmergencyCodeServo[iAxis] = (int) nErrData[iAxis];
                            g_wEmergencyErrStateCode[iAxis] = wErrStateCode[iAxis];
                            g_nErrAxis = iAxis;
                            VERBOSE_MESSAGE("nErrData(Ax: %d): %x\n", iAxis, (int) nErrData[iAxis]);    //for test
                        }
		                VERBOSE_ERROR("Axis %d: Emergency Code: %x, Data: %x, Time: %d\n"
                                      "Error Content: %s\n",
		                              iAxis, wErrStateCode[iAxis],
                                      (int) nErrData[iAxis], (int) nTimeStamp[iAxis],
                                      ERR_GetErrorDescription(g_nEmergencyCodeServo[iAxis]));
		            }
		        }
                else
		        {
		            VERBOSE_ERROR("Couldn't get Emergency Msg. Error code: %x (%s).\n",
		                          erResult,
		                          ECATLIB_GetErrorDescription(erResult));
		        }
            }

            /* ------------------------------- */
            /* 4. Err Information input to SHM */
            /* ------------------------------- */
            if(g_pShmem_sc->sysstate.fErrorState == TRUE)
            {
                if(g_wEmergencyErrStateCode[g_nErrAxis] == 0xff00)  // in error state, set to 0xff00
                {
                    g_pShmem_sc->sysstate.nErrorCode = g_nEmergencyCodeServo[g_nErrAxis];
                }
                else
                {
                    for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
                    {
                        if(g_nErrCodeServo[iAxis] != 0)
                        {
                            g_nErrAxis = iAxis;
                        }
                    }

                    g_pShmem_sc->sysstate.nErrorCode = g_nErrCodeServo[g_nErrAxis];
                }

                g_pShmem_sc->sysstate.nErrorAxis = g_nErrAxis;
                CRT_strncpy(g_pShmem_sc->sysstate.g_szErrorDescription,
                            ERROR_MESSAGE_BUFFER_SIZE,
                            g_szEcatErrorDescription,
                            ERROR_MESSAGE_BUFFER_SIZE);
            }
            else
            {
                g_pShmem_sc->sysstate.nErrorAxis = -1;
            }

            THREAD_Sleep(5);
            //THREAD_SleepMillisec(5);
            //DANDY_SLEEP(5);
        } // end of while

        THREAD_Sleep(5);
        //THREAD_SleepMillisec(5);
        //DANDY_SLEEP(5);
    } // end of while

    EcatIODevSetTraceMode(g_hMaster, EcatErrorTraceDisabled, &dwID);

#endif

    g_fThreadEcatTraceExitState = TRUE;
    
	return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// SVC_ServiceProcThreadRoutine()
//
THREAD_ENTRY_TYPE SVC_ServiceProcThreadRoutine(void* pParam)
{
    int nRet = -1;             // return value of function
    MSG_INFO info;

    ////////////////////////////////////////////////////////////////////////////
    // message receive & reply 
    //
    while(g_fExit == FALSE)
    {
        // receive message
        g_rcvidSC = MSG_Receive(g_chidSC, &SC_msg, sizeof(SC_msg), &info);

        if (SC_msg.code == SC_SERV_ALIVE)
        {
            ;
        }
        else
        {
            if(SC_msg.code != SC_SERV_SCAN_WELD_IO &&
               SC_msg.code != SC_SERV_SCAN_SERVO_IO &&
               SC_msg.code != SC_SERV_SCAN_SYS_STATE)
            {
                VERBOSE_VERBOSE("Received service code: <%d>, value: <%d>\n",
            	    	        SC_msg.code, SC_msg.value);
            }
        }
        
        // set reply data(packet)
        nRet = SERV_DoService(&SC_msg, &SC_reply);

        // message reply
        if (g_rcvidSC == RCVID_PULSE)   // pulse
        {
            SC_reply.size = 0;
        }
        else if (g_rcvidSC > 0)         // message
        {
            //MSG_Reply(g_rcvidSC, nRet, &SC_reply, SC_REPLY_PACKET_SIZE);
            
            //for broker communication nRet must be zero!
            MSG_Reply(g_rcvidSC, 0, &SC_reply, SC_REPLY_PACKET_SIZE);
        }
        else
        {
            VERBOSE_ERROR("Failed to receive data...\n");
        }
    
        THREAD_Sleep(2);
        //THREAD_SleepMillisec(5);
        //DANDY_SLEEP(5);
    } // end while
    
    g_fServiceProcThreadExitState = TRUE;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// CheckTimeLimitThreadRoutine()
//
THREAD_ENTRY_TYPE CheckTimeLimitThreadRoutine(void* pParam)
{
    int iCnt = 0;
    
    // check state of external initialization
    while(iCnt < g_nTime_Limit)
    {
        iCnt++;
        THREAD_Sleep(1000);

        if (g_rcvidSC == INVALID_RCVID)
        {
            ;
        }
        else
        {
            break;
        }

        if (iCnt == g_nTime_Limit)
        {
            VERBOSE_WARNING("Elapsed time of receive message : %d [sec]\n", 
                            g_nTime_Limit);

            VERBOSE_ERROR("Disconnected RM process.\n");

            g_pShmem_sc->sysstate.fErrorState = TRUE;
            g_pShmem_sc->sysstate.nErrorCode  = SYS_ERR_INIT_SC;

            if (g_coidSC != INVALID_COID)
            {
                // send exit cmd
                MSG_SendPulse(g_coidSC, SC_SERV_EXIT, 0);
            }
            
            THREAD_Sleep(10);
            
            break;
        }
    } // end while

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_UpdatePI_ALLInOut()
//
static int _loc_FUNC_UpdatePI_ALLInOut(void)
{
    int nRet;

#if defined (__QNXNTO__)
#if defined (ECAT_MODE_SYNC2B)
    EcatIODevUpdateProcessImageInputs(g_hMaster);      //sync2 mode (b)
#endif
#endif
    ECATSERV_StartReadInputWriteOutput();

    nRet = ECATSERV_WriteTotalOutput();
    
    nRet = ECATSERV_ReadTotalInput();
    
    ECATSERV_DoneReadInputWriteOutput();

#if defined (__QNXNTO__)
#if defined (ECAT_MODE_SYNC2A)
    EcatIODevUpdateProcessImage(g_hMaster);           //sync2 mode (a)
#endif
#if defined (ECAT_MODE_SYNC2B)
    EcatIODevUpdateProcessImageOutputs(g_hMaster);    //sync2 mode (b)
#endif
#endif

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_InputScanResultLoadToShmem()
//

#define     INPUT_TO_VAR        0
#define     INPUT_TO_ARRAY      1

static void _loc_FUNC_InputScanResultLoadToShmem(int nOpt)
{
    int iAxis;
    int iSlave, iPort;

    if(nOpt == INPUT_TO_VAR)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            // Din Slave 0 - Brake State
            g_pShmem_sc->inputstate.fBrakeStatusInState[iAxis] =
                g_DinPortVal[BRAKESTATE_DI_SLAVE_NO][iAxis];
            // Din Slave 1 - Brake Clear
            g_pShmem_sc->inputstate.fBrakeClearInState[iAxis] = 
                g_DinPortVal[BRAKECLEAR_DI_SLAVE_NO][iAxis];
        }

        // Din Slave 0 - System I/O State
        g_pShmem_sc->inputstate.fController_ResetButton =
            g_DinPortVal[SYSTEMIO_DI_SLAVE_NO][CONT_RESET_DI_PORT_NO];
        g_pShmem_sc->inputstate.fController_EstopInState =
            g_DinPortVal[SYSTEMIO_DI_SLAVE_NO][CONT_ESTOP_DI_PORT_NO];

        // Din Slave 2  - Welder Input
            // input to varialbes
#if defined(__QNXNTO__)
        g_pShmem_sc->inputstate.fArcOnInState = 
            g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO];
#else
        g_pShmem_sc->inputstate.fArcOnInState = 
            g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO];

        if(g_fArcOnSigHigh == OFF && g_fArcOnVirtualProc == OFF)
        {
            if(g_pShmem_sc->outputstate.fArcOnOutState == ON)
            {
                g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO] = ON;
            }
            else
            {
                g_DinPortVal[WELD_DI_SLAVE_NO][ARCON_DI_PORT_NO] = OFF;
            }
        }
#endif
        g_pShmem_sc->inputstate.fNoGasInState =
            g_DinPortVal[WELD_DI_SLAVE_NO][NOGAS_DI_PORT_NO];
        g_pShmem_sc->inputstate.fNoWireInState =
            g_DinPortVal[WELD_DI_SLAVE_NO][NOWIRE_DI_PORT_NO];
        g_pShmem_sc->inputstate.fWeldPowerFailInState=
            g_DinPortVal[WELD_DI_SLAVE_NO][WELDERPWRFAIL_DI_PORT_NO];
#if defined(__QNXNTO__)
        g_pShmem_sc->inputstate.fTouchProcessInState =
            g_DinPortVal[WELD_DI_SLAVE_NO][TOUCHPROCESS_DI_PORT_NO];
#else
        g_pShmem_sc->inputstate.fTouchProcessInState =
            g_DinPortVal[WELD_DI_SLAVE_NO][TOUCHPROCESS_DI_PORT_NO];

        if(g_pShmem_sc->outputstate.fTouchStartOutState == ON)
        {
            g_DinPortVal[WELD_DI_SLAVE_NO][TOUCHPROCESS_DI_PORT_NO] = ON;
        }
        else
        {
            g_DinPortVal[WELD_DI_SLAVE_NO][TOUCHPROCESS_DI_PORT_NO] = OFF;
        }
#endif
        g_pShmem_sc->inputstate.fTouchSignalInState =
            g_DinPortVal[WELD_DI_SLAVE_NO][TOUCHSIGNAL_DI_PORT_NO];

        // Din Slave 3  - Cart Input
        g_pShmem_sc->inputstate.fTP_EstopInState = 
            g_DinPortVal[CART_DI_SLAVE_NO][TP_ESTOP_DI_PORT_NO];
        g_pShmem_sc->inputstate.fDeadManSwithInState = 
            g_DinPortVal[CART_DI_SLAVE_NO][DEADMAN_DI_PORT_NO];
        g_pShmem_sc->inputstate.fShockSensorInState = 
            g_DinPortVal[CART_DI_SLAVE_NO][SHOCKSENSOR_DI_PORT_NO];
        g_pShmem_sc->inputstate.fCartJobStartButton = 
            g_DinPortVal[CART_DI_SLAVE_NO][JOBSTART_DI_PORT_NO];
        g_pShmem_sc->inputstate.fCart_EstopInState = 
            g_DinPortVal[CART_DI_SLAVE_NO][CART_ESTOP_DI_PORT_NO];
        g_pShmem_sc->inputstate.fCartCylinderState =
            g_DinPortVal[CART_DI_SLAVE_NO][CART_CYLIND_DI_PORT_NO];

        // Digital Out
        if(g_fArcOnVirtualProc == OFF)
        {
            g_pShmem_sc->outputstate.fArcOnOutState =
                g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nArcOn];
            g_pShmem_sc->outputstate.fGasOnOutState =
                g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nGasOn];
            g_pShmem_sc->outputstate.fTouchReadyOutState =
                g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nTouchReady];
        }
        else if(g_fArcOnVirtualProc == ON)
        {
            g_pShmem_sc->outputstate.fArcOnOutState =
                g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn];
            g_pShmem_sc->outputstate.fGasOnOutState =
                g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn];
            g_pShmem_sc->outputstate.fTouchReadyOutState =
                g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady];
        }

        g_pShmem_sc->outputstate.fInchingPosOutState =
            g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nInchPos];
        g_pShmem_sc->outputstate.fInchingNegOutState =
            g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nInchNeg];
        g_pShmem_sc->outputstate.fTouchStartOutState =
            g_pShmem_sc->outputstate.nDoutPortVal[g_Dout_portno.nTouchStart];

        // Ain Slave 0  - Welder Input
            // input to variables
#if 0
        g_pShmem_sc->inputstate.dbWeldVoltInVal =
           g_AinPortVal[WELD_AI_SLAVE_NO][VOLT_AI_PORT_NO];
        g_pShmem_sc->inputstate.dbWeldCurrInVal =
           g_AinPortVal[WELD_AI_SLAVE_NO][CURR_AI_PORT_NO];
#endif
    }

    if(nOpt == INPUT_TO_ARRAY)
    {
        // All Inputs put to array (2dim array put to 1dim array)
        for(iSlave = 0; iSlave < ROBOT_DI_SLAVE_COUNT; iSlave++)
        {
            for(iPort = 0; iPort < SLAVE_DI_PORT_COUNT; iPort++)
            {
                g_pShmem_sc->inputstate.nDinPortVal[iSlave * SLAVE_DI_PORT_COUNT + iPort] = 
                                                                g_DinPortVal[iSlave][iPort];
            }
        }

        for(iSlave = 0; iSlave < ROBOT_AI_SLAVE_COUNT; iSlave++)
        {
            for(iPort = 0; iPort < SLAVE_AI_PORT_COUNT; iPort++)
            {
                g_pShmem_sc->inputstate.dbAinPortVal[iSlave * SLAVE_AI_PORT_COUNT + iPort] = 
                                                                g_AinPortVal[iSlave][iPort];
            }
        }

        for(iSlave = 0; iSlave < ROBOT_DO_SLAVE_COUNT; iSlave++)
        {
            for(iPort = 0; iPort < SLAVE_DO_PORT_COUNT; iPort++)
            {
                g_pShmem_sc->outputstate.nDoutPortVal[iSlave * SLAVE_DO_PORT_COUNT + iPort] = 
                                                                g_DoutPortVal[iSlave][iPort];
            }
        }

        for(iSlave = 0; iSlave < ROBOT_AO_SLAVE_COUNT; iSlave++)
        {
            for(iPort = 0; iPort < SLAVE_AO_PORT_COUNT; iPort++)
            {
                g_pShmem_sc->outputstate.dbAoutPortVal[iSlave * SLAVE_AO_PORT_COUNT + iPort] = 
                                                                g_AoutPortVal[iSlave][iPort];
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_ReplyPacketMapping()
//
static void _loc_FUNC_ReplyPacketMapping(void)
{
    SC_reply.data.servo_mon.servoout  = SC_reply.data.servoout;
    SC_reply.data.welder_mon.weldDout = SC_reply.data.weldDout;
    SC_reply.data.welder_mon.weldAout = SC_reply.data.weldAout;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_ScanServoState()
//
static void _loc_FUNC_ScanServoState(void)
{
    int iAxis = 0;

    /* 1. check error & servo */
    
        // Check Servo State
    SERV_GetServoState();

        // Check Alarm Code
    //SERV_GetServoAlarmCode();

    if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            // Check Alarm Code & Enter to E-Stop mode
            if(g_nErrCodeServo[iAxis] != 0)
            {
                SERV_EStop(ON);
                VERBOSE_ERROR("[Axis-%d] Err: %x, State: %x\n",
                              iAxis, g_nErrCodeServo[iAxis], g_nReadStatusValue[iAxis]);
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_FUNC_ControlPannelLampControl()
//
static void _loc_FUNC_ControlPannelLampControl(void)
{
        /* 1. EtherCAT Run */
    if(g_pShmem_sc->sysstate.fEcatInitState == ON)
    {
        SERV_LampEtherCATRun_out(ON);
    }
    else
    {
        SERV_LampEtherCATRun_out(OFF);
    }
    
        /* 2. Servo On */
    if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        SERV_LampServoOn_out(ON);
    }
    else
    {
        SERV_LampServoOn_out(OFF);
    }
        /* 3. Controller Ready */
    if(g_pShmem_sc->outputcmd.fLampControllerReadyCmd == ON)
    {
        SERV_LampControllerReady_out(ON);
    }
    else
    {
        SERV_LampControllerReady_out(OFF);
    }
    
        /* 4. Under Operating */
    if(g_pShmem_sc->outputcmd.fLampUnderOperatingCmd == ON)
    {
        SERV_LampUnderOperating_out(ON);
    }
    else
    {
        SERV_LampUnderOperating_out(OFF);
    }
    
        /* 5. Error(Spare) */
    if(g_pShmem_sc->outputcmd.fLampErrorCmd == ON)
    {
        SERV_LampError_out(ON);
    }
    else
    {
        SERV_LampError_out(OFF);
    }

        /* 6. Cart Alarm */
    if(g_pShmem_sc->outputcmd.fCartLampAlarmCmd == ON)
    {
        SERV_CartLampAlarm_out(ON);
    }
    else
    {
        SERV_CartLampAlarm_out(OFF);
    }

        /* 7. Cart Run */
    if(g_pShmem_sc->outputcmd.fCartLampRunCmd == ON)
    {
        SERV_CartLampRun_out(ON);
    }
    else
    {
        SERV_CartLampRun_out(OFF);
    }

        /* 8. Job Start Button Light */
    if(g_pShmem_sc->outputcmd.fCartJobStartConfirmCmd == ON)
    {
        SERV_CartJobStartConfirm_out(ON);
    }
    else
    {
        SERV_CartJobStartConfirm_out(OFF);
    }

        /* 9. Cart Dout Spare */
    if(g_pShmem_sc->outputcmd.fCartDoutSpareCmd == ON)
    {
        SERV_CartDoutSpare_out(ON);
    }
    else
    {
        SERV_CartDoutSpare_out(OFF);
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// ScanEStopStateThreadRoutine()
//
#define PERIOD_OF_NETWORK_STATE_CHECK       200
//#define CHECK_ECAT_NETWORK                  1

THREAD_ENTRY_TYPE ScanEStopStateThreadRoutine(void* pParam)
{
    int iAxis = 0;

    int fEStopSwtichStateChanged;
    int oldControllerEStopSwitchState = 0;
    int oldTPEStopSwitchState = 0;
    int oldCartEStopSwitchState = 0;

    int fECATNetworkStateChanged;
    int oldECATNetworkState = 0;

#if defined(CHECK_ECAT_NETWORK)    
    int nRet;
    int nCnt = 0;
#endif
    
    int fShockSensorInStateChanged;
    int oldShockSensorInState = 0;

    int fHWLimitOnStateChanged;
    int oldHWLimitOnState = 0;
    int fHWLimitOnState[ROB_AXIS_COUNT];
    int fTmpDeterminLimitState = 0;
    int fShockSensorDetected = 0;

    while (g_fExit == FALSE)
	{
        g_fEStopStateThreadRunState = TRUE;

        /* 1. Check Button State(for triggering event) */
            /* E-Stop Button */
        if(oldControllerEStopSwitchState != g_pShmem_sc->inputstate.fController_EstopInState ||
           oldTPEStopSwitchState         != g_pShmem_sc->inputstate.fTP_EstopInState         ||
           oldCartEStopSwitchState       != g_pShmem_sc->inputstate.fCart_EstopInState)
        {
            fEStopSwtichStateChanged = ON;
        }
        else
        {
            fEStopSwtichStateChanged = OFF;
        }

        oldControllerEStopSwitchState = g_pShmem_sc->inputstate.fController_EstopInState;
        oldTPEStopSwitchState         = g_pShmem_sc->inputstate.fTP_EstopInState;
        oldCartEStopSwitchState       = g_pShmem_sc->inputstate.fCart_EstopInState;

            /* Shock Sensor */
        if(oldShockSensorInState != g_pShmem_sc->inputstate.fShockSensorInState)
        {
            fShockSensorInStateChanged = ON;
        }
        else
        {
            fShockSensorInStateChanged = OFF;
        }

        oldShockSensorInState = g_pShmem_sc->inputstate.fShockSensorInState;
        
        /* 2. Check EtherCAT Network State(for triggering event) */
        if(oldECATNetworkState != g_pShmem_sc->sysstate.fEcatInitState)
        {
            fECATNetworkStateChanged = ON;
        }
        else
        {
            fECATNetworkStateChanged = OFF;
        }

        oldECATNetworkState = g_pShmem_sc->sysstate.fEcatInitState;

        /* 3. Check State for trigger event */
        if(oldHWLimitOnState != g_fHWLimitOnState)
        {
            fHWLimitOnStateChanged = ON;
        }
        else
        {
            fHWLimitOnStateChanged = OFF;
        }

        oldHWLimitOnState = g_fHWLimitOnState;

        /* 4. E-STOP Exec */
        if(fEStopSwtichStateChanged == ON)
        {
            if(g_Arg.bNoTPEstop != TRUE && g_Arg.bNoCartEstop != TRUE)
            {
                if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
                   g_pShmem_sc->inputstate.fTP_EstopInState         == ESTOP_ACT ||
                   g_pShmem_sc->inputstate.fCart_EstopInState       == ESTOP_ACT)
                {
                    SERV_EStop(ON);

                    if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                            VERBOSE_MESSAGE("Estop Activated By ControBox\n");
                        }
                    }
                    if(g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_TP;
                            VERBOSE_MESSAGE("Estop Activated By TP\n");
                        }
                    }
                    if(g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CART;
                            VERBOSE_MESSAGE("Estop Activated By Cart\n");
                        }
                    }
                }
                else
                {
                    SERV_EStop(OFF);

                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                    }
                }
            }
            else if(g_Arg.bNoTPEstop == TRUE && g_Arg.bNoCartEstop != TRUE)
            {
                if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
                   g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
                {
                    SERV_EStop(ON);
                    if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                        }
                    }
                    if(g_pShmem_sc->inputstate.fCart_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CART;
                        }
                    }
                }
                else
                {
                    SERV_EStop(OFF);

                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                    }
                }
            }
            else if(g_Arg.bNoTPEstop != TRUE && g_Arg.bNoCartEstop == TRUE)
            {
                if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT ||
                   g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
                {
                    SERV_EStop(ON);

                    if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                        }
                    }
                    if(g_pShmem_sc->inputstate.fTP_EstopInState == ESTOP_ACT)
                    {
                        if(g_pShmem_SysStatus_rm != NULL)
                        {
                            g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_TP;
                        }
                    }
                }
                else
                {
                    SERV_EStop(OFF);

                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                    }
                }
            }
            else if(g_Arg.bNoTPEstop == TRUE && g_Arg.bNoCartEstop == TRUE)
            {
                if(g_pShmem_sc->inputstate.fController_EstopInState == ESTOP_ACT)
                {
                    SERV_EStop(ON);

                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_CONTROLBOX;
                    }
                }
                else
                {
                    SERV_EStop(OFF);

                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                    }
                }
            }
        }

        /* 5. Shock Sensor */
        if(fShockSensorInStateChanged == ON)
        {
            if(g_Arg.bNoShockSensor != TRUE && g_fShockSensorRelease == OFF)
            {
                if(g_pShmem_sc->inputstate.fShockSensorInState == ESTOP_ACT)
                {
                    SERV_EStop(ON);

                    fShockSensorDetected = ON;
                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_SHOCKSENSOR;
                        VERBOSE_MESSAGE("Estop Activated By ShockSensor\n");
                    }
                }
                else if(g_pShmem_sc->inputstate.fShockSensorInState == ESTOP_DEACT &&
                        fShockSensorDetected == ON)
                {
                    SERV_EStop(OFF);

                    fShockSensorDetected = OFF;
                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                        VERBOSE_MESSAGE("Estop Released By ShockSensor\n");
                    }
                }
            }
            else if(g_Arg.bNoShockSensor != TRUE && g_fShockSensorRelease == ON)
            {
                if(g_pShmem_sc->inputstate.fShockSensorInState == ESTOP_ACT)
                {
                    ;
                }
                else if(g_pShmem_sc->inputstate.fShockSensorInState == ESTOP_DEACT)
                {
                    SERV_EStop(OFF);

                    fShockSensorDetected = OFF;
                    if(g_pShmem_SysStatus_rm != NULL)
                    {
                        g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                        VERBOSE_MESSAGE("Estop Released By ShockSensor\n");
                    }
                }
                
                g_fShockSensorRelease = OFF;
            }
        }

        /* 6. Check Slave Network State */
#if defined(CHECK_ECAT_NETWORK)
        if(g_fMasterInitEventActive == OFF && nCnt >= PERIOD_OF_NETWORK_STATE_CHECK)
        {
            nRet = SERV_GetNetworkState();
            
            if(nRet == RESULT_OK)
            {
                if(g_pShmem_sc != NULL)
                {
                    g_pShmem_sc->sysstate.fEcatInitState = TRUE;
                }
            }
            else
            {
                if(g_pShmem_sc != NULL)
                {
                    // Can't Control Lamp at Error State
                    g_pShmem_sc->sysstate.fEcatInitState = FALSE; 

                    // Error State Inform
                    if(fECATNetworkStateChanged == ON)
                    {
                        VERBOSE_ERROR("EtherCAT Network Unstable!\n");
                        g_nErrCodeEcat = ECAT_ERR_SLAVE_CONNECTION_CLOSED;
                        ERR_GetErrorDescription(ECAT_ERR_SLAVE_CONNECTION_CLOSED);
                    }
                }
            }

            nCnt = 0;
        }

        nCnt++;
#endif

        /* 7. H/W Limit Monitoring */
        if(g_fHWLimitMonAct == ON)
        {
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                if(g_rgfHwLimitUsed[iAxis][LIMIT_NEG] == ON &&
                  (g_dbAct_Pos[iAxis] < g_rgdbHwLimit[iAxis][LIMIT_NEG]))
                {
                    fHWLimitOnState[iAxis] = ON;

                    if(fHWLimitOnStateChanged == ON)
                    {
                        VERBOSE_ERROR("%d Axis H/W Limit Negative(-) Detected!\n", iAxis);
                    }
                }
                
                if(g_rgfHwLimitUsed[iAxis][LIMIT_POS] == ON &&
                  (g_dbAct_Pos[iAxis] > g_rgdbHwLimit[iAxis][LIMIT_POS]))
                {
                    fHWLimitOnState[iAxis] = ON;

                    g_nErrAxis = iAxis;
                    g_pShmem_sc->sysstate.nErrorAxis = g_nErrAxis;

                    if(fHWLimitOnStateChanged == ON)
                    {
                        VERBOSE_ERROR("%d Axis H/W Limit Positive(+) Detected!\n", iAxis);
                    }
                }

                if((g_dbAct_Pos[iAxis] > g_rgdbHwLimit[iAxis][LIMIT_NEG]) &&
                   (g_dbAct_Pos[iAxis] < g_rgdbHwLimit[iAxis][LIMIT_POS]))
                {
                    fHWLimitOnState[iAxis] = OFF;
                }

                fTmpDeterminLimitState = fHWLimitOnState[AXIS_0_INDEX] + fHWLimitOnState[AXIS_1_INDEX] +
                                         fHWLimitOnState[AXIS_2_INDEX] + fHWLimitOnState[AXIS_3_INDEX] +
                                         fHWLimitOnState[AXIS_4_INDEX] + fHWLimitOnState[AXIS_5_INDEX];

                if(fTmpDeterminLimitState == OFF && iAxis == (g_nAxisCount - 1))
                {
                    g_fHWLimitOnState = OFF;
                }
                else if(fTmpDeterminLimitState != OFF && iAxis == (g_nAxisCount - 1))
                {
                    g_fHWLimitOnState = ON;
                }
            }
        }
        else
        {
            g_fHWLimitOnState = OFF;
        }

        if(fHWLimitOnStateChanged == ON)
        {
            if(g_fHWLimitOnState == ON)
            {
                SERV_EStop(ON);
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_HWLIMIT_ACT;
                }
                
                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode  = IO_ERR_HW_LIMIT_DETECT;
            }
            else if(g_fHWLimitOnState == OFF)
            {
                SERV_EStop(OFF);
                if(g_pShmem_SysStatus_rm != NULL)
                {
                    g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                }
            }

            fTmpDeterminLimitState = OFF;
        }

        THREAD_Sleep(g_nIoScanTime);  // period of I/O monitoring
        //THREAD_SleepMillisec(g_nIoScanTime);    // period of I/O monitoring
        //DANDY_SLEEP(g_nIoScanTime);  // period of I/O monitoring
    }

    g_fEStopStateThreadExitState = TRUE;
    g_fEStopStateThreadRunState = FALSE;

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// ScanButtonStateThreadRoutine()
//

THREAD_ENTRY_TYPE ScanButtonStateThreadRoutine(void* pParam)
{
    int iAxis = 0;

    int fResetButtonStateChanged;
    int oldResetButtonState = 0;
    
    int fDeadManSwtichStateChanged;
    int oldDeadManSwitchState = 0;

    int fCartJobStartButtonStateChanged;
    int oldCartJobStartButtonState = 0;

    int fEncClearButtonStateChanged[ROB_AXIS_COUNT] = {0};
    int oldEncClearButtonState[ROB_AXIS_COUNT] = {0};

	while (g_fExit == FALSE)
	{
        g_fScanButtonThreadRunState = TRUE;
#if 0
        /* Input map to SHM array */
        _loc_FUNC_InputScanResultLoadToShmem(INPUT_TO_VAR);
        _loc_FUNC_InputScanResultLoadToShmem(INPUT_TO_ARRAY);
#endif
        /* 1. Check Button State(for triggering event) */
            /* Deadman Switch */
        if(oldDeadManSwitchState != g_pShmem_sc->inputstate.fDeadManSwithInState)
        {
            fDeadManSwtichStateChanged = ON;
        }
        else
        {
            fDeadManSwtichStateChanged = OFF;
        }

        oldDeadManSwitchState = g_pShmem_sc->inputstate.fDeadManSwithInState;

            /* Reset Button */
        if(oldResetButtonState != g_pShmem_sc->inputstate.fController_ResetButton)
        {
            fResetButtonStateChanged = ON;
        }
        else
        {
            fResetButtonStateChanged = OFF;
        }

        oldResetButtonState = g_pShmem_sc->inputstate.fController_ResetButton;

            /* Cart Job Start Button */
        if(oldCartJobStartButtonState != g_pShmem_sc->inputstate.fCartJobStartButton)
        {
            fCartJobStartButtonStateChanged = ON;
        }
        else
        {
            fCartJobStartButtonStateChanged = OFF;
        }

        oldCartJobStartButtonState = g_pShmem_sc->inputstate.fCartJobStartButton;

            /* Encoder Clear Button */
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(oldEncClearButtonState[iAxis] != g_pShmem_sc->inputstate.fBrakeClearInState[iAxis])
            {
                fEncClearButtonStateChanged[iAxis] = ON;
            }
            else
            {
                fEncClearButtonStateChanged[iAxis] = OFF;
            }

            oldEncClearButtonState[iAxis] = g_pShmem_sc->inputstate.fBrakeClearInState[iAxis];
        }
        
        /* 2. Reset Button */
        if(fResetButtonStateChanged == ON)
        {
            if(g_pShmem_sc->inputstate.fController_ResetButton == ON)
            {
                if(g_Arg.bSingleExec != TRUE)
                {
                    MSG_SendPulse(g_coidRM, RCON_SERV_ERROR_RESET, 0);
                }
                else
                {
                    SERV_ClearServoAlarm();
                }
            }
        }

        /* 3. DeadMan Swtich */
        if(fDeadManSwtichStateChanged == ON)
        {
            if(g_pShmem_SysStatus_rm != NULL &&
               g_pShmem_SysStatus_rm->nSystemMode != MODE_STAT_AUTORUN)
            {
                /* Dead Man Switch Servo On/Off */
                if(g_pShmem_sc->inputstate.fDeadManSwithInState == ON)
                {
                    SERV_ServoOnCmd(ON);
                }
                else if(g_pShmem_sc->inputstate.fDeadManSwithInState == OFF)
                {
                    SERV_ServoOffStop();

                    if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
                    {
                        SERV_ServoOnCmd(OFF);
                    }
                }
            }
            else if(g_pShmem_SysStatus_rm == NULL)
            {
                /* Dead Man Switch Servo On/Off */
                if(g_pShmem_sc->inputstate.fDeadManSwithInState == ON)
                {
                    SERV_ServoOnCmd(ON);
                }
                else if(g_pShmem_sc->inputstate.fDeadManSwithInState == OFF)
                {
                    SERV_ServoOffStop();

                    if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
                    {
                        SERV_ServoOnCmd(OFF);
                    }
                }
            }
       }
        
        /* 4. Cart Job Start Button */
        if(fCartJobStartButtonStateChanged == ON)
        {
            if(g_pShmem_sc->inputstate.fCartJobStartButton == ON)
            {
                if(g_pShmem_SysStatus_rm != NULL)
                {
#if 0
                    if(g_pShmem_SysStatus_rm->nWorkType == WORK_TYPE_JOB &&
                       g_pShmem_SysStatus_rm->nExecStat != EXEC_STAT_EXECUTING)
#endif
                    if(g_pShmem_SysStatus_rm->nExecStat != EXEC_STAT_EXECUTING)
                    {
                        MSG_SendPulse(g_coidRM,
                                      RSVC_SERV_JOBEXECAUTO,
                                      0);
                                      //g_pShmem_SysStatus_rm->nJobRunIndex);
                        g_pShmem_sc->outputcmd.fCartJobStartConfirmCmd = ON;
                    }
                    else if(g_pShmem_SysStatus_rm->nWorkType == WORK_TYPE_JOB &&
                            g_pShmem_SysStatus_rm->nExecStat == EXEC_STAT_EXECUTING)
                    {
                        MSG_SendPulse(g_coidRM,
                                      RSVC_SERV_JOBEXESTOP,
                                      0);
                        g_pShmem_sc->outputcmd.fCartJobStartConfirmCmd = OFF;
                    }
                }
            }
        }
        
        /* 5. Encoder Clear Button */
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(fEncClearButtonStateChanged[iAxis] == ON)
            {
                if(g_pShmem_sc->inputstate.fBrakeClearInState[iAxis] == ON)
                {
                    SERV_ABSEncoderReset(iAxis);
                }
            }
        }

        THREAD_Sleep(g_nIoScanTime);  // period of I/O monitoring
	}

    g_fScanButtonThreadExitState = TRUE;
    g_fScanButtonThreadRunState = FALSE;

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// ScanSDOInputStateThreadRoutine()
//

THREAD_ENTRY_TYPE ScanSDOInputStateThreadRoutine(void* pParam)
{
    int iAxis = 0;
    int nCnt = 0;
    ECAT_DWORD dwReadBuff = 0;
    //int nReadBuff = 0;
    int nActTorque[ROB_AXIS_COUNT];

    int fEStopModeStateChanged;
    int oldEStopModeState = 0;

    while (g_fExit == FALSE)
	{
        g_fScanSDOInputThreadRunState = RUN;

        /* E-stop Mode */
        if(oldEStopModeState != g_pShmem_sc->sysstate.fEStopState)
        {
            fEStopModeStateChanged = ON;
        }
        else
        {
            fEStopModeStateChanged = OFF;
        }

        oldEStopModeState = g_pShmem_sc->sysstate.fEStopState;

        /* 1. E-Stop mode I/O Handling */
        if(fEStopModeStateChanged == ON)
        {
            if(g_pShmem_sc->sysstate.fEStopState == TRUE)
            {
                SERV_DoutsSetZero(SMOOTH_MODE);

                SERV_DoutsSetZero(URGENT_MODE);
            }
        }

#if 0
        // Check Error Register
        if(nCnt >= 100)
        {
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                g_nErrorRegister[iAxis] =
                        ECATLIB_ReadSlaveCoEObjectInteger(iAxis, 0x1001, 0, dwReadBuff);
            }
        
            nCnt = 0;
        }
#endif

#if 1
        /* 2. Check Actual Torque */
        if(nCnt >= 10)
        {
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                // WriteIndex: 6077, SubIndex: 0, Name: Torque Actual Value, Unit: 0.1%
                nActTorque[iAxis] =
                        ECATLIB_ReadSlaveCoEObjectInteger(iAxis, 0x6077, 0, dwReadBuff);

                //if((nActTorque[iAxis] & 0xf0000) < 0x10000)
                if(nActTorque[iAxis] < 10000)
                {
                    g_dbActTorque[iAxis] = (double) nActTorque[iAxis] * 0.1;
                }
                //else if((nActTorque[iAxis] & 0xf0000) >= 0x10000)
                else if(nActTorque[iAxis] >= 10000)
                {
                    nActTorque[iAxis] = nActTorque[iAxis] - 65535;
                    g_dbActTorque[iAxis] = (double) nActTorque[iAxis] * 0.1 ;
                }
            }
        
            nCnt = 0;
        }

        // Counter for SDO Check
        nCnt++;
#endif

        // if Single Exec mode, send pulse to Runtime Thread
        if(g_Arg.bSingleExec == TRUE)
        {
            MSG_SendPulse(g_coidSCTime, RUNSERV_TIMER, 0);
        }

        THREAD_Sleep(g_nIoScanTime);  // period of I/O monitoring
    }

    g_fScanSDOInputThreadExitState = TRUE;
    g_fScanSDOInputThreadRunState = FALSE;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// MappingInputThreadRoutine()
//

THREAD_ENTRY_TYPE MappingInputThreadRoutine(void* pParam)
{
    while (g_fExit == FALSE)
	{
        g_fMappingInputThreadRunState = RUN;

        /* Digital & Analog inport ports data copy to Packet & SHM */

        /* Input map to SHM array */
        _loc_FUNC_InputScanResultLoadToShmem(INPUT_TO_VAR);
        _loc_FUNC_InputScanResultLoadToShmem(INPUT_TO_ARRAY);

        /* Reply Packet Mapping */
        _loc_FUNC_ReplyPacketMapping();

        THREAD_Sleep(g_nIoScanTime);  // period of I/O monitoring
    }

    g_fMappingInputThreadExitState = TRUE;
    g_fMappingInputThreadRunState = FALSE;

    return 0;
}
