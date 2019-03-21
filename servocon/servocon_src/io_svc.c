/////////////////////////////////////////////////////////////////////////////
//
//  io_svc.c: I/O Monitor & Control Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable
unsigned short g_nReadStatusValue[ROB_AXIS_COUNT];
unsigned short g_DinPortVal[ROBOT_DI_SLAVE_COUNT][SLAVE_DI_PORT_COUNT];
unsigned short g_DoutPortVal[ROBOT_DO_SLAVE_COUNT][SLAVE_DO_PORT_COUNT];
ECAT_REAL32    g_AinPortVal[ROBOT_AI_SLAVE_COUNT][SLAVE_AI_PORT_COUNT];     //-10~10
ECAT_REAL32    g_AoutPortVal[ROBOT_AO_SLAVE_COUNT][SLAVE_AO_PORT_COUNT];    //-10~10
int g_fIoTestModeActive = OFF;
int g_nAnalogIOUnitTypeBySVC = 0;
int g_fAnalogOutUnitPreDefined = OFF;
int g_fShockSensorRelease = OFF;

/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// SERV_EcatDigitalOut()
//
// -nIndex: index of IO device
// -nValue: TRUE(on)/FALSE(off)
//
int SERV_EcatDigitalOut(int nSlave, int nIndex, BOOL bValue)
{
#if defined (__QNXNTO__)
	int nRet;

    if (bValue == TRUE)
    {
        nRet = ECATSERV_WriteDigitalOut(nSlave, nIndex, TRUE);
    }
    else
    {
        nRet = ECATSERV_WriteDigitalOut(nSlave, nIndex, FALSE);
    }
    
    return nRet;
#else
    if (bValue == TRUE)
    {
        //VERBOSE_VERBOSE("%d - %d IO is ON\n", nSlave, nIndex);
    }
    else
    {
        //VERBOSE_VERBOSE("%d - %d IO is OFF\n", nSlave, nIndex);        
    }

    return RESULT_OK;
#endif    
}


////////////////////////////////////////////////////////////////////////////////
//
// Scan Input Service

////////////////////////////////////////////////////////////////////////////////
//
// SERV_Scan_Welder_IO()
//
// -nOpt: option for scan range (Din,Ain,Dout,Aout..etc)
// 0: All
// 1: Weld Digital Input        2: Weld Analog Intput
// 3: Weld Digital Output       4: Weld Analog Output
//
// Welder I/O Config
// Din -> 0: Arc    1: No Gas   2: No Wire  3: Welder Power Fail
//        4: Touch Process      5: Touch Signal
// Ain -> 0: Volt In            1: Current In
// Dout-> 0: Arc ON 1: Gas On   2: Wire Inching +   3: Wrie Inching -
//        4: Touch Start        5: Touch Ready(MC On)
// Aout-> 0: Volt Out           1: Current Out
//

int SERV_Scan_Welder_IO(int nOpt)
{
    if(nOpt == 0)
    {
        // Digital In
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[ARCON_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fArcOnInState;
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[NOGAS_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fNoGasInState;
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[NOWIRE_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fNoWireInState;
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[WELDERPWRFAIL_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fWeldPowerFailInState;
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[TOUCHPROCESS_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fTouchProcessInState;
        SC_reply.data.welder_mon.weldDin.nWeldDInPortVal[TOUCHSIGNAL_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fTouchSignalInState;

        // Analog In
        SC_reply.data.welder_mon.weldAin.dbWeldAInPortVal[VOLT_AI_PORT_NO] = 
            g_pShmem_sc->inputstate.dbWeldVoltInVal;
        SC_reply.data.welder_mon.weldAin.dbWeldAInPortVal[CURR_AI_PORT_NO] = 
            g_pShmem_sc->inputstate.dbWeldCurrInVal;

        // Digital Out
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[ARCON_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fArcOnOutState;
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[GASON_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fGasOnOutState;
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fInchingPosOutState;
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fInchingNegOutState;
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fTouchStartOutState;
        SC_reply.data.welder_mon.weldDout.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fTouchReadyOutState;

        // Analog Out
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_RAW)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_RAW)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_WELD)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_WELD)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_INCH)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_INCH)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
    }
    else if(nOpt == 1)
    {
        // Digital In
        SC_reply.data.weldDin.nWeldDInPortVal[ARCON_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fArcOnInState;
        SC_reply.data.weldDin.nWeldDInPortVal[NOGAS_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fNoGasInState;
        SC_reply.data.weldDin.nWeldDInPortVal[NOWIRE_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fNoWireInState;
        SC_reply.data.weldDin.nWeldDInPortVal[WELDERPWRFAIL_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fWeldPowerFailInState;
        SC_reply.data.weldDin.nWeldDInPortVal[TOUCHPROCESS_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fTouchProcessInState;
        SC_reply.data.weldDin.nWeldDInPortVal[TOUCHSIGNAL_DI_PORT_NO] = 
            g_pShmem_sc->inputstate.fTouchSignalInState;
    }
    else if(nOpt == 2)
    {
        // Analog In
        SC_reply.data.weldAin.dbWeldAInPortVal[VOLT_AI_PORT_NO] = 
            g_pShmem_sc->inputstate.dbWeldVoltInVal;
        SC_reply.data.weldAin.dbWeldAInPortVal[CURR_AI_PORT_NO] = 
            g_pShmem_sc->inputstate.dbWeldCurrInVal;
    }
    else if(nOpt == 3)
    {
        // Digital Out
        SC_reply.data.weldDout.nWeldDOutPortVal[ARCON_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fArcOnOutState;
        SC_reply.data.weldDout.nWeldDOutPortVal[GASON_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fGasOnOutState;
        SC_reply.data.weldDout.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fInchingPosOutState;
        SC_reply.data.weldDout.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fInchingNegOutState;
        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fTouchStartOutState;
        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = 
            g_pShmem_sc->outputstate.fTouchReadyOutState;
    }
    else if(nOpt == 4)
    {
        // Analog Out
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_RAW)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_RAW)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_WELD)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_WELD)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[VOLT_AO_PORT_NO] == ANALOG_IO_TYPE_INCH)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[VOLT_AO_PORT_NO];
        }
        if(g_pShmem_sc->sysstate.nAoutMappingType[CURR_AO_PORT_NO] == ANALOG_IO_TYPE_INCH)
        {
            SC_reply.data.welder_mon.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] =
                g_pShmem_sc->outputcmd.dbAoutPortCmd[CURR_AO_PORT_NO];
        }
    }

    // Determine Packet Size
    if(nOpt == 0)
        SC_reply.size = sizeof(SC_reply.data.welder_mon);
    else if(nOpt == 1)
        SC_reply.size = sizeof(SC_reply.data.weldDin);
    else if(nOpt == 2)
        SC_reply.size = sizeof(SC_reply.data.weldAin);
    else if(nOpt == 3)
        SC_reply.size = sizeof(SC_reply.data.weldDout);
    else if(nOpt == 4)
        SC_reply.size = sizeof(SC_reply.data.weldAout);

    SC_reply.code = SC_msg.code;
    SC_reply.value = SC_msg.value;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_Scan_Servo_IO()
//
// -nOpt: option for scan range (Din,Ain,Dout,Aout..etc)
// 0: All
// 1: Brake Digital Input       2: Brake ECAT Output
//
// Servo I/O Config
// Din -> 0~5: Brake Status   6~11: Brake Clear
// ECATout -> 0~5: Brake Release Out
//
int SERV_Scan_Servo_IO(int nOpt)
{
    int iAxis;

    if(nOpt == 0)
    {
        // Brake State & Clear Digtal In
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.servo_mon.servoin.nBrakeDInPortVal[iAxis] =
                g_pShmem_sc->inputstate.fBrakeStatusInState[iAxis];

            SC_reply.data.servo_mon.servoin.nBrakeDInPortVal[g_nAxisCount + iAxis] =
                g_pShmem_sc->inputstate.fBrakeClearInState[iAxis];
        }

        // Servo On Output State
        SC_reply.data.servo_mon.servoout.fServoOnOutState = 
            g_pShmem_sc->outputstate.fServoOnOutState;

        // Brake Release Output State
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.servo_mon.servoout.fBrakeReleaseOutState[iAxis] = 
              g_pShmem_sc->outputstate.fBrakeReleaseOutState[iAxis];
        }
    }
    else if(nOpt == 1)
    {
        // Brake State & Clear Digtal In
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.servoin.nBrakeDInPortVal[iAxis] =
                g_pShmem_sc->inputstate.fBrakeStatusInState[iAxis];

            SC_reply.data.servoin.nBrakeDInPortVal[g_nAxisCount + iAxis] =
                g_pShmem_sc->inputstate.fBrakeClearInState[iAxis];
        }
    }
    else if(nOpt == 2)
    {
        // Servo On Output State
        SC_reply.data.servoout.fServoOnOutState = 
            g_pShmem_sc->outputstate.fServoOnOutState;

         // Brake Release Output State
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.servoout.fBrakeReleaseOutState[iAxis] = 
              g_pShmem_sc->outputstate.fBrakeReleaseOutState[iAxis];
        }
    }
    else if(nOpt == 3)
    {
        // ABS Encoder Reset State
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.absEncstate.fABSEncResetRequestState[iAxis] = 
                                            g_fABSEncResetRequestflag[iAxis];
            SC_reply.data.absEncstate.fABSEncResetDoneState[iAxis] = 
                                            g_fABSEncResetDoneflag[iAxis];
        }
    }
    
    // Determine Packet Size
    if(nOpt == 0)
    {
        SC_reply.size = sizeof(SC_reply.data.servo_mon);
    }
    else if(nOpt == 1)
    {
        SC_reply.size = sizeof(SC_reply.data.servoin);
    }
    else if(nOpt == 2)
    {
        SC_reply.size = sizeof(SC_reply.data.servoout);
    }
    else if(nOpt == 3)
    {
        SC_reply.size = sizeof(SC_reply.data.absEncstate);
    }
    
    SC_reply.code = SC_msg.code;
    SC_reply.value = SC_msg.value;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_Scan_System_State()
//
// -nOpt: option for scan range (Din,Ain,Dout,Aout..etc)
// 0: All                       1: System State
// 2: System I/O                3: Actual Position
// 4: Actual Position (Joint Coord Only)
int SERV_Scan_System_State(int nOpt)
{
    int iAxis;

    if(nOpt == 0)
    {
        // System State
        SC_reply.data.realtime_sysmon.sysstate.fEcatInitState = 
            g_pShmem_sc->sysstate.fEcatInitState;
        SC_reply.data.realtime_sysmon.sysstate.fErrorState = 
            g_pShmem_sc->sysstate.fErrorState;
        SC_reply.data.realtime_sysmon.sysstate.fEStopState = 
            g_pShmem_sc->sysstate.fEStopState;
        SC_reply.data.realtime_sysmon.sysstate.nErrorCode = 
            g_pShmem_sc->sysstate.nErrorCode;

        // System I/O State
        SC_reply.data.realtime_sysmon.sysiostate.fController_EstopInState = 
            g_pShmem_sc->inputstate.fController_EstopInState;
        SC_reply.data.realtime_sysmon.sysiostate.fDeadManSwithInState = 
            g_pShmem_sc->inputstate.fDeadManSwithInState;
        SC_reply.data.realtime_sysmon.sysiostate.fTP_EstopInState = 
            g_pShmem_sc->inputstate.fTP_EstopInState;
        SC_reply.data.realtime_sysmon.sysiostate.fShockSensorInState = 
            g_pShmem_sc->inputstate.fShockSensorInState;

        // Actual Position
        if(g_pShmem_Status_te != NULL)
        {
            SC_reply.data.realtime_sysmon.pos.nCoordIdx = g_pShmem_Status_te->coord_ref;
        
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
            {
                if(g_pShmem_Status_te->coord_ref == COORDINDEX_JOINT)
                {
                    SC_reply.data.realtime_sysmon.pos.dbCurrPos[iAxis] = 
                        g_pShmem_sc->inputstate.dbActPos[iAxis] * (180/M_PI);
                        //g_pShmem_Status_te->xyzrpy_act[iAxis] * (180/M_PI);
                }
                else
                {
                    if(iAxis >= 0 && iAxis < 3)
                    {
                        SC_reply.data.realtime_sysmon.pos.dbCurrPos[iAxis] = 
                           g_pShmem_Status_te->xyzrpy_act[iAxis];                   //mm
                    }
                    else
                    {
                        SC_reply.data.realtime_sysmon.pos.dbCurrPos[iAxis] = 
                            g_pShmem_Status_te->xyzrpy_act[iAxis] * (180/M_PI);    //deg
                    }
                }
            }
        }
    }
    else if(nOpt == 1)
    {
        // System State
        SC_reply.data.sysstate.fEcatInitState = 
            g_pShmem_sc->sysstate.fEcatInitState;
        SC_reply.data.sysstate.fErrorState = 
            g_pShmem_sc->sysstate.fErrorState;
        SC_reply.data.sysstate.fEStopState = 
            g_pShmem_sc->sysstate.fEStopState;
        SC_reply.data.sysstate.nErrorCode = 
            g_pShmem_sc->sysstate.nErrorCode;
        SC_reply.data.sysstate.nErrorAxis = 
            g_pShmem_sc->sysstate.nErrorAxis;
    }
    else if(nOpt == 2)
    {
        // System I/O State
        SC_reply.data.sysiostate.fController_EstopInState = 
            g_pShmem_sc->inputstate.fController_EstopInState;
        SC_reply.data.sysiostate.fDeadManSwithInState = 
            g_pShmem_sc->inputstate.fDeadManSwithInState;
        SC_reply.data.sysiostate.fTP_EstopInState = 
            g_pShmem_sc->inputstate.fTP_EstopInState;
        SC_reply.data.sysiostate.fShockSensorInState = 
            g_pShmem_sc->inputstate.fShockSensorInState;
    }
    else if(nOpt == 3)
    {
        if(g_pShmem_Status_te != NULL)
        {
            SC_reply.data.pos.nCoordIdx = g_pShmem_Status_te->coord_ref;

            // Actual Position
            for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
            {
                if(g_pShmem_Status_te->coord_ref== COORDINDEX_JOINT)
                {
                    SC_reply.data.pos.dbCurrPos[iAxis] = 
                        g_pShmem_sc->inputstate.dbActPos[iAxis] * (180/M_PI);
                        //g_pShmem_Status_te->xyzrpy_act[iAxis] * (180/M_PI);
                }
                else
                {
                    if(iAxis >= 0 && iAxis < 3)
                    {
                        SC_reply.data.pos.dbCurrPos[iAxis] = 
                            g_pShmem_Status_te->xyzrpy_act[iAxis];       // mm
                    }
                    else
                    {
                        SC_reply.data.pos.dbCurrPos[iAxis] = 
                            g_pShmem_Status_te->xyzrpy_act[iAxis] * (180/M_PI);  // deg
                    }
                }
            }
        }
    }
    else if(nOpt == 4)
    {
        // Actual Position
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.jnt_pos.dbCurrJntPos[iAxis] = 
                g_pShmem_sc->inputstate.dbActPos[iAxis] * (180/M_PI);
        }
    }
    else if(nOpt == 5)
    {
        // Actual Torque
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.jnt_torque.dbActTorque[iAxis] = g_dbActTorque[iAxis];
        }
    }
    else if(nOpt == 6)
    {
        // Target Position
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis ++)
        {
            SC_reply.data.trg_pos.dbTargetJntPos[iAxis] = 
                           g_pShmem_sc->outputcmd.dbTrgPos[iAxis] * (180/M_PI);
        }
    }


    // Determine Packet Size
    if(nOpt == 0)
    {
        SC_reply.size = sizeof(SC_reply.data.realtime_sysmon);
    }
    else if(nOpt == 1)
    {
        SC_reply.size = sizeof(SC_reply.data.sysstate);
    }
    else if(nOpt == 2)
    {
        SC_reply.size = sizeof(SC_reply.data.sysiostate);
    }
    else if(nOpt == 3)
    {
        SC_reply.size = sizeof(SC_reply.data.pos);
    }
    else if(nOpt == 4)
    {
        SC_reply.size = sizeof(SC_reply.data.jnt_pos);      // except realtime_sysmon
    }
    else if(nOpt == 5)
    {
        SC_reply.size = sizeof(SC_reply.data.jnt_torque);   // except realtime_sysmon
    }
    else if(nOpt == 6)
    {
        SC_reply.size = sizeof(SC_reply.data.trg_pos);      // except realtime_sysmon
    }

    SC_reply.code = SC_msg.code;
    SC_reply.value = SC_msg.value;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// Analog Out Service


////////////////////////////////////////////////////////////////////////////////
//
// SERV_WeldVolt_out()
//
// -dbValue: value of analog output
//
int SERV_WeldVolt_out(double dbValue)
{
    int nRet = 0;

    dbValue = dbValue * 0.1;
    
    if(dbValue >= ANALOG_MAX_CMD_VOLTAGE)
    {
        dbValue = ANALOG_MAX_CMD_VOLTAGE;
    }
    else if(dbValue <= -1 * ANALOG_MAX_CMD_VOLTAGE)
    {
        dbValue = -1 * ANALOG_MAX_CMD_VOLTAGE;
    }

    g_pShmem_sc->sysstate.nAoutMappingType[g_Aout_portno.nVoltageOutPortNo] = ANALOG_IO_TYPE_RAW;
    g_pShmem_sc->outputcmd.dbAoutPortCmd[g_Aout_portno.nVoltageOutPortNo] = (float) dbValue;
    VERBOSE_MESSAGE("[Raw Data]Weld Voltage Out: %.2lf [V]\n", dbValue);
    
    //SC_reply.data.weldAout.dbWeldAOutPortVal[VOLT_AO_PORT_NO] = g_pShmem_sc->outputstate.dbWeldVoltOutVal;
    g_pShmem_sc->outputstate.dbWeldVoltOutVal = (dbValue * g_nMaxWeldVoltage) / ANALOG_MAX_CMD_VOLTAGE;
//    g_pShmem_sc->outputstate.weldAoutState.dbWeldAOutPortVal[VOLT_AO_PORT_NO] = dbValue;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_WeldCurr_out()
//
// -dbValue: value of analog output
//
int SERV_WeldCurr_out(double dbValue)
{
    int nRet = 0;

    dbValue = dbValue * 0.1;

    if(dbValue >= ANALOG_MAX_CMD_VOLTAGE)
    {
        dbValue = ANALOG_MAX_CMD_VOLTAGE;
    }
    else if(dbValue <= -1 * ANALOG_MAX_CMD_VOLTAGE)
    {
        dbValue = -1 * ANALOG_MAX_CMD_VOLTAGE;
    }

    g_pShmem_sc->sysstate.nAoutMappingType[g_Aout_portno.nCurrentOutPortNo] = ANALOG_IO_TYPE_RAW;
    g_pShmem_sc->outputcmd.dbAoutPortCmd[g_Aout_portno.nCurrentOutPortNo] = (float) dbValue;
    VERBOSE_MESSAGE("[Raw Data]Weld Current Out: %.2lf [V]\n", dbValue);

    //SC_reply.data.weldAout.dbWeldAOutPortVal[CURR_AO_PORT_NO] = g_pShmem_sc->outputstate.dbWeldCurrOutVal;
    g_pShmem_sc->outputstate.dbWeldCurrOutVal = (dbValue * g_nMaxWeldCurrent) / ANALOG_MAX_CMD_VOLTAGE;
//    g_pShmem_sc->outputstate.weldAoutState.dbWeldAOutPortVal[CURR_AO_PORT_NO] = dbValue;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// Digital Out Service (Weld Dout)

////////////////////////////////////////////////////////////////////////////////
//
// SERV_ArcOn_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_ArcOn_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, ARCON_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Arc On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[ARCON_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fArcOnOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[ARCON_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nArcOn] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, ARCON_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Arc Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[ARCON_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fArcOnOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[ARCON_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_GasOn_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_GasOn_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, GASON_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Gas On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[GASON_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fGasOnOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[GASON_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nGasOn] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, GASON_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Gas Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[GASON_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fGasOnOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[GASON_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_InchingPos_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_InchingPos_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nInchPos] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, INCHPOS_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Inching Positive On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fInchingPosOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nInchPos] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, INCHPOS_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Inching Positive Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fInchingPosOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[INCHPOS_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_InchingNeg_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_InchingNeg_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nInchNeg] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, INCHNEG_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Inching Negative On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fInchingNegOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nInchNeg] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, INCHNEG_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Inching Negative Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fInchingNegOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[INCHNEG_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_TouchStart_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_TouchStart_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, TOUCHSTART_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Touch Start On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fTouchStartOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchStart] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, TOUCHSTART_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Touch Start Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fTouchStartOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[TOUCHSTART_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_TouchReady_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_TouchReady_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady] = ON;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, TOUCHREADY_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Touch Ready On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fTouchReadyOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[g_Dout_portno.nTouchReady] = OFF;
        //nRet = SERV_EcatDigitalOut(WELD_DO_SLAVE_NO, TOUCHREADY_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Touch Ready Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fTouchReadyOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[TOUCHREADY_DO_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// Digital Out Service (Lamp Dout)



////////////////////////////////////////////////////////////////////////////////
//
// SERV_LampControllerReady_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_LampControllerReady_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_CONT_READY_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_CONT_READY_DO_PORT_NO, ON);
        //VERBOSE_MESSAGE("Controller Ready Lamp On.\n");

        g_pShmem_sc->outputstate.fLampControllerReadyState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_CONT_READY_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_CONT_READY_DO_PORT_NO, OFF);
        //VERBOSE_MESSAGE("Controller Ready Lamp Off.\n");

        g_pShmem_sc->outputstate.fLampControllerReadyState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_LampUnderOperating_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_LampUnderOperating_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_OPERATING_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_OPERATING_DO_PORT_NO, ON);
        //VERBOSE_MESSAGE("Under Operating Lamp On.\n");

        g_pShmem_sc->outputstate.fLampUnderOperatingState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_OPERATING_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_OPERATING_DO_PORT_NO, OFF);
        //VERBOSE_MESSAGE("Under Operating Lamp Off.\n");

        g_pShmem_sc->outputstate.fLampUnderOperatingState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_LampServoOn_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_LampServoOn_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_SERVOON_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_SERVOON_DO_PORT_NO, ON);
        //VERBOSE_MESSAGE("Servo On Lamp On.\n");

        g_pShmem_sc->outputstate.fLampServoOnState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_SERVOON_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_SERVOON_DO_PORT_NO, OFF);
        //VERBOSE_MESSAGE("Servo On Lamp Off.\n");

        g_pShmem_sc->outputstate.fLampServoOnState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_LampEtherCATRun_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_LampEtherCATRun_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_ECAT_RUN_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_ECAT_RUN_DO_PORT_NO, ON);
        //VERBOSE_MESSAGE("EtherCAT Run Lamp On.\n");

        g_pShmem_sc->outputstate.fLampEtherCATRunState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_ECAT_RUN_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_ECAT_RUN_DO_PORT_NO, OFF);
        //VERBOSE_MESSAGE("EtherCAT Run Lamp Off.\n");

        g_pShmem_sc->outputstate.fLampEtherCATRunState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_LampError_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_LampError_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_ERROR_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_ERROR_DO_PORT_NO, ON);
        //VERBOSE_MESSAGE("Error Lamp On.\n");

        g_pShmem_sc->outputstate.fLampErrorState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[LAMP_DO_SLAVE_OFFSET + LAMP_ERROR_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(LAMP_DO_SLAVE_NO, LAMP_ERROR_DO_PORT_NO, OFF);
        //VERBOSE_MESSAGE("Error Lamp Off.\n");

        g_pShmem_sc->outputstate.fLampErrorState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// Digital Out Service (Lamp Dout)

#define CART_WIRECUT_SEND_PORT_NO       6

////////////////////////////////////////////////////////////////////////////////
//
// SERV_CartLampAlarm_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_CartLampAlarm_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_LAMP_ALARM_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_LAMP_ALARM_DO_PORT_NO, ON);

        g_pShmem_sc->outputstate.fCartLampAlarmState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_LAMP_ALARM_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_LAMP_ALARM_DO_PORT_NO, OFF);

        g_pShmem_sc->outputstate.fCartLampAlarmState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_CartLampRun_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_CartLampRun_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_LAMP_RUN_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_LAMP_RUN_DO_PORT_NO, ON);

        g_pShmem_sc->outputstate.fCartLampRunState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_LAMP_RUN_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_LAMP_RUN_DO_PORT_NO, OFF);

        g_pShmem_sc->outputstate.fCartLampRunState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_WireCut_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_WireCut_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_WIRECUT_DO_PORT_NO] = ON;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_WIRECUT_DO_PORT_NO, ON);
        VERBOSE_MESSAGE("Wire Cut On.\n");

        SC_reply.data.weldDout.nWeldDOutPortVal[CART_WIRECUT_SEND_PORT_NO] = ON;
        g_pShmem_sc->outputstate.fWireCutOutState = ON;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[CART_WIRECUT_SEND_PORT_NO] = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_WIRECUT_DO_PORT_NO] = OFF;
        //nRet = SERV_EcatDigitalOut(CART_DO_SLAVE_NO, CART_WIRECUT_DO_PORT_NO, OFF);
        if(g_fDoutsSetZero == OFF)
        {
            VERBOSE_MESSAGE("Wire Cut Off.\n");
        }

        SC_reply.data.weldDout.nWeldDOutPortVal[CART_WIRECUT_SEND_PORT_NO] = OFF;
        g_pShmem_sc->outputstate.fWireCutOutState = OFF;
//        g_pShmem_sc->outputstate.weldDoutState.nWeldDOutPortVal[CART_WIRECUT_SEND_PORT_NO] = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_CartJobStartConfirm_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_CartJobStartConfirm_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_JOBSTARTCON_DO_PORT_NO] = ON;
        //VERBOSE_MESSAGE("Job Start Button Light On.\n");

        g_pShmem_sc->outputstate.fCartJobStartConfirmState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_JOBSTARTCON_DO_PORT_NO] = OFF;
        if(g_fDoutsSetZero == OFF)
        {
            //VERBOSE_MESSAGE("Job Start Button Light Off.\n");
        }

        g_pShmem_sc->outputstate.fCartJobStartConfirmState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_CartDoutSpare_out()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_CartDoutSpare_out(int nValue)
{
    int nRet = 0;

    if(nValue == ON)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_SPARE_DO_PORT_NO] = ON;
        //VERBOSE_MESSAGE("Cart Spare Dout On.\n");

        g_pShmem_sc->outputstate.fCartDoutSpareState = ON;
    }
    else if(nValue == OFF)
    {
        g_pShmem_sc->outputcmd.nDoutPortCmd[CART_DO_SLAVE_OFFSET + CART_SPARE_DO_PORT_NO] = OFF;
        if(g_fDoutsSetZero == OFF)
        {
            //VERBOSE_MESSAGE("Cart Spare Dout Off.\n");
        }

        g_pShmem_sc->outputstate.fCartDoutSpareState = OFF;
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_WriteDoutPort()
//

int SERV_WriteDoutPort(int nRecvValue)
{
    VALUE_OUT* pOutVal;
    int nPort;
    int nPortVal;

    pOutVal = (VALUE_OUT*)&nRecvValue;

    nPort = pOutVal->nPort;
    nPortVal = (0xffff) & (pOutVal->nVal);
    g_pShmem_sc->outputcmd.nDoutPortCmd[nPort] = nPortVal;

    if(nPort < 0 || nPort >= ROBOT_DO_PORT_COUNT)
    {
        VERBOSE_ERROR("Invalid Port No: %d\n", nPort);
        return RESULT_ERROR;
    }

    if(nPortVal < 0)
    {
        VERBOSE_ERROR("Invalid Port Value: %d\n", nPortVal);
        return RESULT_ERROR;
    }
    
    VERBOSE_MESSAGE("[Dout]Port No: %d, Port Value: %d\n", nPort, nPortVal);

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_WriteAoutPort()
//

int SERV_WriteAoutPort(int nRecvValue, int nOpt)
{
    VALUE_OUT* pOutVal;
    int nPort;
    int nPortVal;
    double dbPortVal = 0.;
    double dbMaxPortVal = 0.;

    /* Define Aout Unit Type */
    if(g_fAnalogOutUnitPreDefined == OFF)
    {
        if(nOpt == ANALOG_IO_TYPE_RAW)
        {
            g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_RAW;
            VERBOSE_MESSAGE("Analog I/O Unit Set to Controller Based Raw Data Unit [-5~5 V]\n");
        }
        else if(nOpt == ANALOG_IO_TYPE_WELD)
        {
            g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_WELD;
            VERBOSE_MESSAGE("Analog I/O Unit Set to Welding Current/Voltage Data Unit [500A, 45V]\n");
        }
        else if(nOpt == ANALOG_IO_TYPE_INCH)
        {
            g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_INCH;
            VERBOSE_MESSAGE("Analog I/O Unit Set to Inching Speed Data Unit [0~1000 mm/s]\n");
        }
    }

    /* Define Aout Port & Value */
    pOutVal = (VALUE_OUT*)&nRecvValue;

    nPort = pOutVal->nPort;
    nPortVal = (0xffff) & (pOutVal->nVal);

    // Cmd Vaule: -50~50(-5~5Volt), Unit:Vaule 1=0.1 Volt
    if(g_nAnalogIOUnitTypeBySVC == ANALOG_IO_TYPE_RAW)
    {
        dbPortVal = (double) nPortVal * 0.1;
        dbMaxPortVal = ANALOG_MAX_CMD_VOLTAGE;

        if(dbPortVal >= dbMaxPortVal)
        {
            dbPortVal = dbMaxPortVal;
        }
        else if(dbPortVal <= -1 * dbMaxPortVal)
        {
            dbPortVal = -1 * dbMaxPortVal;
        }

        g_pShmem_sc->sysstate.nAoutMappingType[nPort] = ANALOG_IO_TYPE_RAW;
        g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] = dbPortVal;
        VERBOSE_MESSAGE("[Raw-Aout]Port No: %d, Port Value: %.2lf [V]\n",
                        nPort, dbPortVal);
    }
    
    // Cmd Vaule: 0~450(0~5Volt)
    if(g_nAnalogIOUnitTypeBySVC == ANALOG_IO_TYPE_WELD && nPort == g_Aout_portno.nVoltageOutPortNo)
    {
        dbPortVal = (double) nPortVal / g_nMaxWeldVoltage;
        dbMaxPortVal = g_nMaxWeldVoltage * ANALOG_MAX_ABS_VOLTAGE;

        if(dbPortVal >= dbMaxPortVal)
        {
            dbPortVal = dbMaxPortVal;
        }
        else if(dbPortVal <= -1 * dbMaxPortVal)
        {
            dbPortVal = -1 * dbMaxPortVal;
        }

        g_pShmem_sc->sysstate.nAoutMappingType[nPort] = ANALOG_IO_TYPE_WELD;
        g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] = (dbPortVal * g_nMaxWeldVoltage) / ANALOG_MAX_ABS_VOLTAGE;
        
        VERBOSE_MESSAGE("[Weld-Aout]Port No: %d, Port Value: %.2lf [V]\n",
                        nPort, (double) nPortVal / ANALOG_MAX_ABS_VOLTAGE);
    }
    // Cmd Vaule: 0~5000(0~5Volt)
    else if(g_nAnalogIOUnitTypeBySVC == ANALOG_IO_TYPE_WELD && nPort == g_Aout_portno.nCurrentOutPortNo)
    {
        dbPortVal = (double) nPortVal / g_nMaxWeldCurrent;
        dbMaxPortVal = g_nMaxWeldCurrent * ANALOG_MAX_ABS_VOLTAGE;

        if(dbPortVal >= dbMaxPortVal)
        {
            dbPortVal = dbMaxPortVal;
        }
        else if(dbPortVal <= -1 * dbMaxPortVal)
        {
            dbPortVal = -1 * dbMaxPortVal;
        }

        g_pShmem_sc->sysstate.nAoutMappingType[nPort] = ANALOG_IO_TYPE_WELD;
        g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] = (dbPortVal * g_nMaxWeldCurrent) / ANALOG_MAX_ABS_VOLTAGE;
        
        VERBOSE_MESSAGE("[Weld-Aout]Port No: %d, Port Value: %.2lf [A]\n",
                        nPort, (double) nPortVal / ANALOG_MAX_ABS_VOLTAGE);
    }
    // Cmd Vaule: 0
    if(g_nAnalogIOUnitTypeBySVC == ANALOG_IO_TYPE_INCH && nPort == g_Aout_portno.nVoltageOutPortNo)
    {
        dbPortVal = 0;
        
        g_pShmem_sc->sysstate.nAoutMappingType[nPort] = ANALOG_IO_TYPE_INCH;
        g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] = dbPortVal;
        
        VERBOSE_WARNING("Now, A-out Type Setting is Inching! Force Voltage out Set to Zero!\n");
        VERBOSE_MESSAGE("[Inch-Aout]Port No: %d, Port Value: %.2lf [V]\n",
                        nPort, g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort]);
    }
    // Cmd Vaule: 0~10000
    else if(g_nAnalogIOUnitTypeBySVC == ANALOG_IO_TYPE_INCH && nPort == g_Aout_portno.nCurrentOutPortNo)
    {
        dbPortVal = (double) nPortVal / g_nMaxInchSpeed;
        dbMaxPortVal = g_nMaxInchSpeed * ANALOG_MAX_ABS_VOLTAGE;

        if(dbPortVal >= dbMaxPortVal)
        {
            dbPortVal = dbMaxPortVal;
        }
        else if(dbPortVal <= -1 * dbMaxPortVal)
        {
            dbPortVal = -1 * dbMaxPortVal;
        }

        g_pShmem_sc->sysstate.nAoutMappingType[nPort] = ANALOG_IO_TYPE_INCH;
        g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort] = (dbPortVal * g_nMaxInchSpeed) / (ANALOG_MAX_ABS_VOLTAGE * 2);
        
        VERBOSE_MESSAGE("[Incy-Aout]Port No: %d, Port Value: %.2lf [V](%.2lf[mm/s])\n",
                        //nPort, g_pShmem_sc->outputcmd.dbAoutPortCmd[nPort], (double) nPortVal * 0.1);
                        nPort, (dbPortVal / 2), (double) nPortVal * 0.1);
    }

    if(nPort < 0 || nPort >= ROBOT_AO_PORT_COUNT)
    {
        VERBOSE_ERROR("Invalid Port No: %d\n", nPort);
        return RESULT_ERROR;
    }
#if 0
    if(nPortVal < -100 && nPortVal > 100)
    {
        VERBOSE_ERROR("Invalid Port Value: %d\n", nPortVal);
        return RESULT_ERROR;
    }
#endif

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_ChangeAnalogIOUnit()
//
#define ANALOG_OUT_UNIT_PREDEFINED_ON      3
#define ANALOG_OUT_UNIT_PREDEFINED_OFF     4

int SERV_ChangeAnalogIOUnit(int nOpt)
{
    if(nOpt == ANALOG_IO_TYPE_RAW)
    {
        g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_RAW;
        VERBOSE_MESSAGE("Analog I/O Unit Set to Controller Based Raw Data Unit [-5~5 V]\n");
    }
    else if(nOpt == ANALOG_IO_TYPE_WELD)
    {
        g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_WELD;
        VERBOSE_MESSAGE("Analog I/O Unit Set to Welding Current/Voltage Data Unit [500A, 45V]\n");
    }
    else if(nOpt == ANALOG_IO_TYPE_INCH)
    {
        g_nAnalogIOUnitTypeBySVC = ANALOG_IO_TYPE_INCH;
        VERBOSE_MESSAGE("Analog I/O Unit Set to Inching Speed Data Unit [0~1000 mm/s]\n");
    }
    else if(nOpt == ANALOG_OUT_UNIT_PREDEFINED_ON)
    {
        g_fAnalogOutUnitPreDefined = ON;
    }
    else if(nOpt == ANALOG_OUT_UNIT_PREDEFINED_OFF)
    {
        g_fAnalogOutUnitPreDefined = OFF;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// SERV_ShockSensorRelease()
//

int  SERV_ShockSensorRelease(int nValue)
{
    if(nValue == ON)
    {
        g_fShockSensorRelease = ON;
        VERBOSE_MESSAGE("Shock Sensor E-Stop Check Released!\n");

        if(g_pShmem_sc->sysstate.fEStopState == TRUE)
           //&& g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_SHOCKSENSOR)
        {
            g_pShmem_sc->sysstate.fEStopState = FALSE;
        }
    }
    else if(nValue == OFF)
    {
        g_fShockSensorRelease = OFF;
        VERBOSE_MESSAGE("Shock Sensor E-Stop Check Start!\n");
    }
    
    return 0;
}