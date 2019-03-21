/////////////////////////////////////////////////////////////////////////////
//
//  motor_svc.c: Motor Setup & Control Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include <math.h>
#include "service.h"

///////////////////////////////////////


///////////////////////////////////////
//Global_variable
int g_fTrgPosDiff = 0;
int g_nProfSecState = 0;
int g_fDoutsSetZero = OFF;
int g_fABSEncResetRequestflag[ROB_AXIS_COUNT] = {0};
int g_fABSEncResetDoneflag[ROB_AXIS_COUNT] = {0};
int g_fPosFileWrite = OFF;

/////////////////////////////////////////////////////////////////////////////

int FUNC_SyncActualPosToTargetPos(void);

////////////////////////////////////////////////////////////////////////////////
//
// SERV_ABSEncoderReset()
//
int SERV_ABSEncoderReset(int nAxis)
{
    int nRet = 0;

    if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
    {
        SERV_ServoOnCmd(OFF);
    }

    g_fABSEncResetRequestflag[nAxis] = ON;
    g_fABSEncResetDoneflag[nAxis] = OFF;
    
    g_fAbsEncResetEventActive = ON;
#if defined (__QNXNTO__)
    // Servo State set to switch on disable
    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);

    // Make ABS value Initialize
    nRet = ECATSERV_ABSEncoderReset(nAxis);
#else
    g_nAct_Pulse[nAxis] = 0;
            
    FUNC_SyncActualPosToTargetPos();
    g_nTrg_Pulse[nAxis] = g_nAct_Pulse[nAxis];
    
    FUNC_ConvertPosToPulse(nAxis, TARGET_POS_IDX);

#endif
    if(nRet == RESULT_OK)
    {
        // Home offset set to new value
        nRet = ECATSERV_WriteHomeOffsetVal(nAxis);
        
        if(nRet == RESULT_OK)
        {
            VERBOSE_MESSAGE("Axis-%d ABS Encoder Reset Done!\n", nAxis);
        }
        else
        {
            VERBOSE_ERROR("Axis-%d Home Offset Apply Fail!\n", nAxis);
        }
    }
    else
    {
        VERBOSE_ERROR("Axis-%d ABS Encoder Reset Fail!\n", nAxis);
    }
#if defined (__QNXNTO__)    
    // User Parameter set to new value
    ECATSERV_ServoUserParamReset(nAxis);

    // Store Parameters to EEPROM
    ECATSERV_StoreParameters(nAxis);
    
    // Servo State set to switch on disable
    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);
#endif
    g_fABSEncResetRequestflag[nAxis] = OFF;
    
    if(nRet == RESULT_OK)
    {
        g_fABSEncResetDoneflag[nAxis] = ON;
    }
    else if(nRet == RESULT_ERROR)
    {
        g_pShmem_sc->sysstate.fErrorState = TRUE;
        g_pShmem_sc->sysstate.nErrorCode  = SVC_ERR_ABS_ENC_RESET;
        g_nErrAxis = nAxis;
        g_pShmem_sc->sysstate.nErrorAxis = g_nErrAxis;
    }

    g_fAbsEncResetEventActive = OFF;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_ABSEncoderResetOnly()
//
int SERV_ABSEncoderResetOnly(int nAxis)
{
    int nRet = 0;

    if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
    {
        SERV_ServoOnCmd(OFF);
    }

    g_fABSEncResetRequestflag[nAxis] = ON;
    g_fABSEncResetDoneflag[nAxis] = OFF;
    
    g_fAbsEncResetEventActive = ON;

#if defined (__QNXNTO__)
    // Servo State set to switch on disable
    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);

    // Make ABS value Initialize
    nRet = ECATSERV_ABSEncoderReset(nAxis);
#else
    g_nAct_Pulse[nAxis] = 0;
            
    FUNC_SyncActualPosToTargetPos();
    g_nTrg_Pulse[nAxis] = g_nAct_Pulse[nAxis];
    
    FUNC_ConvertPosToPulse(nAxis, TARGET_POS_IDX);

#endif
    if(nRet == RESULT_OK)
    {
        // Home offset set to new value
        //nRet = ECATSERV_WriteHomeOffsetVal(nAxis);
        VERBOSE_MESSAGE("Axis-%d ABS Encoder Reset Only Done!\n", nAxis);
    }
    else
    {
        VERBOSE_ERROR("Axis-%d ABS Encoder Reset Only Fail!\n", nAxis);
    }

#if defined (__QNXNTO__)    
    // User Parameter set to new value
    ECATSERV_ServoUserParamReset(nAxis);

    // Store Parameters to EEPROM
    ECATSERV_StoreParameters(nAxis);
    
    // Servo State set to switch on disable
    ECATSERV_ServoStateSetSwitchOnDisable(nAxis);
#endif

    g_fABSEncResetRequestflag[nAxis] = OFF;
    
    if(nRet == RESULT_OK)
    {
        g_fABSEncResetDoneflag[nAxis] = ON;
    }

    g_fAbsEncResetEventActive = OFF;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetZeroPosition()
//
int SERV_SetZeroPosition(int nAxis)
{
    int nRet = 0;

    if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
    {
        SERV_ServoOnCmd(OFF);
    }

    g_fABSEncResetRequestflag[nAxis] = ON;
    g_fABSEncResetDoneflag[nAxis] = OFF;

    g_fAbsEncResetEventActive = ON;

#if defined (__QNXNTO__)
    // Servo State set to switch on disable
    nRet = ECATSERV_ServoStateSetSwitchOnDisable(nAxis);

    // Home offset set to new value
    nRet = ECATSERV_WriteHomeOffsetVal(nAxis);
    
    // User Parameter set to new value
    ECATSERV_ServoUserParamReset(nAxis);
    
    // Store Parameters to EEPROM
    ECATSERV_StoreParameters(nAxis);
#else
    g_nAct_Pulse[nAxis] = 0;
            
    FUNC_SyncActualPosToTargetPos();
    g_nTrg_Pulse[nAxis] = g_nAct_Pulse[nAxis];
    
    FUNC_ConvertPosToPulse(nAxis, TARGET_POS_IDX);

#endif

    VERBOSE_MESSAGE("Axis-%d Position Value Reset Done!\n", nAxis);
    
    g_fABSEncResetRequestflag[nAxis] = OFF;
    
    if(nRet == RESULT_OK)
    {
        g_fABSEncResetDoneflag[nAxis] = ON;
    }
    else if(nRet == RESULT_ERROR)
    {
        g_pShmem_sc->sysstate.fErrorState = TRUE;
        g_pShmem_sc->sysstate.nErrorCode  = SVC_ERR_SET_ZERO_POSITION;
        g_nErrAxis = nAxis;
        g_pShmem_sc->sysstate.nErrorAxis = g_nErrAxis;
    }

    g_fAbsEncResetEventActive = OFF;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetHomeOffsetZeroValue()
//
int SERV_SetHomeOffsetZeroValue(int nAxis)
{
    int nRet = 0;

    if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
    {
        SERV_ServoOnCmd(OFF);
    }

    g_fABSEncResetRequestflag[nAxis] = ON;
    g_fABSEncResetDoneflag[nAxis] = OFF;

    g_fAbsEncResetEventActive = ON;

#if defined (__QNXNTO__)
    // Servo State set to switch on disable
    nRet = ECATSERV_ServoStateSetSwitchOnDisable(nAxis);

    // Home offset set to new value
    //nRet = ECATSERV_WriteHomeOffsetVal(nAxis);

    // WriteIndex: 607C, SubIndex: 0, Name: Home offset
    nRet = ECATLIB_WriteSignedIntegerSlaveCoEObject(nAxis, 0x607C, 0, 0, LEN_DINT_BYTE);
    
    // User Parameter set to new value
    ECATSERV_ServoUserParamReset(nAxis);
    
    // Store Parameters to EEPROM
    ECATSERV_StoreParameters(nAxis);
#else
    g_nAct_Pulse[nAxis] = 0;
            
    FUNC_SyncActualPosToTargetPos();
    g_nTrg_Pulse[nAxis] = g_nAct_Pulse[nAxis];
    
    FUNC_ConvertPosToPulse(nAxis, TARGET_POS_IDX);

#endif
    VERBOSE_MESSAGE("Axis-%d HomeOffset Value Reset Done!\n", nAxis);
    
    g_fABSEncResetRequestflag[nAxis] = OFF;
    
    if(nRet == RESULT_OK)
    {
        g_fABSEncResetDoneflag[nAxis] = ON;
    }

    g_fAbsEncResetEventActive = OFF;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_SERV_CheckMotionProfile()
//
#define PROF_SEC_NONE            0
#define PROF_ON_MOTION           1
#define PROF_ON_DEC_MOTION       2
#define PROF_NOT_ON_MOTION       3

void _loc_SERV_CheckMotionProfile(void)
{
    if (g_pShmem_Status_te != NULL)
    {
        if(g_pShmem_Status_te->prof_sect == PROF_A1 ||
           g_pShmem_Status_te->prof_sect == PROF_AU ||
           g_pShmem_Status_te->prof_sect == PROF_A2 ||
           g_pShmem_Status_te->prof_sect == PROF_U)
        {
            g_nProfSecState = PROF_ON_MOTION;
        }
        else if(g_pShmem_Status_te->prof_sect == PROF_D1 ||
                g_pShmem_Status_te->prof_sect == PROF_DU ||
                g_pShmem_Status_te->prof_sect == PROF_D2)
        {
            g_nProfSecState = PROF_ON_DEC_MOTION;
        }
        else
        {
            g_nProfSecState = PROF_NOT_ON_MOTION;
        }
        
        if(g_fAxisDebugMsg == TRUE)
        {
            VERBOSE_MESSAGE("Prof Sect: %d\n", g_pShmem_Status_te->prof_sect);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// _loc_SERV_Wait_CheckStopPositionValue()
//      - Option: ESTOP_MODE, STOP_MODE
//
#define TARGET_DIFF_POSITION_THRESHOLD      1.0
#define POSTION_CHECK_TIME                  10
#define SERVOOFF_POSTION_CHECK_TIME         100

void _loc_SERV_Wait_CheckStopPositionValue(int nOption)
{
    double  dbPositionVal[ROB_AXIS_COUNT];
    double  dbPositionValBuff[ROB_AXIS_COUNT];
    int     iAxis;
    double  dbTrgPosDiff = 0;
    unsigned long dbWaitCheckTime = 0;

    g_fTrgPosDiff = OFF;

    if(nOption == ESTOP_MODE)
    {
        dbWaitCheckTime = (int) g_dbRobotDecel_Estop + POSTION_CHECK_TIME;
    }
    else if(nOption == STOP_MODE)
    {
        dbWaitCheckTime = (int) g_dbRobotDecel_NormalStop + POSTION_CHECK_TIME;
    }
    else if(nOption == SRVOFF_MODE)
    {
        dbWaitCheckTime = SERVOOFF_POSTION_CHECK_TIME;
    }
    
    THREAD_Sleep(dbWaitCheckTime);

    if(nOption == ESTOP_MODE)
    {
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            dbPositionVal[iAxis] = g_dbTrg_Pos[iAxis] * (180/M_PI);
            //dbPositionVal[iAxis] = g_dbAct_Pos[iAxis] * (180/M_PI);
        }

        dbWaitCheckTime = POSTION_CHECK_TIME;
        THREAD_Sleep(dbWaitCheckTime);

        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            dbPositionValBuff[iAxis] = g_dbTrg_Pos[iAxis] * (180/M_PI);
            //dbPositionValBuff[iAxis] = g_dbAct_Pos[iAxis] * (180/M_PI);
        }

        // Compare with Target Position Value
        for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            dbTrgPosDiff = (fabs)(dbPositionVal[iAxis] - dbPositionValBuff[iAxis]);

            if(dbTrgPosDiff > TARGET_DIFF_POSITION_THRESHOLD)
            {
                g_fTrgPosDiff = ON;
            }
        }

        if(g_fAxisDebugMsg == TRUE)
        {
            VERBOSE_MESSAGE("Diffence Target Position: %7.3lf (%d)\n",
                            dbTrgPosDiff, g_fTrgPosDiff);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_DoutsSetZero()
//

void SERV_DoutsSetZero(int nOpt)
{
    int iSlave, iPort;

    //0: Arc ON           1: Gas On
    //2: Wire Inching +   3: Wrie Inching -
    //4: Touch Start      5: Touch Ready(MC On)
    g_fDoutsSetZero = ON;   // for Verbose print

    if(g_pShmem_sc->outputstate.fInchingPosOutState == ON)
    {
        SERV_InchingPos_out(OFF);
    }

    if(g_pShmem_sc->outputstate.fInchingNegOutState == ON)
    {
        SERV_InchingNeg_out(OFF);
    }

    if(g_pShmem_sc->outputstate.fTouchStartOutState == ON)
    {
        SERV_TouchStart_out(OFF);
    }
        
    if(g_pShmem_sc->outputstate.fWireCutOutState == ON)
    {
        SERV_WireCut_out(OFF);
    }

    if(nOpt == URGENT_MODE)
    {
        if(g_pShmem_sc->outputstate.fArcOnOutState == ON)
        {
            SERV_ArcOn_out(OFF);
        }

        if(g_pShmem_sc->outputstate.fGasOnOutState == ON)
        {
            SERV_GasOn_out(OFF);
        }

        if(g_pShmem_sc->outputstate.fTouchReadyOutState == ON)
        {
            SERV_TouchReady_out(OFF);
        }

        for(iPort = 0; iPort < ROBOT_AO_PORT_COUNT; iPort++)    // analog cmd set zero
        {
            g_pShmem_sc->outputcmd.dbAoutPortCmd[iPort] = 0;
        }
    }
    else if(nOpt == SMOOTH_MODE)
    {
        if(g_pShmem_sc->outputstate.fArcOnOutState == ON ||
           g_pShmem_sc->outputstate.fGasOnOutState == ON)
        {
            SERV_ArcOn_out(OFF);                                    // arc off

            THREAD_Sleep(g_nEstopGasOffDelayTime);                  // delay for smooth arc off

            SERV_GasOn_out(OFF);                                    // gas off

            for(iPort = 0; iPort < ROBOT_AO_PORT_COUNT; iPort++)    // analog cmd set zero
            {
                g_pShmem_sc->outputcmd.dbAoutPortCmd[iPort] = 0;
            }

            THREAD_Sleep(g_nEstopTouchReadyOffDelayTime);           // delay for smooth arc off

            SERV_TouchReady_out(OFF);                               // Touch Ready off

            VERBOSE_VERBOSE("Smooth Arc Off Processing Done!\n");
        }
    }

#if 1
    //////////////////////////
    // Digital Output
    for(iSlave = 0; iSlave < ROBOT_DO_SLAVE_COUNT; iSlave++)
    {
        for(iPort = 0; iPort < SLAVE_DO_PORT_COUNT; iPort++)
        {
            if(iSlave != WELD_DO_SLAVE_NO && iPort != GASON_DO_PORT_NO)
            {
                if(g_DoutPortVal[iSlave][iPort] == ON)
                {
                    g_DoutPortVal[iSlave][iPort] = OFF;
                }
            }
        }
    }
     
    for(iPort = 0; iPort < ROBOT_DO_PORT_COUNT; iPort++)
    {
        if(iPort != g_Dout_portno.nGasOn)
        {
            if(g_pShmem_sc->outputcmd.nDoutPortCmd[iPort] == ON)
            {
                g_pShmem_sc->outputcmd.nDoutPortCmd[iPort] = OFF;
            }
        }
    }
#endif
    g_fDoutsSetZero = OFF;

#if 0
    VERBOSE_MESSAGE("All Outputs Set Zero!\n");
#endif
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_EStop()
//
// -nValue: value of receive packet (0: off, 1: on)
//
#define ESTOPMODE_WAIT_COUNT     10

int SERV_EStop(int nValue)
{
    int nCnt;

    g_nProfSecState = PROF_SEC_NONE;

    if(nValue == OFF)
    {
        g_pShmem_sc->sysstate.fEStopState = FALSE;
        VERBOSE_MESSAGE("E-stop state OFF.\n");
    }
    if(nValue == ON)
    {
        g_pShmem_sc->sysstate.fEStopState = TRUE;
        VERBOSE_MESSAGE("E-stop state ON.\n");

        if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
        {
            // if hard e-stop flag is TRUE, apply quick stop mode
            if(g_Arg.bHardEstop == TRUE)
            {
                ECATSERV_QuickStop();
            }
            // if if hard e-stop flag is FALSE, monitoring motion profile section
            else
            {
                // Check motion profile state
                _loc_SERV_CheckMotionProfile();

                // case1: acc or uniform mode profile
                if(g_nProfSecState == PROF_ON_MOTION)
                {
                    // Wait defined sampling count
                    for(nCnt = 0; nCnt < ESTOPMODE_WAIT_COUNT; nCnt++)
                    {
                        THREAD_Sleep(2);
                    }

                    // Check motion profile state
                    _loc_SERV_CheckMotionProfile();

                    if(g_nProfSecState == PROF_ON_DEC_MOTION || g_nProfSecState == PROF_ON_MOTION)
                    {
                        VERBOSE_VERBOSE("Case1: On Acc/Uni mode E-stop Normal mode(%d)\n",
                                        g_nProfSecState);
                        
                        _loc_SERV_Wait_CheckStopPositionValue(ESTOP_MODE);
                        
                        if(g_fTrgPosDiff > 0)
                        {
                            ECATSERV_QuickStop();

                            VERBOSE_ERROR("Case1: Normal E-stop Fail! Enter to Quick Stop!\n");
                        }
                        else
                        {
                            if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
                            {
                                SERV_ServoOnCmd(OFF);
                            }
                        }
                    }
                    else
                    {
                        VERBOSE_VERBOSE("Case1: On Acc/Uni mode E-stop Not motion mode(%d)\n",
                                        g_nProfSecState);
#if 0
                        VERBOSE_MESSAGE("2Axis Targ Pos: %7.3lf, Act Pos: %7.3lf\n",
                            g_dbTrg_Pos[1], g_dbAct_Pos[1]);
                        VERBOSE_MESSAGE("3Axis Targ Pos: %7.3lf, Act Pos: %7.3lf\n",
                            g_dbTrg_Pos[2], g_dbAct_Pos[2]);

                        THREAD_Sleep(2);

                        VERBOSE_MESSAGE("2Axis Targ Pos: %7.3lf, Act Pos: %7.3lf\n",
                            g_dbTrg_Pos[1], g_dbAct_Pos[1]);
                        VERBOSE_MESSAGE("3Axis Targ Pos: %7.3lf, Act Pos: %7.3lf\n",
                            g_dbTrg_Pos[2], g_dbAct_Pos[2]);
#endif

                        THREAD_Sleep(100);   // Wait for Actual React (prevent jerk stop)

                        if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
                        {
                            SERV_ServoOnCmd(OFF);
                        }
                    }
                }
                // case2: dec mode profile
                else if(g_nProfSecState == PROF_ON_DEC_MOTION)
                {
                    // Wait E-stop dec time
                    for(nCnt = 0; nCnt < g_dbRobotDecel_Estop; nCnt++)
                    {
                        THREAD_Sleep(2);
                    }

                    // Check motion profile state
                    _loc_SERV_CheckMotionProfile();

                    if(g_nProfSecState == PROF_NOT_ON_MOTION)
                    {
                        VERBOSE_VERBOSE("Case2: On Dec mode E-stop Normal mode(%d)\n",
                                        g_nProfSecState);
                        
                        _loc_SERV_Wait_CheckStopPositionValue(ESTOP_MODE);
                        
                        if(g_fTrgPosDiff > 0)
                        {
                            ECATSERV_QuickStop();
                            
                            VERBOSE_ERROR("Case2: Normal E-stop Fail! Enter to Quick Stop!\n");
                        }
                        else
                        {
                            if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
                            {
                                SERV_ServoOnCmd(OFF);
                            }
                        }
                    }
                    else
                    {
                        VERBOSE_VERBOSE("Case2: On Dec mode E-stop Quick mode(%d)\n",
                                        g_nProfSecState);

                        ECATSERV_QuickStop();
                    }
                }
                // case3: etc mode profile
                else if(g_nProfSecState == PROF_NOT_ON_MOTION)
                {
                    VERBOSE_VERBOSE("Case3: On None motion E-stop mode(%d)\n",
                                    g_nProfSecState);

                    if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
                    {
                        SERV_ServoOnCmd(OFF);
                    }
                }
            }
        }
        
#if 1
        if(g_pShmem_sc->outputstate.fServoOnOutState == TRUE)
        {
            SC_reply.data.servoout.fServoOnOutState = OFF;
            g_pShmem_sc->outputstate.fServoOnOutState = OFF;
        }
#endif
    }

    return RESULT_OK;
}


int SERV_ServoOffStop(void)
{
    int nCnt;
    int fNormalStopEvent = OFF;

    g_nProfSecState = PROF_SEC_NONE;
    
    /* Decel Stop (applied to E-stop decel value) */
    MSG_SendPulse(g_coidTE, TESERV_STOP, 1);
    if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        g_pShmem_sc->outputstate.fServoOnOutState = OFF;    // for Stop Jog Request
        fNormalStopEvent = ON;
    }

    // if hard e-stop flag is TRUE, apply quick stop mode
    if(g_Arg.bHardEstop == TRUE)
    {
        if(fNormalStopEvent == ON)
        {
            g_pShmem_sc->outputstate.fServoOnOutState = ON;
        }
        ECATSERV_QuickStop();
    }
    // if if hard e-stop flag is FALSE, monitoring motion profile section
    else
    {
        // Check motion profile state
        _loc_SERV_CheckMotionProfile();

        // case1: acc or uniform mode profile
        if(g_nProfSecState == PROF_ON_MOTION)
        {
            // Wait defined sampling count
            for(nCnt = 0; nCnt < ESTOPMODE_WAIT_COUNT; nCnt++)
            {
                THREAD_Sleep(2);
            }

            // Check motion profile state
            _loc_SERV_CheckMotionProfile();

            if(g_nProfSecState == PROF_ON_DEC_MOTION || g_nProfSecState == PROF_ON_MOTION)
            {
                if(g_fAxisDebugMsg == TRUE)
                {
                    VERBOSE_VERBOSE("Case1: On Acc/Uni mode Servo off stop Normal mode(%d)\n",
                                    g_nProfSecState);
                }
                
                _loc_SERV_Wait_CheckStopPositionValue(SRVOFF_MODE);
                
                if(g_fTrgPosDiff > 0)
                {
                    if(fNormalStopEvent == ON)
                    {
                        g_pShmem_sc->outputstate.fServoOnOutState = ON;
                    }
                    ECATSERV_QuickStop();
                    if(g_fAxisDebugMsg == TRUE)
                    {
                        VERBOSE_ERROR("Case1: Normal Servo off stop Fail! Enter to Quick Stop!\n");
                    }
                }
                else
                {
                    if(g_pShmem_sc != NULL)
                    {
                        if(fNormalStopEvent == ON)
                        {
                            g_pShmem_sc->outputstate.fServoOnOutState = ON; // for Stop Jog Request
                        }
                        SERV_ServoOnCmd(OFF);
                    }
                }
            }
            else
            {
                if(g_fAxisDebugMsg == TRUE)
                {
                    VERBOSE_VERBOSE("Case1: On Acc/Uni mode Servo off stop Not motion mode(%d)\n",
                                    g_nProfSecState);
                }
#if 1
                THREAD_Sleep(100);   // Wait for Actual React (prevent jerk stop)

                if(fNormalStopEvent == ON)
                {
                    g_pShmem_sc->outputstate.fServoOnOutState = ON; // for Stop Jog Request
                }
                SERV_ServoOnCmd(OFF);
#endif
            }
        }
        // case2: dec mode profile
        else if(g_nProfSecState == PROF_ON_DEC_MOTION)
        {
            // Wait E-stop dec time
            for(nCnt = 0; nCnt < g_dbRobotDecel_Estop; nCnt++)
            {
                THREAD_Sleep(2);
            }

            // Check motion profile state
            _loc_SERV_CheckMotionProfile();

            if(g_nProfSecState == PROF_NOT_ON_MOTION)
            {
                if(g_fAxisDebugMsg == TRUE)
                {
                    VERBOSE_VERBOSE("Case2: On Dec mode Servo off stop Normal mode(%d)\n",
                                    g_nProfSecState);
                }
                
                _loc_SERV_Wait_CheckStopPositionValue(SRVOFF_MODE);
                
                if(g_fTrgPosDiff > 0)
                {
                    if(fNormalStopEvent == ON)
                    {
                        g_pShmem_sc->outputstate.fServoOnOutState = ON;
                    }
                    ECATSERV_QuickStop();
                    
                    if(g_fAxisDebugMsg == TRUE)
                    {
                        VERBOSE_ERROR("Case2: Normal Servo off stop Fail! Enter to Quick Stop!\n");
                    }
                }
                else
                {
                    if(g_pShmem_sc != NULL)
                    {
                        if(fNormalStopEvent == ON)
                        {
                            g_pShmem_sc->outputstate.fServoOnOutState = ON; // for Stop Jog Request
                        }
                        SERV_ServoOnCmd(OFF);
                    }
                }
            }
            else
            {
                if(g_fAxisDebugMsg == TRUE)
                {
                    VERBOSE_VERBOSE("Case2: On Dec mode Servo off stop Quick mode(%d)\n",
                                    g_nProfSecState);
                }

                if(fNormalStopEvent == ON)
                {
                    g_pShmem_sc->outputstate.fServoOnOutState = ON;
                }
                ECATSERV_QuickStop();
            }
        }
        // case3: etc mode profile
        else if(g_nProfSecState == PROF_NOT_ON_MOTION)
        {
            if(g_fAxisDebugMsg == TRUE)
            {
                VERBOSE_VERBOSE("Case3: On None motion Servo off stop mode(%d)\n",
                                g_nProfSecState);
            }

            if(g_pShmem_sc != NULL)
            {
                if(fNormalStopEvent == ON)
                {
                    g_pShmem_sc->outputstate.fServoOnOutState = ON; // for Stop Jog Request
                }
                SERV_ServoOnCmd(OFF);
            }
        }
    }

    if (g_pShmem_sc->outputstate.fServoOnOutState == ON)
    {
        SC_reply.data.servoout.fServoOnOutState = OFF;
        g_pShmem_sc->outputstate.fServoOnOutState = OFF;
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_JobStopProcess()
//
int SERV_JobStopProcess(void)
{
    _loc_SERV_Wait_CheckStopPositionValue(STOP_MODE);

    if(g_pShmem_sc->outputstate.fArcOnOutState      == ON ||
       g_pShmem_sc->outputstate.fGasOnOutState      == ON ||
       g_pShmem_sc->outputstate.fInchingPosOutState == ON ||
       g_pShmem_sc->outputstate.fInchingNegOutState == ON ||
       g_pShmem_sc->outputstate.fTouchStartOutState == ON ||
       g_pShmem_sc->outputstate.fTouchReadyOutState == ON)
    {
        g_pShmem_SysStatus_rm->fEstopDone = OFF;

        SERV_DoutsSetZero(SMOOTH_MODE);
        
        SERV_DoutsSetZero(URGENT_MODE);
    }
    
    g_pShmem_SysStatus_rm->fEstopDone = ON;

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNC_SyncActualPosToTargetPos()
//
int FUNC_SyncActualPosToTargetPos(void)
{
    int iAxis;

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

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// SERV_GetPosition()
//
int SERV_GetPosition(void)
{
	int nRet = 0;
    int iAxis;

    // get actual pos.
    nRet = ECATSERV_ReadPosition();

    //g_pShmem_sc->inputstate.nCoordIdx = nCoorIdx;

    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        if (nRet != 0)
        {
            VERBOSE_ERROR("Cannot get %d axis position\n", iAxis);
        }

        // write actual pos. to SC_SHM
#if defined (__QNXNTO__)
        FUNC_ConvertPulseToPos(iAxis, ACTUAL_POS_IDX);
        g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbAct_Pos[iAxis];
        //g_dbTrg_Pos[iAxis] = g_pShmem_sc->outputcmd.dbTrgPos[iAxis];
        //FUNC_SyncActualPosToTargetPos();
#else
        FUNC_ConvertPulseToPos(iAxis, TARGET_POS_IDX);
        g_pShmem_sc->inputstate.dbActPos[iAxis] = g_dbTrg_Pos[iAxis];
#endif
    }
    if(g_pShmem_Status_te != NULL)
    {
        SC_reply.data.pos.nCoordIdx = g_pShmem_Status_te->coord_ref;

        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(g_pShmem_Status_te->coord_ref == COORDINDEX_JOINT)
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
                        g_pShmem_Status_te->xyzrpy_act[iAxis];   // mm
                }
                else
                {
                    SC_reply.data.pos.dbCurrPos[iAxis] = 
                        g_pShmem_Status_te->xyzrpy_act[iAxis] * (180/M_PI);  // deg
                }
            }
        }
    }
    
    SC_reply.size = sizeof(SC_reply.data.pos);
    SC_reply.code = SC_msg.code;
    SC_reply.value = SC_msg.value;

    return nRet;

}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_GetServoAlarmCode()
//
int SERV_GetServoAlarmCode(void)
{
    int nRet;

    if(g_pShmem_sc->sysstate.fEcatInitState == FALSE)
    {
    	return RESULT_ERROR;
    }
    
    nRet = ECATSERV_GetServoAlarmCode();

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_BrakeReleaseCmd()
// -nAxis: Axis No (0 ~ 5)
// -nValue: boolean value (0: off, 1: on)
//
int SERV_BrakeReleaseCmd(int nAxis, int nValue)
{
    int nRet = 0;
    int iAxis;

    if(nAxis == ALL_AXES) // All Axis
    {
        if(nValue == OFF)
        {
            nRet = ECATSERV_WritePhysicalOutput(nAxis, OFF, ALL);
            THREAD_Sleep(10);
            
            if(nRet == RESULT_OK)
            {
                for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
                {
                    SC_reply.data.servoout.fBrakeReleaseOutState[iAxis] = OFF;
                    g_pShmem_sc->outputstate.fBrakeReleaseOutState[iAxis] = OFF;
                }
                
                VERBOSE_MESSAGE("ALL Axes Brake Lock.\n");
            }
        }
        else if(nValue == ON)
        {
            nRet = ECATSERV_WritePhysicalOutput(nAxis, ON, ALL);
            THREAD_Sleep(10);
            
            if(nRet == RESULT_OK)
            {
                for(iAxis = 0; iAxis < g_nAxisCount; iAxis++)
                {
                    SC_reply.data.servoout.fBrakeReleaseOutState[iAxis] = ON;
                    g_pShmem_sc->outputstate.fBrakeReleaseOutState[iAxis] = ON;
                }
                
                VERBOSE_MESSAGE("ALL Axes Brake Released.\n");
            }
        }
    }
    else
    {
        if(nValue == OFF)  // Brake Lock
        {
            nRet = ECATSERV_WritePhysicalOutput(nAxis, OFF, EACH);
            THREAD_Sleep(10);
            
            if(nRet == RESULT_OK)
            {
                SC_reply.data.servoout.fBrakeReleaseOutState[nAxis] = OFF;
                g_pShmem_sc->outputstate.fBrakeReleaseOutState[nAxis] = OFF;
            }

            VERBOSE_MESSAGE("Axis%d Brake Lock.\n", nAxis);
        }
        else if(nValue == ON)  // Brake Release
        {
            nRet = ECATSERV_WritePhysicalOutput(nAxis, ON, EACH);
            THREAD_Sleep(10);
            
            if(nRet == RESULT_OK)
            {
                SC_reply.data.servoout.fBrakeReleaseOutState[nAxis] = ON;
                g_pShmem_sc->outputstate.fBrakeReleaseOutState[nAxis] = ON;
            }

            VERBOSE_MESSAGE("Axis%d Brake Release.\n", nAxis);
        }
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_ServoOnCmd()
//
// -nValue: boolean value (0: off, 1: on)
//
int SERV_ServoOnCmd(int nValue)
{
    int nResult = -1;
#if defined (__QNXNTO__)
    int nRet[ROB_AXIS_COUNT];
    int iAxis;
    int nCnt = 0;

	// check master instance state
	if(g_hMaster == NULL)
	{
		VERBOSE_WARNING("Not ready to use Master.\n");
		return RESULT_ERROR;
	}
#endif
    // check ethercat state
    if(g_pShmem_sc->sysstate.fEcatInitState == FALSE)
    {
        VERBOSE_ERROR("Check the ethercat network\n");
    	return RESULT_ERROR;
    }
    
    // check alarm code
    if(g_pShmem_SysStatus_rm != NULL &&
       g_pShmem_SysStatus_rm->fErrorState == ON && nValue == ON)
    {
        VERBOSE_WARNING("Cannot servo on. Check alarm code.\n");
        return RESULT_ERROR;
    }
#if 0
    // check alarm code
    if(g_pShmem_SysStatus_rm != NULL &&
       g_pShmem_SysStatus_rm->fErrorState == ON && nValue == OFF)
    {
        VERBOSE_WARNING("Cannot servo off. Check alarm code.\n");
        return RESULT_ERROR;
    }
#endif

    // check E-stop state
    if(g_pShmem_SysStatus_rm != NULL &&
       g_pShmem_SysStatus_rm->fEStopState == ON && nValue == ON)
    {
        VERBOSE_WARNING("Cannot servo on. Check E-Stop State.\n");
        return RESULT_ERROR;
    }
#if 0
    // check E-stop state
    if(g_pShmem_SysStatus_rm != NULL &&
       g_pShmem_SysStatus_rm->fEStopState == ON && nValue == OFF)
    {
        VERBOSE_WARNING("Cannot servo off. Check E-Stop State.\n");
        return RESULT_ERROR;
    }
#endif
    // check same state request
    if(g_pShmem_sc->outputstate.fServoOnOutState == ON && nValue == ON)
    {
        VERBOSE_WARNING("All Servo States are already SERVO-ON status\n");
        return RESULT_ERROR;
    }

    if(g_pShmem_sc->outputstate.fServoOnOutState == OFF && nValue == OFF)
    {
        VERBOSE_WARNING("All Servo States are already SERVO-OFF status\n");
        return RESULT_ERROR;
    }
#if defined (__QNXNTO__)
    for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
    {
        nRet[iAxis] = -1;
    }

    if (nValue == OFF)  // servo off
    {
        ECATSERV_ServoOff(iAxis, ALL);

        if(nCnt == 0)
        {
            THREAD_Sleep(5);   //org: 200
            SERV_GetServoState();
        }

        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            nRet[iAxis] = g_nServoOnCmdState[iAxis];

            if(g_nServoOnCmdState[iAxis] != RESULT_OK)
            {
                nRet[iAxis] = -1;
            }
        }

SERVO_OFF_EACH:
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(nRet[iAxis] == -1)
            {
                nRet[iAxis] = ECATSERV_ServoOff(iAxis, EACH);
            }

            THREAD_Sleep(1);    //org: 20
            SERV_GetServoState();

            if(g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_NR &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_QUICKSTOP &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_STOP &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_CTRWORD_PROCEEDING)
            {
                nRet[iAxis] = RESULT_ERROR;
            }

            if(g_pShmem_sc->sysstate.fEStopState == TRUE)
            {
                return RESULT_ERROR;
            }

            if(nRet[iAxis] != RESULT_OK)
            {
                VERBOSE_WARNING("Axis %d Servo Off Fail[State: %x]. Retry..\n",
                                iAxis, g_nReadStatusValue[iAxis]);
            }
        }

        nResult = nRet[0] + nRet[1] + nRet[2] + nRet[3] + nRet[4] + nRet[5];
        
        if(nResult == RESULT_OK)
        {
            VERBOSE_MESSAGE("Servo off.\n");

            SC_reply.data.servoout.fServoOnOutState = OFF;
            g_pShmem_sc->outputstate.fServoOnOutState = OFF;
        }
        else if(nResult != RESULT_OK && nCnt < 10)
        {
            nCnt++;
            goto SERVO_OFF_EACH;
        }
    }
    else if (nValue == ON)  // servo on
    {
        ECATSERV_ServoOn(ALL_AXES, ALL);
        
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            nRet[iAxis] = g_nServoOnCmdState[iAxis];

            if(g_nServoOnCmdState[iAxis] != RESULT_OK)
            {
                nRet[iAxis] = -1;
            }
        }

        if(nCnt == 0)
        {
            THREAD_Sleep(1);    //org: 10
            SERV_GetServoState();
        }

SERVO_ON_EACH:
        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(nRet[iAxis] == -1)
            {
                nRet[iAxis] = ECATSERV_ServoOn(iAxis, EACH);
            }

            THREAD_Sleep(1);    //org: 10
            SERV_GetServoState();

            if(g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_R &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_ON_NR &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_CTRWORD_PROCEEDING)
            {
                nRet[iAxis] = RESULT_ERROR;
            }
            
            if(g_pShmem_sc->sysstate.fEStopState == TRUE)
            {
                return RESULT_ERROR;
            }

            if(nRet[iAxis] != RESULT_OK)
            {
                VERBOSE_WARNING("Axis %d Servo On Fail[State: %x]. Retry..\n",
                                iAxis, g_nReadStatusValue[iAxis]);
            }
        }
        
        nResult = nRet[0] + nRet[1] + nRet[2] + nRet[3] + nRet[4] + nRet[5];

        if(nResult == RESULT_OK)
        {
            VERBOSE_MESSAGE("Servo on.\n");

            SC_reply.data.servoout.fServoOnOutState = ON;
            g_pShmem_sc->outputstate.fServoOnOutState = ON;
            THREAD_Sleep(2);
        }
        else if(nResult != RESULT_OK && nCnt < 10)
        {
            nCnt++;
            goto SERVO_ON_EACH;
        }
    }
    
#else
    nResult = RESULT_OK;

    if (nValue == OFF)  // servo off
    {
        g_pShmem_sc->outputstate.fServoOnOutState = OFF;
    }
    else if (nValue == ON)  // servo on
    {
        g_pShmem_sc->outputstate.fServoOnOutState = ON;
    }
#endif

    return nResult;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_SetPosition()
//
// -nValue: value of position (pulse)
//
int SERV_SetPosition(int nValue)
{
    int nRet = 0;
    g_fSetPosFuncActive = TRUE;
    THREAD_Sleep(100);

#if defined (__QNXNTO__)
    nRet = ECATSERV_WriteTargetPosition(ALL_AXES, ALL);
#endif
    
    THREAD_Sleep(100);
    g_fSetPosFuncActive = FALSE;

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_ClearServoAlarm()
//
int SERV_ClearServoAlarm(void)
{
    int nRet;
    int iAxis, i;
    int fErrorState = 0;
    static int s_nSlave;

    nRet = ECATSERV_AlarmClear();
    
    SERV_GetServoState();

#if defined (__QNXNTO__)
    for(s_nSlave = 0; s_nSlave < g_nSlaveCount; s_nSlave++)
    {
        EcatIODevClearSlaveStateError(g_hMaster, s_nSlave);
    }
#endif
    if (g_pShmem_sc != NULL)
    {
        if(g_pShmem_sc->outputstate.fServoOnOutState == ON)
        {
            SERV_ServoOnCmd(OFF);
        }

        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            if(g_nErrCodeServo[iAxis] != 0 ||
              (g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_READY_SWON &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SWON &&
               g_nReadStatusValue[iAxis] != SRVSTATE_CODE_SERVO_OFF_SHUT))
            {
                fErrorState = fErrorState + 1;
            }
        }
#if 0
        VERBOSE_MESSAGE("ErrState: %d, ErrCode: %d, %d, %d, %d, %d, %d\n",
                        fErrorState,
                        g_nErrCodeServo[0], g_nErrCodeServo[1],
                        g_nErrCodeServo[2], g_nErrCodeServo[3],
                        g_nErrCodeServo[4], g_nErrCodeServo[5]);
#endif
        if(fErrorState != 0)
        {
            fErrorState = TRUE;
        }
        else
        {
            fErrorState = FALSE;
        }

        if(fErrorState == TRUE)
        {
            VERBOSE_MESSAGE("Not Cleared servo alarm. Try Again.\n");
        }
        else
        {
            if(g_pShmem_sc->sysstate.fErrorState == TRUE)
            {
                g_pShmem_sc->sysstate.fErrorState = FALSE;
                g_pShmem_sc->sysstate.nErrorCode = SYS_ERR_OK;
            }
            
            for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                g_nEmergencyCodeServo[iAxis] = 0;
                g_wEmergencyErrStateCode[iAxis] = 0;
            }
            for(i = 0; i < ERROR_MESSAGE_BUFFER_SIZE; i++)
            {
                g_szEcatErrorDescription[i] = ' ';
            }
#if 0
            CRT_strncpy(g_szEcatErrorDescription,
                        ERROR_MESSAGE_BUFFER_SIZE,
                        DANDY_DEFINE_NAME_STR(SERVO_ERR_OK) + 6,
                        ERROR_MESSAGE_BUFFER_SIZE);
#endif
            g_nErrAxis = 0;
            g_nErrCodeEcat = 0;
            g_pShmem_sc->sysstate.nErrorCode = 0;
#if 0
            if(g_pShmem_sc->sysstate.fEStopState == ON)
            {
                g_pShmem_sc->sysstate.fEStopState = OFF;
            }
#endif
            VERBOSE_MESSAGE("Cleared servo alarm.\n");
        }
    }

#if defined (__QNXNTO__)
    if(g_pShmem_sc->sysstate.fEcatInitState == TRUE)
    {
        EcatIODevResetStatistics(g_hMaster);
    }
#endif

    return RESULT_OK;
}
