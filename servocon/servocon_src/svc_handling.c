/////////////////////////////////////////////////////////////////////////////
//
//  svc_handling.c: Service Request Receive & Process
//                                            2013.11.11  Ryu SinWook
///////////////////////////////////////
#include "service.h"

////////////////////////////////////////////////////////////////////////////////
//
// Variables
//
int g_fHWLimitMonAct = OFF;
int g_fAxisDebugMsg = FALSE;
int g_fArcOnSigHigh = OFF;
int g_fArcOnVirtualProc = OFF;

SC_MSG         SC_msg;           // msg packet
SC_REPLY       SC_reply;         // reply packet

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
// SERV_DoService()
//
// -pMsg : pointer to receive packet
// -pReply: pointer to send packet
//
int SERV_DoService(const SC_MSG* pMsg, SC_REPLY* pReply)
{
    int nRet = -1;
    int i;

    //memcpy(pReply, pMsg, 12 + pMsg->size);
    memcpy(pReply, pMsg, sizeof(SC_MSG));

    switch(pMsg->code)
    {
    case SC_SERV_EXIT:
        nRet = MAIN_Exit();
        g_fLoopGo = FALSE;
        break;

    case SC_SERV_VERSION:
        nRet = SERV_SetVersion(pReply);
        break;

    case SC_SERV_INIT:
        nRet = MAIN_Init_External();
        break;

    case SC_SERV_SERVO:
        nRet = SERV_ServoOnCmd(pMsg->value);
        break;

    case SC_SERV_BRAKE_RELEASE:
        nRet = SERV_BrakeReleaseCmd(pMsg->value, ON);   // value is Axis No
        break;

    case SC_SERV_BRAKE_LOCK:
        nRet = SERV_BrakeReleaseCmd(pMsg->value, OFF);  // value is Axis No
        break;

    case SC_SERV_GET_SERVO_ALARM:
        nRet = SERV_GetServoAlarmCode();        
    	break;

    case SC_SERV_ALARM_RESET:
        nRet = SERV_ClearServoAlarm();
    	break;

    case SC_SERV_ECAT_RESTART:
        nRet = SERV_RestartMaster();
#if defined (__QNXNTO__)
        EcatIODevResetStatistics(g_hMaster);
#endif
    	break;

    case SC_SERV_ESTOP:
        if(pMsg->value == ON)
        {
            nRet = SERV_EStop(pMsg->value);

            if(g_pShmem_SysStatus_rm != NULL)
            {
                g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_SERV;
                VERBOSE_MESSAGE("Estop Activated By Service\n");
            }
        }
        else if(pMsg->value == OFF)
        {
            nRet = SERV_EStop(pMsg->value);

            if(g_pShmem_SysStatus_rm != NULL)
            {
                g_pShmem_SysStatus_rm->nEstopCode = SYS_ESTOP_NONE;
                VERBOSE_MESSAGE("Estop Released By Service\n");
            }
        }
    	break;

    case SC_SERV_SCAN_WELD_IO:
        if(g_pShmem_sc != NULL)
        {
            nRet = SERV_Scan_Welder_IO(pMsg->value);
        }
    	break;

    case SC_SERV_SCAN_SERVO_IO:
        if(g_pShmem_sc != NULL)
        {
            nRet = SERV_Scan_Servo_IO(pMsg->value);
        }
    	break;
        
    case SC_SERV_SCAN_SYS_STATE:
        if(g_pShmem_sc != NULL)
        {
            nRet = SERV_Scan_System_State(pMsg->value);
        }
    	break;

    case SC_SERV_ARCON_OUT:
        nRet = SERV_ArcOn_out(pMsg->value);
    	break;

    case SC_SERV_GASON_OUT:
        nRet = SERV_GasOn_out(pMsg->value);
    	break;

    case SC_SERV_INCHING_POS:
        nRet = SERV_InchingPos_out(pMsg->value);
    	break;

    case SC_SERV_INCHING_NEG:
        nRet = SERV_InchingNeg_out(pMsg->value);
    	break;

    case SC_SERV_TOUCH_START:
        nRet = SERV_TouchStart_out(pMsg->value);
    	break;

    case SC_SERV_TOUCH_READY:
        nRet = SERV_TouchReady_out(pMsg->value);
    	break;

    case SC_SERV_WIRE_CUT:
        nRet = SERV_WireCut_out(pMsg->value);
        break;

    case SC_SERV_VOLT_OUT:
        nRet = SERV_WeldVolt_out(pMsg->value);
    	break;

    case SC_SERV_CURR_OUT:
        nRet = SERV_WeldCurr_out(pMsg->value);
    	break;

    case SC_SERV_SET_POS_ZERO:
        if(pMsg->value == ALL_AXES)
        {
            for(i = 0; i < g_nAxisCount; i++)
            {
                nRet = SERV_SetZeroPosition(i);
            }
        }
        else
        {
            nRet = SERV_SetZeroPosition(pMsg->value);
            THREAD_Sleep(10);
        }
        break;

    case SC_SERV_SET_HOME_ZERO:
        if(pMsg->value == ALL_AXES)
        {
            for(i = 0; i < g_nAxisCount; i++)
            {
                nRet = SERV_SetHomeOffsetZeroValue(i);
            }
        }
        else
        {
            nRet = SERV_SetHomeOffsetZeroValue(pMsg->value);
            THREAD_Sleep(10);
        }
        break;

    case SC_SERV_ENC_RESET_ONLY:
        if(pMsg->value == ALL_AXES)
        {
            for(i = 0; i < g_nAxisCount; i++)
            {
                nRet = SERV_ABSEncoderResetOnly(i);
            }
        }
        else
        {
            nRet = SERV_ABSEncoderResetOnly(pMsg->value);
            THREAD_Sleep(10);
        }
        break;
        
    case SC_SERV_ABS_ENC_RESET:
        if(pMsg->value == ALL_AXES)
        {
            for(i = 0; i < g_nAxisCount; i++)
            {
                nRet = SERV_ABSEncoderReset(i);
            }
        }
        else
        {
            nRet = SERV_ABSEncoderReset(pMsg->value);
            THREAD_Sleep(10);
        }
        break;

    case SC_SERV_WRITE_DO_PORT:
        nRet = SERV_WriteDoutPort(pMsg->value);
        break;

    case SC_SERV_WRITE_AO_PORT:
        nRet = SERV_WriteAoutPort(pMsg->value, pMsg->data.serv_opt.nOpt);
        break;

    case SC_SET_INTERP_PERIOD:
        nRet = SERV_SetInterpolationTimePeriod(pMsg->value, pMsg->data.serv_opt.nOpt);
        break;

    case SC_SET_INTERP_INDEX:
        nRet = SERV_SetInterpolationTimeIndex(pMsg->value, pMsg->data.serv_opt.nOpt);
        break;

    case SC_SET_QNX_TIMER_REG:
        g_hTimer = TIME_RegTimerPulse(g_coidSCTime, RUNSERV_TIMER, 0, pMsg->value, 0);
        break;

    case SC_CHANGE_AOUT_UNIT:
        nRet = SERV_ChangeAnalogIOUnit(pMsg->value);
        break;

    case SC_RELEASE_SHOCKSENSOR:
        nRet = SERV_ShockSensorRelease(pMsg->value);
        break;

    case SC_DOUT_SET_ZERO:
        SERV_DoutsSetZero(pMsg->value);
        break;

    case SC_SERV_ERRORSTOP:
        nRet = SERV_ServoOffStop();
        SERV_DoutsSetZero(STOP_MODE);
        break;

    case SC_SERV_JOBSTOP_EVENT:
        nRet = SERV_JobStopProcess();
        break;

    case SC_HWLIMIT_MON_ACT:
        if(pMsg->value == ON)
        {
            g_fHWLimitMonAct = ON;
            VERBOSE_MESSAGE("H/W Limit Monitoring Activated!\n");
        }
        else if(pMsg->value == OFF)
        {
            g_fHWLimitMonAct = OFF;
            g_fHWLimitOnState = OFF;
            VERBOSE_MESSAGE("H/W Limit Monitoring Deactivated!\n");
            SERV_EStop(OFF);        // E-stop State Released!
        }
        else
        {
            VERBOSE_ERROR("Invalid H/W Limit Act Value! 0 or 1 is allowable.\n");
        }
    	break;

    case SC_ARCSENS_FILE_SAVE:
        nRet = SERV_ArcSensorDataFileSave(pMsg->value);
        break;

    case SC_SHM_PARAM_RELOAD:
        // system parameter load by sysconf RM SHM
        nRet = SHM_LoadSysConfigParam(ROBOT_0_INDEX);
        if(nRet == RESULT_OK)
        {
            VERBOSE_MESSAGE("Parameter Reload Done!\n");
        }
        break;

    case SC_ARCSEN_RDATA_FILELOAD:
        nRet = SERV_ArcSensorRDataFileLoad();
        break;

    case SC_SET_SYNCERROR_CNT:
        nRet = SERV_SetSyncErrorCountLimit(pMsg->value);
        break;

    case SC_SET_OPERATEMODE:
        //nRet = SERV_GetNetworkState();
        nRet = SERV_SetControlMode(pMsg->value, pMsg->data.serv_opt.nOpt);
        break;

    case SC_READ_OPERATEMODE:
        nRet = SERV_ReadControlMode();
        break;

    case SC_SERV_SERVOON_1AX:
        nRet = ECATSERV_ServoOn(pMsg->value, EACH);
        VERBOSE_MESSAGE("Servo On %d Axis\n", pMsg->value);
        break;

    case SC_SERV_SERVOOFF_1AX:
        nRet = ECATSERV_ServoOff(pMsg->value, EACH);
        VERBOSE_MESSAGE("Servo Off %d Axis\n", pMsg->value);
        break;
    
    case SC_CONTROL_WORD_TEST:
        nRet = ECATSERV_WriteControlWord(0, pMsg->value, EACH);
        break;

    case SC_VGA_DISPLAY_ON_OFF:
        if(pMsg->value == ON)
        {
            g_Arg.bNoVGA = FALSE;
            VERBOSE_MESSAGE("Start VGA Display!\n");
        }
        if(pMsg->value == OFF)
        {
            g_Arg.bNoVGA = TRUE;
            VERBOSE_MESSAGE("Stop VGA Display!\n");
        }
        break;

    case SC_AXIS_DEBUG:
        if(pMsg->value == ON)
        {
            g_fAxisDebugMsg = TRUE;
            VERBOSE_MESSAGE("Start Axis Debug Message Display!\n");
        }
        if(pMsg->value == OFF)
        {
            g_fAxisDebugMsg = FALSE;
            VERBOSE_MESSAGE("Stop Axis Debug Message Display!\n");
        }
        break;

    case SC_IO_TEST_MODE:
        if(pMsg->value == ON)
        {
            g_fIoTestModeActive = ON;
            VERBOSE_MESSAGE("Start I/O Test mode!\n");
        }
        if(pMsg->value == OFF)
        {
            g_fIoTestModeActive = OFF;
            VERBOSE_MESSAGE("Stop I/O Test mode!\n");
        }
        break;

    case SC_TEST_IO_ON:
        for(i = 0; i < 8; i++)
        {
            nRet = SERV_EcatDigitalOut(pMsg->value, i, TRUE);
        }
        VERBOSE_MESSAGE("%d-Slave All Ports Douts ON!\n", pMsg->value);
    	break;

    case SC_TEST_IO_OFF:
        for(i = 0; i < 8; i++)
        {
            nRet = SERV_EcatDigitalOut(pMsg->value, i, FALSE);
        }
        VERBOSE_MESSAGE("%d-Slave All Ports Douts Off!\n", pMsg->value);
    	break;

    case SC_SERV_SET_POSITION:
        nRet = SERV_SetPosition(pMsg->value);
    	break;

    case SC_SERV_GET_POSITION:
        //nRet = SERV_GetPosition();
        // for test
        if(pMsg->value == ON)
        {
            g_pShmem_sc->sysstate.nErrorCode = ECAT_ERR_INIT_ECAT_FAIL;
            g_pShmem_sc->sysstate.nErrorAxis = 1;
            g_pShmem_sc->sysstate.fErrorState = ON;
        }
        else if(pMsg->value == OFF)
        {
            g_pShmem_sc->sysstate.nErrorCode = SYS_ERR_OK;
            g_pShmem_sc->sysstate.nErrorAxis = -1;
            g_pShmem_sc->sysstate.fErrorState = OFF;
        }
        break;
    
    case SC_POS_FILE_WRITE:
        if(pMsg->value == ON)
        {
            VERBOSE_MESSAGE("Start Position Data File Write!\n");
            g_fPosFileWrite = ON;
        }
        else if(pMsg->value == OFF)
        {
            VERBOSE_MESSAGE("Stop Position Data File Write!\n");
            g_fPosFileWrite = OFF;
        }
    	break;

    case SC_SERV_SHOW_LOADED_SHM:
        SHM_ShowSystemParam();
        break;

    case SC_SERV_ADC_GET_REQ:
        if(pMsg->value == ON)
        {
            VERBOSE_MESSAGE("ADC_gathering_request ON!\n");
            if(g_pShmem_Task_te != NULL)
            {
                g_pShmem_Task_te->ADC_gathering_request = ON;
            }
        }
        else if(pMsg->value == OFF)
        {
            VERBOSE_MESSAGE("ADC_gathering_request OFF!\n");
            if(g_pShmem_Task_te != NULL)
            {
                g_pShmem_Task_te->ADC_gathering_request = OFF;
            }
        }
        break;

    case SC_ACRON_INPORT_SIGHIGH:
        if(pMsg->value == ON)
        {
            VERBOSE_MESSAGE("Arc On Signal Force to High ON!\n");
            g_fArcOnSigHigh = ON;
        }
        else if(pMsg->value == OFF)
        {
            VERBOSE_MESSAGE("Arc On Signal Force to High OFF!\n");
            g_fArcOnSigHigh = OFF;
        }
        break;

    case SC_ACRON_VIRTUAL_PROC:
        if(pMsg->value == ON)
        {
            VERBOSE_MESSAGE("Arc On Virtual Process ON!\n");
            g_fArcOnVirtualProc = ON;
        }
        else if(pMsg->value == OFF)
        {
            VERBOSE_MESSAGE("Arc On Virtual Process OFF!\n");
            g_fArcOnVirtualProc = OFF;
        }
        break;
        
    case SC_SERV_ALIVE:             // 127(ignore service)
        break;
        
    default:
        VERBOSE_WARNING("Not defined service code : <%d>\n", pMsg->code);
    }

    return nRet;
}
