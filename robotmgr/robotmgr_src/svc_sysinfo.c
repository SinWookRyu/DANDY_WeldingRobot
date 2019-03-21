/////////////////////////////////////////////////////////////////////////////
//
//  svc_sysinfo.c: System Information Service
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

#define RECENT_ERRHIST_IDX              -1
#define DEF_ERRHIST_REQ_CNT             32

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int  g_nErrHistoryStartIdx;
int  g_nErrHistoryReqCnt;
int  g_fErrHistPacketDef = OFF;

char g_szErrContent[ERROR_NAME_LEN];
char g_szErrModeContent[ERROR_MODE_NAME_LEN];
char g_szEstopContent[ESTOP_NAME_LEN];

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_SetSystemStatePacketDefine()
//      - Service Name: RMGR_SERV_SYSSTATE

#define Reply_SysState     RM_reply_packet.Data.reply_state

int SVC_SetSystemStatePacketDefine(void)
{
    RM_reply_packet.nCode      = RM_packet.nCode;
    RM_reply_packet.nValue     = RM_packet.nValue;
    RM_reply_packet.nDataSize  = sizeof(RMGR_STATE_REPLY_DATA);

    Reply_SysState.nExecStat   = g_pShm_SysStatus->nExecStat;
    Reply_SysState.nWorkType   = g_pShm_SysStatus->nWorkType;
    Reply_SysState.nSystemMode = g_pShm_SysStatus->nSystemMode;
    Reply_SysState.nErrCode    = g_pShm_SysStatus->nErrCode;
    Reply_SysState.nEstopCode  = g_pShm_SysStatus->nEstopCode;

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_Fn_CheckErrHistoryReqArgValidation()
//
static int _loc_Fn_CheckErrHistoryReqArgValidation(void)
{
    static int nCheckReqValid;

    nCheckReqValid = g_nErrHistoryStartIdx + g_nErrHistoryReqCnt;

    // Check Start Index valid
    if(g_nErrHistoryStartIdx != RECENT_ERRHIST_IDX && g_nErrHistoryStartIdx < 0)
    {
        VERBOSE_WARNING("Error History Data Request Start Idx too Small."
                        "(%d ->",
                        g_nErrHistoryStartIdx);
        
        if(g_ErrCodeStack.nTop == 0)
        {
            g_nErrHistoryStartIdx = 0;
        }
        else if(g_ErrCodeStack.nTop < DEF_ERRHIST_REQ_CNT)
        {
            g_nErrHistoryStartIdx = 0;
            g_nErrHistoryReqCnt   = g_ErrCodeStack.nTop;
        }
        else
        {
            g_nErrHistoryStartIdx = g_ErrCodeStack.nTop - DEF_ERRHIST_REQ_CNT;
        }

        VERBOSE_WARNING("\v %d)\n",
                        g_nErrHistoryStartIdx);
    }
    else if(g_nErrHistoryStartIdx >= g_nErrHistorySaveMaxCnt)
    {
        VERBOSE_WARNING("Error History Data Request Start Idx too Big."
                        "(%d -> %d)\n",
                        g_nErrHistoryStartIdx,
                        0);
        g_nErrHistoryStartIdx = 0;
    }


    // Check Request Count valid
    if(g_nErrHistoryReqCnt <= 0)
    {
        VERBOSE_WARNING("Error History Data Request Count too Small. "
                        "(%d -> %d)\n",
                        g_nErrHistoryReqCnt,
                        DEF_ERRHIST_REQ_CNT);
        g_nErrHistoryReqCnt = DEF_ERRHIST_REQ_CNT;
    }
    else if(g_nErrHistoryReqCnt >= MAX_ERROR_HISTORY_SND_SIZE)
    {
        VERBOSE_WARNING("Error History Data Request Count too Big. "
                        "(%d -> %d)\n",
                        g_nErrHistoryReqCnt,
                        MAX_ERROR_HISTORY_SND_SIZE);
        g_nErrHistoryReqCnt = MAX_ERROR_HISTORY_SND_SIZE;
    }
    
    // Check Range valid
    if(nCheckReqValid > MAX_ERROR_STACK_SIZE)
    {
        VERBOSE_WARNING("Error History Data Request Range Invalid. "
                        "(%d ~ %d) -> (%d ~ %d)\n",
                        g_nErrHistoryStartIdx,
                       (g_nErrHistoryStartIdx + g_nErrHistoryReqCnt),
                        g_ErrCodeStack.nTop,
                       (g_ErrCodeStack.nTop + DEF_ERRHIST_REQ_CNT));
        
        g_nErrHistoryStartIdx = g_ErrCodeStack.nTop;
        g_nErrHistoryReqCnt   = DEF_ERRHIST_REQ_CNT;
    }
    
    // Check Recent Error Index
    if(g_nErrHistoryStartIdx == RECENT_ERRHIST_IDX)
    {
        if(g_ErrCodeStack.nTop == 0)
        {
            g_nErrHistoryStartIdx = 0;
        }
        else if(g_ErrCodeStack.nTop < g_nErrHistoryReqCnt)
        {
            g_nErrHistoryStartIdx = 0;
            g_nErrHistoryReqCnt   = g_ErrCodeStack.nTop;
        }
        else
        {
            g_nErrHistoryStartIdx = g_ErrCodeStack.nTop - g_nErrHistoryReqCnt;
        }
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SetSystemStatePacketDefine()
//      - Service Name: RMGR_SERV_ERRHISTORY

int SVC_SetErrorHistoryPacketDefine(void)
{
    static int iCount;
    int nMsgIdx = 0;

    g_nErrHistoryStartIdx = RM_packet.Data.errhistory_req.nErrHistoryStartIdx;
    g_nErrHistoryReqCnt = RM_packet.Data.errhistory_req.nErrHistoryReqCnt;

    _loc_Fn_CheckErrHistoryReqArgValidation();

    RM_reply_packet.Data.errhistory_reply.nErrCnt = g_ErrCodeStack.nErrCnt;

    //for(iCount = 0; iCount < MAX_ERROR_HISTORY_SND_SIZE; iCount++)
    for(iCount = g_nErrHistoryStartIdx;
        iCount < g_nErrHistoryStartIdx + MAX_ERROR_HISTORY_SND_SIZE;
        iCount++)
    {
        if(iCount >= g_nErrHistoryStartIdx &&
           iCount < (g_nErrHistoryStartIdx + g_nErrHistoryReqCnt))
        {
            CRT_strcpy(RM_reply_packet.Data.errhistory_reply.szErrStackSysTime[nMsgIdx],
                       SYSTIME_DATA_LEN,
                       g_ErrCodeStack.szErrStackSysTime[iCount]);
            RM_reply_packet.Data.errhistory_reply.nErrorCodeStack[nMsgIdx]   = 
                                        g_ErrCodeStack.nErrStack[iCount];
            RM_reply_packet.Data.errhistory_reply.fErrorActiveState[nMsgIdx] = 
                                        g_ErrCodeStack.fErrorActiveState[iCount];
            CRT_strcpy(RM_reply_packet.Data.errhistory_reply.szErrContent[nMsgIdx],
                       ERROR_NAME_LEN,
                       g_ErrCodeStack.g_szErrContent[iCount]);

            nMsgIdx++;
        }
        else
        {
            CRT_strcpy(RM_reply_packet.Data.errhistory_reply.szErrStackSysTime[iCount - g_nErrHistoryStartIdx],
                       SYSTIME_DATA_LEN,
                       "0");
            RM_reply_packet.Data.errhistory_reply.nErrorCodeStack[iCount - g_nErrHistoryStartIdx]   = 
                                        SYS_ERR_OK;
            RM_reply_packet.Data.errhistory_reply.fErrorActiveState[iCount - g_nErrHistoryStartIdx] = 
                                        OFF;
            CRT_strcpy(RM_reply_packet.Data.errhistory_reply.szErrContent[iCount - g_nErrHistoryStartIdx],
                       ERROR_NAME_LEN,
                       " ");
        }
    }

    g_fErrHistPacketDef = ON;

    RM_reply_packet.nCode      = RM_packet.nCode;
    RM_reply_packet.nValue     = RM_packet.nValue;
    RM_reply_packet.nDataSize  = sizeof(RMGR_ERR_HISTORY_REPLY_DATA);

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_ShowErrorHistoryDisplay()
//      - Service Name: RMGR_SERV_ERRHISTORY

int SVC_ShowErrorHistoryDisplay(void)
{
    static int iCount;

    if(g_fErrHistPacketDef != ON)
    {
        g_nErrHistoryStartIdx = RECENT_ERRHIST_IDX;
        g_nErrHistoryReqCnt = DEF_ERRHIST_REQ_CNT;

        _loc_Fn_CheckErrHistoryReqArgValidation();
    }
    
    g_fErrHistPacketDef = OFF;

    VERBOSE_VERBOSE("\v\n<- Error History ->\n");
    VERBOSE_VERBOSE("\v Total ErrCnt: %d, StackTop: %d, ReqIdx: %d ~ %d\n",
                    g_ErrCodeStack.nErrCnt,
                    g_ErrCodeStack.nTop,
                    g_nErrHistoryStartIdx,
                   (g_nErrHistoryStartIdx + g_nErrHistoryReqCnt));

    VERBOSE_VERBOSE("\v No \tTime\t\t\tCode\tAct\tContents\n");
    for(iCount = g_nErrHistoryStartIdx;
        iCount < (g_nErrHistoryStartIdx + g_nErrHistoryReqCnt);
        iCount++)
    {
        SYSMON_ParceErrCodeToErrContent(g_ErrCodeStack.nErrStack[iCount]);
        VERBOSE_VERBOSE("\v[%d]\t%s\t%x\t%d\t%s\n",
                        iCount,
                        g_ErrCodeStack.szErrStackSysTime[iCount],
                        g_ErrCodeStack.nErrStack[iCount],
                        g_ErrCodeStack.fErrorActiveState[iCount],
                        g_ErrCodeStack.g_szErrContent[iCount]);

        if(iCount != 0 && (iCount % 10) == 0)
        {
            VERBOSE_VERBOSE("\v\n");
        }
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_ShowSystemStateDisplay()
//      - Service Name: RMGR_SERV_SYSSTATE

int SVC_ShowSystemStateDisplay(void)
{
    VERBOSE_VERBOSE("\v\n<- System State Info ->\n");

    VERBOSE_VERBOSE("\v   ExecStat : %d\t"
                    "   WorkType : %d\t"
                    "   SysMode  : %d\t",
                    g_pShm_SysStatus->nExecStat,
                    g_pShm_SysStatus->nWorkType,
                    g_pShm_SysStatus->nSystemMode);

    VERBOSE_VERBOSE("\v   ErrCode(fErr): %d(%d)\n" 
                    "   TE fInit: %d, fExit: %d\t"
                    "   SC fInit: %d, fExit: %d\n"
                    "   Error Code History\n",
                    g_pShm_SysStatus->nErrCode,
                    g_pShm_SysStatus->fErrorState,
                    g_pShm_SysStatus->fInitProcTE,
                    g_pShm_SysStatus->fExitProcTE,
                    g_pShm_SysStatus->fInitProcSC,
                    g_pShm_SysStatus->fExitProcSC);
    
    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SetSystemInfoPacketDefine()
//      - Service Name: RMGR_SERV_SYSINFO
//                  Value: 0 -> Global(Control), 1-> Global(Ecat),
//                         2 -> Robot, 3 -> Axis, 4 -> Motor, 5 -> Welder

int SVC_SetSystemInfoPacketDefine(int nValue)
{
    static int nRobot, nAxis, nMotor, nWelder;
    static int iAxis;
    static int nAxesCount;

    nRobot  = 0;
    nAxis   = 0;
    nMotor  = 0;
    nWelder = 0;

    if(nValue == 0)         // Global Param(Control)
    {
        ;
    }
    if(nValue == 1)         // Global Param(Ecat)
    {
        ;
    }
    else if(nValue == 2)    // Robot Param
    {
        nRobot = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 3)    // Axis Param
    {
        nAxis = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 4)    // Motor Param
    {
        nMotor = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 5)    // Welder Param
    {
        nWelder = RM_packet.Data.sys_info.nReqSystemIndex;
    }

    ///////////////////////////////////
    // GLOBAL
    if(nValue == 0)
    {
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nQNXTimerRes    =
                                        g_nControlParam.nQNXTimerRes;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nQNXTimerTick   =
                                        g_nControlParam.nQNXTimerTick;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nIoScanTime     =
                                        g_nControlParam.nIoScanTime;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nTrajUpdateTime =
                                        g_nControlParam.nTrajUpdateTime;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nServoInterpolationTime =
                                        g_nControlParam.nServoInterpolationTime;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nServoOnBrakeDelayTime =
                                        g_nControlParam.nServoOnBrakeDelayTime;
        RM_reply_packet.Data.sys_info.INFO.shm_ctrl.nServoOffBrakeDelayTime =
                                        g_nControlParam.nServoOffBrakeDelayTime;
    }

    if(nValue == 1)
    {
        CRT_strcpy(RM_reply_packet.Data.sys_info.INFO.shm_ecat.szPathName,
                   CONFIG_PATH_NAME_LEN,
                   g_pszEcatConfigDir);
        RM_reply_packet.Data.sys_info.INFO.shm_ecat.nSlaveCount      =
                                        g_nECATParam.nSlaveCount;
        RM_reply_packet.Data.sys_info.INFO.shm_ecat.nWriteOffsetSize =
                                        g_nECATParam.nWriteOffsetSize;
        RM_reply_packet.Data.sys_info.INFO.shm_ecat.nReadOffsetSize  =
                                        g_nECATParam.nReadOffsetSize;
    }
    
    g_pRobot = g_pShm_SysConfig->robot + nRobot;
    g_pAxis  = g_pShm_SysConfig->robot->axis;
    g_pMotor = g_pShm_SysConfig->motor + nMotor;

    ///////////////////////////////////
    // ROBOT
    // svc value: 2
    if(nValue == 2)
    {
        RM_reply_packet.Data.sys_info.INFO.shm_robot.fUsed = g_rgfRobotUsed[nRobot];
        CRT_strcpy(RM_reply_packet.Data.sys_info.INFO.shm_robot.szRobotName,
                   ROBOT_NAME_LEN,
                   g_pRobot->szRobotName);
        RM_reply_packet.Data.sys_info.INFO.shm_robot.nRobotType = g_pRobot->nRobotType;
        
        // maximum speed
        for (iAxis = 0; iAxis < g_rgnRobotAxisCount[nRobot]; iAxis++)
        {
            RM_reply_packet.Data.sys_info.INFO.shm_robot.dbMaxJointSpeed[iAxis] = 
                                              g_pRobot->dbMaxJointSpeed[iAxis] * (180/M_PI) * 1000;
            RM_reply_packet.Data.sys_info.INFO.shm_robot.dh[iAxis].l  = g_pRobot->dh[iAxis].l;
            RM_reply_packet.Data.sys_info.INFO.shm_robot.dh[iAxis].al = g_pRobot->dh[iAxis].al * (180/M_PI);
            RM_reply_packet.Data.sys_info.INFO.shm_robot.dh[iAxis].d  = g_pRobot->dh[iAxis].d;
            RM_reply_packet.Data.sys_info.INFO.shm_robot.dh[iAxis].th = g_pRobot->dh[iAxis].th * (180/M_PI);
        }
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbMaxLinearSpeed =
                                              g_pRobot->dbMaxLinearSpeed * 1000;
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbMaxOrientSpeed =
                                              g_pRobot->dbMaxOrientSpeed * (180/M_PI) * 1000;
        
        // jerk
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbJerk = g_pRobot->dbJerk;

        // accel/decel pattern determined by jerk(0: trapezoidal)
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbAccel = g_pRobot->dbAccel;
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbDecel = g_pRobot->dbDecel;
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbDecel_Error = g_pRobot->dbDecel_Error;
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbDecel_Estop = g_pRobot->dbDecel_Estop;
        RM_reply_packet.Data.sys_info.INFO.shm_robot.dbDecel_Touch = g_pRobot->dbDecel_Touch;
    }
    
    nAxesCount = 0;

    RM_reply_packet.Data.sys_info.INFO.shm_robot.nAxesCount = g_pRobot->nAxesCount;

    ///////////////////////////////////
    // Axis
    // svc value: 3
    nAxesCount = g_pRobot->nAxesCount;
    DANDY_ASSERT(nAxesCount >= 0 && nAxesCount <= MAX_AXIS_COUNT);

    if(nValue == 3)
    {
        //g_pAxis = g_pShm_SysConfig->robot->axis + nAxis;
        g_pAxis = g_pShm_SysConfig->robot->axis;

        CRT_strcpy(RM_reply_packet.Data.sys_info.INFO.shm_axis.szName,
                   AXIS_NAME_LEN, g_pAxis[nAxis].szName);
        RM_reply_packet.Data.sys_info.INFO.shm_axis.nAxisType = g_pAxis[nAxis].nAxisType;
        RM_reply_packet.Data.sys_info.INFO.shm_axis.nAxisIndex = g_pAxis[nAxis].nAxisIndex;

        RM_reply_packet.Data.sys_info.INFO.shm_axis.fHwLim_min = g_pAxis[nAxis].fHwLim_min;
        RM_reply_packet.Data.sys_info.INFO.shm_axis.fHwLim_max = g_pAxis[nAxis].fHwLim_max;
        RM_reply_packet.Data.sys_info.INFO.shm_axis.pos_hwlim_min = g_pAxis[nAxis].pos_hwlim_min * (180/M_PI);
        RM_reply_packet.Data.sys_info.INFO.shm_axis.pos_hwlim_max = g_pAxis[nAxis].pos_hwlim_max * (180/M_PI);

        RM_reply_packet.Data.sys_info.INFO.shm_axis.fSwLim_min = g_pAxis[nAxis].fSwLim_min;
        RM_reply_packet.Data.sys_info.INFO.shm_axis.fHwLim_max = g_pAxis[nAxis].fSwLim_max;
        RM_reply_packet.Data.sys_info.INFO.shm_axis.pos_swlim_min = g_pAxis[nAxis].pos_swlim_min * (180/M_PI);
        RM_reply_packet.Data.sys_info.INFO.shm_axis.pos_swlim_max = g_pAxis[nAxis].pos_swlim_max * (180/M_PI);

        RM_reply_packet.Data.sys_info.INFO.shm_axis.ori[0] = g_pAxis[nAxis].ori[0];
        RM_reply_packet.Data.sys_info.INFO.shm_axis.red[0] = g_pAxis[nAxis].red[0];
        RM_reply_packet.Data.sys_info.INFO.shm_axis.dir[0] = g_pAxis[nAxis].dir[0];
        RM_reply_packet.Data.sys_info.INFO.shm_axis.nMotorCount = g_pAxis[nAxis].nMotorCount;
    }

    ///////////////////////////////////
    // MOTOR
    // svc value: 4
    if(nValue == 4)
    {
        CRT_strcpy(RM_reply_packet.Data.sys_info.INFO.shm_motor.szName,
                   MOTOR_NAME_LEN, g_pMotor->szName);
        RM_reply_packet.Data.sys_info.INFO.shm_motor.nMotorType = g_pMotor->nMotorType;
        RM_reply_packet.Data.sys_info.INFO.shm_motor.nMotorIndex = g_pMotor->nMotorIndex;

        RM_reply_packet.Data.sys_info.INFO.shm_motor.jrk = g_pMotor->jrk;

        RM_reply_packet.Data.sys_info.INFO.shm_motor.acc = g_pMotor->acc;
        RM_reply_packet.Data.sys_info.INFO.shm_motor.dec = g_pMotor->dec;

        RM_reply_packet.Data.sys_info.INFO.shm_motor.dec_error = g_pMotor->dec_error;
        RM_reply_packet.Data.sys_info.INFO.shm_motor.dec_estop = g_pMotor->dec_estop;

        RM_reply_packet.Data.sys_info.INFO.shm_motor.vellim_max = g_pMotor->vellim_max * (180/M_PI);
        RM_reply_packet.Data.sys_info.INFO.shm_motor.nEncRes = g_pMotor->nEncRes;
    }

    ///////////////////////////////////
    // Welder
    // svc value: 5
    if(nValue == 5)
    {
        CRT_strcpy(RM_reply_packet.Data.sys_info.INFO.shm_welder.szName,
                   WELDER_NAME_LEN,
                   g_pShm_SysConfig->welder[nWelder].szName);
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nType =
                                g_pShm_SysConfig->welder[nWelder].nType;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nAbility =
                                g_pShm_SysConfig->welder[nWelder].nAbility;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nWelder =
                                g_pShm_SysConfig->welder[nWelder].nWelder;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nRobot =
                                g_pShm_SysConfig->welder[nWelder].nRobot;

        RM_reply_packet.Data.sys_info.INFO.shm_welder.nDinSlaveNo =
                                g_pShm_SysConfig->welder[nWelder].nDinSlaveNo;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.din_portno =
                                g_pShm_SysConfig->welder[nWelder].din_portno;

        RM_reply_packet.Data.sys_info.INFO.shm_welder.nDoutSlaveNo =
                                g_pShm_SysConfig->welder[nWelder].nDoutSlaveNo;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.dout_portno =
                                g_pShm_SysConfig->welder[nWelder].dout_portno;

        RM_reply_packet.Data.sys_info.INFO.shm_welder.nCurrentInPortNo =
                                g_pShm_SysConfig->welder[nWelder].nCurrentInPortNo;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nVoltageInPortNo =
                                g_pShm_SysConfig->welder[nWelder].nVoltageInPortNo;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nCurrentOutPortNo =
                                g_pShm_SysConfig->welder[nWelder].nCurrentOutPortNo;
        RM_reply_packet.Data.sys_info.INFO.shm_welder.nVoltageOutPortNo =
                                g_pShm_SysConfig->welder[nWelder].nVoltageOutPortNo;
    }

    RM_reply_packet.Data.sys_info.nReqSystemIndex =
                                        RM_packet.Data.sys_info.nReqSystemIndex;
    RM_reply_packet.nCode  = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    if(nValue == 0)         // Global Param (Control)
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_ctrl) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);

    }
    if(nValue == 1)         // Global Param (Ecat)
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_ecat) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);

    }
    else if(nValue == 2)    // Robot Param
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_robot) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);
    }
    else if(nValue == 3)    // Axis Param
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_axis) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);
    }
    else if(nValue == 4)    // Motor Param
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_motor) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);
    }
    else if(nValue == 5)    // Welder Param
    {
        RM_reply_packet.nDataSize  = sizeof(RM_reply_packet.Data.sys_info.INFO.shm_welder) +
                                     sizeof(RM_reply_packet.Data.sys_info.nReqSystemIndex);
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_ShowSystemInfoDisplay()
//      - Service Name: RMGR_SERV_SYSINFO

int SVC_ShowSystemInfoDisplay(int nValue)
{
    static int nRobot, nAxis, nMotor, nWelder;
    static int iAxis;

    nRobot  = 0;
    nAxis   = 0;
    nMotor  = 0;
    nWelder = 0;

    if(nValue == 0)         // Global Param(Control)
    {
        ;
    }
    else if(nValue == 1)         // Global Param(Ecat)
    {
        ;
    }
    else if(nValue == 2)    // Robot Param
    {
        nRobot = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 3)    // Axis Param
    {
        nAxis = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 4)    // Motor Param
    {
        nMotor = RM_packet.Data.sys_info.nReqSystemIndex;
    }
    else if(nValue == 5)    // Welder Param
    {
        nWelder = RM_packet.Data.sys_info.nReqSystemIndex;
    }

    // Print Global Config (Control)
    if(nValue == 0)
    {
        VERBOSE_NOTIFY("\n<- System Infomation ->\n"
                       "<Control Parameter>\n"
                       "QNX Timer Tick  : %d ns\t"
                       "QNX Timer Res   : %d us\n"
                       "Trajectory Time : %d ms\t"
                       "ServoInterp Time: %d cnt\n"
                       "Io Scan Time    : %d ms\n",
                       g_pShm_SysConfig->ctrl.nQNXTimerTick,
                       g_pShm_SysConfig->ctrl.nQNXTimerRes,
                       g_pShm_SysConfig->ctrl.nTrajUpdateTime,
                       g_pShm_SysConfig->ctrl.nServoInterpolationTime,
                       g_pShm_SysConfig->ctrl.nIoScanTime);
    }

    // Print Global Config (Control)
    if(nValue == 1)
    {
        VERBOSE_NOTIFY("\v<EtherCAT Parameter>\n"
                       "Slave Count : %d cnt\t"
                       "Write Offset Size: 0x%x \n"
                       "Read Offset Size: 0x%x \t"
                       "ECAT Conf File  : %s\n", 
                       g_pShm_SysConfig->ecat.nSlaveCount,
                       g_pShm_SysConfig->ecat.nWriteOffsetSize,
                       g_pShm_SysConfig->ecat.nReadOffsetSize,
                       g_pShm_SysConfig->ecat.szPathName);
    }
	
    // Print Robot Config
    if(g_pShm_SysConfig->robot[nRobot].fUsed == TRUE)
    {
        // svc value: 2
        if(nValue == 2)
        {
            VERBOSE_NOTIFY("\v\n<Robot %d Parameter: Total %d Robot Available>\n"
                           "Robot Name: %s\t",
                           nRobot, MAX_ROBOT_COUNT,
                           g_pShm_SysConfig->robot[nRobot].szRobotName);

            if(g_pShm_SysConfig->robot[nRobot].nRobotType == ROBTYPE_DR6 ||
               g_pShm_SysConfig->robot[nRobot].nRobotType == ROBTYPE_10KG)
                VERBOSE_NOTIFY("\vRobot Type: Vertical 5Bar Link\n");
            else if(g_pShm_SysConfig->robot[nRobot].nRobotType == ROBTYPE_DANDY_II ||
                    g_pShm_SysConfig->robot[nRobot].nRobotType == ROBTYPE_DANDY)
                VERBOSE_NOTIFY("\vRobot Type: Vertical 4Bar Link\n");
            else if(g_pShm_SysConfig->robot[nRobot].nRobotType == ROBTYPE_RECTANGULAR)
                VERBOSE_NOTIFY("\vRobot Type: Cartesian\n");
            
            // Motion Parameters
            VERBOSE_NOTIFY("\vRobot Max Joint Spd[deg/s]: ");
            for (iAxis = 0; iAxis < g_rgnRobotAxisCount[nRobot]; iAxis++)
            {
                VERBOSE_NOTIFY("\v%.1lf  ",
                               g_pShm_SysConfig->robot[nRobot].dbMaxJointSpeed[iAxis] * (180/M_PI) * 1000);
            }
            VERBOSE_NOTIFY("\v\n");

            VERBOSE_NOTIFY("\vRobot Max Linear Spd: %.1lf mm/s\t"
                           "Robot Max Orient Spd: %.1lf deg/s\n",
                           g_pShm_SysConfig->robot[nRobot].dbMaxLinearSpeed * 1000,
                           g_pShm_SysConfig->robot[nRobot].dbMaxOrientSpeed * (180/M_PI) * 1000);

            VERBOSE_NOTIFY("\vJerk: %.2lf ms\t"
                           "Accel: %.2lf ms\t"
                           "Decel: %.2lf ms\n"
                           "Robot Error Decel: %.2lf ms\t\t"
                           "Robot Estop Decel: %.2lf ms\n"
                           "Robot Touch Decel: %.2lf ms\t\t"
                           "Robot Axis Count: %d cnt\n",
                           g_pShm_SysConfig->robot[nRobot].dbJerk,
                           g_pShm_SysConfig->robot[nRobot].dbAccel,
                           g_pShm_SysConfig->robot[nRobot].dbDecel,
                           g_pShm_SysConfig->robot[nRobot].dbDecel_Error,
                           g_pShm_SysConfig->robot[nRobot].dbDecel_Estop,
                           g_pShm_SysConfig->robot[nRobot].dbDecel_Touch,
                           g_pShm_SysConfig->robot[nRobot].nAxesCount);
            
            // DH Parameters
           VERBOSE_NOTIFY("\vLink_th: %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].dh[0].th * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[1].th * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[2].th * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[3].th * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[4].th * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[5].th * (180/M_PI));
           VERBOSE_NOTIFY("\vLink_d : %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].dh[0].d,
                           g_pShm_SysConfig->robot[nRobot].dh[1].d,
                           g_pShm_SysConfig->robot[nRobot].dh[2].d,
                           g_pShm_SysConfig->robot[nRobot].dh[3].d,
                           g_pShm_SysConfig->robot[nRobot].dh[4].d,
                           g_pShm_SysConfig->robot[nRobot].dh[5].d);
           VERBOSE_NOTIFY("\vLink_al: %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].dh[0].al * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[1].al * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[2].al * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[3].al * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[4].al * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].dh[5].al * (180/M_PI));
           VERBOSE_NOTIFY("\vLink_l : %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].dh[0].l,
                           g_pShm_SysConfig->robot[nRobot].dh[1].l,
                           g_pShm_SysConfig->robot[nRobot].dh[2].l,
                           g_pShm_SysConfig->robot[nRobot].dh[3].l,
                           g_pShm_SysConfig->robot[nRobot].dh[4].l,
                           g_pShm_SysConfig->robot[nRobot].dh[5].l);
           
           // Coordinate Parameters
           VERBOSE_NOTIFY("\vTCP : %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].coordTool.x,
                           g_pShm_SysConfig->robot[nRobot].coordTool.y,
                           g_pShm_SysConfig->robot[nRobot].coordTool.z,
                           g_pShm_SysConfig->robot[nRobot].coordTool.rol * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].coordTool.pit * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].coordTool.yaw * (180/M_PI));
           VERBOSE_NOTIFY("\vWorld: %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
                           g_pShm_SysConfig->robot[nRobot].world.x,
                           g_pShm_SysConfig->robot[nRobot].world.y,
                           g_pShm_SysConfig->robot[nRobot].world.z,
                           g_pShm_SysConfig->robot[nRobot].world.rol * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].world.pit * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].world.yaw * (180/M_PI));
        }

        // Print Axis Config
        // svc value: 3
        if(nValue == 3)
        {
            VERBOSE_NOTIFY("\v\n   <Axis %d Parameter>\n"
                           "   Axis Name: %s\t",
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].nAxisIndex,
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].szName);

            if(g_pShm_SysConfig->robot[nRobot].axis[nAxis].nAxisType == AXISTYPE_REVOLUTE)
                VERBOSE_NOTIFY("\vAxis Type: Revolute\n");
            else if(g_pShm_SysConfig->robot[nRobot].axis[nAxis].nAxisType == AXISTYPE_PRISMATIC)
                VERBOSE_NOTIFY("\vAxis Type: Prismatic\n");
            else if(g_pShm_SysConfig->robot[nRobot].axis[nAxis].nAxisType == AXISTYPE_NONE)
                VERBOSE_NOTIFY("\vAxis Type: None\n");
            else
                VERBOSE_NOTIFY("\vAxis Type: Error\n");

            VERBOSE_NOTIFY("\v   SW Limit-> +: %d(Pos: %.3lf), -: %d(neg: %.3lf)\n"
                             "   HW Limit-> +: %d(Pos: %.3lf), -: %d(neg: %.3lf)\n"
                             "   Motor Count: %d cnt\n",
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].fSwLim_max,
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].pos_swlim_max * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].fSwLim_min,
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].pos_swlim_min * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].fHwLim_max,
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].pos_hwlim_max * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].fHwLim_min,
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].pos_hwlim_min * (180/M_PI),
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].nMotorCount);
            VERBOSE_NOTIFY("\v      Encoder Origin Value: %d p\t\t"
                           "Gear Reduction Ratio: %.2lf\n"
                           "      Direction: %d\n",
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].ori[0],
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].red[0],
                           g_pShm_SysConfig->robot[nRobot].axis[nAxis].dir[0]);
        }
        // Print Motor Config
        // svc value: 4
        if(nValue == 4)
        {
            if(nValue == 4)
            {
                 VERBOSE_NOTIFY("\v\n");
            }
            VERBOSE_NOTIFY("\v      <Motor %d Parameter>\n"
                           "      Motor Name: %s\t",
                           g_pShm_SysConfig->motor[nMotor].nMotorIndex,
                           g_pShm_SysConfig->motor[nMotor].szName);
            
            if(g_pShm_SysConfig->motor[nMotor].nMotorType == MOTTYPE_SIGMA_V)
                VERBOSE_NOTIFY("\v      Motor Type: Sigma-V(Yaskawa)\n");
            else if(g_pShm_SysConfig->motor[nMotor].nMotorType == MOTTYPE_MINAS)
                VERBOSE_NOTIFY("\v      Motor Type: Minas(Panasonic)\n");

            VERBOSE_NOTIFY("\v      Jerk: %.2lf ms    "
                           "Accel: %.2lf ms    "
                           "Decel: %.2lf ms\n"
                           "      Motor Error Decel: %.2lf ms\t"
                           "Motor Estop Decel: %.2lf ms\n"
                           "      Motor Velocity Max: %.2lf deg/s\t"
                           "Motor Encoder Resolution: %d\n",
                           g_pShm_SysConfig->motor[nMotor].jrk,
                           g_pShm_SysConfig->motor[nMotor].acc,
                           g_pShm_SysConfig->motor[nMotor].dec,
                           g_pShm_SysConfig->motor[nMotor].dec_error,
                           g_pShm_SysConfig->motor[nMotor].dec_estop,
                           g_pShm_SysConfig->motor[nMotor].vellim_max * (180/M_PI),
                           g_pShm_SysConfig->motor[nMotor].nEncRes);
        }
    }

    ///////////////////////////////////
    // Welder
    // svc value: 5
    if(nValue == 5)
    {
        VERBOSE_NOTIFY("\v\n   <Robot %d - Welder %d Parameter>\n"
                       "   Welder Name: %s\n",
                       g_pShm_SysConfig->welder[nWelder].nRobot,
                       g_pShm_SysConfig->welder[nWelder].nWelder,
                       g_pShm_SysConfig->welder[nWelder].szName);

        if(g_pShm_SysConfig->welder[nWelder].nType == WELDER_TYPE_HYOSUNG_UR)
            VERBOSE_NOTIFY("\vWelder Type: HyoSung UR\t");
        else if(g_pShm_SysConfig->welder[nWelder].nType == WELDER_TYPE_DAIHEN_DM)
            VERBOSE_NOTIFY("\vWelder Type: DaiHen DM\t");
        else if(g_pShm_SysConfig->welder[nWelder].nType == WELDER_TYPE_ZEUS)
            VERBOSE_NOTIFY("\vWelder Type: Zeus\t");
        else if(g_pShm_SysConfig->welder[nWelder].nType == WELDER_TYPE_GENERIC)
            VERBOSE_NOTIFY("\vWelder Type: Generic\t");
        else if(g_pShm_SysConfig->welder[nWelder].nType == WELDER_TYPE_NONE)
            VERBOSE_NOTIFY("\vWelder Type: None\t");
        else
            VERBOSE_NOTIFY("\vWelder Type: Error\t");

        if(g_pShm_SysConfig->welder[nWelder].nAbility == WELDER_ABIL_CO2)
            VERBOSE_NOTIFY("\vWelder Ability: CO2\n");
        else if(g_pShm_SysConfig->welder[nWelder].nAbility == WELDER_ABIL_MIG)
            VERBOSE_NOTIFY("\vWelder Ability: MIG\n");
        else if(g_pShm_SysConfig->welder[nWelder].nAbility == WELDER_ABIL_MAG)
            VERBOSE_NOTIFY("\vWelder Ability: MAG\n");
        else if(g_pShm_SysConfig->welder[nWelder].nAbility == WELDER_ABIL_TIG)
            VERBOSE_NOTIFY("\vWelder Ability: TIG\n");
        else if(g_pShm_SysConfig->welder[nWelder].nAbility == WELDER_ABIL_VOID)
            VERBOSE_NOTIFY("\vWelder Ability: Void\n");
        else
            VERBOSE_NOTIFY("\vWelder Ability: Error\n");

        VERBOSE_NOTIFY("\v   Din Slave Cnt: %d\n"
            "   Din Port No-> [ArcOn: %d] [NoGas: %d] [NoWire: %d] [WeldPwrFail: %d]\n"
            "                 [TPorc: %d] [TSignal: %d] [WeldOn: %d] [ArcFail: %d]\n",
            g_pShm_SysConfig->welder[nWelder].nDinSlaveNo,
            g_pShm_SysConfig->welder[nWelder].din_portno.nArcOn,
            g_pShm_SysConfig->welder[nWelder].din_portno.nNoGas,
            g_pShm_SysConfig->welder[nWelder].din_portno.nNoWire,
            g_pShm_SysConfig->welder[nWelder].din_portno.nWeldPowerFail,
            g_pShm_SysConfig->welder[nWelder].din_portno.nTouchProcess,
            g_pShm_SysConfig->welder[nWelder].din_portno.nTouchSignal,
            g_pShm_SysConfig->welder[nWelder].din_portno.nWeld,
            g_pShm_SysConfig->welder[nWelder].din_portno.nArcFail);

        VERBOSE_NOTIFY("\v   Dout Slave Cnt: %d\n"
            "   Din Port No-> [ArcOn: %d] [GasOn: %d] [InchPos: %d] [InchNeg: %d]\n"
            "                 [Tstart: %d] [Tready: %d] [WireCut: %d] [WeldPwr: %d]\n",
            g_pShm_SysConfig->welder[nWelder].nDoutSlaveNo,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nArcOn,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nGasOn,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nInchPos,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nInchNeg,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nTouchStart,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nTouchReady,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nWireCut,
            g_pShm_SysConfig->welder[nWelder].dout_portno.nWeldPower);

        VERBOSE_NOTIFY("\v   Ain Port No -> [CuurentIn: %d] [VoltageIn: %d]\n"
                       "                  [CuurentIn: %d] [VoltageIn: %d]\n",
            g_pShm_SysConfig->welder[nWelder].nCurrentInPortNo,
            g_pShm_SysConfig->welder[nWelder].nVoltageInPortNo,
            g_pShm_SysConfig->welder[nWelder].nCurrentOutPortNo,
            g_pShm_SysConfig->welder[nWelder].nVoltageOutPortNo);

        VERBOSE_NOTIFY("\v   Volt Offset Unit: %3.1lf, Curr Offset Unit: %3.1lf\n",
            g_dbVoltRealTimeCmdOffsetUnit, g_dbCurrRealTimeCmdOffsetUnit);
        
        VERBOSE_NOTIFY("\v   [Volt Calib] Y: %3.1lf ~ %3.1lf,\tX: %3.1lf ~ %3.1lf\n",
                       g_dbUpperBoundY_Volt, g_dbLowerBoundY_Volt,
                       g_dbUpperBoundX_Volt, g_dbLowerBoundX_Volt);
        VERBOSE_NOTIFY("\v   [Curr Calib] Y: %4.1lf ~ %4.1lf,\tX: %4.1lf ~ %4.1lf\n",
                       g_dbUpperBoundY_Curr, g_dbLowerBoundY_Curr,
                       g_dbUpperBoundX_Curr, g_dbLowerBoundX_Curr);
    }
    
    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SendVersionInform()
//      - Service Name: RMGR_SERV_SYSVERSION

int SVC_SendVersionInform(void)
{
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RMGR_VER_REPLY_DATA);
    CRT_strcpy(RM_reply_packet.Data.reply_ver.rgchRM_vers,
               RMGR_VERSION_DATA_LEN,
               SYS_RM_VERSION);
    CRT_strcpy(RM_reply_packet.Data.reply_ver.rgchRM_build,
               RMGR_BUILD_DATA_LEN,
               SYS_RM_BUILD);

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SystemMonDataSendToTP()
//      - Service Name: RSVC_SERV_SYSMON_TO_TP

int SVC_SystemMonDataSendToTP(void)
{
    SHM_DANDY_JOB* pShmem = g_pShmemJobRM;

    RM_reply_packet.Data.sys_mon.nErrCode    = g_pShm_SysStatus->nErrCode;
    RM_reply_packet.Data.sys_mon.fEstopState = g_pShm_SysStatus->fEStopState;
    RM_reply_packet.Data.sys_mon.nEstopCode  = g_pShm_SysStatus->nEstopCode;
    
    if(g_pShmemSC != NULL)
    {
        RM_reply_packet.Data.sys_mon.fEcatRunState = g_pShmemSC->sysstate.fEcatInitState;
    }

    RM_reply_packet.Data.sys_mon.nExecStat   = g_pShm_SysStatus->nExecStat;
    RM_reply_packet.Data.sys_mon.nWorkType   = g_pShm_SysStatus->nWorkType;
    RM_reply_packet.Data.sys_mon.nSystemMode = g_pShm_SysStatus->nSystemMode;

    memcpy(RM_reply_packet.Data.sys_mon.szJobFileName,
           g_rgszLoadedJobModName,
           sizeof(RM_reply_packet.Data.sys_mon.szJobFileName));

    RM_reply_packet.Data.sys_mon.nCmdLoadCount = pShmem->dwCmdLoadCount;

    if(g_pShmemTEStatus != NULL)
    {
        SYSMON_CheckJobExecLineIndex();
        RM_reply_packet.Data.sys_mon.nJobRunIndex = g_nJobExecLineIdx;
    }
    else
    {
        RM_reply_packet.Data.sys_mon.nJobRunIndex = -1;
    }

    if(g_pShmemSC != NULL)
    {
        RM_reply_packet.Data.sys_mon.fDeadManState = 
                    g_pShmemSC->inputstate.fDeadManSwithInState;
    }
    else
    {
        RM_reply_packet.Data.sys_mon.fDeadManState = -1;
    }

    if(g_pShmemSC != NULL)
    {
        RM_reply_packet.Data.sys_mon.fServoOnOutState = 
                    g_pShmemSC->outputstate.fServoOnOutState;
    }
    else
    {
        RM_reply_packet.Data.sys_mon.fServoOnOutState = -1;
    }

    RM_reply_packet.Data.sys_mon.nJobLoadCnt = g_nJobLoadCnt;

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.sys_mon);

    //VERBOSE_VERBOSE("RealTimeMonData Send Done!\n");

#if 0
    VERBOSE_VERBOSE("ErrCode: %d, EstopState: %d, ExecStat: %d, WorkType: %d\n"
                    "Systemmode: %d, FileName: %s, nJobRunIdx: %d, Deadman: %d, Servo: %d\n",
                    RM_reply_packet.Data.sys_mon.nErrCode,
                    RM_reply_packet.Data.sys_mon.fEstopState,
                    RM_reply_packet.Data.sys_mon.nExecStat,
                    RM_reply_packet.Data.sys_mon.nWorkType,
                    RM_reply_packet.Data.sys_mon.nSystemMode,
                    RM_reply_packet.Data.sys_mon.szJobFileName,
                    RM_reply_packet.Data.sys_mon.nJobRunIndex,
                    RM_reply_packet.Data.sys_mon.fDeadManState,
                    RM_reply_packet.Data.sys_mon.fServoOnOutState);
#endif

    return 0;
}

