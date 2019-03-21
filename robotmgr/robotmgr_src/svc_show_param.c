/////////////////////////////////////////////////////////////////////////////
//
//  param_show_svc.c: Show Parameter Service
//                                            2014.06.13  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable


/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_SendHomePositionValue()
//      - Service Name: RSVC_SERV_SHOWHOME_TP

int SVC_SendHomePositionValue(int nHomeIndex)
{
    int iAxis;

    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }

    RM_reply_packet.Data.home_dat.nHomeIndex = nHomeIndex;
    
    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        RM_reply_packet.Data.home_dat.rgdbHomePosVal[iAxis] =
                g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][iAxis] * (180/M_PI);
    }
    
    VERBOSE_MESSAGE("Current HomePos[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nHomeIndex,
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][0] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][1] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][2] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][3] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][4] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][5] * (180/M_PI));

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.home_dat);

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SendUserCoordinateValue()
//      - Service Name: RSVC_SERV_SHOWUSRCRD_TP

int SVC_SendUserCoordinateValue(int nUserCoordIndex)
{
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }

    RM_reply_packet.Data.usrcrd_dat.nUserCoordIndex
        = nUserCoordIndex;
    
    RM_reply_packet.Data.usrcrd_dat.usercoord.x =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].x;
    RM_reply_packet.Data.usrcrd_dat.usercoord.y =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].y;
    RM_reply_packet.Data.usrcrd_dat.usercoord.z =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].z;
    RM_reply_packet.Data.usrcrd_dat.usercoord.rol =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].rol * (180/M_PI);
    RM_reply_packet.Data.usrcrd_dat.usercoord.pit =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].pit * (180/M_PI);
    RM_reply_packet.Data.usrcrd_dat.usercoord.yaw =
                g_pShm_SysConfig->robot->user[nUserCoordIndex].yaw * (180/M_PI);

    VERBOSE_MESSAGE("Current User Coord[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nUserCoordIndex,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].x,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].y,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].z,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].rol * (180/M_PI),
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].pit * (180/M_PI),
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].yaw * (180/M_PI));

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrcrd_dat);

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SendUserParameterValue()
//      - Service Name: RSVC_SERV_SHOWPARAM_TP

int SVC_SendUserParameterValue(int nOption)
{
    if(g_pShm_SysConfig == NULL || g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    if(nOption == 0)    //TCP
    {
        RM_reply_packet.Data.usrparam.PARAM.TCP.x   = g_pShm_SysConfig->robot->coordTool.x;
        RM_reply_packet.Data.usrparam.PARAM.TCP.y   = g_pShm_SysConfig->robot->coordTool.y;
        RM_reply_packet.Data.usrparam.PARAM.TCP.z   = g_pShm_SysConfig->robot->coordTool.z;
        RM_reply_packet.Data.usrparam.PARAM.TCP.rol = g_pShm_SysConfig->robot->coordTool.rol * (180/M_PI);
        RM_reply_packet.Data.usrparam.PARAM.TCP.pit = g_pShm_SysConfig->robot->coordTool.pit * (180/M_PI);
        RM_reply_packet.Data.usrparam.PARAM.TCP.yaw = g_pShm_SysConfig->robot->coordTool.yaw * (180/M_PI);

        VERBOSE_MESSAGE("Current TCP: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                        g_pShm_SysConfig->robot->coordTool.x,
                        g_pShm_SysConfig->robot->coordTool.y,
                        g_pShm_SysConfig->robot->coordTool.z,
                        g_pShm_SysConfig->robot->coordTool.rol * (180/M_PI),
                        g_pShm_SysConfig->robot->coordTool.pit * (180/M_PI),
                        g_pShm_SysConfig->robot->coordTool.yaw * (180/M_PI));

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
	else if(nOption == 1)   //world coord offset
    {
        RM_reply_packet.Data.usrparam.PARAM.world.x   = g_pShm_SysConfig->robot->world.x;
        RM_reply_packet.Data.usrparam.PARAM.world.y   = g_pShm_SysConfig->robot->world.y;
        RM_reply_packet.Data.usrparam.PARAM.world.z   = g_pShm_SysConfig->robot->world.z;
        RM_reply_packet.Data.usrparam.PARAM.world.rol = g_pShm_SysConfig->robot->world.rol * (180/M_PI);
        RM_reply_packet.Data.usrparam.PARAM.world.pit = g_pShm_SysConfig->robot->world.pit * (180/M_PI);
        RM_reply_packet.Data.usrparam.PARAM.world.yaw = g_pShm_SysConfig->robot->world.yaw * (180/M_PI);

        VERBOSE_MESSAGE("Current World Offset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                        g_pShm_SysConfig->robot->world.x,
                        g_pShm_SysConfig->robot->world.y,
                        g_pShm_SysConfig->robot->world.z,
                        g_pShm_SysConfig->robot->world.rol * (180/M_PI),
                        g_pShm_SysConfig->robot->world.pit * (180/M_PI),
                        g_pShm_SysConfig->robot->world.yaw * (180/M_PI));

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
    else if(nOption == 2)   //weld tune input
    {
        RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_a     = g_pShm_SysParam->weld_tune[0].input.dbVolt_a;
        RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_b     = g_pShm_SysParam->weld_tune[0].input.dbVolt_b;
        RM_reply_packet.Data.usrparam.PARAM.input.dbVoltScale  = g_pShm_SysParam->weld_tune[0].input.dbVoltScale;
        RM_reply_packet.Data.usrparam.PARAM.input.dbVoltOffset = g_pShm_SysParam->weld_tune[0].input.dbVoltOffset;
        RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_a     = g_pShm_SysParam->weld_tune[0].input.dbCurr_a;
        RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_b     = g_pShm_SysParam->weld_tune[0].input.dbCurr_b;
        RM_reply_packet.Data.usrparam.PARAM.input.dbCurrScale  = g_pShm_SysParam->weld_tune[0].input.dbCurrScale;
        RM_reply_packet.Data.usrparam.PARAM.input.dbCurrOffset = g_pShm_SysParam->weld_tune[0].input.dbCurrOffset;

        VERBOSE_MESSAGE("[Current Weld Tune Input Param] -> a, b, scale, offset\n"
                        "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf\n"
                        "  Current: %.8lf, %.8lf, %.8lf, %.8lf\n",
                        g_pShm_SysParam->weld_tune[0].input.dbVolt_a,
                        g_pShm_SysParam->weld_tune[0].input.dbVolt_b,
                        g_pShm_SysParam->weld_tune[0].input.dbVoltScale,
                        g_pShm_SysParam->weld_tune[0].input.dbVoltOffset,
                        g_pShm_SysParam->weld_tune[0].input.dbCurr_a,
                        g_pShm_SysParam->weld_tune[0].input.dbCurr_b,
                        g_pShm_SysParam->weld_tune[0].input.dbCurrScale,
                        g_pShm_SysParam->weld_tune[0].input.dbCurrOffset);

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
    else if(nOption == 3)   //weld tune output
    {
        RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_a     = g_pShm_SysParam->weld_tune[0].output.dbVolt_a;
        RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_b     = g_pShm_SysParam->weld_tune[0].output.dbVolt_b;
        RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_c     = g_pShm_SysParam->weld_tune[0].output.dbVolt_c;
        RM_reply_packet.Data.usrparam.PARAM.output.dbVoltScale  = g_pShm_SysParam->weld_tune[0].output.dbVoltScale;
        RM_reply_packet.Data.usrparam.PARAM.output.dbVoltOffset = g_pShm_SysParam->weld_tune[0].output.dbVoltOffset;
        RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_a     = g_pShm_SysParam->weld_tune[0].output.dbCurr_a;
        RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_b     = g_pShm_SysParam->weld_tune[0].output.dbCurr_b;
        RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_c     = g_pShm_SysParam->weld_tune[0].output.dbCurr_c;
        RM_reply_packet.Data.usrparam.PARAM.output.dbCurrScale  = g_pShm_SysParam->weld_tune[0].output.dbCurrScale;
        RM_reply_packet.Data.usrparam.PARAM.output.dbCurrOffset = g_pShm_SysParam->weld_tune[0].output.dbCurrOffset;

        VERBOSE_MESSAGE("[Current Weld Tune Output Param] -> a, b, c, scale, offset\n"
                        "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n"
                        "  Current: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n",
                        g_pShm_SysParam->weld_tune[0].output.dbVolt_a,
                        g_pShm_SysParam->weld_tune[0].output.dbVolt_b,
                        g_pShm_SysParam->weld_tune[0].output.dbVolt_c,
                        g_pShm_SysParam->weld_tune[0].output.dbVoltScale,
                        g_pShm_SysParam->weld_tune[0].output.dbVoltOffset,
                        g_pShm_SysParam->weld_tune[0].output.dbCurr_a,
                        g_pShm_SysParam->weld_tune[0].output.dbCurr_b,
                        g_pShm_SysParam->weld_tune[0].output.dbCurr_c,
                        g_pShm_SysParam->weld_tune[0].output.dbCurrScale,
                        g_pShm_SysParam->weld_tune[0].output.dbCurrOffset);

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
    else if(nOption == 4)   //restart parameters
    {
        RM_reply_packet.Data.usrparam.PARAM.restart.moving_type    = g_pShm_SysParam->TE_restart[0].moving_type;
        RM_reply_packet.Data.usrparam.PARAM.restart.d_overlap_horz = g_pShm_SysParam->TE_restart[0].d_overlap_horz;
        RM_reply_packet.Data.usrparam.PARAM.restart.d_overlap_vert = g_pShm_SysParam->TE_restart[0].d_overlap_vert;

        if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_LIN)
        {
            RM_reply_packet.Data.usrparam.PARAM.restart.path_speed     = g_pShm_SysParam->TE_restart[0].path_speed * 1000;
        }
        else if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_JNT)
        {
            RM_reply_packet.Data.usrparam.PARAM.restart.path_speed     = g_pShm_SysParam->TE_restart[0].path_speed * 100;
        }
        RM_reply_packet.Data.usrparam.PARAM.restart.hori_start_vol = g_pShm_SysParam->TE_restart[0].hori_start_vol;
        RM_reply_packet.Data.usrparam.PARAM.restart.hori_start_cur = g_pShm_SysParam->TE_restart[0].hori_start_cur;
        RM_reply_packet.Data.usrparam.PARAM.restart.hori_main_vol  = g_pShm_SysParam->TE_restart[0].hori_main_vol;
        RM_reply_packet.Data.usrparam.PARAM.restart.hori_main_cur  = g_pShm_SysParam->TE_restart[0].hori_main_cur;
        RM_reply_packet.Data.usrparam.PARAM.restart.vert_start_vol = g_pShm_SysParam->TE_restart[0].vert_start_vol;
        RM_reply_packet.Data.usrparam.PARAM.restart.vert_start_cur = g_pShm_SysParam->TE_restart[0].vert_start_cur;
        RM_reply_packet.Data.usrparam.PARAM.restart.vert_main_vol  = g_pShm_SysParam->TE_restart[0].vert_main_vol;
        RM_reply_packet.Data.usrparam.PARAM.restart.vert_main_cur  = g_pShm_SysParam->TE_restart[0].vert_main_cur;

        VERBOSE_MESSAGE("[Restart Param] -> moving_type, overlap_distance, path_speed\n"
                        "                   hori_st_vol, hori_st_cur, hori_main_vol, hori_main_cur\n"
                        "                   vert_st_vol, vert_st_cur, vert_main_vol, vert_main_cur\n");
        if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_LIN)
        {
            VERBOSE_MESSAGE("\v  Mov Type: %d, HorzOverLap: %.2lfmm, VertOverLap: %.2lfmm, PathSpeed: %.2lf[mm/s]\n",
                            g_pShm_SysParam->TE_restart[0].moving_type,
                            g_pShm_SysParam->TE_restart[0].d_overlap_horz,
                            g_pShm_SysParam->TE_restart[0].d_overlap_vert,
                            g_pShm_SysParam->TE_restart[0].path_speed * 1000);
        }
        else if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_JNT)
        {
            VERBOSE_MESSAGE("\v  Mov Type: %d, HorzOverLap: %.2lfmm, VertOverLap: %.2lfmm, PathSpeed: %.2lf[%%]\n",
                            g_pShm_SysParam->TE_restart[0].moving_type,
                            g_pShm_SysParam->TE_restart[0].d_overlap_horz,
                            g_pShm_SysParam->TE_restart[0].d_overlap_vert,
                            g_pShm_SysParam->TE_restart[0].path_speed * 100);
        }
        VERBOSE_MESSAGE("\v  Horizontal: %.2lf[V], %.2lf[A], %.2lf[V], %.2lf[A]\n"
                        "  Vertical  : %.2lf[V], %.2lf[A], %.2lf[V], %.2lf[A]\n",
                        g_pShm_SysParam->TE_restart[0].hori_start_vol,
                        g_pShm_SysParam->TE_restart[0].hori_start_cur,
                        g_pShm_SysParam->TE_restart[0].hori_main_vol,
                        g_pShm_SysParam->TE_restart[0].hori_main_cur,
                        g_pShm_SysParam->TE_restart[0].vert_start_vol,
                        g_pShm_SysParam->TE_restart[0].vert_start_cur,
                        g_pShm_SysParam->TE_restart[0].vert_main_vol,
                        g_pShm_SysParam->TE_restart[0].vert_main_cur);

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
    else if(nOption == 5)   //arcsensor parameters
    {
        RM_reply_packet.Data.usrparam.PARAM.arcsensor.fSaveArcSensorData =
                                g_pShm_SysParam->arcsensor[0].fSaveArcSensorData;
        RM_reply_packet.Data.usrparam.PARAM.arcsensor.nStartSaveNodeNo =
                                g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo;
        RM_reply_packet.Data.usrparam.PARAM.arcsensor.nSaveNodeCount =
                                g_pShm_SysParam->arcsensor[0].nSaveNodeCount;

        VERBOSE_MESSAGE("[Current ArcSensor Param]\n"
                        "\v  Save Data Flag    : %d\n"
                        "\v  Start Save Node No: %d\n"
                        "\v  Save Node Count   : %d\n",
                        g_pShm_SysParam->arcsensor[0].fSaveArcSensorData,
                        g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo,
                        g_pShm_SysParam->arcsensor[0].nSaveNodeCount);

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }
    else if(nOption == 6)   //cart offset
    {
        RM_reply_packet.Data.usrparam.PARAM.cart.x   = g_CartOffset.x;
        RM_reply_packet.Data.usrparam.PARAM.cart.y   = g_CartOffset.y;
        RM_reply_packet.Data.usrparam.PARAM.cart.z   = g_CartOffset.z;
        RM_reply_packet.Data.usrparam.PARAM.cart.rol = g_CartOffset.rol;
        RM_reply_packet.Data.usrparam.PARAM.cart.pit = g_CartOffset.pit;
        RM_reply_packet.Data.usrparam.PARAM.cart.yaw = g_CartOffset.yaw;

        VERBOSE_MESSAGE("Current CartOffset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                        g_CartOffset.x,
                        g_CartOffset.y,
                        g_CartOffset.z,
                        g_CartOffset.rol,
                        g_CartOffset.pit,
                        g_CartOffset.yaw);

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SendWeldTuneInputParameterValue()
//      - Service Name: RSVC_SERV_SHOWWELDTUNE_IN

int SVC_SendWeldTuneInputParameterValue(int nIdx)
{
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_a     = g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_a;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_b     = g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_b;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVoltScale  = g_pShm_SysParam->weld_tune[nIdx].input.dbVoltScale;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVoltOffset = g_pShm_SysParam->weld_tune[nIdx].input.dbVoltOffset;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_a     = g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_a;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_b     = g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_b;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurrScale  = g_pShm_SysParam->weld_tune[nIdx].input.dbCurrScale;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurrOffset = g_pShm_SysParam->weld_tune[nIdx].input.dbCurrOffset;

    VERBOSE_MESSAGE("[Current Weld Tune Input Param] -> a, b, scale, offset\n"
                    "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf\n"
                    "  Current: %.8lf, %.8lf, %.8lf, %.8lf\n",
                    g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_a,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_b,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbVoltScale,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbVoltOffset,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_a,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_b,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbCurrScale,
                    g_pShm_SysParam->weld_tune[nIdx].input.dbCurrOffset);

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SendWeldTuneOutputParameterValue()
//      - Service Name: RSVC_SERV_SHOWWELDTUNE_OUT

int SVC_SendWeldTuneOutputParameterValue(int nIdx)
{
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_a     = g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_a;
    RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_b     = g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_b;
    RM_reply_packet.Data.usrparam.PARAM.output.dbVolt_c     = g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_c;
    RM_reply_packet.Data.usrparam.PARAM.output.dbVoltScale  = g_pShm_SysParam->weld_tune[nIdx].output.dbVoltScale;
    RM_reply_packet.Data.usrparam.PARAM.output.dbVoltOffset = g_pShm_SysParam->weld_tune[nIdx].output.dbVoltOffset;
    RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_a     = g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_a;
    RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_b     = g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_b;
    RM_reply_packet.Data.usrparam.PARAM.output.dbCurr_c     = g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_c;
    RM_reply_packet.Data.usrparam.PARAM.output.dbCurrScale  = g_pShm_SysParam->weld_tune[nIdx].output.dbCurrScale;
    RM_reply_packet.Data.usrparam.PARAM.output.dbCurrOffset = g_pShm_SysParam->weld_tune[nIdx].output.dbCurrOffset;

    VERBOSE_MESSAGE("[Current Weld Tune Output Param] -> a, b, c, scale, offset\n"
                    "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n"
                    "  Current: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n",
                    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_a,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_b,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_c,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbVoltScale,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbVoltOffset,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_a,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_b,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_c,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbCurrScale,
                    g_pShm_SysParam->weld_tune[nIdx].output.dbCurrOffset);

    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ShowSkipCondition()
//      - Service Name: RSVC_SERV_SET_SKIPCOND

int SVC_ShowSkipCondition(void)
{
    VERBOSE_MESSAGE("Left Vert Skip: %d, Right Vert Skip: %d\n",
                    g_fLeftVertSkip, g_fRightVertSkip);
    VERBOSE_MESSAGE("Left Collar Skip: %d, Right Collar Skip: %d, Horizontal Skip: %d\n",
                    g_fLeftCollarSkip, g_fRightCollarSkip, g_fHorizontalSkip);

    RM_reply_packet.Data.skip_cond.fLeftVertSkip    = g_fLeftVertSkip;
    RM_reply_packet.Data.skip_cond.fRightVertSkip   = g_fRightVertSkip;
    RM_reply_packet.Data.skip_cond.fLeftCollarSkip  = g_fLeftCollarSkip;
    RM_reply_packet.Data.skip_cond.fRightCollarSkip = g_fRightCollarSkip;
    RM_reply_packet.Data.skip_cond.fHorizontalSkip  = g_fHorizontalSkip;

    RM_reply_packet.nCode  = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.skip_cond);

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: FUNC_GetBVarValue()
//      - Service Name: RSVC_SERV_SET_GAPSKIPCOND

int FUNC_GetBVarValue(int nVarIndex)
{
    SHM_DANDY_JOB*  pShmem = g_pShmemJobRM;
    BYTE*           pBVarStart;
    BYTE            byBVarVal;

    pBVarStart = GET_SHM_JOB_BVA_BUFFER(pShmem);

    byBVarVal = pBVarStart[nVarIndex];

    return (int) byBVarVal;
}


///////////////////////////////////////
//
//  Function: SVC_ShowGapSkipCondition()
//      - Service Name: RSVC_SERV_SET_GAPSKIPCOND

int SVC_ShowGapSkipCondition(void)
{
    g_nLeftVertGapSkipCond     = FUNC_GetBVarValue(g_nLeftVertSkipBvar);
    g_nRightVertGapSkipCond    = FUNC_GetBVarValue(g_nRightVertSkipBvar);
    g_nLeftCollarGapSkipCond   = FUNC_GetBVarValue(g_nLeftCollarSkipBvar);
    g_nRightCollarGapSkipCond  = FUNC_GetBVarValue(g_nRightCollarSkipBvar);
    g_nHorizontalGapSkipCond   = FUNC_GetBVarValue(g_nHorizontalSkipBvar);
    g_nLeftBracketGapSkipCond  = FUNC_GetBVarValue(g_nLeftBracketSkipBvar);
    g_nRightBracketGapSkipCond = FUNC_GetBVarValue(g_nRightBracketSkipBvar);

    VERBOSE_MESSAGE("\v[Gap/Skip]\tLeft Vert: %d, Right Vert: %d\n",
                    g_nLeftVertGapSkipCond, g_nRightVertGapSkipCond);
    VERBOSE_MESSAGE("\v\tLeft Collar: %d, Right Collar: %d, Horizontal: %d\n",
                    g_nLeftCollarGapSkipCond, g_nRightCollarGapSkipCond, g_nHorizontalGapSkipCond);
    VERBOSE_MESSAGE("\v\tLeft Bracket: %d, Right Bracket: %d\n",
                    g_nLeftBracketGapSkipCond, g_nRightBracketGapSkipCond);

    RM_reply_packet.Data.gapskip_cond.nLeftVertGapSkipCond     = g_nLeftVertGapSkipCond;   
    RM_reply_packet.Data.gapskip_cond.nRightVertGapSkipCond    = g_nRightVertGapSkipCond;  
    RM_reply_packet.Data.gapskip_cond.nLeftCollarGapSkipCond   = g_nLeftCollarGapSkipCond; 
    RM_reply_packet.Data.gapskip_cond.nRightCollarGapSkipCond  = g_nRightCollarGapSkipCond;
    RM_reply_packet.Data.gapskip_cond.nHorizontalGapSkipCond   = g_nHorizontalGapSkipCond; 
    RM_reply_packet.Data.gapskip_cond.nLeftBracketGapSkipCond  = g_nLeftBracketGapSkipCond;
    RM_reply_packet.Data.gapskip_cond.nRightBracketGapSkipCond = g_nRightBracketGapSkipCond;

    RM_reply_packet.nCode  = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.gapskip_cond);

    return RESULT_OK;
}
