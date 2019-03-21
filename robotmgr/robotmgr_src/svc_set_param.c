/////////////////////////////////////////////////////////////////////////////
//
//  param_set_svc.c: Set Parameter Service
//                                            2014.06.13  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

int g_fLeftVertSkip     = 0;   
int g_fRightVertSkip    = 0;  
int g_fLeftCollarSkip   = 0; 
int g_fRightCollarSkip  = 0;
int g_fHorizontalSkip   = 0;

int g_nLeftVertGapSkipCond     = GAPSKIP_COND_GAP_NONE;
int g_nRightVertGapSkipCond    = GAPSKIP_COND_GAP_NONE;
int g_nLeftCollarGapSkipCond   = GAPSKIP_COND_GAP_NONE;
int g_nRightCollarGapSkipCond  = GAPSKIP_COND_GAP_NONE;
int g_nHorizontalGapSkipCond   = GAPSKIP_COND_GAP_NONE;
int g_nLeftBracketGapSkipCond  = GAPSKIP_COND_GAP_NONE;
int g_nRightBracketGapSkipCond = GAPSKIP_COND_GAP_NONE;

int  g_nSkipBvarIdx  = -1;
BYTE g_fSkip     = 0;
#if 0
int  g_nGapSkipBvarIdx  = -1;
int  g_nGapSkipCond  = GAPSKIP_COND_GAP_NONE;
#endif

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: SVC_SetHomePositionValue()
//      - Service Name: RSVC_SERV_HOME_EDIT

int SVC_SetHomePositionValue(int nHomeIndex)
{
    int iAxis;
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }

    // display current shared memory data and requested data
    VERBOSE_MESSAGE("Current HomePos[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nHomeIndex,
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][0] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][1] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][2] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][3] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][4] * (180/M_PI),
                    g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][5] * (180/M_PI));

    VERBOSE_MESSAGE("Modified HomePos[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nHomeIndex,
                    RM_packet.Data.home_dat.rgdbHomePosVal[0],
                    RM_packet.Data.home_dat.rgdbHomePosVal[1],
                    RM_packet.Data.home_dat.rgdbHomePosVal[2],
                    RM_packet.Data.home_dat.rgdbHomePosVal[3],
                    RM_packet.Data.home_dat.rgdbHomePosVal[4],
                    RM_packet.Data.home_dat.rgdbHomePosVal[5]);

    // put the requested data to shared memory
    // unit: shared memory is radian, requested packet is degree
    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][iAxis] =
                                  RM_packet.Data.home_dat.rgdbHomePosVal[iAxis] * (M_PI/180);
    }

    // set reply message data modifed home index
    RM_reply_packet.Data.home_dat.nHomeIndex = nHomeIndex;
    
    // set reply message data modifed home position data
    for(iAxis = 0; iAxis < ROB_AXIS_COUNT; iAxis++)
    {
        RM_reply_packet.Data.home_dat.rgdbHomePosVal[iAxis] =
                g_pShm_SysConfig->robot->rgdbHomePosVal[nHomeIndex][iAxis] * (180/M_PI);
    }

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Home Postion File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.home_dat);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetUserCoordinateValue()
//      - Service Name: RSVC_SERV_USERCRD_EDIT

int SVC_SetUserCoordinateValue(int nUserCoordIndex)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }

    // display current shared memory data and requested data
    VERBOSE_MESSAGE("Current User Coord[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nUserCoordIndex,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].x,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].y,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].z,
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].rol * (180/M_PI),
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].pit * (180/M_PI),
                    g_pShm_SysConfig->robot->user[nUserCoordIndex].yaw * (180/M_PI));
    
    VERBOSE_MESSAGE("Modified User Coord[%d]: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    nUserCoordIndex,
                    RM_packet.Data.usrcrd_dat.usercoord.x,
                    RM_packet.Data.usrcrd_dat.usercoord.y,
                    RM_packet.Data.usrcrd_dat.usercoord.z,
                    RM_packet.Data.usrcrd_dat.usercoord.rol,
                    RM_packet.Data.usrcrd_dat.usercoord.pit,
                    RM_packet.Data.usrcrd_dat.usercoord.yaw);

    // put the requested data to shared memory
    // unit: shared memory is radian, requested packet is degree
    g_pShm_SysConfig->robot->user[nUserCoordIndex].x =
                              RM_packet.Data.usrcrd_dat.usercoord.x;
    g_pShm_SysConfig->robot->user[nUserCoordIndex].y =
                              RM_packet.Data.usrcrd_dat.usercoord.y;
    g_pShm_SysConfig->robot->user[nUserCoordIndex].z =
                              RM_packet.Data.usrcrd_dat.usercoord.z;
    g_pShm_SysConfig->robot->user[nUserCoordIndex].rol =
                              RM_packet.Data.usrcrd_dat.usercoord.rol * (M_PI/180);
    g_pShm_SysConfig->robot->user[nUserCoordIndex].pit =
                              RM_packet.Data.usrcrd_dat.usercoord.pit * (M_PI/180);
    g_pShm_SysConfig->robot->user[nUserCoordIndex].yaw =
                              RM_packet.Data.usrcrd_dat.usercoord.yaw * (M_PI/180);

    // set reply message data modifed user coordinate data
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
    
    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("User Coordinate File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrcrd_dat);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetToolCenterdPointValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT

int SVC_SetToolCenterdPointValue(void)
{
    int nRet = RESULT_OK;
    
    // if RM shared memory not loaded, return error
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }

    // display current shared memory data and requested data
    VERBOSE_MESSAGE("Current TCP: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    g_pShm_SysConfig->robot->coordTool.x,
                    g_pShm_SysConfig->robot->coordTool.y,
                    g_pShm_SysConfig->robot->coordTool.z,
                    g_pShm_SysConfig->robot->coordTool.rol * (180/M_PI),
                    g_pShm_SysConfig->robot->coordTool.pit * (180/M_PI),
                    g_pShm_SysConfig->robot->coordTool.yaw * (180/M_PI));
    
    VERBOSE_MESSAGE("Modified TCP: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    RM_packet.Data.usrparam.PARAM.TCP.x,
                    RM_packet.Data.usrparam.PARAM.TCP.y,
                    RM_packet.Data.usrparam.PARAM.TCP.z,
                    RM_packet.Data.usrparam.PARAM.TCP.rol,
                    RM_packet.Data.usrparam.PARAM.TCP.pit,
                    RM_packet.Data.usrparam.PARAM.TCP.yaw);

    // put the requested data to shared memory
    // unit: shared memory is radian, requested packet is degree
    g_pShm_SysConfig->robot->coordTool.x =
                              RM_packet.Data.usrparam.PARAM.TCP.x;
    g_pShm_SysConfig->robot->coordTool.y =
                              RM_packet.Data.usrparam.PARAM.TCP.y;
    g_pShm_SysConfig->robot->coordTool.z =
                              RM_packet.Data.usrparam.PARAM.TCP.z;
    g_pShm_SysConfig->robot->coordTool.rol =
                              RM_packet.Data.usrparam.PARAM.TCP.rol * (M_PI/180);
    g_pShm_SysConfig->robot->coordTool.pit =
                              RM_packet.Data.usrparam.PARAM.TCP.pit * (M_PI/180);
    g_pShm_SysConfig->robot->coordTool.yaw =
                              RM_packet.Data.usrparam.PARAM.TCP.yaw * (M_PI/180);

    // set reply message data modifed TCP data
    RM_reply_packet.Data.usrparam.PARAM.TCP.x   = g_pShm_SysConfig->robot->coordTool.x;
    RM_reply_packet.Data.usrparam.PARAM.TCP.y   = g_pShm_SysConfig->robot->coordTool.y;
    RM_reply_packet.Data.usrparam.PARAM.TCP.z   = g_pShm_SysConfig->robot->coordTool.z;
    RM_reply_packet.Data.usrparam.PARAM.TCP.rol = g_pShm_SysConfig->robot->coordTool.rol * (180/M_PI);
    RM_reply_packet.Data.usrparam.PARAM.TCP.pit = g_pShm_SysConfig->robot->coordTool.pit * (180/M_PI);
    RM_reply_packet.Data.usrparam.PARAM.TCP.yaw = g_pShm_SysConfig->robot->coordTool.yaw * (180/M_PI);
    
    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("TCP File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetWordCoordinateOffsetValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT

int SVC_SetWordCoordinateOffsetValue(void)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }
    
    // display current shared memory data and requested data
    VERBOSE_MESSAGE("Current World Offset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    g_pShm_SysConfig->robot->world.x,
                    g_pShm_SysConfig->robot->world.y,
                    g_pShm_SysConfig->robot->world.z,
                    g_pShm_SysConfig->robot->world.rol * (180/M_PI),
                    g_pShm_SysConfig->robot->world.pit * (180/M_PI),
                    g_pShm_SysConfig->robot->world.yaw * (180/M_PI));
    
    VERBOSE_MESSAGE("Modified World Offset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    RM_packet.Data.usrparam.PARAM.world.x,
                    RM_packet.Data.usrparam.PARAM.world.y,
                    RM_packet.Data.usrparam.PARAM.world.z,
                    RM_packet.Data.usrparam.PARAM.world.rol,
                    RM_packet.Data.usrparam.PARAM.world.pit,
                    RM_packet.Data.usrparam.PARAM.world.yaw);

    // put the requested data to shared memory
    // unit: shared memory is radian, requested packet is degree
    g_pShm_SysConfig->robot->world.x =
                              RM_packet.Data.usrparam.PARAM.world.x;
    g_pShm_SysConfig->robot->world.y =
                              RM_packet.Data.usrparam.PARAM.world.y;
    g_pShm_SysConfig->robot->world.z =
                              RM_packet.Data.usrparam.PARAM.world.z;
    g_pShm_SysConfig->robot->world.rol =
                              RM_packet.Data.usrparam.PARAM.world.rol * (M_PI/180);
    g_pShm_SysConfig->robot->world.pit =
                              RM_packet.Data.usrparam.PARAM.world.pit * (M_PI/180);
    g_pShm_SysConfig->robot->world.yaw =
                              RM_packet.Data.usrparam.PARAM.world.yaw * (M_PI/180);

    // set reply message data modifed world offset data
    RM_reply_packet.Data.usrparam.PARAM.world.x   = g_pShm_SysConfig->robot->world.x;
    RM_reply_packet.Data.usrparam.PARAM.world.y   = g_pShm_SysConfig->robot->world.y;
    RM_reply_packet.Data.usrparam.PARAM.world.z   = g_pShm_SysConfig->robot->world.z;
    RM_reply_packet.Data.usrparam.PARAM.world.rol = g_pShm_SysConfig->robot->world.rol * (180/M_PI);
    RM_reply_packet.Data.usrparam.PARAM.world.pit = g_pShm_SysConfig->robot->world.pit * (180/M_PI);
    RM_reply_packet.Data.usrparam.PARAM.world.yaw = g_pShm_SysConfig->robot->world.yaw * (180/M_PI);

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("World Offset File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetCartOffsetValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT

int SVC_SetCartOffsetValue(void)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysConfig == NULL)
    {
        return RESULT_ERROR;
    }
    
    // display current shared memory data and requested data
    VERBOSE_MESSAGE("Current Cart Offset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    g_CartOffset.x,
                    g_CartOffset.y,
                    g_CartOffset.z,
                    g_CartOffset.rol,
                    g_CartOffset.pit,
                    g_CartOffset.yaw);
    
    VERBOSE_MESSAGE("Modified Cart Offset: %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf\n",
                    RM_packet.Data.usrparam.PARAM.cart.x,
                    RM_packet.Data.usrparam.PARAM.cart.y,
                    RM_packet.Data.usrparam.PARAM.cart.z,
                    RM_packet.Data.usrparam.PARAM.cart.rol,
                    RM_packet.Data.usrparam.PARAM.cart.pit,
                    RM_packet.Data.usrparam.PARAM.cart.yaw);

    // put the requested data to shared memory
    // unit: shared memory is radian, requested packet is degree
    g_CartOffset.x = RM_packet.Data.usrparam.PARAM.cart.x;
    g_CartOffset.y = RM_packet.Data.usrparam.PARAM.cart.y;
    g_CartOffset.z = RM_packet.Data.usrparam.PARAM.cart.z;
    g_CartOffset.rol = RM_packet.Data.usrparam.PARAM.cart.rol;
    g_CartOffset.pit = RM_packet.Data.usrparam.PARAM.cart.pit;
    g_CartOffset.yaw = RM_packet.Data.usrparam.PARAM.cart.yaw;

    // set reply message data modifed world offset data
    RM_reply_packet.Data.usrparam.PARAM.cart.x   = g_CartOffset.x;
    RM_reply_packet.Data.usrparam.PARAM.cart.y   = g_CartOffset.y;
    RM_reply_packet.Data.usrparam.PARAM.cart.z   = g_CartOffset.z;
    RM_reply_packet.Data.usrparam.PARAM.cart.rol = g_CartOffset.rol;
    RM_reply_packet.Data.usrparam.PARAM.cart.pit = g_CartOffset.pit;
    RM_reply_packet.Data.usrparam.PARAM.cart.yaw = g_CartOffset.yaw;

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Cart Offset File Save Done!\n");
    }
#if 0
    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();
#endif
    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetWeldTuneInputValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT
//                      RSVC_SERV_WELDTUNE_IN_EDIT

int SVC_SetWeldTuneInputValue(int nIdx)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }
    
    // display current shared memory data and requested data
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

    VERBOSE_MESSAGE("[Modified Weld Tune Input Param] -> a, b, scale, offset\n"
                    "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf\n"
                    "  Current: %.8lf, %.8lf, %.8lf, %.8lf\n",
                    RM_packet.Data.usrparam.PARAM.input.dbVolt_a,
                    RM_packet.Data.usrparam.PARAM.input.dbVolt_b,
                    RM_packet.Data.usrparam.PARAM.input.dbVoltScale,
                    RM_packet.Data.usrparam.PARAM.input.dbVoltOffset,
                    RM_packet.Data.usrparam.PARAM.input.dbCurr_a,
                    RM_packet.Data.usrparam.PARAM.input.dbCurr_b,
                    RM_packet.Data.usrparam.PARAM.input.dbCurrScale,
                    RM_packet.Data.usrparam.PARAM.input.dbCurrOffset);

    // put the requested data to shared memory
    g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_a     = RM_packet.Data.usrparam.PARAM.input.dbVolt_a;
    g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_b     = RM_packet.Data.usrparam.PARAM.input.dbVolt_b;
    g_pShm_SysParam->weld_tune[nIdx].input.dbVoltScale  = RM_packet.Data.usrparam.PARAM.input.dbVoltScale;
    g_pShm_SysParam->weld_tune[nIdx].input.dbVoltOffset = RM_packet.Data.usrparam.PARAM.input.dbVoltOffset;
    g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_a     = RM_packet.Data.usrparam.PARAM.input.dbCurr_a;
    g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_b     = RM_packet.Data.usrparam.PARAM.input.dbCurr_b;
    g_pShm_SysParam->weld_tune[nIdx].input.dbCurrScale  = RM_packet.Data.usrparam.PARAM.input.dbCurrScale;
    g_pShm_SysParam->weld_tune[nIdx].input.dbCurrOffset = RM_packet.Data.usrparam.PARAM.input.dbCurrOffset;

    // set reply message data modifed weld tune input data
    RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_a     = g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_a;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVolt_b     = g_pShm_SysParam->weld_tune[nIdx].input.dbVolt_b;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVoltScale  = g_pShm_SysParam->weld_tune[nIdx].input.dbVoltScale;
    RM_reply_packet.Data.usrparam.PARAM.input.dbVoltOffset = g_pShm_SysParam->weld_tune[nIdx].input.dbVoltOffset;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_a     = g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_a;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurr_b     = g_pShm_SysParam->weld_tune[nIdx].input.dbCurr_b;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurrScale  = g_pShm_SysParam->weld_tune[nIdx].input.dbCurrScale;
    RM_reply_packet.Data.usrparam.PARAM.input.dbCurrOffset = g_pShm_SysParam->weld_tune[nIdx].input.dbCurrOffset;
    
    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Weld Tune Input Param File Save Done!(Idx: %d)\n", nIdx);
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetWeldTuneOutputValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT
//                      RSVC_SERV_WELDTUNE_OUT_EDIT

int SVC_SetWeldTuneOutputValue(int nIdx)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    // Check Parameters Validation (Volt)
    nRet = FUNC_CheckTuneOutputValueValidation(TUNE_PARAM_VOLT,
                                               RM_packet.Data.usrparam.PARAM.output.dbVolt_a,
                                               RM_packet.Data.usrparam.PARAM.output.dbVolt_b,
                                               RM_packet.Data.usrparam.PARAM.output.dbVolt_c);
    
    if(nRet != RESULT_OK)
    {
        VERBOSE_ERROR("Volt Coefficient Value is Invalid!\n");

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);
        
        return nRet;
    }

    // Check Parameters Validation (Current)
    nRet = FUNC_CheckTuneOutputValueValidation(TUNE_PARAM_CURR,
                                               RM_packet.Data.usrparam.PARAM.output.dbCurr_a,
                                               RM_packet.Data.usrparam.PARAM.output.dbCurr_b,
                                               RM_packet.Data.usrparam.PARAM.output.dbCurr_c);

    if(nRet != RESULT_OK)
    {
        VERBOSE_ERROR("Curr Coefficient Value is Invalid!\n");

        RM_reply_packet.nCode = RM_packet.nCode;
        RM_reply_packet.nValue = RM_packet.nValue;
        RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

        return nRet;
    }

    // if validation check done,
    // display current shared memory data and requested data
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

    VERBOSE_MESSAGE("[Modified Weld Tune Output Param] -> a, b, c, scale, offset\n"
                    "  Voltage: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n"
                    "  Current: %.8lf, %.8lf, %.8lf, %.8lf, %.8lf\n",
                    RM_packet.Data.usrparam.PARAM.output.dbVolt_a,
                    RM_packet.Data.usrparam.PARAM.output.dbVolt_b,
                    RM_packet.Data.usrparam.PARAM.output.dbVolt_c,
                    RM_packet.Data.usrparam.PARAM.output.dbVoltScale,
                    RM_packet.Data.usrparam.PARAM.output.dbVoltOffset,
                    RM_packet.Data.usrparam.PARAM.output.dbCurr_a,
                    RM_packet.Data.usrparam.PARAM.output.dbCurr_b,
                    RM_packet.Data.usrparam.PARAM.output.dbCurr_c,
                    RM_packet.Data.usrparam.PARAM.output.dbCurrScale,
                    RM_packet.Data.usrparam.PARAM.output.dbCurrOffset);
    
    // put the requested data to shared memory
    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_a     = RM_packet.Data.usrparam.PARAM.output.dbVolt_a;
    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_b     = RM_packet.Data.usrparam.PARAM.output.dbVolt_b;
    g_pShm_SysParam->weld_tune[nIdx].output.dbVolt_c     = RM_packet.Data.usrparam.PARAM.output.dbVolt_c;
    g_pShm_SysParam->weld_tune[nIdx].output.dbVoltScale  = RM_packet.Data.usrparam.PARAM.output.dbVoltScale;
    g_pShm_SysParam->weld_tune[nIdx].output.dbVoltOffset = RM_packet.Data.usrparam.PARAM.output.dbVoltOffset;
    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_a     = RM_packet.Data.usrparam.PARAM.output.dbCurr_a;
    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_b     = RM_packet.Data.usrparam.PARAM.output.dbCurr_b;
    g_pShm_SysParam->weld_tune[nIdx].output.dbCurr_c     = RM_packet.Data.usrparam.PARAM.output.dbCurr_c;
    g_pShm_SysParam->weld_tune[nIdx].output.dbCurrScale  = RM_packet.Data.usrparam.PARAM.output.dbCurrScale;
    g_pShm_SysParam->weld_tune[nIdx].output.dbCurrOffset = RM_packet.Data.usrparam.PARAM.output.dbCurrOffset;
    
    // set reply message data modifed weld tune output data
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

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Weld Tune Input Param File Save Done!(Idx: %d)\n", nIdx);
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetRestartParameterValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT

int SVC_SetRestartParameterValue(void)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    // display current shared memory data and requested data
    /* show current shared memory value */
    VERBOSE_MESSAGE("[Current Restart Param] -> moving_type, horz_overlap_distance, vert_overlap_distance, path_speed\n"
                    "                           hori_st_vol, hori_st_cur, hori_main_vol, hori_main_cur\n"
                    "                           vert_st_vol, vert_st_cur, vert_main_vol, vert_main_cur\n");
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
    
    /* show requested value */
    VERBOSE_MESSAGE("[Modified Restart Param] -> moving_type, horz_overlap_distance, vert_overlap_distance, path_speed\n"
                    "                            hori_st_vol, hori_st_cur, hori_main_vol, hori_main_cur\n"
                    "                            vert_st_vol, vert_st_cur, vert_main_vol, vert_main_cur\n");
    if(RM_packet.Data.usrparam.PARAM.restart.moving_type == RESTART_MOVE_TYPE_LIN)
    {
        VERBOSE_MESSAGE("\v  Mov Type: %d, HorzOverLap: %.2lfmm, VertOverLap: %.2lfmm, PathSpeed: %.2lf[mm/s]\n",
                        RM_packet.Data.usrparam.PARAM.restart.moving_type,
                        RM_packet.Data.usrparam.PARAM.restart.d_overlap_horz,
                        RM_packet.Data.usrparam.PARAM.restart.d_overlap_vert,
                        RM_packet.Data.usrparam.PARAM.restart.path_speed);
    }
    else if(RM_packet.Data.usrparam.PARAM.restart.moving_type == RESTART_MOVE_TYPE_JNT)
    {
        VERBOSE_MESSAGE("\v  Mov Type: %d, HorzOverLap: %.2lfmm, VertOverLap: %.2lfmm, PathSpeed: %.2lf[%%]\n",
                        RM_packet.Data.usrparam.PARAM.restart.moving_type,
                        RM_packet.Data.usrparam.PARAM.restart.d_overlap_horz,
                        RM_packet.Data.usrparam.PARAM.restart.d_overlap_vert,
                        RM_packet.Data.usrparam.PARAM.restart.path_speed);
    }
    VERBOSE_MESSAGE("\v  Horizontal: %.2lf[V], %.2lf[A], %.2lf[V], %.2lf[A]\n"
                    "  Vertical  : %.2lf[V], %.2lf[A], %.2lf[V], %.2lf[A]\n",
                    RM_packet.Data.usrparam.PARAM.restart.hori_start_vol,
                    RM_packet.Data.usrparam.PARAM.restart.hori_start_cur,
                    RM_packet.Data.usrparam.PARAM.restart.hori_main_vol,
                    RM_packet.Data.usrparam.PARAM.restart.hori_main_cur,
                    RM_packet.Data.usrparam.PARAM.restart.vert_start_vol,
                    RM_packet.Data.usrparam.PARAM.restart.vert_start_cur,
                    RM_packet.Data.usrparam.PARAM.restart.vert_main_vol,
                    RM_packet.Data.usrparam.PARAM.restart.vert_main_cur);

    // put the requested data to shared memory
    g_pShm_SysParam->TE_restart[0].moving_type        = RM_packet.Data.usrparam.PARAM.restart.moving_type;
    g_pShm_SysParam->TE_restart[0].d_overlap_horz     = RM_packet.Data.usrparam.PARAM.restart.d_overlap_horz;
    g_pShm_SysParam->TE_restart[0].d_overlap_vert     = RM_packet.Data.usrparam.PARAM.restart.d_overlap_vert;

    if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_LIN)
    {
        g_pShm_SysParam->TE_restart[0].path_speed     = RM_packet.Data.usrparam.PARAM.restart.path_speed * 0.001;
    }
    else if(g_pShm_SysParam->TE_restart[0].moving_type == RESTART_MOVE_TYPE_JNT)
    {
        g_pShm_SysParam->TE_restart[0].path_speed     = RM_packet.Data.usrparam.PARAM.restart.path_speed * 0.01;
    }
    g_pShm_SysParam->TE_restart[0].hori_start_vol =  RM_packet.Data.usrparam.PARAM.restart.hori_start_vol;
    g_pShm_SysParam->TE_restart[0].hori_start_cur =  RM_packet.Data.usrparam.PARAM.restart.hori_start_cur;
    g_pShm_SysParam->TE_restart[0].hori_main_vol  =  RM_packet.Data.usrparam.PARAM.restart.hori_main_vol;
    g_pShm_SysParam->TE_restart[0].hori_main_cur  =  RM_packet.Data.usrparam.PARAM.restart.hori_main_cur;
    g_pShm_SysParam->TE_restart[0].vert_start_vol =  RM_packet.Data.usrparam.PARAM.restart.vert_start_vol;
    g_pShm_SysParam->TE_restart[0].vert_start_cur =  RM_packet.Data.usrparam.PARAM.restart.vert_start_cur;
    g_pShm_SysParam->TE_restart[0].vert_main_vol  =  RM_packet.Data.usrparam.PARAM.restart.vert_main_vol;
    g_pShm_SysParam->TE_restart[0].vert_main_cur  =  RM_packet.Data.usrparam.PARAM.restart.vert_main_cur;
        
    // set reply message data to modifed restart data
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

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Weld Tune Input Param File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();
    SYSC_LoadParamToTE_RestartShmem();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetArcSensorParamValue()
//      - Service Name: RSVC_SERV_PARAM_EDIT

int SVC_SetArcSensorParamValue(void)
{
    int nRet = RESULT_OK;

    // if RM shared memory not loaded, return error
    if(g_pShm_SysParam == NULL)
    {
        return RESULT_ERROR;
    }

    // display current shared memory data and requested data
    VERBOSE_MESSAGE("[Current ArcSensor Param]\n"
                    "\v  Save Data Flag    : %d\n"
                    "\v  Start Save Node No: %d\n"
                    "\v  Save Node Count   : %d\n",
                    g_pShm_SysParam->arcsensor[0].fSaveArcSensorData,
                    g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo,
                    g_pShm_SysParam->arcsensor[0].nSaveNodeCount);

    VERBOSE_MESSAGE("[Modified ArcSensor Param]\n"
                    "\v  Save Data Flag    : %d\n"
                    "\v  Start Save Node No: %d\n"
                    "\v  Save Node Count   : %d\n",
                    RM_packet.Data.usrparam.PARAM.arcsensor.fSaveArcSensorData,
                    RM_packet.Data.usrparam.PARAM.arcsensor.nStartSaveNodeNo,
                    RM_packet.Data.usrparam.PARAM.arcsensor.nSaveNodeCount);
    
    // put the requested data to shared memory
    g_pShm_SysParam->arcsensor[0].fSaveArcSensorData =
                        RM_packet.Data.usrparam.PARAM.arcsensor.fSaveArcSensorData;
    g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo =
                        RM_packet.Data.usrparam.PARAM.arcsensor.nStartSaveNodeNo;
    g_pShm_SysParam->arcsensor[0].nSaveNodeCount =
                        RM_packet.Data.usrparam.PARAM.arcsensor.nSaveNodeCount;
    
    // set reply message data to modifed arc sensor data
    RM_reply_packet.Data.usrparam.PARAM.arcsensor.fSaveArcSensorData =
                        g_pShm_SysParam->arcsensor[0].fSaveArcSensorData;
    RM_reply_packet.Data.usrparam.PARAM.arcsensor.nStartSaveNodeNo =
                        g_pShm_SysParam->arcsensor[0].nStartSaveNodeNo;
    RM_reply_packet.Data.usrparam.PARAM.arcsensor.nSaveNodeCount =
                        g_pShm_SysParam->arcsensor[0].nSaveNodeCount;
    
    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("ArcSensor Param File Save Done!\n");
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.usrparam.PARAM);

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetWeldMeasureParamValue()
//      - Service Name: RSVC_SRV_WELD_MEASURE_WRITE

int SVC_SetWeldMeasureParamValue(int nOpt)
{
    int nRet = RESULT_OK;
    int iIdx;

    if(nOpt == 0)   // show service
    {
        for(iIdx = 0; iIdx < MEASURE_WRITE_COUNT; iIdx++)
        {
            // display current shared memory data and requested data
            VERBOSE_MESSAGE("\v[Current Weld Measure Value (Idx: %d)]\n"
                            "  [Volt] Cmd: %.2lf, Measure: %.2lf\t"
                            "  [Curr] Cmd: %.2lf, Measure: %.2lf\n",
                            iIdx,
                            g_dbControllerCmdVolt[iIdx],
                            g_dbWelderMeasureVolt[iIdx],
                            g_dbControllerCmdCurr[iIdx],
                            g_dbWelderMeasureCurr[iIdx]);

            // set reply message data to modifed weld measured value data
            RM_reply_packet.Data.weld_measure.dbControllerCmdVolt[iIdx] = g_dbControllerCmdVolt[iIdx];
            RM_reply_packet.Data.weld_measure.dbWelderMeasureVolt[iIdx] = g_dbWelderMeasureVolt[iIdx];
            RM_reply_packet.Data.weld_measure.dbControllerCmdCurr[iIdx] = g_dbControllerCmdCurr[iIdx];
            RM_reply_packet.Data.weld_measure.dbWelderMeasureCurr[iIdx] = g_dbWelderMeasureCurr[iIdx];
        }
    }
    else if(nOpt == 1)  // set service
    {
        for(iIdx = 0; iIdx < MEASURE_WRITE_COUNT; iIdx++)
        {
            // display current shared memory data and requested data
            VERBOSE_VERBOSE("\v[Current Weld Measure Value (Idx: %d)]\n"
                            "  [Volt] Cmd: %.2lf, Measure: %.2lf\t"
                            "  [Curr] Cmd: %.2lf, Measure: %.2lf\n",
                            iIdx,
                            g_dbControllerCmdVolt[iIdx],
                            g_dbWelderMeasureVolt[iIdx],
                            g_dbControllerCmdCurr[iIdx],
                            g_dbWelderMeasureCurr[iIdx]);

            VERBOSE_MESSAGE("\v[Modified Weld Measure Value (Idx: %d)]\n"
                            "  [Volt] Cmd: %.2lf, Measure: %.2lf\t"
                            "  [Curr] Cmd: %.2lf, Measure: %.2lf\n",
                            iIdx,
                            RM_packet.Data.weld_measure.dbControllerCmdVolt[iIdx],
                            RM_packet.Data.weld_measure.dbWelderMeasureVolt[iIdx],
                            RM_packet.Data.weld_measure.dbControllerCmdCurr[iIdx],
                            RM_packet.Data.weld_measure.dbWelderMeasureCurr[iIdx]);
            
            // put the requested data to global variables
            g_dbControllerCmdVolt[iIdx] = RM_packet.Data.weld_measure.dbControllerCmdVolt[iIdx];
            g_dbWelderMeasureVolt[iIdx] = RM_packet.Data.weld_measure.dbWelderMeasureVolt[iIdx];
            g_dbControllerCmdCurr[iIdx] = RM_packet.Data.weld_measure.dbControllerCmdCurr[iIdx];
            g_dbWelderMeasureCurr[iIdx] = RM_packet.Data.weld_measure.dbWelderMeasureCurr[iIdx];

            // set reply message data to modifed weld measured value data
            RM_reply_packet.Data.weld_measure.dbControllerCmdVolt[iIdx] = g_dbControllerCmdVolt[iIdx];
            RM_reply_packet.Data.weld_measure.dbWelderMeasureVolt[iIdx] = g_dbWelderMeasureVolt[iIdx];
            RM_reply_packet.Data.weld_measure.dbControllerCmdCurr[iIdx] = g_dbControllerCmdCurr[iIdx];
            RM_reply_packet.Data.weld_measure.dbWelderMeasureCurr[iIdx] = g_dbWelderMeasureCurr[iIdx];
        }
        
        // save the data to user param config file
        nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
        if(nRet == RESULT_OK)
        {
            VERBOSE_MESSAGE("ArcSensor Param File Save Done!\n");
        }

        DANDY_SLEEP(100);
    }

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.weld_measure);

    return nRet;
}


///////////////////////////////////////
//
//  Function: _loc_SendWeldCoeffientABC()
//      - Service Name: RSVC_SRV_CAL_WELDCOEFF

static void _loc_SendWeldCoeffientABC(double dbVolt_a,
                                      double dbVolt_b,
                                      double dbVolt_c,
                                      double dbCurr_a,
                                      double dbCurr_b,
                                      double dbCurr_c)
{
    // set reply message data to calculated volt parameter data
    RM_reply_packet.Data.cal_weldcoeff.dbVolt_a = dbVolt_a;
    RM_reply_packet.Data.cal_weldcoeff.dbVolt_b = dbVolt_b;
    RM_reply_packet.Data.cal_weldcoeff.dbVolt_c = dbVolt_c;
    RM_reply_packet.Data.cal_weldcoeff.dbVoltScale  = DEF_WELD_SCALE;
    RM_reply_packet.Data.cal_weldcoeff.dbVoltOffset = DEF_WELD_OFFSET;

    // set reply message data to calculated current parameter data
    RM_reply_packet.Data.cal_weldcoeff.dbCurr_a = dbCurr_a;
    RM_reply_packet.Data.cal_weldcoeff.dbCurr_b = dbCurr_b;
    RM_reply_packet.Data.cal_weldcoeff.dbCurr_c = dbCurr_c;
    RM_reply_packet.Data.cal_weldcoeff.dbCurrScale  = DEF_WELD_SCALE;
    RM_reply_packet.Data.cal_weldcoeff.dbCurrOffset = DEF_WELD_OFFSET;
   
    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.cal_weldcoeff);
}


///////////////////////////////////////
//
//  Function: SVC_CalculateAndSaveWeldCoefficient()
//      - Service Name: RSVC_SRV_CAL_WELDCOEFF

int SVC_CalculateAndSaveWeldCoefficient(int nIdx)
{
    int     nRet_Volt = RESULT_OK,
            nRet_Curr = RESULT_OK;
    double  dbWelder_Coeff[WELD_COEFF_COUNT] = {0,};
    double  dbVoltWelder_Coeff[WELD_COEFF_COUNT] = {0,},
            dbCurrWelder_Coeff[WELD_COEFF_COUNT] = {0,};

    /* Volt Parameter Calculate */
    // Get A, B, C Parameters (Volt) From Measured Value & Check basic validation
    nRet_Volt = FUNC_GetWelderCoeff(dbWelder_Coeff, 0, VOLT_OUT);

    memcpy(dbVoltWelder_Coeff, dbWelder_Coeff, sizeof(double)*WELD_COEFF_COUNT);

    if(nRet_Volt != RESULT_OK)
    {
        // if parameters are invalid or calculation failed,
        // return the calculated value & error code
        _loc_SendWeldCoeffientABC(dbVoltWelder_Coeff[0],
                                  dbVoltWelder_Coeff[1],
                                  dbVoltWelder_Coeff[2],
                                  dbCurrWelder_Coeff[0],
                                  dbCurrWelder_Coeff[1],
                                  dbCurrWelder_Coeff[2]);

        return nRet_Volt;
    }

    // Check Parameters Validation (Volt)
    nRet_Volt = FUNC_CheckTuneOutputValueValidation(TUNE_PARAM_VOLT,
                                                    dbWelder_Coeff[0],
                                                    dbWelder_Coeff[1],
                                                    dbWelder_Coeff[2]);
    if(nRet_Volt != RESULT_OK)
    {
        // if parameters are invalid,
        // return the calculated value & error code
        _loc_SendWeldCoeffientABC(dbVoltWelder_Coeff[0],
                                  dbVoltWelder_Coeff[1],
                                  dbVoltWelder_Coeff[2],
                                  dbCurrWelder_Coeff[0],
                                  dbCurrWelder_Coeff[1],
                                  dbCurrWelder_Coeff[2]);

        VERBOSE_ERROR("Volt Coefficient Value is Invalid!\n");
        return nRet_Volt;
    }
    
    /* Current Parameter Calculate */
    // Get A, B, C Parameters (Current) From Measured Value & Check basic validation
    nRet_Curr = FUNC_GetWelderCoeff(dbWelder_Coeff, 0, CURRENT_OUT);

    memcpy(dbCurrWelder_Coeff, dbWelder_Coeff, sizeof(double)*WELD_COEFF_COUNT);

    if(nRet_Curr != RESULT_OK)
    {
        // if parameters are invalid or calculation failed,
        // return the calculated value & error code
        _loc_SendWeldCoeffientABC(dbVoltWelder_Coeff[0],
                                  dbVoltWelder_Coeff[1],
                                  dbVoltWelder_Coeff[2],
                                  dbCurrWelder_Coeff[0],
                                  dbCurrWelder_Coeff[1],
                                  dbCurrWelder_Coeff[2]);

        return nRet_Curr;
    }

    // Check Parameters Validation (Current)
    nRet_Curr = FUNC_CheckTuneOutputValueValidation(TUNE_PARAM_CURR,
                                                    dbWelder_Coeff[0],
                                                    dbWelder_Coeff[1],
                                                    dbWelder_Coeff[2]);
    if(nRet_Curr != RESULT_OK)
    {
        // if parameters are invalid,
        // return the calculated value & error code
        _loc_SendWeldCoeffientABC(dbVoltWelder_Coeff[0],
                                  dbVoltWelder_Coeff[1],
                                  dbVoltWelder_Coeff[2],
                                  dbCurrWelder_Coeff[0],
                                  dbCurrWelder_Coeff[1],
                                  dbCurrWelder_Coeff[2]);

        VERBOSE_ERROR("Curr Coefficient Value is Invalid!\n");
        return nRet_Curr;
    }

    // volt & current calculation are successful, index is just indication
    VERBOSE_MESSAGE("Weld Tune Output Param Calculated Done!(Idx: %d)\n", nIdx);

    _loc_SendWeldCoeffientABC(dbVoltWelder_Coeff[0],
                              dbVoltWelder_Coeff[1],
                              dbVoltWelder_Coeff[2],
                              dbCurrWelder_Coeff[0],
                              dbCurrWelder_Coeff[1],
                              dbCurrWelder_Coeff[2]);
    
    if(nRet_Volt == RESULT_OK && nRet_Curr == RESULT_OK)
    {
        return RESULT_OK;
    }
    else
    {
        return RESULT_ERROR;
    }
}


///////////////////////////////////////
//
//  Function: SVC_ApplyWeldTuneInputParameterValue()
//      - Service Name: RSVC_SERV_SHOWWELDTUNE_IN

int SVC_ApplyWeldTuneInputParameterValue(int nIdx)
{
    int nRet = RESULT_OK;

    // put the requested data to shared memory
    g_pShm_SysParam->nWeldTuneInParamApplyIndex = nIdx;

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Weld Tune In Param File Save Done!(Active Idx: %d)\n",
                        nIdx);
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_ApplyWeldTuneOutputParameterValue()
//      - Service Name: RSVC_SERV_SHOWWELDTUNE_OUT

int SVC_ApplyWeldTuneOutputParameterValue(int nIdx)
{
    int nRet = RESULT_OK;

    // put the requested data to shared memory
    g_pShm_SysParam->nWeldTuneOutParamApplyIndex = nIdx;

    // save the data to user param config file
    nRet = SVC_SaveUserParamConfigToFile(ROBOT_0_INDEX);
    if(nRet == RESULT_OK)
    {
        VERBOSE_MESSAGE("Weld Tune Out Param File Save Done!(Active Idx: %d)\n",
                        nIdx);
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    
    return nRet;
}


///////////////////////////////////////
//
//  Function: SVC_SetGapCondition() -- not applied
//      - Service Name: RSVC_SERV_SET_GAPCOND

/* ---------------------------------------------------------- */
/* traditional method for setting the skip and gap condition  */
/* using B variables(B10, B20) ------------------------------ */
/* set shift value shared memory, TE act weld condition shift */
/* ---------------------------------------------------------- */
int SVC_SetGapCondition(void)
{
    // if RM shared memory not loaded, return error
    if(g_pShm_SysStatus == NULL)
    {
        return RESULT_ERROR;
    }

#if 0   // for no use B10, B20 to discriminate left, right welding job
    g_pShm_SysStatus->fLeftSkip =  RM_packet.Data.gap_cond.fLeftSkip;
    g_pShm_SysStatus->fRightSkip = RM_packet.Data.gap_cond.fRightSkip;
#endif

    // Horizontal Gap
    if(RM_packet.Data.gap_cond.nHorzGapCond == GAP_COND_NONE)
    {
        g_pShm_SysStatus->nHorzGapShift = 0;
    }
    else if(RM_packet.Data.gap_cond.nHorzGapCond == GAP_COND_SMALL)
    {
        g_pShm_SysStatus->nHorzGapShift = 10;
    }
    
    // Vertical Left Gap
    if(RM_packet.Data.gap_cond.nVertLeftGapCond == GAP_COND_NONE)
    {
        g_pShm_SysStatus->nVertLeftGapShift = 0;
    }
    else if(RM_packet.Data.gap_cond.nVertLeftGapCond == GAP_COND_SMALL)
    {
        g_pShm_SysStatus->nVertLeftGapShift = 10;
    }
    else if(RM_packet.Data.gap_cond.nVertLeftGapCond == GAP_COND_BIG)
    {
        g_pShm_SysStatus->nVertLeftGapShift = 20;
    }

    // Vertical Right Gap
    if(RM_packet.Data.gap_cond.nVertRightGapCond == GAP_COND_NONE)
    {
        g_pShm_SysStatus->nVertRightGapShift = 0;
    }
    else if(RM_packet.Data.gap_cond.nVertRightGapCond == GAP_COND_SMALL)
    {
        g_pShm_SysStatus->nVertRightGapShift = 10;
    }
    else if(RM_packet.Data.gap_cond.nVertRightGapCond == GAP_COND_BIG)
    {
        g_pShm_SysStatus->nVertRightGapShift = 20;
    }

    // apply modifed contents to sibling process
    DANDY_SLEEP(100);
    SVC_ApplyParamToSibling();

    // display skip and gap condition
    VERBOSE_MESSAGE("Left Skip: %d, Right Skip: %d\n",
                    g_pShm_SysStatus->fLeftSkip, g_pShm_SysStatus->fRightSkip);

    VERBOSE_MESSAGE("Horizontal Gap: %d, Vertical Left Gap: %d, Vertical Right Gap: %d\n",
                    RM_packet.Data.gap_cond.nHorzGapCond,
                    RM_packet.Data.gap_cond.nVertLeftGapCond,
                    RM_packet.Data.gap_cond.nVertRightGapCond);

    VERBOSE_MESSAGE("Horizontal Shift: %d, Vertical Left Shift: %d, Vertical Right Shift: %d\n",
                    g_pShm_SysStatus->nHorzGapShift,
                    g_pShm_SysStatus->nVertLeftGapShift,
                    g_pShm_SysStatus->nVertRightGapShift);

    // define reply packet
    RM_reply_packet.nCode = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;
    RM_reply_packet.nDataSize = sizeof(RM_reply_packet.Data.gap_cond);

    return 0;
}


///////////////////////////////////////
//
//  Function: SVC_SetSkipCondition() -- not applied
//      - Service Name: RSVC_SERV_SET_SKIPCOND

int SVC_SetSkipCondition(int nOption)
{
    // set skip variable
    /* skip on (left vertical, right vertical, left collar, right collar, horizontal) */
    if(nOption == LEFT_VERT_SKIP_ON)
    {
        g_fLeftVertSkip = ON;
        g_fSkip = g_fLeftVertSkip;
        g_nSkipBvarIdx = g_nLeftVertSkipBvar;
    }
    else if(nOption == RIGHT_VERT_SKIP_ON)
    {
        g_fRightVertSkip = ON;
        g_fSkip = g_fRightVertSkip;
        g_nSkipBvarIdx = g_nRightVertSkipBvar;
    }
    else if(nOption == LEFT_COLLAR_SKIP_ON)
    {
        g_fLeftCollarSkip = ON;
        g_fSkip = g_fLeftCollarSkip;
        g_nSkipBvarIdx = g_nLeftCollarSkipBvar;
    }
    else if(nOption == RIGHT_COLLAR_SKIP_ON)
    {
        g_fRightCollarSkip = ON;
        g_fSkip = g_fRightCollarSkip;
        g_nSkipBvarIdx = g_nRightCollarSkipBvar;
    }
    else if(nOption == HORIZONTAL_SKIP_ON)
    {
        g_fHorizontalSkip = ON;
        g_fSkip = g_fHorizontalSkip;
        g_nSkipBvarIdx = g_nHorizontalSkipBvar;
    }

    /* skip off (left vertical, right vertical, left collar, right collar, horizontal) */
    if(nOption == LEFT_VERT_SKIP_OFF)
    {
        g_fLeftVertSkip = OFF;
        g_fSkip = g_fLeftVertSkip;
        g_nSkipBvarIdx = g_nLeftVertSkipBvar;
    }
    else if(nOption == RIGHT_VERT_SKIP_OFF)
    {
        g_fRightVertSkip = OFF;
        g_fSkip = g_fRightVertSkip;
        g_nSkipBvarIdx = g_nRightVertSkipBvar;
    }
    else if(nOption == LEFT_COLLAR_SKIP_OFF)
    {
        g_fLeftCollarSkip = OFF;
        g_fSkip = g_fLeftCollarSkip;
        g_nSkipBvarIdx = g_nLeftCollarSkipBvar;
    }
    else if(nOption == RIGHT_COLLAR_SKIP_OFF)
    {
        g_fRightCollarSkip = OFF;
        g_fSkip = g_fRightCollarSkip;
        g_nSkipBvarIdx = g_nRightCollarSkipBvar;
    }
    else if(nOption == HORIZONTAL_SKIP_OFF)
    {
        g_fHorizontalSkip = OFF;
        g_fSkip = g_fHorizontalSkip;
        g_nSkipBvarIdx = g_nHorizontalSkipBvar;
    }
    
    // display skip condition
    VERBOSE_MESSAGE("Left Vert Skip: %d, Right Vert Skip: %d\n",
                    g_fLeftVertSkip, g_fRightVertSkip);
    VERBOSE_MESSAGE("Left Collar Skip: %d, Right Collar Skip: %d, Horizontal Skip: %d\n",
                    g_fLeftCollarSkip, g_fRightCollarSkip, g_fHorizontalSkip);

    // define reply packet
    RM_reply_packet.nCode  = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SetGapSkipCondition()
//      - Service Name: RSVC_SERV_SET_GAPSKIPCOND

/* ------------------------------------------------------------- */
/* Final applied service, skip and gap condition set at one time */
/* ------------------------------------------------------------- */

int SVC_SetGapSkipCondition(void)
{
    // set gap and skip condition at one time by requested message
    // every cases are using one variable each (4 case: skip, gap none, middle, big)
    // : left vertical, right vertical, left collar, right collar, horizontal

    g_nLeftVertGapSkipCond     = RM_packet.Data.gapskip_cond.nLeftVertGapSkipCond;
    g_nRightVertGapSkipCond    = RM_packet.Data.gapskip_cond.nRightVertGapSkipCond;
    g_nLeftCollarGapSkipCond   = RM_packet.Data.gapskip_cond.nLeftCollarGapSkipCond;
    g_nRightCollarGapSkipCond  = RM_packet.Data.gapskip_cond.nRightCollarGapSkipCond;
    g_nHorizontalGapSkipCond   = RM_packet.Data.gapskip_cond.nHorizontalGapSkipCond;
    g_nLeftBracketGapSkipCond  = RM_packet.Data.gapskip_cond.nLeftBracketGapSkipCond;
    g_nRightBracketGapSkipCond = RM_packet.Data.gapskip_cond.nRightBracketGapSkipCond;
    
    // display skip and gap condition
    VERBOSE_MESSAGE("\v[Gap/Skip]\tLeft Vert: %d, Right Vert: %d\n",
                    g_nLeftVertGapSkipCond, g_nRightVertGapSkipCond);
    VERBOSE_MESSAGE("\v\tLeft Collar: %d, Right Collar: %d, Horizontal: %d\n",
                    g_nLeftCollarGapSkipCond, g_nRightCollarGapSkipCond, g_nHorizontalGapSkipCond);
    VERBOSE_MESSAGE("\v\tLeft Bracket: %d, Right Bracket: %d\n",
                    g_nLeftBracketGapSkipCond, g_nRightBracketGapSkipCond);

    // define reply packet
    RM_reply_packet.nCode  = RM_packet.nCode;
    RM_reply_packet.nValue = RM_packet.nValue;

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_ApplyParamToSibling()
//
int SVC_ApplyParamToSibling(void)
{
    // apply to TE process
    MSG_SendPulse(g_coidTE, TESERV_INIT, 0);

    // apply to SC process
    MSG_SendPulse(g_coidSC, SC_SHM_PARAM_RELOAD, 0);

    return RESULT_OK;
}
