/////////////////////////////////////////////////////////////////////////////
//
//  conf_default.c: Parameter Config Default Information Load & Management
//                                            2013.07.10  Ryu SinWook

#include <math.h>

///////////////////////////////////////

#include "robotmgr_main.h"

///////////////////////////////////////

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(__var)       (__var = __var)
#endif

/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfigDefault()
//

int SYSC_LoadConfigDefault(void)
{
    SYSC_ClearConfig();

    VERBOSE_VERBOSE("Try to set default configuration\n");

    SYSC_ArrangeConfigGlobal();
    SYSC_ArrangeConfigRobot();
    SYSC_ArrangeConfigAxis();
    SYSC_ArrangeConfigMotor();
    SYSC_ArrangeConfigWelder();
    //SYSC_ArrangeConfigSensor();

    // all configuration is good, because all values are cleared
    DANDY_ASSERT(SYSC_CheckConfigGlobal() != -1);
    DANDY_ASSERT(SYSC_CheckConfigRobot() != -1);
    DANDY_ASSERT(SYSC_CheckConfigAxis() != -1);
    DANDY_ASSERT(SYSC_CheckConfigMotor() != -1);
    DANDY_ASSERT(SYSC_CheckConfigWelder() != -1);
    //DANDY_ASSERT(SYSC_CheckConfigSensor() != -1);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_NONE()
//

static void _loc_SYSC_ResetRobotInfo_NONE(int nRobot, int nStartAxis)
{
    // unused robot

    UNREFERENCED_PARAMETER(nStartAxis);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 0;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "unused");
    g_rgnRobotType[nRobot] = ROBTYPE_NONE;

        // masters
    g_rgnRobotAxisCount[nRobot] = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_VOID()
//

static void _loc_SYSC_ResetRobotInfo_VOID(int nRobot, int nStartAxis)
{
    // VOID robot, defined as a robot, but no joints and no axes

    UNREFERENCED_PARAMETER(nStartAxis);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 1;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "VOID");
    g_rgnRobotType[nRobot] = ROBTYPE_EXCEPT;

        // masters
    g_rgnRobotAxisCount[nRobot] = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_DR6()
//       -  Doosan welding robot 'DR6'

static void _loc_SYSC_ResetRobotInfo_DR6(int nRobot, int nStartAxis)
{
    // reduction gear ratio
    static const double rgdbGearRatio[6] = { 161, 161,  161, 8262./99., 200./3., 50};

    // software limit (limit angle is degree)
    // axis 1, 2, 4, 5, 6 is absolute range values
    // axis 3 is conditinal range about axis 2
    static const double rgdbMinAngle[6] = {-170, -90, -65, -180, -132, -380};
    static const double rgdbMaxAngle[6] = { 170, 150,  60,  180,  132,  380};

    // maximum velocity and acceleration
    // 3780 rpm (original DR6)
    //static const double rgdbMaxVel[6]   = { 141, 141,  141,  273,  342,  456};
    //static const double rgdbMaxAccel[6] = { 587, 587,  587, 1132, 1417, 1890};
    // 3000 rpm (modified DR6)
    static const double rgdbMaxVel[6]    = { 111.8, 111.8, 111.8, 215.6,  270.0,  360.0};
    static const double rgdbMaxAccel[6]  = { 465.8, 465.8, 465.8, 898.6, 1125.0, 1500.0};

    // encoder resolution for one motor turn
    //static const int    rgnEncoderRes[6] = { -8192,  8192, -8192, 8192, -8192, -8192};
    static const int    rgnEncoderRes[6] =    { 8192, 8192, 8192, 8192, 8192, 8192};
    static const int    rgnAxisDirection[6] = { REV,  FOR,  REV,  FOR,  REV,  REV};
    // D-H parameter
    static const double rgdbParam_al[6]  = { 90.0,   0.0,  90.0, -90.0, 90.0,   0.0};
    static const double rgdbParam_l[6]   = {150.0, 450.0, 115.0,   0.0,  0.0,   0.0};
    static const double rgdbParam_th[6]  = { 90.0,  90.0,   0.0,   0.0,  0.0,   0.0};
    static const double rgdbParam_d[6]   = {  0.0,   0.0,   0.0, 735.0,  0.0, 108.5};

    int iJoint, iAxis;
    double dbPI;

    dbPI = acos((double) -1);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 1;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "DR6");
    g_rgnRobotType[nRobot] = ROBTYPE_DR6;

    // comm
    g_rgnCommProtocol[nRobot] = ROBOT_COMM_RS485;

#if defined(_WIN32)
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "com5");
#else
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "/dev/ser5");
#endif

    // masters
    g_rgnRobotAxisCount[nRobot] = 6;

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotAxis[nRobot][iJoint].nMotorCount = nStartAxis + iJoint;
        //g_rgRobotAxis[nRobot][iJoint].nSlaveCount = 0;
    }

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotDHParam[nRobot][iJoint].l  = rgdbParam_l[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].al = rgdbParam_al[iJoint] * dbPI / 180.;
        g_rgRobotDHParam[nRobot][iJoint].d  = rgdbParam_d[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].th = rgdbParam_th[iJoint] * dbPI / 180.;
    }

    // maximum speed
    g_rgRobotMotion[nRobot].dbMaxLinearSpeed = 1000;
    g_rgRobotMotion[nRobot].dbMaxOrientSpeed = 180. * dbPI / 180.;

    // maximum accel
    //g_rgRobotMotion[nRobot].dbMaxLinearAccel = 5000;
    //g_rgRobotMotion[nRobot].dbMaxOrientAccel = 900. * dbPI / 180.;

    ///////////////////////////////////
    // AXIS configuration

    for (iAxis = nStartAxis, iJoint = 0; iJoint < 6; iAxis++, iJoint++)
    {
        CRT_sprintf(g_rgszAxisName[iAxis], AXIS_NAME_LEN, "Joint %d", iJoint);

        g_rgnAxisType[iAxis] = MOTTYPE_MINAS;
        g_rgnAxisIndex[iAxis] = iAxis + 1;

        g_rgfSwLimitUsed[iAxis][0] = 1;
        g_rgfSwLimitUsed[iAxis][1] = 1;

        g_rgdbSwLimit[iAxis][0] = rgdbMinAngle[iJoint] * dbPI / 180.;
        g_rgdbSwLimit[iAxis][1] = rgdbMaxAngle[iJoint] * dbPI / 180.;

        g_rgdbGearRatio[iAxis] = rgdbGearRatio[iJoint];
        g_rgdbRotaionDist[iAxis] = 2. * dbPI;
        g_rgnEncoderRes[iAxis] = rgnEncoderRes[iJoint];
        g_rgnAxisDirection[iAxis] = rgnAxisDirection[iJoint];

        g_rgdbMotorMaxVel[iAxis]   = rgdbMaxVel[iJoint] * dbPI / 180.;
        g_rgdbMotorMaxAccel[iAxis] = rgdbMaxAccel[iJoint] * dbPI / 180.;
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_BLAST()
//       -  DSME blasting working robot 'BLAST'

static void _loc_SYSC_ResetRobotInfo_BLAST(int nRobot, int nStartAxis)
{
    // 1, 2 axes are linear motion (ball screw) = mm unit
    // 3, 4, 5 axes are rotation motion = deg unit (should be convert to radian)

    // reduction gear ratio
    static const double rgdbGearRatio[5] = {   1,       1,    51,    81,    50};

    // travel distance for ball screw one turn
    static const double rgdbRotDist[5]   = {  20,      10,   360,   360,   360}; 

    // software limit (axis 1, 2 axes are [mm].  axis 3, 4, 5 is degree)
    static const double rgdbMinPos[5]    = {-300,    -125,  -180,  -135,  -180};
    static const double rgdbMaxPos[5]    = { 300,     125,   180,   135,   180};
    static const double rgdbMaxVel[5]    = {1000,     500,   352,   222,   360};
    static const double rgdbMaxAccel[5]  = {4000,    2000,  1764,  1111,  1800};

    // encoder resolution for one motor turn
    //static const int    rgnEncoderRes[5] = { -8192,  8192, -8192, -8192, -8192};
    static const int    rgnEncoderRes[5] =    { 8192, 8192, 8192, 8192, 8192};
    static const int    rgnAxisDirection[5] = { REV,  FOR,  REV,  REV,  REV};

    // D-H parameter
    // all D-H parameter values will be ignored

    int iJoint, iAxis;
    double dbPI;

    dbPI = acos((double) -1);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 1;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "Blast");
    g_rgnRobotType[nRobot] = ROBTYPE_EXCEPT;

    // comm
    g_rgnCommProtocol[nRobot] = ROBOT_COMM_RS485;

#if defined(_WIN32)
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "com5");
#else
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "/dev/ser5");
#endif

    // masters
    g_rgnRobotAxisCount[nRobot] = 5;

    for (iJoint = 0; iJoint < 5; iJoint++)
    {
        g_rgRobotAxis[nRobot][iJoint].nMotorCount = nStartAxis + iJoint;
        //g_rgRobotAxis[nRobot][iJoint].nSlaveCount = 0;
    }

    for (iJoint = 0; iJoint < 5; iJoint++)
    {
        g_rgRobotDHParam[nRobot][iJoint].l  = 0;
        g_rgRobotDHParam[nRobot][iJoint].al = 0;
        g_rgRobotDHParam[nRobot][iJoint].d  = 0;
        g_rgRobotDHParam[nRobot][iJoint].th = 0;
    }

    // maximum speed
    g_rgRobotMotion[nRobot].dbMaxLinearSpeed = 500;
    g_rgRobotMotion[nRobot].dbMaxOrientSpeed = 222. * dbPI / 180.;

    // maximum accel
    //g_rgRobotMotion[nRobot].dbMaxLinearAccel = 2000;
    //g_rgRobotMotion[nRobot].dbMaxOrientAccel = 1111. * dbPI / 180.;

    ///////////////////////////////////
    // AXIS configuration

    for (iAxis = nStartAxis, iJoint = 0; iJoint < 5; iAxis++, iJoint++)
    {
        CRT_sprintf(g_rgszAxisName[iAxis], AXIS_NAME_LEN, "Joint %d", iJoint + 1);
        g_rgnAxisType[iAxis] = MOTTYPE_MINAS;
        g_rgnAxisIndex[iAxis] = iAxis + 1;

        g_rgfSwLimitUsed[iAxis][0] = 1;
        g_rgfSwLimitUsed[iAxis][1] = 1;

        g_rgdbGearRatio[iAxis] = rgdbGearRatio[iJoint];
        g_rgnEncoderRes[iAxis] = rgnEncoderRes[iJoint];
        g_rgnAxisDirection[iAxis] = rgnAxisDirection[iJoint];

        if (iJoint < 2)
        {
            // linear axis (ball screw)
            g_rgdbSwLimit[iAxis][0] = rgdbMinPos[iJoint];
            g_rgdbSwLimit[iAxis][1] = rgdbMaxPos[iJoint];

            g_rgdbRotaionDist[iAxis] = rgdbRotDist[iJoint];

            g_rgdbMotorMaxVel[iAxis]   = rgdbMaxVel[iJoint];
            g_rgdbMotorMaxAccel[iAxis] = rgdbMaxAccel[iJoint];
        }
        else
        {
            // rotation axis
            g_rgdbSwLimit[iAxis][0] = rgdbMinPos[iJoint] * dbPI / 180.;
            g_rgdbSwLimit[iAxis][1] = rgdbMaxPos[iJoint] * dbPI / 180.;

            g_rgdbRotaionDist[iAxis] = rgdbRotDist[iJoint] * dbPI / 180.;

            g_rgdbMotorMaxVel[iAxis]   = rgdbMaxVel[iJoint] * dbPI / 180.;
            g_rgdbMotorMaxAccel[iAxis] = rgdbMaxAccel[iJoint] * dbPI / 180.;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_10KG()
//       - Doosan welding robot '10KG' (DR6 compatible type)

static void _loc_SYSC_ResetRobotInfo_10KG(int nRobot, int nStartAxis)
{
    // reduction gear ratio
    static const double rgdbGearRatio[6] = { 120, 120, 120, 81.6, 81.0, 50};

    // software limit (limit angle is degree)
    // axis 1, 2, 4, 5, 6 is absolute range values
    // axis 3 is conditinal range about axis 2
    static const double rgdbMinAngle[6] = {-160, -90, -63, -180, -135, -360};
    static const double rgdbMaxAngle[6] = { 160, 150,  65,  180,  135,  360};

    // maximum velocity and acceleration
    static const double rgdbMaxVel[6]   = { 150.0, 150.0, 150.0, 220.8, 222.2,  360.0};
    static const double rgdbMaxAccel[6] = { 600.0, 600.0, 600.0, 882.3, 888.8, 1440.0};

    // encoder resolution for one motor turn
    //static const int    rgnEncoderRes[6] = { -8192, 8192, -8192, 8192,  8192, -8192};
    static const int    rgnEncoderRes[6] =    { 8192, 8192, 8192, 8192, 8192, 8192};
    static const int    rgnAxisDirection[6] = { REV,  FOR,  REV,  FOR,  FOR,  REV};

    // D-H parameter
    static const double rgdbParam_al[6]  = { 90.0,   0.0,  90.0, -90.0, 90.0,   0.0};
    static const double rgdbParam_l[6]   = {200.0, 500.0, 115.0,   0.0,  0.0,   0.0};
    static const double rgdbParam_th[6]  = { 90.0,  90.0,   0.0,   0.0,  0.0,   0.0};
    static const double rgdbParam_d[6]   = {  0.0,   0.0,   0.0, 800.0,  0.0, 110.0};

    int iJoint, iAxis;
    double dbPI;

    dbPI = acos((double) -1);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 1;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "10KG");
    g_rgnRobotType[nRobot] = ROBTYPE_DR6;    // DR6 Compatible

    // comm
    g_rgnCommProtocol[nRobot] = ROBOT_COMM_RS485;

#if defined(_WIN32)
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "com5");
#else
    CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "/dev/ser5");
#endif

    // masters
    g_rgnRobotAxisCount[nRobot] = 6;

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotAxis[nRobot][iJoint].nMotorCount = nStartAxis + iJoint;
        //g_rgRobotAxis[nRobot][iJoint].nSlaveCount = 0;
    }

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotDHParam[nRobot][iJoint].l  = rgdbParam_l[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].al = rgdbParam_al[iJoint] * dbPI / 180.;
        g_rgRobotDHParam[nRobot][iJoint].d  = rgdbParam_d[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].th = rgdbParam_th[iJoint] * dbPI / 180.;
    }

    // maximum speed
    g_rgRobotMotion[nRobot].dbMaxLinearSpeed = 1000;
    g_rgRobotMotion[nRobot].dbMaxOrientSpeed = 180. * dbPI / 180.;

    // maximum accel
    //g_rgRobotMotion[nRobot].dbMaxLinearAccel = 5000;
    //g_rgRobotMotion[nRobot].dbMaxOrientAccel = 900. * dbPI / 180.;

    // job program buffer
    g_rgdwCmdSize[nRobot] = DEF_CMD_COUNT;
    g_rgdwTVarSize[nRobot] = DEF_EMB_POS_COUNT;
    g_rgdwPVarSize[nRobot] = DEF_GLB_POS_COUNT;
    g_rgdwBVarSize[nRobot] = DEF_VAR_COUNT;
    g_rgdwIVarSize[nRobot] = DEF_VAR_COUNT;
    g_rgdwRVarSize[nRobot] = DEF_VAR_COUNT;

    ///////////////////////////////////
    // AXIS configuration

    for (iAxis = nStartAxis, iJoint = 0; iJoint < 6; iAxis++, iJoint++)
    {
        CRT_sprintf(g_rgszAxisName[iAxis], AXIS_NAME_LEN, "Joint %d", iJoint);

        g_rgnAxisType[iAxis] = MOTTYPE_MINAS;
        g_rgnAxisIndex[iAxis] = iAxis + 1;

        g_rgfSwLimitUsed[iAxis][0] = 1;
        g_rgfSwLimitUsed[iAxis][1] = 1;

        g_rgdbSwLimit[iAxis][0] = rgdbMinAngle[iJoint] * dbPI / 180.;
        g_rgdbSwLimit[iAxis][1] = rgdbMaxAngle[iJoint] * dbPI / 180.;

        g_rgdbGearRatio[iAxis] = rgdbGearRatio[iJoint];
        g_rgdbRotaionDist[iAxis] = 2. * dbPI;
        g_rgnEncoderRes[iAxis] = rgnEncoderRes[iJoint];
        g_rgnAxisDirection[iAxis] = rgnAxisDirection[iJoint];

        g_rgdbMotorMaxVel[iAxis]   = rgdbMaxVel[iJoint] * dbPI / 180.;
        g_rgdbMotorMaxAccel[iAxis] = rgdbMaxAccel[iJoint] * dbPI / 180.;
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetRobotInfo_DANDY_II()
//       - DANDY-II robot Prototype 'DANDY-II'

static void _loc_SYSC_ResetRobotInfo_DANDY_II(int nRobot, int nStartAxis)
{
    // reduction gear ratio
    static const double rgdbGearRatio[6] = { 141, 141, 160, 100, 180, 100};

    // software limit (limit angle is degree)
    // all axes are absolute range values, Unit: deg
    static const double rgdbSwMinAngle[6] = {-151.997, -89.986, -71.995, -228.985, -113.993, -358.989};
    static const double rgdbSwMaxAngle[6] = { 151.997, 111.987, 106.029,  228.985,  113.993,  358.989};

    // hardware limit (limit angle is degree)
    // all axes are absolute range values, Unit: deg
    static const double rgdbHwMinAngle[6] = {-152.997, -90.986, -72.995, -229.985, -114.993, -359.989};
    static const double rgdbHwMaxAngle[6] = { 152.997, 112.987, 107.029,  229.985,  114.993,  359.989};

    // maximum velocity and acceleration, Unit: deg/ms, ms
    static const double rgdbMaxVel[6]   = { 0.1276596, 0.1276596, 0.1125, 0.18, 0.1, 0.18};
    static const double rgdbMaxAccel[6] = {       400,       400,    400,  400, 300,  300};

    // encoder resolution for one motor turn
    static const int    rgnEncoderRes[6] =    { 1048576, 1048576, 1048576, 1048576, 1048576, 1048576};
    static const int    rgnAxisDirection[6] = { FOR,  REV,  FOR,  REV,  FOR,  REV};

    // D-H parameter
    static const double rgdbParam_al[6]  = {-90,   0, -90,  90, -90, 0};
    static const double rgdbParam_l[6]   = { 60, 350,  90,   0,   0, 0};
    static const double rgdbParam_th[6]  = {  0, -90,   0,   0,   0, 0};
    static const double rgdbParam_d[6]   = {400,   0,   0, 390,   0, 0};

    int iJoint, iAxis;
    double dbPI;

    dbPI = acos((double) -1);

    ///////////////////////////////////
    // ROBOT configuration

    g_rgfRobotUsed[nRobot] = 1;
    CRT_strcpy(g_rgszRobotName[nRobot], ROBOT_NAME_LEN, "DANDY-II");
    g_rgnRobotType[nRobot] = ROBTYPE_DANDY_II;

    // comm
    g_rgnCommProtocol[nRobot] = ROBOT_COMM_NONE;

#if defined(_WIN32)
    //CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "com5");
#else
    //CRT_strcpy(g_rgszCommPort[nRobot], ROBOT_COMM_PORT_LEN, "/dev/ser5");
#endif

    // masters
    g_rgnRobotAxisCount[nRobot] = 6;

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotAxis[nRobot][iJoint].nMotorCount = nStartAxis + iJoint;
    }

    for (iJoint = 0; iJoint < 6; iJoint++)
    {
        g_rgRobotDHParam[nRobot][iJoint].l  = rgdbParam_l[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].al = rgdbParam_al[iJoint] * dbPI / 180.;
        g_rgRobotDHParam[nRobot][iJoint].d  = rgdbParam_d[iJoint];
        g_rgRobotDHParam[nRobot][iJoint].th = rgdbParam_th[iJoint] * dbPI / 180.;
    }

    // maximum speed
    g_rgRobotMotion[nRobot].dbMaxLinearSpeed = 0.5;         //Unit: mm/ms
    g_rgRobotMotion[nRobot].dbMaxOrientSpeed = 0.00628;     //Unit: rad/ms

    // job program buffer
    g_rgdwCmdSize[nRobot]  = DEF_CMD_COUNT;
    g_rgdwTVarSize[nRobot] = DEF_EMB_POS_COUNT;
    g_rgdwPVarSize[nRobot] = DEF_GLB_POS_COUNT;
    g_rgdwBVarSize[nRobot] = DEF_VAR_COUNT;
    g_rgdwIVarSize[nRobot] = DEF_VAR_COUNT;
    g_rgdwRVarSize[nRobot] = DEF_VAR_COUNT;

    ///////////////////////////////////
    // AXIS configuration

    for (iAxis = nStartAxis, iJoint = 0; iJoint < 6; iAxis++, iJoint++)
    {
        CRT_sprintf(g_rgszAxisName[iAxis], AXIS_NAME_LEN, "Joint %d", iJoint);

        g_rgnAxisType[iAxis] = MOTTYPE_SIGMA_V;
        g_rgnAxisIndex[iAxis] = iAxis + 1;

        g_rgfSwLimitUsed[iAxis][0] = 1;
        g_rgfSwLimitUsed[iAxis][1] = 1;

        g_rgdbSwLimit[iAxis][0] = rgdbSwMinAngle[iJoint] * dbPI / 180.;
        g_rgdbSwLimit[iAxis][1] = rgdbSwMaxAngle[iJoint] * dbPI / 180.;

        g_rgfHwLimitUsed[iAxis][0] = 1;
        g_rgfHwLimitUsed[iAxis][1] = 1;

        g_rgdbHwLimit[iAxis][0] = rgdbHwMinAngle[iJoint] * dbPI / 180.;
        g_rgdbHwLimit[iAxis][1] = rgdbHwMaxAngle[iJoint] * dbPI / 180.;

        g_rgdbGearRatio[iAxis] = rgdbGearRatio[iJoint];
        g_rgdbRotaionDist[iAxis] = 2. * dbPI;
        g_rgnEncoderRes[iAxis] = rgnEncoderRes[iJoint];
        g_rgnAxisDirection[iAxis] = rgnAxisDirection[iJoint];

        g_rgdbMotorMaxVel[iAxis]   = rgdbMaxVel[iJoint] * dbPI / 180.;
        g_rgdbMotorMaxAccel[iAxis] = rgdbMaxAccel[iJoint];
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ResetRobotInfo()
//

int SYSC_ResetRobotInfo(int nCount,
                        const char* const rgpszRobotType[],
                        const int rgnStartAxis[])
{
    int iRobot;

    // clear all robot information
    SYSC_ClearConfigRobot();
    SYSC_ClearConfigAxis();
    SYSC_ClearConfigMotor();

    // create new robot

    for (iRobot = 0; iRobot < nCount; iRobot++)
    {
        ///////////////////////////////////
        // robot configuration

        if (stricmp(rgpszRobotType[iRobot], "none") == 0 ||
            stricmp(rgpszRobotType[iRobot], "unused") == 0 ||
            stricmp(rgpszRobotType[iRobot], "null") == 0)
        {
            _loc_SYSC_ResetRobotInfo_NONE(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with 'NONE' type\n");
        }
        else if (stricmp(rgpszRobotType[iRobot], "VOID") == 0)
        {
            _loc_SYSC_ResetRobotInfo_VOID(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with 'VOID' type\n");
        }
        else if (stricmp(rgpszRobotType[iRobot], "DR6") == 0)
        {
            _loc_SYSC_ResetRobotInfo_DR6(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with 'DR6' type\n");
        }
        else if (stricmp(rgpszRobotType[iRobot], "10KG") == 0)
        {
            _loc_SYSC_ResetRobotInfo_10KG(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with '10KG' type\n");
        }
        else if (stricmp(rgpszRobotType[iRobot], "BLAST") == 0 ||
            stricmp(rgpszRobotType[iRobot], "BLASTING") == 0)
        {
            _loc_SYSC_ResetRobotInfo_BLAST(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with 'BLAST' type\n");
            break;
        }
        else if (stricmp(rgpszRobotType[iRobot], "DANDY-II") == 0 ||
            stricmp(rgpszRobotType[iRobot], "DANDY-II Robot") == 0)
        {
            _loc_SYSC_ResetRobotInfo_DANDY_II(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Robot info was reset with 'DANDY-II' type\n");
            break;
        }
        else
        {
            VERBOSE_WARNING("Unsupported robot type : '%s'\n", rgpszRobotType[iRobot]);

            _loc_SYSC_ResetRobotInfo_VOID(iRobot, rgnStartAxis[iRobot]);
            VERBOSE_VERBOSE("Temporary, Robot info was reset with VOID type\n");
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetWelderInfo_Void()
//      - reset welder info, unimplemented welder

static void _loc_SYSC_ResetWelderInfo_Void(int nWelder,
                                           int nWelderAddr,
                                           const char* pszName)
{
    DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);

    if (pszName != NULL)
    {
        CRT_strcpy(g_rgWelderConfig[nWelder].szName, WELDER_NAME_LEN, pszName);
    }

    // type
    g_rgWelderConfig[nWelder].nType = WELDER_TYPE_NONE;

    // din (Welder -> Controller)
    g_rgWelderConfig[nWelder].nDinSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].din_portno.nArcOn         = -1;
    g_rgWelderConfig[nWelder].din_portno.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_portno.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_portno.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_portno.nTouchProcess  = -1;
    g_rgWelderConfig[nWelder].din_portno.nTouchSignal   = -1;
    g_rgWelderConfig[nWelder].din_portno.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_portno.nArcFail       = -1;

    g_rgWelderConfig[nWelder].din_actlev.nArcOn         = -1;
    g_rgWelderConfig[nWelder].din_actlev.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_actlev.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_actlev.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchProcess  = -1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchSignal   = -1;
    g_rgWelderConfig[nWelder].din_actlev.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_actlev.nArcFail       = -1;

    // dout (Controller -> Welder)
    g_rgWelderConfig[nWelder].nDoutSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].dout_portno.nArcOn        = -1;
    g_rgWelderConfig[nWelder].dout_portno.nGasOn        = -1;
    g_rgWelderConfig[nWelder].dout_portno.nInchPos      = -1;
    g_rgWelderConfig[nWelder].dout_portno.nInchNeg      = -1;
    g_rgWelderConfig[nWelder].dout_portno.nTouchStart   = -1;
    g_rgWelderConfig[nWelder].dout_portno.nTouchReady   = -1;
    g_rgWelderConfig[nWelder].dout_portno.nWireCut      = -1;
    g_rgWelderConfig[nWelder].dout_portno.nWeldPower    = -1;

    g_rgWelderConfig[nWelder].dout_actlev.nArcOn        = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nGasOn        = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchPos      = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchNeg      = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchStart   = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchReady   = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nWireCut      = -1;
    g_rgWelderConfig[nWelder].dout_actlev.nWeldPower    = -1;

    // ain (A/D)
    g_rgWelderConfig[nWelder].nCurrentInPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageInPortNo = 0;

    // aout (D/A)
    g_rgWelderConfig[nWelder].nCurrentOutPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageOutPortNo = 0;

    g_dbVoltRealTimeCmdOffsetUnit = DEF_VOLT_OFFSET_UNIT;
    g_dbCurrRealTimeCmdOffsetUnit = DEF_CURR_OFFSET_UNIT;

    g_dbUpperBoundY_Volt = DEF_VOLT_UPPER_Y;
    g_dbLowerBoundY_Volt = DEF_VOLT_LOWER_Y;
    g_dbUpperBoundX_Volt = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Volt = 0;
    g_dbUpperBoundY_Curr = DEF_CURR_UPPER_Y;
    g_dbLowerBoundY_Curr = DEF_CURR_LOWER_Y;
    g_dbUpperBoundX_Curr = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Curr = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetWelderInfo_Generic()
//      - generic welder

static void _loc_SYSC_ResetWelderInfo_Generic(int nWelder,
                                              int nWelderAddr,
                                              const char* pszName)
{
    DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);

    if (pszName != NULL)
    {
        CRT_strcpy(g_rgWelderConfig[nWelder].szName, WELDER_NAME_LEN, pszName);
    }

    // type & ability
    g_rgWelderConfig[nWelder].nType = WELDER_TYPE_GENERIC;

    // din (UR 501R -> Controller)
    g_rgWelderConfig[nWelder].nDinSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].din_portno.nArcOn         = 0;
    g_rgWelderConfig[nWelder].din_portno.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_portno.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_portno.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_portno.nTouchProcess  = 4;
    g_rgWelderConfig[nWelder].din_portno.nTouchSignal   = 5;
    g_rgWelderConfig[nWelder].din_portno.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_portno.nArcFail       = 7;

    g_rgWelderConfig[nWelder].din_actlev.nArcOn         = 1;
    g_rgWelderConfig[nWelder].din_actlev.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_actlev.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_actlev.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchProcess  = 1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchSignal   = 0;
    g_rgWelderConfig[nWelder].din_actlev.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_actlev.nArcFail       = 1;

    // dout (Controller -> UR 501R)
    g_rgWelderConfig[nWelder].nDoutSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].dout_portno.nArcOn        = 0;
    g_rgWelderConfig[nWelder].dout_portno.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_portno.nInchPos      = 2;
    g_rgWelderConfig[nWelder].dout_portno.nInchNeg      = 3;
    g_rgWelderConfig[nWelder].dout_portno.nTouchStart   = 4;
    g_rgWelderConfig[nWelder].dout_portno.nTouchReady   = 5;
    g_rgWelderConfig[nWelder].dout_portno.nWireCut      = 6;
    g_rgWelderConfig[nWelder].dout_portno.nWeldPower    = 7;

    g_rgWelderConfig[nWelder].dout_actlev.nArcOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchPos      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchNeg      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchStart   = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchReady   = 0;
    g_rgWelderConfig[nWelder].dout_actlev.nWireCut      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nWeldPower    = 1;

    // ain (A/D)
    g_rgWelderConfig[nWelder].nCurrentInPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageInPortNo = 0;

    // aout (D/A)
    g_rgWelderConfig[nWelder].nCurrentOutPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageOutPortNo = 0;

    g_dbVoltRealTimeCmdOffsetUnit = DEF_VOLT_OFFSET_UNIT;
    g_dbCurrRealTimeCmdOffsetUnit = DEF_CURR_OFFSET_UNIT;

    g_dbUpperBoundY_Volt = DEF_VOLT_UPPER_Y;
    g_dbLowerBoundY_Volt = DEF_VOLT_LOWER_Y;
    g_dbUpperBoundX_Volt = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Volt = 0;
    g_dbUpperBoundY_Curr = DEF_CURR_UPPER_Y;
    g_dbLowerBoundY_Curr = DEF_CURR_LOWER_Y;
    g_dbUpperBoundX_Curr = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Curr = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetWelderInfo_Hyosung()
//      - Hyosung UR 351R/501R

static void _loc_SYSC_ResetWelderInfo_Hyosung(int nWelder,
                                              int nWelderAddr,
                                              const char* pszName)
{
    DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);

    if (pszName != NULL)
    {
        CRT_strcpy(g_rgWelderConfig[nWelder].szName, WELDER_NAME_LEN, pszName);
    }

    // type & ability
    g_rgWelderConfig[nWelder].nType = WELDER_TYPE_HYOSUNG_UR;

    // din (UR 501R -> Controller)
    g_rgWelderConfig[nWelder].nDinSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].din_portno.nArcOn         = 0;
    g_rgWelderConfig[nWelder].din_portno.nNoGas         = 1;
    g_rgWelderConfig[nWelder].din_portno.nNoWire        = 2;
    g_rgWelderConfig[nWelder].din_portno.nWeldPowerFail = 3;
    g_rgWelderConfig[nWelder].din_portno.nTouchProcess  = 4;
    g_rgWelderConfig[nWelder].din_portno.nTouchSignal   = 5;
    g_rgWelderConfig[nWelder].din_portno.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_portno.nArcFail       = -1;

    g_rgWelderConfig[nWelder].din_actlev.nArcOn         = 1;
    g_rgWelderConfig[nWelder].din_actlev.nNoGas         = 1;
    g_rgWelderConfig[nWelder].din_actlev.nNoWire        = 1;
    g_rgWelderConfig[nWelder].din_actlev.nWeldPowerFail = 1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchProcess  = 1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchSignal   = 0;
    g_rgWelderConfig[nWelder].din_actlev.nWeld          = -1;
    g_rgWelderConfig[nWelder].din_actlev.nArcFail       = -1;

    // dout (Controller -> UR 501R)
    g_rgWelderConfig[nWelder].nDoutSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].dout_portno.nArcOn        = 0;
    g_rgWelderConfig[nWelder].dout_portno.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_portno.nInchPos      = 2;
    g_rgWelderConfig[nWelder].dout_portno.nInchNeg      = 3;
    g_rgWelderConfig[nWelder].dout_portno.nTouchStart   = 4;
    g_rgWelderConfig[nWelder].dout_portno.nTouchReady   = 5;
    g_rgWelderConfig[nWelder].dout_portno.nWireCut      = 6;
    g_rgWelderConfig[nWelder].dout_portno.nWeldPower    = -1;

    g_rgWelderConfig[nWelder].dout_actlev.nArcOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchPos      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchNeg      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchStart   = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchReady   = 0;
    g_rgWelderConfig[nWelder].dout_actlev.nWireCut      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nWeldPower    = -1;

    // ain (A/D)
    g_rgWelderConfig[nWelder].nCurrentInPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageInPortNo = 0;

    // aout (D/A)
    g_rgWelderConfig[nWelder].nCurrentOutPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageOutPortNo = 0;

    g_dbVoltRealTimeCmdOffsetUnit = DEF_VOLT_OFFSET_UNIT;
    g_dbCurrRealTimeCmdOffsetUnit = DEF_CURR_OFFSET_UNIT;

    g_dbUpperBoundY_Volt = DEF_VOLT_UPPER_Y;
    g_dbLowerBoundY_Volt = DEF_VOLT_LOWER_Y;
    g_dbUpperBoundX_Volt = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Volt = 0;
    g_dbUpperBoundY_Curr = DEF_CURR_UPPER_Y;
    g_dbLowerBoundY_Curr = DEF_CURR_LOWER_Y;
    g_dbUpperBoundX_Curr = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Curr = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ResetWelderInfo_Daihen()
//      - Daihen DM 350/500

static void _loc_SYSC_ResetWelderInfo_Daihen(int nWelder,
                                             int nWelderAddr,
                                             const char* pszName)
{
    DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);

    if (pszName != NULL)
    {
        CRT_strcpy(g_rgWelderConfig[nWelder].szName, WELDER_NAME_LEN, pszName);
    }

    // type & ability
    g_rgWelderConfig[nWelder].nType = WELDER_TYPE_DAIHEN_DM;

    // din (DM 500 -> Controller)
    g_rgWelderConfig[nWelder].nDinSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].din_portno.nArcOn         = -1;
    g_rgWelderConfig[nWelder].din_portno.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_portno.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_portno.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_portno.nTouchProcess  = 4;
    g_rgWelderConfig[nWelder].din_portno.nTouchSignal   = 5;
    g_rgWelderConfig[nWelder].din_portno.nWeld          = 6;
    g_rgWelderConfig[nWelder].din_portno.nArcFail       = -1;

    g_rgWelderConfig[nWelder].din_actlev.nArcOn         = -1;
    g_rgWelderConfig[nWelder].din_actlev.nNoGas         = -1;
    g_rgWelderConfig[nWelder].din_actlev.nNoWire        = -1;
    g_rgWelderConfig[nWelder].din_actlev.nWeldPowerFail = -1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchProcess  = 1;
    g_rgWelderConfig[nWelder].din_actlev.nTouchSignal   = 0;
    g_rgWelderConfig[nWelder].din_actlev.nWeld          = 1;
    g_rgWelderConfig[nWelder].din_actlev.nArcFail       = -1;

    // dout (Controller -> DM 500)
    g_rgWelderConfig[nWelder].nDoutSlaveNo = nWelderAddr;
    g_rgWelderConfig[nWelder].dout_portno.nArcOn        = 0;
    g_rgWelderConfig[nWelder].dout_portno.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_portno.nInchPos      = 2;
    g_rgWelderConfig[nWelder].dout_portno.nInchNeg      = 3;
    g_rgWelderConfig[nWelder].dout_portno.nTouchStart   = 4;
    g_rgWelderConfig[nWelder].dout_portno.nTouchReady   = 5;
    g_rgWelderConfig[nWelder].dout_portno.nWireCut      = 6;
    g_rgWelderConfig[nWelder].dout_portno.nWeldPower    = 7;

    g_rgWelderConfig[nWelder].dout_actlev.nArcOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nGasOn        = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchPos      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nInchNeg      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchStart   = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nTouchReady   = 0;
    g_rgWelderConfig[nWelder].dout_actlev.nWireCut      = 1;
    g_rgWelderConfig[nWelder].dout_actlev.nWeldPower    = 1;

    // ain (A/D)
    g_rgWelderConfig[nWelder].nCurrentInPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageInPortNo = 0;

    // aout (D/A)
    g_rgWelderConfig[nWelder].nCurrentOutPortNo = 1;
    g_rgWelderConfig[nWelder].nVoltageOutPortNo = 0;

    g_dbVoltRealTimeCmdOffsetUnit = DEF_VOLT_OFFSET_UNIT;
    g_dbCurrRealTimeCmdOffsetUnit = DEF_CURR_OFFSET_UNIT;

    g_dbUpperBoundY_Volt = DEF_VOLT_UPPER_Y;
    g_dbLowerBoundY_Volt = DEF_VOLT_LOWER_Y;
    g_dbUpperBoundX_Volt = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Volt = 0;
    g_dbUpperBoundY_Curr = DEF_CURR_UPPER_Y;
    g_dbLowerBoundY_Curr = DEF_CURR_LOWER_Y;
    g_dbUpperBoundX_Curr = MAX_WELDOUT_VOLTAGE;
    g_dbLowerBoundX_Curr = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ResetWelderInfo()
//

int SYSC_ResetWelderInfo(int nCount, const int rgnWelderRobot[],
                         const char* const rgpszWelderType[],
                         const int rgnWelderAddr[])
{
    int iWelder;
    int nRobot, nRobotWelder;

    // clear all welder information
    SYSC_ClearConfigWelder();

    // create new robot
    for (iWelder = 0; iWelder < nCount; iWelder++)
    {
        if (rgpszWelderType[iWelder] == NULL || rgpszWelderType[iWelder][0] == 0)
        {
            VERBOSE_WARNING("Welder type was not specified\n");

            _loc_SYSC_ResetWelderInfo_Void(iWelder,
                                           rgnWelderAddr[iWelder],
                                           "Void");

            VERBOSE_VERBOSE("Temporary, Welder type was reset with 'Void' type\n");
        }
        else if (strnicmp(rgpszWelderType[iWelder], "hyosung_ur", 10) == 0)
        {
            // Hyosung UR 501R
            _loc_SYSC_ResetWelderInfo_Hyosung(iWelder,
                                              rgnWelderAddr[iWelder],
                                              rgpszWelderType[iWelder]);
            VERBOSE_VERBOSE("Welder type was reset with 'HYOSUNG_UR' series type\n");
        }
        else if (strnicmp(rgpszWelderType[iWelder], "daihen_dm", 9) == 0)
        {
            // Daihen DM 500
            _loc_SYSC_ResetWelderInfo_Daihen(iWelder,
                                              rgnWelderAddr[iWelder],
                                              rgpszWelderType[iWelder]);
            VERBOSE_VERBOSE("Welder type was reset with 'DAIHEN_DM' series type\n");
        }
        else
        {
            VERBOSE_WARNING("Unsupported welder type : '%s'\n", rgpszWelderType[iWelder]);

            _loc_SYSC_ResetWelderInfo_Generic(iWelder,
                                              rgnWelderAddr[iWelder],
                                              rgpszWelderType[iWelder]);

            VERBOSE_VERBOSE("Temporary, Welder type was reset with 'Generic' type\n");
        }

        // add the welder info to robot info
        nRobot = rgnWelderRobot[iWelder];
        DANDY_ASSERT(nRobot >= 0 && nRobot < MAX_ROBOT_COUNT);

        nRobotWelder = g_rgnRobotWelderCount[nRobot]++;
        DANDY_ASSERT(nRobotWelder >= 0 && nRobotWelder < MAX_WELDER_COUNT);
            
        g_rgnRobotWelderList[nRobot][nRobotWelder] = iWelder;
    }

    return 0;
}
