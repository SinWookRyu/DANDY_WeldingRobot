/////////////////////////////////////////////////////////////////////////////
//
//  conf_check.c: Config Parameter Validation Check & Arrange
//                                            2013.06.25  Ryu SinWook

#include <math.h>

///////////////////////////////////////

#include "robotmgr_main.h"

///////////////////////////////////////

///////////////////////////////////////
//  Config related Extern Variables Definition
///////////////////////////////////////

///////////////////////////////////////
// robot parameter

int    g_rgfRobotUsed[MAX_ROBOT_COUNT];
char   g_rgszRobotName[MAX_ROBOT_COUNT][ROBOT_NAME_LEN];
int    g_rgnRobotType[MAX_ROBOT_COUNT];

int    g_rgnCommProtocol[MAX_ROBOT_COUNT];     // comm protocol
char   g_rgszCommPort[MAX_ROBOT_COUNT][MAX_COMM_COUNT];

int    g_rgnRobotAxisCount[MAX_ROBOT_COUNT];
CONFIG_AXIS g_rgRobotAxis[MAX_ROBOT_COUNT][ROB_AXIS_COUNT];

int    g_rgnRobotDefJogPercent[MAX_ROBOT_COUNT];
int    g_rgnRobotDefExtraJogPercent[MAX_ROBOT_COUNT];

DH_PARAMS    g_rgRobotDHParam[MAX_ROBOT_COUNT][ROB_AXIS_COUNT];
CONFIG_ROBOT g_rgRobotMotion[MAX_ROBOT_COUNT];

CONFIG_ROBOT g_rgCoordInfo[MAX_ROBOT_COUNT];

double g_rgdbHomePosVal[MAX_HOME_COUNT][ROB_AXIS_COUNT];    // Unit: deg

COORD_EULER    g_CartOffset;

unsigned long  g_rgdwCmdSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwTVarSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwPVarSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwBVarSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwIVarSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwRVarSize[MAX_ROBOT_COUNT];

unsigned long  g_rgdwWeaveSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwSWFSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwMWFSize[MAX_ROBOT_COUNT];
unsigned long  g_rgdwEWFSize[MAX_ROBOT_COUNT];

int    g_rgnRobotWelderCount[MAX_ROBOT_COUNT];
int    g_rgnRobotWelderList[MAX_ROBOT_COUNT][MAX_WELDER_COUNT];

CONFIG_WELD_FUNC    g_Weld_Function[MAX_ROBOT_COUNT];

TE_RESTART_PARAM  g_rgRestartParam[MAX_ROBOT_COUNT];
ARCSENSOR_PARAM   g_rgArcSensorParam[MAX_ROBOT_COUNT];

///////////////////////////////////////
// axis parameter

int    g_rgfAxisUsed[MAX_AXIS_COUNT];
char   g_rgszAxisName[MAX_AXIS_COUNT][AXIS_NAME_LEN];  // axis's name
int    g_rgnAxisType[MAX_AXIS_COUNT];         // axis's type
int    g_rgnAxisIndex[MAX_AXIS_COUNT];        // axis's id

int    g_rgfHwLimitUsed[MAX_AXIS_COUNT][2];    // HW limit used?
double g_rgdbHwLimit[MAX_AXIS_COUNT][2];       // HW limit
int    g_rgfSwLimitUsed[MAX_AXIS_COUNT][2];    // SW limit used?
double g_rgdbSwLimit[MAX_AXIS_COUNT][2];       // SW limit

double g_rgdbGearRatio[MAX_AXIS_COUNT];        // reduction gear ratio
double g_rgdbRotaionDist[MAX_AXIS_COUNT];      // axis's terminal rataion distance
int    g_rgnAxisDirection[MAX_AXIS_COUNT];     // axis's direction
int    g_rgnEncoderResetVal[MAX_AXIS_COUNT];   // INT_MIN = unused
int    g_rgnEcnoderHomeVal[MAX_AXIS_COUNT];    // INT_MIN = unused

int    g_nMotorCount[MAX_AXIS_COUNT];          // Motor count include in axis

///////////////////////////////////////
// motor parameter

char   g_rgszMotorName[MAX_MOTOR_COUNT][MOTOR_NAME_LEN];  // motor name 
int    g_rgnMotorType[MAX_MOTOR_COUNT];         // motor type
int    g_rgnMotorIndex[MAX_MOTOR_COUNT];         // motor id

int    g_rgnEncoderRes[MAX_MOTOR_COUNT];        // axis's encoder resolution
DWORD  g_rgdwAbsEncoderRes[MAX_MOTOR_COUNT];    // axis's ABS encoder resolution

int    g_rgnEncoderType[MAX_MOTOR_COUNT];       // encoder type (ABS | INC)
int    g_rgnHwHome[MAX_MOTOR_COUNT];            // bit, negtive value is unused

double g_rgdbMotorMaxVel[MAX_MOTOR_COUNT];
double g_rgdbMotorMaxAccel[MAX_MOTOR_COUNT];

CONFIG_MOTOR g_rgMotorConfig[MAX_MOTOR_COUNT];

///////////////////////////////////////
// welder parameter

CONFIG_WELDER    g_rgWelderConfig[MAX_WELDER_COUNT];

WELD_PARAM_TUNE  g_rgWeldTuneParam[MAX_WELDER_COUNT];

int g_nWeldTuneInParamApplyIndex;
int g_nWeldTuneOutParamApplyIndex;

double  g_dbAinMaxVolt;
double  g_dbADCMaxBit;

double  g_dbControllerCmdVolt[MAX_WELDER_COUNT];
double  g_dbWelderMeasureVolt[MAX_WELDER_COUNT];
double  g_dbControllerCmdCurr[MAX_WELDER_COUNT];
double  g_dbWelderMeasureCurr[MAX_WELDER_COUNT];

///////////////////////////////////////

// config dir buffer (config dir is program startup directory)
static char s_szConfigDir[PATH_NAME_BUFFER_SIZE];


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigGloabl()
//      - Clear Config Global Parameter

void SYSC_ClearConfigGloabl(void)
{
    int i;

    // some values are illegally initialized to check duplication

    g_nReqVersion = -1;
    g_nTerminalInetPort = -1;

#if defined(_WIN32)
    GetCurrentDirectory(sizeof(s_szConfigDir), s_szConfigDir);
#else
    getcwd(s_szConfigDir, sizeof(s_szConfigDir));
#endif
    g_pszConfigDir = s_szConfigDir;

    if (g_pszWorkDir != NULL)
    {
        DEBUG_FREE(g_pszWorkDir);
        g_pszWorkDir = NULL;
    }

    if (g_pszJobDir != NULL)
    {
        DEBUG_FREE(g_pszJobDir);
        g_pszJobDir = NULL;
    }

    if (g_pszSensDir != NULL)
    {
        DEBUG_FREE(g_pszSensDir);
        g_pszSensDir = NULL;
    }

    if (g_pszWireCutJobFileName != NULL)
    {
        DEBUG_FREE(g_pszWireCutJobFileName);
        g_pszWireCutJobFileName = NULL;
    }

    if (g_pszNewCreatedJobFileName != NULL)
    {
        DEBUG_FREE(g_pszNewCreatedJobFileName);
        g_pszNewCreatedJobFileName = NULL;
    }

    if (g_pszEcatConfigDir != NULL)
    {
        DEBUG_FREE(g_pszEcatConfigDir);
        g_pszEcatConfigDir = NULL;
    }

    if (g_pszLoadedJobFileName != NULL)
    {
        DEBUG_FREE(g_pszLoadedJobFileName);
        g_pszLoadedJobFileName = NULL;
    }
    
    if (g_pszTerminalGreeting != NULL)
    {
        DEBUG_FREE(g_pszTerminalGreeting);
        g_pszTerminalGreeting = NULL;
    }

    if (g_pszLocale != NULL)
    {
        DEBUG_FREE(g_pszLocale);
        g_pszLocale = NULL;
    }

    if (g_pszPrompt != NULL)
    {
        DEBUG_FREE(g_pszPrompt);
        g_pszPrompt = NULL;
    }

    g_pszTerminalGreeting = NULL;
    g_pszLocale = NULL;
    g_pszPrompt = NULL;

    g_nQNXTimerTick = 0;
    g_nQNXTimerRes  = 0;
    g_nIoTime = 0;
    g_nTrajUpdateTime = 0;
    g_nServoInterpolationTime = 0;

    g_nServoOnBrakeDelayTime = 0;
    g_nServoOffBrakeDelayTime = 0;

    g_nEstopGasOffDelayTime = 0;
    g_nEstopTouchReadyOffDelayTime = 0;

    for(i = 0; i < MAX_MASTER_RESET_CNT_DAY; i++)
    {
        g_rgnEcatMasterResetHours[i] = 0;
    }
    g_nEcatMasterResetHoursCount = 0;
    g_nEcatMasterResetMin = -1;

    g_nSlaveCount = 0;
    g_nWriteOffsetSize = 0;
    g_nReadOffsetSize = 0;

    g_nErrHistorySaveMaxCnt = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigRobot()
//      - Clear Config Robot Parameter

void SYSC_ClearConfigRobot(void)
{
    int iRobot;

    memset(g_rgfRobotUsed,          0, sizeof(g_rgfRobotUsed));
    memset(g_rgszRobotName,         0, sizeof(g_rgszRobotName));
    memset(g_rgnRobotType,          0, sizeof(g_rgnRobotType));

    memset(g_rgnCommProtocol,       0, sizeof(g_rgnCommProtocol));
    memset(g_rgszCommPort,          0, sizeof(g_rgszCommPort));

    memset(g_rgnRobotAxisCount,     0, sizeof(g_rgnRobotAxisCount));
    memset(g_rgRobotAxis,           0, sizeof(g_rgRobotAxis));
    memset(g_rgnRobotDefJogPercent, 0, sizeof(g_rgnRobotDefJogPercent));
    memset(g_rgnRobotDefExtraJogPercent, 0, sizeof(g_rgnRobotDefExtraJogPercent));
    memset(g_nHomeSpeed,            0, sizeof(g_nHomeSpeed));
    memset(g_rgRobotDHParam,        0, sizeof(g_rgRobotDHParam));
    memset(g_rgRobotMotion,         0, sizeof(g_rgRobotMotion));

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        g_rgRobotMotion[iRobot].dbJerk = -1;
    }

    memset(g_rgCoordInfo, 0xff, sizeof(g_rgCoordInfo));  // init with (-1)

    memset(g_rgdwCmdSize,   0, sizeof(g_rgdwCmdSize));
    memset(g_rgdwTVarSize,  0, sizeof(g_rgdwTVarSize));
    memset(g_rgdwPVarSize,  0, sizeof(g_rgdwPVarSize));
    memset(g_rgdwBVarSize,  0, sizeof(g_rgdwBVarSize));
    memset(g_rgdwIVarSize,  0, sizeof(g_rgdwIVarSize));
    memset(g_rgdwRVarSize,  0, sizeof(g_rgdwRVarSize));
    memset(g_rgdwWeaveSize, 0, sizeof(g_rgdwWeaveSize));
    memset(g_rgdwSWFSize,   0, sizeof(g_rgdwSWFSize));
    memset(g_rgdwMWFSize,   0, sizeof(g_rgdwMWFSize));
    memset(g_rgdwEWFSize,   0, sizeof(g_rgdwEWFSize));

    memset(g_rgnRobotWelderCount, 0,    sizeof(g_rgnRobotWelderCount));
    memset(g_rgnRobotWelderList,  0xff, sizeof(g_rgnRobotWelderList));
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigAxis()
//      - Clear Config Axis Parameter

void SYSC_ClearConfigAxis(void)
{
    int iAxis;

    memset(g_rgfAxisUsed,      0, sizeof(g_rgfAxisUsed));

    memset(g_rgszAxisName,     0, sizeof(g_rgszAxisName));
    memset(g_rgnAxisType,      0, sizeof(g_rgnAxisType));
    memset(g_rgnAxisIndex,  0xff, sizeof(g_rgnAxisIndex));  // init with (-1)

    memset(g_rgfHwLimitUsed,   0, sizeof(g_rgfHwLimitUsed));
    memset(g_rgdbHwLimit,      0, sizeof(g_rgdbHwLimit));
    memset(g_rgfSwLimitUsed,   0, sizeof(g_rgfSwLimitUsed));
    memset(g_rgdbSwLimit,      0, sizeof(g_rgdbSwLimit));

    memset(g_rgdbGearRatio,    0, sizeof(g_rgdbGearRatio));
    memset(g_rgdbRotaionDist,  0, sizeof(g_rgdbRotaionDist));

    memset(g_rgnAxisDirection, 0, sizeof(g_rgnAxisDirection));
    memset(g_nMotorCount,      0, sizeof(g_nMotorCount));
    
    for (iAxis = 0; iAxis < MAX_AXIS_COUNT; iAxis++)
    {
        g_rgnEncoderResetVal[iAxis] = INT_MIN;  // unused
        g_rgnEcnoderHomeVal[iAxis]  = INT_MIN;  // unused
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigMotor()
//      - Clear Config Motor Parameter

void SYSC_ClearConfigMotor(void)
{
    memset(g_rgszMotorName,     0, sizeof(g_rgszMotorName));
    memset(g_rgnMotorType,      0, sizeof(g_rgnMotorType));
    memset(g_rgnMotorIndex,  0xff, sizeof(g_rgnMotorIndex)); // init with (-1)

    memset(g_rgnEncoderRes,     0, sizeof(g_rgnEncoderRes));
    memset(g_rgdwAbsEncoderRes, 0, sizeof(g_rgdwAbsEncoderRes));

    memset(g_rgnEncoderType,    0, sizeof(g_rgnEncoderType));


    memset(g_rgnHwHome,      0xff, sizeof(g_rgnHwHome));     // init with (-1)

    memset(g_rgdbMotorMaxVel,   0, sizeof(g_rgdbMotorMaxVel));
    memset(g_rgdbMotorMaxAccel, 0, sizeof(g_rgdbMotorMaxAccel));
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigWelder()
//      - Clear Config Welder Parameter

void SYSC_ClearConfigWelder(void)
{
    int iWelder;

    memset(g_rgnRobotWelderCount, 0, sizeof(g_rgnRobotWelderCount));
    memset(g_rgWelderConfig,   0xff, sizeof(g_rgWelderConfig));
    //memset(g_rgWeldTuneParam,   0xff, sizeof(g_rgWeldTuneParam));
    
    for (iWelder = 0; iWelder < MAX_WELDER_COUNT; iWelder++)
    {
        g_rgWelderConfig[iWelder].fUsed = 0;
        g_rgWelderConfig[iWelder].szName[0] = 0;
        g_rgWelderConfig[iWelder].nType = WELDER_TYPE_NONE;
    }

    g_nWeldTuneInParamApplyIndex = 0;
    g_nWeldTuneOutParamApplyIndex = 0;

    g_dbAinMaxVolt  = 0.0;
    g_dbADCMaxBit   = 0.0;

    g_dbVoltRealTimeCmdOffsetUnit = 0;
    g_dbCurrRealTimeCmdOffsetUnit = 0;

    g_dbUpperBoundY_Volt = 0;
    g_dbLowerBoundY_Volt = 0;
    g_dbUpperBoundX_Volt = 0;
    g_dbLowerBoundX_Volt = 0;
    g_dbUpperBoundY_Curr = 0;
    g_dbLowerBoundY_Curr = 0;
    g_dbUpperBoundX_Curr = 0;
    g_dbLowerBoundX_Curr = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfigSensor()
//      - Clear Config Sensor Parameter

void SYSC_ClearConfigSensor(void)
{
   /* memset(g_rgnRobotSensorCount, 0, sizeof(g_rgnRobotSensorCount));
    memset(g_rgSensorConfig, 0, sizeof(g_rgSensorConfig));*/
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ClearConfig()
//

void SYSC_ClearConfig(void)
{
    SYSC_ClearConfigGloabl();
    SYSC_ClearConfigRobot();
    SYSC_ClearConfigAxis();
    SYSC_ClearConfigMotor();
    SYSC_ClearConfigWelder();
    //SYSC_ClearConfigSensor();
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigGlobal()
//      - Check Corrections Global Parameter

int SYSC_CheckConfigGlobal(void)
{
    // Check QNX Timer Tick
    if (g_nQNXTimerTick < 1)
    {
        VERBOSE_ERROR("QNX Timer Tick is Invalid : %d [ms]\n",
                      g_nQNXTimerTick);
        return RESULT_ERROR;
    }

    // Check QNX Timer Res
    if (g_nQNXTimerRes < 1)
    {
        VERBOSE_ERROR("QNX Timer Resolution is Invalid : %d [ms]\n",
                      g_nQNXTimerRes);
        return RESULT_ERROR;
    }

    // Check I/O Scan Time
    if (g_nIoTime < 1)
    {
        VERBOSE_ERROR("I/O Scan Time is Invalid : %d [ms]\n",
                      g_nIoTime);
        return RESULT_ERROR;
    }

    // Check Slave Count
    if (g_nSlaveCount < 1)
    {
        VERBOSE_ERROR("Slave Count is Invalid : %d [EA]\n",
                      g_nSlaveCount);
        return RESULT_ERROR;
    }

    // Check Error History Save Max Count
    if (g_nErrHistorySaveMaxCnt < 1)
    {
        VERBOSE_ERROR("Error History Save Max Count is Invalid : %d [EA]\n",
                      g_nErrHistorySaveMaxCnt);
        return RESULT_ERROR;
    }

    // Check Write Offset Size
    if (g_nWriteOffsetSize < 0x00)
    {
        VERBOSE_ERROR("Write Offset Size is Invalid : %x\n",
                      g_nWriteOffsetSize);
        return RESULT_ERROR;
    }

    // Check Read Offset Size
    if (g_nReadOffsetSize < 0x00)
    {
        VERBOSE_ERROR("Read Offset Size is Invalid : %x\n",
                      g_nReadOffsetSize);
        return RESULT_ERROR;
    }

    // Trajectory Update Time
    if (g_nTrajUpdateTime < MIN_TRJ_TIME || g_nTrajUpdateTime > MAX_TRJ_TIME)
    {
        VERBOSE_ERROR("Trajectory Update Time is invalid : %d [ms] (%d ~ %d)\n",
                         g_nTrajUpdateTime, MIN_TRJ_TIME, MAX_TRJ_TIME);
        return RESULT_ERROR;
    }
    
    // Check Trajectory Buffer Count
    if (g_nServoInterpolationTime < 0 || g_nServoInterpolationTime > MAX_INTERPOLATION_TIME)
    {
        VERBOSE_ERROR("Trajectory Buffer should be larger than "
                         "0 and less than %d  : %d\n",
                         MAX_INTERPOLATION_TIME, g_nServoInterpolationTime);
        return RESULT_ERROR;
    }
    
    // Check Servo On Delay Time
    if (g_nServoOnBrakeDelayTime < 0 || g_nServoOnBrakeDelayTime > 1000)
    {
        VERBOSE_ERROR("Servo On Delay Time is invalid : %d [ms] (%d ~ %d)\n",
                         g_nServoOnBrakeDelayTime, 0, 1000);
        return RESULT_ERROR;
    }

    // Check Servo Off Delay Time
    if (g_nServoOffBrakeDelayTime < 0 || g_nServoOffBrakeDelayTime > 1000)
    {
        VERBOSE_ERROR("Servo Off Delay Time is invalid : %d [ms] (%d ~ %d)\n",
                         g_nServoOffBrakeDelayTime, 0, 1000);
        return RESULT_ERROR;
    }

    // Check Estop Gas Off Delay Time
    if (g_nEstopGasOffDelayTime < 0 || g_nEstopGasOffDelayTime > 10000)
    {
        VERBOSE_ERROR("Estop Gas Off Delay Time is invalid : %d [ms] (%d ~ %d)\n",
                         g_nEstopGasOffDelayTime, 0, 10000);
        return RESULT_ERROR;
    }

    // Check Estop TouchReady Off Delay Time
    if (g_nEstopTouchReadyOffDelayTime < 0 || g_nEstopTouchReadyOffDelayTime > 10000)
    {
        VERBOSE_ERROR("Estop TouchReady Off Delay Time is invalid : %d [ms] (%d ~ %d)\n",
                         g_nEstopTouchReadyOffDelayTime, 0, 10000);
        return RESULT_ERROR;
    }

    // Check Analog Input Max Volt
    if (g_dbAinMaxVolt < 0.0)
    {
        VERBOSE_ERROR("Analog Input Max Volt is invalid : %.1lf [V]"
                      "(Bigger than %d)\n",
                      g_dbAinMaxVolt, 0);
        return RESULT_ERROR;
    }

    // Check Analog Input Convert Max Bit
    if (g_dbADCMaxBit < 0.0 || g_dbADCMaxBit > 65535.0)
    {
        VERBOSE_ERROR("Analog Input Convert Max Bit is invalid : %.0lf [bit]"
                      "(%d ~ %d)\n",
                      g_dbADCMaxBit, 0, 65535);
        return RESULT_ERROR;
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigRobot()
//      - Check Corrections Robot Parameter

int SYSC_CheckConfigRobot(void)
{
    int rgnAxisRobot[ROB_AXIS_COUNT];
    int rgnWelderRobot[MAX_WELDER_COUNT];

    int nAxesUsed, nMasterAxis, nUsedCount;
    int iRobot, iAxis;

    int iWelder, nWelder;

    memset(rgnAxisRobot, 0xff, sizeof(rgnAxisRobot));       // init with (-1)
    memset(rgnWelderRobot, 0xff, sizeof(rgnWelderRobot));   // init with (-1)

    nUsedCount = 0;

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        ///////////////////////////////////////////////////
        // check robot's joints and axes

        DANDY_ASSERT(g_rgszRobotName[iRobot][0] != 0);

        if (g_rgfRobotUsed[iRobot] &&
            g_rgnRobotType[iRobot] == ROBTYPE_NONE)
        {
            VERBOSE_ERROR("Robot %d : Invalid robot type\n",
                             iRobot+1);
            return RESULT_ERROR;
        }
        
        if (!g_rgfRobotUsed[iRobot])
            continue;

        nUsedCount++;

        if ((g_rgnRobotType[iRobot] == ROBTYPE_NONE ||
             g_rgnRobotType[iRobot] == ROBTYPE_EXCEPT) &&
            g_rgnRobotAxisCount[iRobot] != 0)
        {
            VERBOSE_ERROR("Robot %d : VOID robot was specified joints\n",
                             iRobot+1);
            return RESULT_ERROR;
        }

        // axes count, axis duplication using check
        nAxesUsed = 0;

        for (iAxis = 0; iAxis < g_rgnRobotAxisCount[iRobot]; iAxis++)
        {
            nMasterAxis = g_rgRobotAxis[iRobot][iAxis].nMotorCount;
            DANDY_ASSERT(nMasterAxis >= 0 && nMasterAxis < MAX_AXIS_COUNT);
            DANDY_ASSERT(g_rgfAxisUsed[nMasterAxis]);

            if (rgnAxisRobot[nMasterAxis] != -1)
            {
                VERBOSE_ERROR("Robot %d : Axis %d is already assigned "
                              "to Robot %d\n",
                              iRobot+1, nMasterAxis, rgnAxisRobot[nMasterAxis]+1);
                return RESULT_ERROR;
            }

            DANDY_ASSERT(g_rgfAxisUsed[nMasterAxis]);

            rgnAxisRobot[nMasterAxis] = iRobot;
            nAxesUsed++;
        } // end of for(iAxis)

        if (g_rgnRobotType[iRobot] == ROBTYPE_NONE ||
            g_rgnRobotType[iRobot] == ROBTYPE_EXCEPT)
        {
            if (nAxesUsed != 0)
            {
                VERBOSE_ERROR("Robot %d was defined as non-motional robot, "
                              "but assigned %d axes\n",
                              iRobot+1, nAxesUsed);
                return RESULT_ERROR;
            }
        }
        else
        {
            if (nAxesUsed == 0)
            {
                VERBOSE_ERROR("Robot %d was defined as motional robot, "
                              "but not assigned any axes\n",
                              iRobot+1);
                return RESULT_ERROR;
            }
        }

        // linear speed
        if (g_rgfRobotUsed[iRobot] && g_rgnRobotType[iRobot] != ROBTYPE_EXCEPT)
        {
            DANDY_ASSERT(g_rgnRobotType[iRobot] != ROBTYPE_NONE);

            for (iAxis = 0; iAxis < g_rgnRobotAxisCount[iRobot]; iAxis++)
            {
                // maximum speed
                if (g_rgRobotMotion[iRobot].dbMaxJointSpeed[iAxis] < 0)
                {
                    VERBOSE_ERROR("Robot %d, Joint %d :"
                                  "max joint speed is invalid or not specified\n",
                                  iRobot+1, iAxis+1);
                    return RESULT_ERROR;
                }
            }

            // maximum speed
            if (g_rgRobotMotion[iRobot].dbMaxLinearSpeed < 0)
            {
                VERBOSE_ERROR("Robot %d : Linear speed is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbMaxOrientSpeed < 0)
            {
                VERBOSE_ERROR("Robot %d : Orientation speed is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbJerk < 0 ||
                g_rgRobotMotion[iRobot].dbJerk > g_rgRobotMotion[iRobot].dbAccel)
            {
                VERBOSE_ERROR("Robot %d : jerk value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbAccel <= MIN_ACCDEC_VAL)
            {
                VERBOSE_ERROR("Robot %d : acceleration value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbDecel <= MIN_ACCDEC_VAL)
            {
                VERBOSE_ERROR("Robot %d : deceleration value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbDecel_Error < 0)
            {
                VERBOSE_ERROR("Robot %d : error deceleration value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbDecel_Estop < 0)
            {
                VERBOSE_ERROR("Robot %d : estop deceleration value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_rgRobotMotion[iRobot].dbDecel_Touch < 0)
            {
                VERBOSE_ERROR("Robot %d : touch deceleration value is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

            if (g_nHomeSpeed[iRobot] < 0)
            {
                VERBOSE_ERROR("Robot %d : home speed is invalid or not specified\n",
                              iRobot+1);
                return RESULT_ERROR;
            }

           if(g_Weld_Function[iRobot].nGapRefBvar   == g_Weld_Function[iRobot].nLeftWeldBvar  ||
              g_Weld_Function[iRobot].nGapRefBvar   == g_Weld_Function[iRobot].nRightWeldBvar ||
              g_Weld_Function[iRobot].nLeftWeldBvar == g_Weld_Function[iRobot].nRightWeldBvar)
            {
                VERBOSE_ERROR("Can not Use Same Bvar LeftWeld(%d), RightWeld(%d), GapRef(%d) Variable!\n",
                              g_Weld_Function[iRobot].nGapRefBvar,
                              g_Weld_Function[iRobot].nLeftWeldBvar,
                              g_Weld_Function[iRobot].nRightWeldBvar);

                g_Weld_Function[iRobot].fGapRefVarUsed  = 0;
                g_Weld_Function[iRobot].nGapRefBvar     = -1;
                g_Weld_Function[iRobot].nLeftWeldBvar   = -1;
                g_Weld_Function[iRobot].nRightWeldBvar  = -1;

                return RESULT_ERROR;
            }

           if(g_nLeftVertSkipBvar    == g_nRightVertSkipBvar   ||
              g_nRightVertSkipBvar   == g_nLeftCollarSkipBvar  ||
              g_nLeftCollarSkipBvar  == g_nRightCollarSkipBvar ||
              g_nRightCollarSkipBvar == g_nHorizontalSkipBvar  ||
              g_nHorizontalSkipBvar  == g_nLeftVertSkipBvar    ||
              g_nLeftVertSkipBvar    == g_nLeftBracketSkipBvar ||
              g_nLeftBracketSkipBvar == g_nRightBracketSkipBvar)
            {
                VERBOSE_ERROR("Can not Use Same Bvar LVert(%d), RVert(%d), "
                              "LCollar(%d), RCollar(%d), Horiz(%d), LBracket(%d), RBracket(%d) Variable!\n",
                              g_nLeftVertSkipBvar,
                              g_nRightVertSkipBvar,
                              g_nLeftCollarSkipBvar,
                              g_nRightCollarSkipBvar,
                              g_nHorizontalSkipBvar,
                              g_nLeftBracketSkipBvar,
                              g_nRightBracketSkipBvar);

                g_nLeftVertSkipBvar     = -1;
                g_nRightVertSkipBvar    = -1;
                g_nLeftCollarSkipBvar   = -1;
                g_nRightCollarSkipBvar  = -1;
                g_nHorizontalSkipBvar   = -1; 
                g_nLeftBracketSkipBvar  = -1;
                g_nRightBracketSkipBvar = -1;

                return RESULT_ERROR;
            }
        }

        ///////////////////////////////////////////////////
        // check robot's job program area

        if (!g_rgfRobotUsed[iRobot])
        {
            DANDY_ASSERT(g_rgdwCmdSize[iRobot]   == 0);
            DANDY_ASSERT(g_rgdwTVarSize[iRobot]  == 0);
            DANDY_ASSERT(g_rgdwPVarSize[iRobot]  == 0);
            DANDY_ASSERT(g_rgdwBVarSize[iRobot]  == 0);
            DANDY_ASSERT(g_rgdwIVarSize[iRobot]  == 0);
            DANDY_ASSERT(g_rgdwRVarSize[iRobot]  == 0);
            DANDY_ASSERT(g_rgdwWeaveSize[iRobot] == 0);
            DANDY_ASSERT(g_rgdwSWFSize[iRobot]   == 0);
            DANDY_ASSERT(g_rgdwMWFSize[iRobot]   == 0);
            DANDY_ASSERT(g_rgdwEWFSize[iRobot]   == 0);
        }
        else
        {
            DANDY_ASSERT(g_rgdwCmdSize[iRobot]  >= MIN_CMD_COUNT && g_rgdwCmdSize[iRobot] <= MAX_CMD_COUNT);
            DANDY_ASSERT(g_rgdwTVarSize[iRobot] > 0 && g_rgdwTVarSize[iRobot] <= MAX_EMB_POS_COUNT);
            DANDY_ASSERT(g_rgdwPVarSize[iRobot] > 0 && g_rgdwPVarSize[iRobot] <= MAX_GLB_POS_COUNT);
            DANDY_ASSERT(g_rgdwBVarSize[iRobot] > 0 && g_rgdwBVarSize[iRobot] <= MAX_VAR_COUNT);
            DANDY_ASSERT(g_rgdwIVarSize[iRobot] > 0 && g_rgdwIVarSize[iRobot] <= MAX_VAR_COUNT);
            DANDY_ASSERT(g_rgdwRVarSize[iRobot] > 0 && g_rgdwRVarSize[iRobot] <= MAX_VAR_COUNT);
            DANDY_ASSERT(g_rgdwWeaveSize[iRobot]> 0 && g_rgdwWeaveSize[iRobot]<= MAX_WEAVE_COUNT);
            DANDY_ASSERT(g_rgdwSWFSize[iRobot]  > 0 && g_rgdwSWFSize[iRobot]  <= MAX_WELD_COND_COUNT);
            DANDY_ASSERT(g_rgdwMWFSize[iRobot]  > 0 && g_rgdwMWFSize[iRobot]  <= MAX_WELD_COND_COUNT);
            DANDY_ASSERT(g_rgdwEWFSize[iRobot]  > 0 && g_rgdwEWFSize[iRobot]  <= MAX_WELD_COND_COUNT);
        }

        ///////////////////////////////////////////////////
        // check welder duplication

        for (iWelder = 0; iWelder < g_rgnRobotWelderCount[iRobot]; iWelder++)
        {
            nWelder = g_rgnRobotWelderList[iRobot][iWelder];
            DANDY_ASSERT(nWelder >= 0 && nWelder < MAX_WELDER_COUNT);
            DANDY_ASSERT(g_rgWelderConfig[nWelder].fUsed);

            if (rgnWelderRobot[nWelder] != -1)
            {
                VERBOSE_ERROR("Robot %d : Welder %d is already assigned "
                              "to Robot %d\n",
                              iRobot+1, nWelder, rgnWelderRobot[nWelder]+1);
                return RESULT_ERROR;
            }

            rgnWelderRobot[nWelder] = iRobot;
        }
    } // end of for(iRobot)

    if (nUsedCount == 0)
        VERBOSE_ALERT("no robots are used\n");
    else
        VERBOSE_NOTIFY("total %d robots are used\n", nUsedCount);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigAxis()
//      - Check Corrections Axis Parameter

int SYSC_CheckConfigAxis(void)
{
    int iAxis;
    int nUsedCount;

    nUsedCount = 0;

    for (iAxis = 0; iAxis < MAX_AXIS_COUNT; iAxis++)
    {
        DANDY_ASSERT(g_rgszAxisName[iAxis][0] != 0);

        if (!g_rgfAxisUsed[iAxis])
            continue;

        nUsedCount++;

        if (g_rgnAxisType[iAxis] == AXISTYPE_NONE)
        {
            VERBOSE_ERROR("Axis %d : axis type was not specified\n",
                             iAxis);
            return RESULT_ERROR;
        }

        if (g_rgnAxisIndex[iAxis] < 0)
        {
            VERBOSE_ERROR("Axis %d : axis index was not specified\n",
                             iAxis);
            return RESULT_ERROR;
        }

        if (g_nMotorCount[iAxis] < 0)
        {
            VERBOSE_ERROR("Axis %d : motor count was not specified\n",
                             iAxis);
            return RESULT_ERROR;
        }

        DANDY_ASSERT(g_rgdbGearRatio[iAxis] >= 1.e-8);

        if (g_rgnAxisDirection[iAxis] != 1 && g_rgnAxisDirection[iAxis] != -1)
        {
            VERBOSE_ERROR("Axis %d : axis direction was not specified\n",
                             iAxis);
            return RESULT_ERROR;
        }
    }

    if (nUsedCount == 0)
        VERBOSE_ALERT("no axes are used\n");
    else
        VERBOSE_NOTIFY("total %d axes are used\n", nUsedCount);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigMotor()
//      - Check Corrections Motor Parameter

int SYSC_CheckConfigMotor(void)
{
    int iMotor, fUsed;
    int nUsedCount;
    
    nUsedCount = 0;

    for (iMotor = 0; iMotor < MAX_MOTOR_COUNT; iMotor++)
    {
        fUsed = (g_rgnMotorType[iMotor] != MOTTYPE_NONE);

        if (fUsed != TRUE)
            break;
        
        DANDY_ASSERT(g_rgszMotorName[iMotor][0] != 0);
        
        nUsedCount++;

        if (g_rgnMotorType[iMotor] == MOTTYPE_NONE)
        {
            VERBOSE_ERROR("Motor %d : motor type was not specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgnMotorIndex[iMotor] < 0)
        {
            VERBOSE_ERROR("Motor %d : motor index was not specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgnEncoderRes[iMotor] == 0)
        {
            VERBOSE_ERROR("Motor %d : Encoder resolution was not specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].jrk < 0 ||
            g_rgMotorConfig[iMotor].jrk > g_rgMotorConfig[iMotor].acc)
        {
            VERBOSE_ERROR("Motor %d : jerk value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].acc <= MIN_ACCDEC_VAL)
        {
            VERBOSE_ERROR("Motor %d : acc value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].dec <= MIN_ACCDEC_VAL)
        {
            VERBOSE_ERROR("Motor %d : dec value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].dec_error < 0)
        {
            VERBOSE_ERROR("Motor %d : error dec value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].dec_estop < 0)
        {
            VERBOSE_ERROR("Motor %d : estop dec value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgMotorConfig[iMotor].vellim_max < 0)
        {
            VERBOSE_ERROR("Motor %d : max velocity limit value was not valid or specified\n",
                             iMotor);
            return RESULT_ERROR;
        }

        if (g_rgdbMotorMaxVel[iMotor] < 1.e-8)
        {
            // just warning
            VERBOSE_WARNING("Motor %d : Max velocity is too little : %g\n",
                               iMotor, g_rgdbMotorMaxVel[iMotor]);
        }
    }

    if (nUsedCount == 0)
        VERBOSE_ALERT("no motors are used\n");
    else
        VERBOSE_NOTIFY("total %d motors are used\n", nUsedCount);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigWelder()
//      - Check Corrections Welder Parameter

int SYSC_CheckConfigWelder(void)
{
    int iWelder;
    int nUsedCount;

    nUsedCount = 0;

    for (iWelder = 0; iWelder < MAX_WELDER_COUNT; iWelder++)
    {
        DANDY_ASSERT(g_rgWelderConfig[iWelder].szName[0] != 0);

        if (!g_rgWelderConfig[iWelder].fUsed)
            continue;

        nUsedCount++;

        if (g_rgWelderConfig[iWelder].nType == WELDER_TYPE_NONE)
        {
            VERBOSE_ERROR("Welder %d : welder type was not specified\n",
                          iWelder);
            //return RESULT_ERROR;
        }

        if (g_rgWelderConfig[iWelder].nWelder != iWelder)
        {
            VERBOSE_ERROR("Welder %d : welder index was not specified : %d\n",
                          iWelder,
                          g_rgWelderConfig[iWelder].nWelder);
            //return RESULT_ERROR;
        }

        if(iWelder == 0 && g_rgWelderConfig[iWelder].nWelder != iWelder)
            break;
    }

    if (nUsedCount == 0)
        VERBOSE_ALERT("no welders are used\n");
    else
        VERBOSE_NOTIFY("total %d welders are used\n", nUsedCount);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_CheckConfigSensor()
//      - Check Corrections Sensor Parameter

int SYSC_CheckConfigSensor(void)
{
#if 0
    int iSensor;
    int nUsedCount;

    nUsedCount = 0;

    for (iSensor = 0; iSensor < MAX_SENSOR_COUNT; iSensor++)
    {
        DANDY_ASSERT(g_rgSensorConfig[iSensor].szName[0] != 0);

        if (!g_rgSensorConfig[iSensor].fUsed)
            continue;

        nUsedCount++;

        if (g_rgSensorConfig[iSensor].nType == SENSOR_TYPE_NONE)
        {
            VERBOSE_ERROR("Sensor %d : sensor type was not specified\n",
                             iSensor);
            return RESULT_ERROR;
        }
    }

    if (nUsedCount == 0)
        VERBOSE_ALERT("no sensors are used\n");
    else
        VERBOSE_NOTIFY("total %d sensors are used\n", nUsedCount);
#endif

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigGlobal()
//      - Cross Information Arrangement for Global Config

int SYSC_ArrangeConfigGlobal(void)
{
    if (g_nReqVersion < 0)
        g_nReqVersion = 0;

    // inet terminal port
    if (g_nTerminalInetPort == 0 || g_nTerminalInetPort == -1)
    {
        g_nTerminalInetPort = DEF_TERMINAL_INET_PORT;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Inet Terminal Port to default : %d\n",
                         g_nTerminalInetPort);
#endif
    }
    else if (g_nTerminalInetPort < 0)
    {
        g_nTerminalInetPort = -1;
    }

    // working directory
    if (g_pszWorkDir == NULL)
    {
        g_pszWorkDir = (char *)DEBUG_MALLOC(strlen("./")+1);
        CRT_strcpy(g_pszWorkDir, strlen("./")+1, "./");
    }

    // job directory
    if (g_pszJobDir == NULL)
    {
        g_pszJobDir = (char *)DEBUG_MALLOC(strlen("./job/")+1);
        memcpy(g_pszJobDir, "./job/", strlen("./job/")+1);
    }

    // sensor directory
    if (g_pszSensDir == NULL)
    {
        g_pszSensDir = (char *)DEBUG_MALLOC(strlen("./sdata/")+1);
        memcpy(g_pszSensDir, "./sdata/", strlen("./sdata/")+1);
    }

    // WireCut Job File Name
    if (g_pszWireCutJobFileName == NULL)
    {
        g_pszWireCutJobFileName = (char *)DEBUG_MALLOC(JOB_MODULE_NAME_SIZE);
        CRT_strcpy(g_pszWireCutJobFileName,
                   JOB_MODULE_NAME_SIZE,
                   "wirecut");
    }

    // Newly Created Job File Name
    if (g_pszNewCreatedJobFileName == NULL)
    {
        g_pszNewCreatedJobFileName = (char *)DEBUG_MALLOC(JOB_MODULE_NAME_SIZE);
        CRT_strcpy(g_pszNewCreatedJobFileName,
                   JOB_MODULE_NAME_SIZE,
                   "blank.pgm");
    }

    // EtherCAT Config directory
    if (g_pszEcatConfigDir == NULL)
    {
        g_pszEcatConfigDir = (char *)DEBUG_MALLOC(strlen(DEF_ECAT_CONF_FILE)+1);
        CRT_strcpy(g_pszEcatConfigDir,
                   strlen(DEF_ECAT_CONF_FILE)+1,
                   DEF_ECAT_CONF_FILE);
    }

    // Loaded Job File Name
    if (g_pszLoadedJobFileName == NULL)
    {
        g_pszLoadedJobFileName = (char *)DEBUG_MALLOC(JOB_MODULE_NAME_SIZE);
        CRT_strcpy(g_pszLoadedJobFileName,
                   JOB_MODULE_NAME_SIZE,
                   "");
        //g_fAseembleDone == FALSE;
    }

    // terminal connection greeting
    if (g_pszTerminalGreeting == NULL)
    {
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("None Terminal Greeting\n");
#endif
    }

    // locale
    if (g_pszLocale == NULL)
    {
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Keep Current Locale\n");
#endif
    }

    // terminal prompt
    if (g_pszPrompt == NULL)
    {
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Used default prompt\n");
#endif
    }

    if (g_nQNXTimerTick <= 0)
    {
        g_nQNXTimerTick = DEF_TIMER_TICK;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("QNX Timer Tick to default : %d [us]\n",
                        g_nQNXTimerTick);
#endif
    }

    if (g_nQNXTimerRes <= 0)
    {
        g_nQNXTimerRes = DEF_TIMER_RES;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("QNX Timer Resolution to default : %d [us]\n",
                        g_nQNXTimerRes);
#endif
    }

    if (g_nIoTime <= 0)
    {
        g_nIoTime = DEF_IO_TIME;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("IO Sample Time to default : %d [ms]\n",
                         g_nIoTime);
#endif
    }

    if (g_nTrajUpdateTime <= 0)
    {
        g_nTrajUpdateTime = DEF_TRAJ_TIME;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Trajectory Update Time to default : %d [ms]\n",
                        g_nTrajUpdateTime);
#endif
    }

    if (g_nServoInterpolationTime <= 0)
    {
#if defined(_WIN32)
        g_nServoInterpolationTime = 1;  //3;
#else
        g_nServoInterpolationTime = 1;
#endif
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Servo Interpolation time to default : %d\n",
                        g_nServoInterpolationTime);
#endif
    }

    if (g_nServoOnBrakeDelayTime < 0)
    {
        g_nServoOnBrakeDelayTime = DEF_SERVOON_DELAY;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Servo On Delay time to default : %d\n",
                        g_nServoOnBrakeDelayTime);
#endif
    }

    if (g_nServoOffBrakeDelayTime < 0)
    {
        g_nServoOffBrakeDelayTime = DEF_SERVOOFF_DELAY;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Servo Off Delay time to default : %d\n",
                        g_nServoOffBrakeDelayTime);
#endif
    }

    if (g_nEstopGasOffDelayTime < 0)
    {
        g_nEstopGasOffDelayTime = DEF_ESTOP_GASOFF_DELAY;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Estop Gas Off Delay time to default : %d\n",
                        g_nEstopGasOffDelayTime);
#endif
    }

    if (g_nEstopTouchReadyOffDelayTime < 0)
    {
        g_nEstopTouchReadyOffDelayTime = DEF_ESTOP_TOUCHREADY_DELAY;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Estop TouchReady Off Delay time to default : %d\n",
                        g_nEstopTouchReadyOffDelayTime);
#endif
    }

    if (g_nSlaveCount <= 0)
    {
        g_nSlaveCount = DEF_SLAVE_CNT;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Slave Count to default : %d [EA]\n",
                        g_nSlaveCount);
#endif
    }

    if (g_nErrHistorySaveMaxCnt <= 0)
    {
        g_nErrHistorySaveMaxCnt = DEF_ERRHIST_SAVE_MAX_CNT;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Error History Save Max Count to default : %d [EA]\n",
                        g_nSlaveCount);
#endif
    }

    if (g_nWriteOffsetSize < 0)
    {
        g_nWriteOffsetSize = DEF_WRITE_OFFSET_SIZE;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Write Offset Size to default : %d\n",
                        g_nWriteOffsetSize);
#endif
    }

    if (g_nReadOffsetSize < 0)
    {
        g_nReadOffsetSize = DEF_READ_OFFSET_SIZE;
#if defined(CONF_VERBOSE)
        VERBOSE_VERBOSE("Read Offset Size to default : %d\n",
                        g_nReadOffsetSize);
#endif
    }

    // Arrange Analog Input Max Volt
    if (g_dbAinMaxVolt <= 0.0)
    {
        g_dbAinMaxVolt = DEF_AIN_MAX_VOLT;
    }

    // Arrange Analog Input Convert Max Bit
    if (g_dbADCMaxBit <= 0.0)
    {
        g_dbADCMaxBit  = DEF_ADC_MAX_BIT;
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_AssignConfigRobotMotion()
//      - Cross Information Arrangement for Robot Motion Config

static void _loc_SYSC_AssignConfigRobotMotion(int nRobot)
{
    int iAxis;

    DANDY_ASSERT(nRobot >= 0 && nRobot < MAX_ROBOT_COUNT);

    // default max joint speed by axis
    for (iAxis = 0; iAxis < g_rgnRobotAxisCount[nRobot]; iAxis++)
    {
        if (g_rgRobotMotion[nRobot].dbMaxJointSpeed[iAxis] < 0)
        {
            g_rgRobotMotion[nRobot].dbMaxJointSpeed[iAxis] = DEF_MAX_JOINT_SPEED;
        }
    }

    // default max linear speed
    if (g_rgRobotMotion[nRobot].dbMaxLinearSpeed < 0)
    {
        g_rgRobotMotion[nRobot].dbMaxLinearSpeed = DEF_MAX_LINEAR_SPEED;
    }

    // default max orient speed
    if (g_rgRobotMotion[nRobot].dbMaxOrientSpeed < 0)
    {
        g_rgRobotMotion[nRobot].dbMaxOrientSpeed = DEF_MAX_ORIENT_SPEED;
    }

    // default jerk
    if (g_rgRobotMotion[nRobot].dbJerk < 0 ||
        g_rgRobotMotion[nRobot].dbJerk > g_rgRobotMotion[nRobot].dbAccel)
    {
        g_rgRobotMotion[nRobot].dbJerk = DEF_MOTION_JERK;
    }

    // default accel
    if (g_rgRobotMotion[nRobot].dbAccel == ACCEL_NONE)
    {
        g_rgRobotMotion[nRobot].dbAccel = DEF_ACCEL_TIME;
    }

    // default decel
    if (g_rgRobotMotion[nRobot].dbDecel == ACCEL_NONE)
    {
        g_rgRobotMotion[nRobot].dbDecel = DEF_DECEL_TIME;
    }

    // default stop
    if (g_rgRobotMotion[nRobot].dbDecel_Error == ACCEL_NONE)
    {
        g_rgRobotMotion[nRobot].dbDecel_Error = DEF_ERROR_STOP_DEC_TIME;
    }

    // default estop
    if (g_rgRobotMotion[nRobot].dbDecel_Estop == ACCEL_NONE)
    {
        g_rgRobotMotion[nRobot].dbDecel_Estop = DEF_ESTOP_DEC_TIME;
    }

    // default tstop
    if (g_rgRobotMotion[nRobot].dbDecel_Touch == ACCEL_NONE)
    {
        g_rgRobotMotion[nRobot].dbDecel_Touch = DEF_TSTOP_DEC_TIME;
    }

    if(g_nLeftVertSkipBvar <= 0)
    {
        g_nLeftVertSkipBvar = DEF_LEFT_VERT_SKIP_BVAR;
    }

    if(g_nRightVertSkipBvar <= 0)
    {
        g_nRightVertSkipBvar = DEF_RIGHT_VERT_SKIP_BVAR;
    }

    if(g_nLeftCollarSkipBvar <= 0)
    {
        g_nLeftCollarSkipBvar = DEF_LEFT_COLLAR_SKIP_BVAR;
    }

    if(g_nRightCollarSkipBvar <= 0)
    {
        g_nRightCollarSkipBvar = DEF_RIGHT_COLLAR_SKIP_BVAR;
    }

    if(g_nHorizontalSkipBvar <= 0)
    {
        g_nHorizontalSkipBvar = DEF_HORIZONTAL_SKIP_BVAR;
    }

    if(g_nLeftBracketSkipBvar <= 0)
    {
        g_nLeftBracketSkipBvar = DEF_LEFT_BRACKET_SKIP_BVAR;
    }

    if(g_nRightBracketSkipBvar <= 0)
    {
        g_nRightBracketSkipBvar = DEF_RIGHT_BRACKET_SKIP_BVAR;
    }
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigRobot()
//      - Cross Information Arrangement for Robot Config

int SYSC_ArrangeConfigRobot(void)
{
    int iRobot;
    int fUsed;

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        if (g_rgszRobotName[iRobot][0] == 0)
        {
            CRT_sprintf(g_rgszRobotName[iRobot], ROBOT_NAME_LEN,
                        "ROBOT%d", iRobot + 1);
        }

        fUsed = (g_rgnRobotType[iRobot] != ROBTYPE_NONE);

        g_rgfRobotUsed[iRobot] = fUsed;

        if (!fUsed || g_rgnRobotType[iRobot] == ROBTYPE_EXCEPT)
        {
            // clear all joint and axes info
            //              if unused robot or non-motional robot

            g_rgnRobotAxisCount[iRobot] = 0;
        }

        if (g_rgnRobotDefJogPercent[iRobot] <= 0)
        {
            g_rgnRobotDefJogPercent[iRobot] = DEF_JOG_PERCENT;
        }

        if (g_rgnRobotDefExtraJogPercent[iRobot] <= 0)
        {
            g_rgnRobotDefExtraJogPercent[iRobot] = DEF_EXJOG_PERCENT;
        }

        if (g_nHomeSpeed[iRobot] <= 0)
        {
            g_nHomeSpeed[iRobot] = DEF_HOME_SPEED;
        }

        // accel/decel/stop/estop/tstop
        _loc_SYSC_AssignConfigRobotMotion(iRobot);

        // job program
        if (g_fJobLoadDoneCheck == OFF)
        {
            if (g_rgdwCmdSize[iRobot] == 0)
                g_rgdwCmdSize[iRobot] = DEF_CMD_COUNT;

            if (g_rgdwTVarSize[iRobot] == 0)
                g_rgdwTVarSize[iRobot] = DEF_EMB_POS_COUNT;

            if (g_rgdwPVarSize[iRobot] == 0)
                g_rgdwPVarSize[iRobot] = DEF_GLB_POS_COUNT;

            if (g_rgdwBVarSize[iRobot] == 0)
                g_rgdwBVarSize[iRobot] = DEF_VAR_COUNT;

            if (g_rgdwIVarSize[iRobot] == 0)
                g_rgdwIVarSize[iRobot] = DEF_VAR_COUNT;

            if (g_rgdwRVarSize[iRobot] == 0)
                g_rgdwRVarSize[iRobot] = DEF_VAR_COUNT;

            if (g_rgdwWeaveSize[iRobot] == 0)
                g_rgdwWeaveSize[iRobot] = DEF_WEAVE_COUNT;

            if (g_rgdwSWFSize[iRobot] == 0)
                g_rgdwSWFSize[iRobot] = DEF_WELD_COND_COUNT;

            if (g_rgdwMWFSize[iRobot] == 0)
                g_rgdwMWFSize[iRobot] = DEF_WELD_COND_COUNT;

            if (g_rgdwEWFSize[iRobot] == 0)
                g_rgdwEWFSize[iRobot] = DEF_WELD_COND_COUNT;
        }
        else
        {
#if 0
            g_rgdwCmdSize[iRobot] = 0;
            g_rgdwTVarSize[iRobot] = 0;
            g_rgdwPVarSize[iRobot] = 0;
            g_rgdwBVarSize[iRobot] = 0;
            g_rgdwIVarSize[iRobot] = 0;
            g_rgdwRVarSize[iRobot] = 0;
            g_rgdwWeaveSize[iRobot] = 0;
            g_rgdwSWFSize[iRobot] = 0;
            g_rgdwMWFSize[iRobot] = 0;
            g_rgdwEWFSize[iRobot] = 0;
#endif
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigAxis()
//      - Cross Information Arrangement for Axis Config

int SYSC_ArrangeConfigAxis(void)
{
    int iRobot, iAxis;
    int nAxis;

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        if (!g_rgfRobotUsed[iRobot])
            continue;

        for (iAxis = 0; iAxis < g_rgnRobotAxisCount[iRobot]; iAxis++)
        {
            nAxis = g_rgRobotAxis[iRobot][iAxis].nMotorCount;
            DANDY_ASSERT(nAxis >= 0 && nAxis < MAX_AXIS_COUNT);

            if (g_rgfAxisUsed[nAxis])
            {
                VERBOSE_ERROR("Axis %d is already used before robot #%d\n",
                              nAxis, iRobot);
                return RESULT_ERROR;
            }

            g_rgfAxisUsed[nAxis] = 1;
        }
    }

    for (iAxis = 0; iAxis < MAX_AXIS_COUNT; iAxis++)
    {
        if (g_rgszAxisName[iAxis][0] == 0)
        {
            CRT_sprintf(g_rgszAxisName[iAxis], AXIS_NAME_LEN, "AXIS%d", iAxis);
        }

        if (g_rgnAxisIndex[iAxis] == -1)
        {
            g_rgnAxisIndex[iAxis] = iAxis + 1;
        }

        if (g_rgdbGearRatio[iAxis] < 1.e-8)
        {
            g_rgdbGearRatio[iAxis] = 1.;
        }

        if (g_rgnAxisDirection[iAxis] != 1 && g_rgnAxisDirection[iAxis] != -1)
        {
            g_rgnAxisDirection[iAxis] = 1;
        }

        if (g_rgnEcnoderHomeVal[iAxis] < 1.e-8)
        {
            g_rgnEcnoderHomeVal[iAxis] = 0;
        }

        if (g_nMotorCount[iAxis] < 1)
        {
            g_nMotorCount[iAxis] = 1;
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigMotor()
//      - Cross Information Arrangement for Motor Config

int SYSC_ArrangeConfigMotor(void)
{
    int iMotor;

    for (iMotor = 0; iMotor < MAX_MOTOR_COUNT; iMotor++)
    {
        if (g_rgszMotorName[iMotor][0] == 0)
        {
            CRT_sprintf(g_rgszMotorName[iMotor], MOTOR_NAME_LEN, "MOTOR%d", iMotor);
        }

        if (g_rgnMotorIndex[iMotor] == -1)
        {
            g_rgnMotorIndex[iMotor] = iMotor + 1;
        }

        if (g_rgdwAbsEncoderRes[iMotor] == 0)
        {
            g_rgdwAbsEncoderRes[iMotor] = 0x20000;   // 2^17
        }

        if (g_rgnEncoderRes[iMotor] == 0)
        {
            g_rgnEncoderRes[iMotor] = 0x20000;
        }

        if (g_rgMotorConfig[iMotor].jrk == 0)
        {
            g_rgMotorConfig[iMotor].jrk = 0;
        }

        if (g_rgMotorConfig[iMotor].acc == 0)
        {
            g_rgMotorConfig[iMotor].acc = DEF_ACCEL_TIME;
        }

        if (g_rgMotorConfig[iMotor].dec == 0)
        {
            g_rgMotorConfig[iMotor].dec = DEF_DECEL_TIME;
        }

        if (g_rgMotorConfig[iMotor].dec_error == 0)
        {
            g_rgMotorConfig[iMotor].dec_error = DEF_ERROR_STOP_DEC_TIME;
        }

        if (g_rgMotorConfig[iMotor].dec_estop == 0)
        {
            g_rgMotorConfig[iMotor].dec_estop = DEF_ESTOP_DEC_TIME;
        }

        if (g_rgMotorConfig[iMotor].vellim_max == 0)
        {
            g_rgMotorConfig[iMotor].vellim_max = DEF_MAX_MOTOR_SPEED;
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigMotor()
//      - Cross Information Arrangement for Welder Config

int SYSC_ArrangeConfigWelder(void)
{
    int iRobot, iWelderRobot;
    int iWelder;

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        if (!g_rgfRobotUsed[iRobot])
            continue;

        for (iWelderRobot = 0; iWelderRobot < g_rgnRobotWelderCount[iRobot]; iWelderRobot++)
        {
            iWelder = g_rgnRobotWelderList[iRobot][iWelderRobot];
            DANDY_ASSERT(iWelder >= 0 && iWelder < MAX_WELDER_COUNT);

            if (g_rgWelderConfig[iWelder].fUsed)
            {
                DANDY_ASSERT(g_rgWelderConfig[iWelder].nRobot >= 0 &&
                       g_rgWelderConfig[iWelder].nRobot < MAX_ROBOT_COUNT);

                VERBOSE_ERROR("Welder %d is already used before robot #%d\n",
                              iWelder, iRobot);
                return RESULT_ERROR;
            }

            if (g_rgWelderConfig[iWelder].nType == WELDER_TYPE_NONE)
            {
                VERBOSE_WARNING("Invalid welder %d is used by robot #%d\n",
                                iWelder, iRobot);
                return RESULT_ERROR;
            }

            g_rgWelderConfig[iWelder].fUsed = 1;
            g_rgWelderConfig[iWelder].nWelder = iWelder;
            g_rgWelderConfig[iWelder].nRobot = iRobot;
        }
    }

    // default name
    for (iWelder = 0; iWelder < MAX_WELDER_COUNT; iWelder++)
    {
        if (g_rgWelderConfig[iWelder].szName[0] == 0)
        {
            CRT_sprintf(g_rgWelderConfig[iWelder].szName, WELDER_NAME_LEN,
                        "WELDER%d", iWelder);
        }
    }

    if(g_dbVoltRealTimeCmdOffsetUnit < 0)
    {
        g_dbVoltRealTimeCmdOffsetUnit = DEF_VOLT_OFFSET_UNIT;
    }

    if(g_dbCurrRealTimeCmdOffsetUnit < 0)
    {
        g_dbCurrRealTimeCmdOffsetUnit = DEF_CURR_OFFSET_UNIT;
    }
#if 0
    if(g_dbUpperBoundY_Volt > DEF_VOLT_UPPER_Y)
    {
        g_dbUpperBoundY_Volt = DEF_VOLT_UPPER_Y;
    }

    if(g_dbLowerBoundY_Volt < DEF_VOLT_LOWER_Y)
    {
        g_dbLowerBoundY_Volt = DEF_VOLT_LOWER_Y;
    }
#else
    if(g_dbUpperBoundY_Volt > HYOSUNG_WELD_MAX_VOLT_VAL)
    {
        g_dbUpperBoundY_Volt = HYOSUNG_WELD_MAX_VOLT_VAL;
    }

    if(g_dbLowerBoundY_Volt < 0)
    {
        g_dbLowerBoundY_Volt = 0;
    }
#endif

    if(g_dbUpperBoundX_Volt > MAX_WELDOUT_VOLTAGE)
    {
        g_dbUpperBoundX_Volt = MAX_WELDOUT_VOLTAGE;
    }

    if(g_dbLowerBoundX_Volt < 0)
    {
        g_dbLowerBoundX_Volt = 0;
    }
#if 0
    if(g_dbUpperBoundY_Curr > DEF_CURR_UPPER_Y)
    {
        g_dbUpperBoundY_Curr = DEF_CURR_UPPER_Y;
    }

    if(g_dbLowerBoundY_Curr < DEF_CURR_LOWER_Y)
    {
        g_dbLowerBoundY_Curr = DEF_CURR_LOWER_Y;
    }
#else
    if(g_dbUpperBoundY_Curr > HYOSUNG_WELD_MAX_CURR_VAL)
    {
        g_dbUpperBoundY_Curr = HYOSUNG_WELD_MAX_CURR_VAL;
    }

    if(g_dbLowerBoundY_Curr < 0)
    {
        g_dbLowerBoundY_Curr = 0;
    }
#endif

    if(g_dbUpperBoundX_Curr > MAX_WELDOUT_VOLTAGE)
    {
        g_dbUpperBoundX_Curr = MAX_WELDOUT_VOLTAGE;
    }

    if(g_dbLowerBoundX_Curr < 0)
    {
        g_dbLowerBoundX_Curr = 0;
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ArrangeConfigSensor()
//      - Cross Information Arrangement for Sensor Config

/*
int SYSC_ArrangeConfigSensor(void)
{
    int iRobot, iSensorRobot;
    int iSensor;

    for (iRobot = 0; iRobot < MAX_ROBOT_COUNT; iRobot++)
    {
        if (!g_rgfRobotUsed[iRobot])
            continue;

        for (iSensorRobot = 0; iSensorRobot < g_rgnRobotSensorCount[iRobot]; iSensorRobot++)
        {
            iSensor = g_rgnRobotSensorList[iRobot][iSensorRobot];
            DANDY_ASSERT(iSensor >= 0 && iSensor < MAX_SENSOR_COUNT);

            if (g_rgSensorConfig[iSensor].fUsed)
            {
                DANDY_ASSERT(g_rgSensorConfig[iSensor].nRobot >= 0 &&
                       g_rgSensorConfig[iSensor].nRobot < MAX_ROBOT_COUNT);

                VERBOSE_ERROR2(SYSC_ArrangeConfigSensor,
                               "Sensor %d is already used before robot #%d\n",
                               iSensor, iRobot);
                return RESULT_ERROR;
            }

            if (g_rgSensorConfig[iSensor].nType == SENSOR_TYPE_NONE)
            {
                VERBOSE_WARNING2(SYSC_ArrangeConfigSensor,
                                 "Invalid Sensor %d is used by robot #%d\n",
                                 iSensor, iRobot);
                return RESULT_ERROR;
            }

            g_rgSensorConfig[iSensor].fUsed = 1;
            g_rgSensorConfig[iSensor].nRobot = iRobot;
        }
    }

    // default name
    for (iSensor = 0; iSensor < MAX_SENSOR_COUNT; iSensor++)
    {
        if (g_rgSensorConfig[iSensor].szName[0] == 0)
        {
            sprintf(g_rgSensorConfig[iSensor].szName, "SENSOR%d", iSensor);
        }

        if (g_rgSensorConfig[iSensor].nSampTime == 0)
            g_rgSensorConfig[iSensor].nSampTime = DEF_SENSOR_SAMP_TIME;

        if (g_rgSensorConfig[iSensor].nSensorJogRate == 0)
            g_rgSensorConfig[iSensor].nSampTime = DEF_SENSOR_JOG_RATE;
    }

    return 0;
}
*/
