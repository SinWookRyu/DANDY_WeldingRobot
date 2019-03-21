#ifndef __CONF_MGR_H__
#define __CONF_MGR_H__

///////////////////////////////////////////////////////////////////////////////
// configuration searching following sequence

// (1) find the file is defined in "DSME_ROBOT_CONFIG" environment variable
// (2) OS dependant configuration file
//     Windows : find "dsmert.win.conf" file
//     QNX     : find "dsmert.qnx.conf" file
//     -- If you want to and/insert the platform dependant configuration file,
//        Modify the function SYSC_LoadConfig() in param_mgr.c
// (3) find "dsmert.conf" file

#include <ctype.h>
#include <locale.h>
#include <time.h>
#include "dandy_platform.h"
#include "dandy_debug.h"
#include "sys_conf.h"

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#include <dirent.h>
#include <glob.h>
#include <sys/utsname.h>
#endif

#undef SYSC_VERBOSE      //for config file handling debugging
//#define SYSC_VERBOSE
#undef CONF_VERBOSE
//#undef _INIT_RETRY

/////////////////////////////////////////////////////////////////////////////

#define JOB_CACHE_FILE_NAME     "cachejob.bin"  // job cache file
#define HOME_FILE_NAME          "home.cfg"      // home position config file
#define COORD_FILE_NAME         "coord.cfg"     // coordination config file
#define WELD_COND_FILE_NAME     "weldcond.cfg"  // global welding condition config file
#define STATISTICS_FILE_NAME    "statisti.cfg"  // using time figure 

// job cache file writing info name
#define JOB_CACHE_DEF_NAME      "cached"        // cached file loading module name
#define JOB_CACHE_COMMENT_TAG   "[cached] "     // comment tag of cache file

#define CONF_PATHNAME_LEN           256
#define CONF_FILENAME_LEN           256

/////////////////////////////////////////////////////////////////////////////

#define CONFIG_ENV_NAME                     "DSME_ROBOT_CONFIG"
#define DEF_CONFIG_BASENAME                 "DANDY_II"
#define DEF_SYSCONFIG_FILENAME              "DANDY_II.conf"
#define DEF_USERPARAM_FILENAME              "DANDY_II.UserParam.conf"
#define DEF_STATISTIC_FILENAME              "DANDY_II.Statistics.dat"
#define DEF_RESTARTPARAM_FILENAME           "DANDY_II.RestartParam.bin"

#define DEF_CONST_PVAR_FILENAME             "DANDY_II.ConstPVar.bin"
#define DEF_CONST_BVAR_FILENAME             "DANDY_II.ConstBVar.bin"
#define DEF_CONST_IVAR_FILENAME             "DANDY_II.ConstIVar.bin"
#define DEF_CONST_RVAR_FILENAME             "DANDY_II.ConstRVar.bin"

#define OPTION_PVAR                         0
#define OPTION_BVAR                         1
#define OPTION_IVAR                         2
#define OPTION_RVAR                         3

#define OPTION_WRITE                        0
#define OPTION_READ                         1

#define OPTION_SYSCONFIG_LOAD               1
#define OPTION_USERPARAM_LOAD               2
#define OPTION_STATISTIC_LOAD               3

#define MIN_ACCDEC_VAL              0.001

///////////////////////////////////////////////////////////////////////////////
//
// [GLOBAL] configuration file key names
//

#define SYSCONF_KEY_VERSION                 "VERSION"

// terminal inet config
#define SYSCONF_KEY_TERMINAL_INET_PORT      "TERMINAL_INET_PORT"
#define SYSCONF_KEY_TERMINAL_UART_PORT      "TERMINAL_UART_PORT"

#define SYSCONF_KEY_WORK_DIR                "WORK_DIR"
#define SYSCONF_KEY_JOB_DIR                 "JOB_DIR"
#define SYSCONF_KEY_SENS_DIR                "SENS_DIR"
#define SYSCONF_KEY_WIRECUT_JOB_NAME        "WIRECUT_JOBNAME"
#define SYSCONF_KEY_NEW_CREATED_JOB_NAME    "NEW_JOBNAME"

#define SYSCONF_KEY_TERMINAL_GREETING       "TERMINAL_GREETING"
#define SYSCONF_KEY_LOCALE                  "LOCALE"
#define SYSCONF_KEY_PROMPT                  "PROMPT"

#define SYSCONF_KEY_QNXTIMER_TICK           "TIMER_TICK"
#define SYSCONF_KEY_QNXTIMER_RES            "TIMER_RES"
#define SYSCONF_KEY_IO_TIME                 "IO_TIME"
#define SYSCONF_KEY_TRAJ_UPDATE_TIME        "TRAJ_TIME"
#define SYSCONF_KEY_SRV_INTPOLATION_TIME    "INTP_TIME"

#define SYSCONF_KEY_ECAT_CONFIG_DIR         "ECAT_CONF_DIR"
#define SYSCONF_KEY_SLAVE_COUNT             "SLAVE_CNT"
#define SYSCONF_KEY_WRITE_OFFSET_SIZE       "WRITE_OFFSET_SIZE"
#define SYSCONF_KEY_READ_OFFSET_SIZE        "READ_OFFSET_SIZE"

#define SYSCONF_KEY_ERRHITORY_SAVE_COUNT    "ERRHIST_SAVE_CNT"

#define SYSCONF_KEY_ANALOG_IN_MAX_VOLT      "AIN_MAX_VOLT"
#define SYSCONF_KEY_AD_CONVERT_MAX_BIT      "ADC_MAX_BIT"

///////////////////////////////////////////////////////////////////////////////
//
// [ROBOT] configuration file key names
//

#define SYSCONF_KEY_ROBOT_NAME              "NAME"      // robot name
#define SYSCONF_KEY_ROBOT_TYPE              "TYPE"
#define SYSCONF_KEY_ROBOT_AXES              "AXES"      // 
#define SYSCONF_KEY_ROBOT_SYNC_MOTOR        "SYNC_MOTOR" // 

#define SYSCONF_KEY_ROBOT_LINK_th           "LINK_th"   // theta
#define SYSCONF_KEY_ROBOT_LINK_d            "LINK_d"    // d
#define SYSCONF_KEY_ROBOT_LINK_al           "LINK_al"   // alpha
#define SYSCONF_KEY_ROBOT_LINK_l            "LINK_l"    // l

#define SYSCONF_KEY_ROBOT_JOG_SPEED         "JOG_SPEED"
#define SYSCONF_KEY_ROBOT_EXJOG_SPEED       "EXJOG_SPEED"
#define SYSCONF_KEY_ROBOT_MAX_SPEED         "MAX_SPEED"
#define SYSCONF_KEY_ROBOT_MAX_ACCEL         "MAX_ACCEL"
#define SYSCONF_KEY_ROBOT_MAX_JOINT_SPEED   "MAX_JOINT_SPEED"

#define SYSCONF_KEY_ROBOT_JERK              "JERK"

#define SYSCONF_KEY_ROBOT_ACCEL             "ACCEL"
#define SYSCONF_KEY_ROBOT_DECEL             "STOP"
#define SYSCONF_KEY_ROBOT_ERROR_STOP        "ERROR_STOP"
#define SYSCONF_KEY_ROBOT_ESTOP             "ESTOP"
#define SYSCONF_KEY_ROBOT_TSTOP             "TSTOP"

#define SYSCONF_KEY_ROBOT_COORD_TCP         "TCP"
#define SYSCONF_KEY_ROBOT_COORD_WORLD       "WORLD"
#define SYSCONF_KEY_ROBOT_COORD_CART        "CART"
#define SYSCONF_KEY_ROBOT_COORD_USER        "USER"
#define SYSCONF_KEY_ROBOT_COORD_HOME        "HOME"
#define SYSCONF_KEY_LOADED_JOB_NAME         "JOB_NAME"

#define SYSCONF_KEY_WELD_TUNE_INDEX         "TUNE_APPLY_IDX"
#define SYSCONF_KEY_WELD_TUNE_VOLT_IN       "TUNE_VOLT_IN"
#define SYSCONF_KEY_WELD_TUNE_CURR_IN       "TUNE_CURR_IN"
#define SYSCONF_KEY_WELD_TUNE_VOLT_OUT      "TUNE_VOLT_OUT"
#define SYSCONF_KEY_WELD_TUNE_CURR_OUT      "TUNE_CURR_OUT"

#define SYSCONF_KEY_WELD_MEASURE_VAL        "WELD_MEASURE_VAL"

#define SYSCONF_KEY_ROBOT_RESTART_INFO      "RESTART_INFO"
#define SYSCONF_KEY_ARCSENSOR_SAVENODE      "SAVE_NODE"

#define SYSCONF_KEY_ROBOT_HOME_SPEED        "HOME_SPEED"

#define SYSCONF_KEY_ROBOT_WELDERS           "WELDERS"   // welder list included in the robot

#define SYSCONF_KEY_ROBOT_CMD_SIZE          "CMD_SIZE"
#define SYSCONF_KEY_ROBOT_TVA_SIZE          "TVA_SIZE"
#define SYSCONF_KEY_ROBOT_PVA_SIZE          "PVA_SIZE"
#define SYSCONF_KEY_ROBOT_BVA_SIZE          "BVA_SIZE"
#define SYSCONF_KEY_ROBOT_IVA_SIZE          "IVA_SIZE"
#define SYSCONF_KEY_ROBOT_RVA_SIZE          "RVA_SIZE"
#define SYSCONF_KEY_ROBOT_WVF_SIZE          "WVF_SIZE"
#define SYSCONF_KEY_ROBOT_SWF_SIZE          "SWF_SIZE"
#define SYSCONF_KEY_ROBOT_MWF_SIZE          "MWF_SIZE"
#define SYSCONF_KEY_ROBOT_EWF_SIZE          "EWF_SIZE"

#define SYSCONF_KEY_GAP_REF_BVAR_USED       "GAP_REF_BVAR_USED"
#define SYSCONF_KEY_GAP_REF_BVAR            "GAP_REF_BVAR"
#define SYSCONF_KEY_LEFT_WELD_BVAR          "LEFT_WELD_BVAR"
#define SYSCONF_KEY_RIGHT_WELD_BVAR         "RIGHT_WELD_BVAR"

#define SYSCONF_KEY_LEFT_VERT_SKIP_REF_BVAR     "LEFT_VERT_SKIP_REF_BVAR"
#define SYSCONF_KEY_RIGHT_VERT_SKIP_REF_BVAR    "RIGHT_VERT_SKIP_REF_BVAR"
#define SYSCONF_KEY_LEFT_COLLAR_SKIP_REF_BVAR   "LEFT_COLLAR_SKIP_REF_BVAR"
#define SYSCONF_KEY_RIGHT_COLLAR_SKIP_REF_BVAR  "RIGHT_COLLAR_SKIP_REF_BVAR"
#define SYSCONF_KEY_HORIZONTAL_SKIP_REF_BVAR    "HORIZONTAL_SKIP_REF_BVAR"
#define SYSCONF_KEY_LEFT_BRACKET_SKIP_REF_BVAR  "LEFT_BRACKET_SKIP_REF_BVAR"
#define SYSCONF_KEY_RIGHT_BRACKET_SKIP_REF_BVAR "RIGHT_BRACKET_SKIP_REF_BVAR"

#define SYSCONF_KEY_CWEAV_TOUCHUPDIS        "CWEAV_TOUCHUPDIS"
#define SYSCONF_KEY_CWEAV_HORMARGIN         "CWEAV_HORMARGIN"
#define SYSCONF_KEY_CWEAV_WELDLEGDIS        "CWEAV_WELDLEGDIS"

#define SYSCONF_KEY_SERVOON_BRAKE_DELAY     "SERVOON_DELAY"
#define SYSCONF_KEY_SERVOOFF_BRAKE_DELAY    "SERVOOFF_DELAY"

#define SYSCONF_KEY_ESTOP_GASOFF_DELAY          "ESTOP_GASOFF_DELAY"
#define SYSCONF_KEY_ESTOP_TOUCHREADYOFF_DELAY   "ESTOP_TOUCHREADYOFF_DELAY"

#define SYSCONF_KEY_MASTER_RESET_HOURS      "MASTER_RESET_HOURS"
#define SYSCONF_KEY_MASTER_RESET_MIN        "MASTER_RESET_MIN"

///////////////////////////////////////////////////////////////////////////////
//
// [AXIS] configuration file key names
//

#define SYSCONF_KEY_AXIS_NAME               "NAME"      // axis's name
#define SYSCONF_KEY_AXIS_TYPE               "TYPE"
#define SYSCONF_KEY_AXIS_ID                 "ID"
#define SYSCONF_KEY_AXIS_COMM_PORT          "COMM_PORT"

#define SYSCONF_KEY_AXIS_HW_LIMIT           "HW_LIMIT"
#define SYSCONF_KEY_AXIS_SW_LIMIT           "SW_LIMIT"

#define SYSCONF_KEY_AXIS_GEAR_RATIO         "GEAR_RATIO"
#define SYSCONF_KEY_AXIS_TERM_DIST          "TERM_DIST"
#define SYSCONF_KEY_AXIS_DIRECTION          "DIRECTION"
#define SYSCONF_KEY_AXIS_MOTOR_COUNT        "MOTOR_CNT"

#define SYSCONF_KEY_MOTOR_ENCODER_ORIGIN    "ENCODER_ORIGIN"

///////////////////////////////////////////////////////////////////////////////
//
// [MOTOR] configuration file key names
//
#define SYSCONF_KEY_MOTOR_NAME              "NAME"      // axis's name
#define SYSCONF_KEY_MOTOR_TYPE              "TYPE"
#define SYSCONF_KEY_MOTOR_ID                "ID"

#define SYSCONF_KEY_MOTOR_ACCEL             "ACCEL"
#define SYSCONF_KEY_MOTOR_DECEL             "STOP"
#define SYSCONF_KEY_MOTOR_JERK              "JERK"

#define SYSCONF_KEY_MOTOR_ERROR_STOP        "ERROR_STOP"
#define SYSCONF_KEY_MOTOR_ESTOP             "ESTOP"

#define SYSCONF_KEY_MOTOR_ENCODER_RES       "ENCODER_RES"
#define SYSCONF_KEY_MOTOR_ABS_ENCODER_RES   "ABS_ENCODER_RES"

#define SYSCONF_KEY_MOTOR_ENCODER_TYPE      "ENCODER_TYPE"
#define SYSCONF_KEY_MOTOR_HW_HOME           "HW_HOME"

#define SYSCONF_KEY_MOTOR_MAX_VEL           "MAX_VEL"
#define SYSCONF_KEY_MOTOR_MAX_ACCEL         "MAX_ACCEL"

///////////////////////////////////////////////////////////////////////////////
//
// [WELDER] configuration file key names
//

#define SYSCONF_KEY_WELDER_NAME              "NAME"      // welder's name
#define SYSCONF_KEY_WELDER_TYPE              "TYPE"      // welder's type (Hyosung_UR, Daihen_DM, Zeus)
#define SYSCONF_KEY_WELDER_ABILITY           "ABILITY"
#define SYSCONF_KEY_WELDER_INPORT_NO         "INPORT_NO"
#define SYSCONF_KEY_WELDER_OUTPORT_NO        "OUTPORT_NO"
#define SYSCONF_KEY_WELDER_INPORT_LEVEL      "INPORT_LEVEL"
#define SYSCONF_KEY_WELDER_OUTPORT_LEVEL     "OUTPORT_LEVEL"

#define SYSCONF_KEY_WELDER_VOLT_IN           "VoltIn"
#define SYSCONF_KEY_WELDER_CURR_IN           "CurrIn"
#define SYSCONF_KEY_WELDER_VOLT_OUT          "VoltOut"
#define SYSCONF_KEY_WELDER_CURR_OUT          "CurrOut"

#define SYSCONF_KEY_WELDER_VOLT_OFFSET_UNIT  "VOLT_OFFSET_UNIT"
#define SYSCONF_KEY_WELDER_CURR_OFFSET_UNIT  "CURR_OFFSET_UNIT"

#define SYSCONF_KEY_WELD_VOLT_CALIB_UPPER_Y  "VOLT_CALIB_UPPER_Y"
#define SYSCONF_KEY_WELD_VOLT_CALIB_LOWER_Y  "VOLT_CALIB_LOWER_Y"
#define SYSCONF_KEY_WELD_VOLT_CALIB_UPPER_X  "VOLT_CALIB_UPPER_X"
#define SYSCONF_KEY_WELD_VOLT_CALIB_LOWER_X  "VOLT_CALIB_LOWER_X"

#define SYSCONF_KEY_WELD_CURR_CALIB_UPPER_Y  "CURR_CALIB_UPPER_Y"
#define SYSCONF_KEY_WELD_CURR_CALIB_LOWER_Y  "CURR_CALIB_LOWER_Y"
#define SYSCONF_KEY_WELD_CURR_CALIB_UPPER_X  "CURR_CALIB_UPPER_X"
#define SYSCONF_KEY_WELD_CURR_CALIB_LOWER_X  "CURR_CALIB_LOWER_X"

// welding global
#define SYSCONF_KEY_WELDER_WELDMAP          "WELDMAP"

///////////////////////////////////////////////////////////////////////////////
// Set default config value
///////////////////////////////////////

// min/max trajectory time
#define MAX_TRJ_TIME                500     // 500 [ms]
#if defined(_WIN32)
#define MIN_TRJ_TIME                1       // Windows : 2 or 1 [ms]
#elif defined(__QNXNTO__)
#define MIN_TRJ_TIME                1       // QNX : 1 [ms]
#else
#define MIN_TRJ_TIME                2       // Other : 2 [ms]
#endif

#define MAX_MASTER_RESET_CNT_DAY    5

#define DEF_TIMER_TICK              1       // [ms]
#define DEF_TIMER_RES               100     // [us]
#define DEF_IO_TIME                 1       // [ms]
#define DEF_TRAJ_TIME               1       // [ms]

#define DEF_SERVOON_DELAY           70      // [ms]
#define DEF_SERVOOFF_DELAY          150     // [ms]

#define DEF_ESTOP_GASOFF_DELAY      3000    // [ms]
#define DEF_ESTOP_TOUCHREADY_DELAY  500     // [ms]

#define DEF_ECAT_CONF_FILE          "/root/ethercat/dandy_master_140120_Kon.xml"
#define DEF_SLAVE_CNT               11      // EA
#define DEF_WRITE_OFFSET_SIZE       0X28    //
#define DEF_READ_OFFSET_SIZE        0X38    //

#define DEF_ERRHIST_SAVE_MAX_CNT    256    // EA

#define DEF_AIN_MAX_VOLT            10.0
#define DEF_ADC_MAX_BIT             32767.0

#define DEF_QNX_WORK_DIR_NAME       "/works/"
#define DEF_QNX_JOB_DIR_NAME        "/works/job/"

#define DEF_WIN_WORK_DIR_NAME       "./"
#define DEF_WIN_JOB_DIR_NAME        "./job/"
#define DEF_WIN_SENS_DIR_NAME       "./sdata/"

// acceleration type
#define ACCEL_NONE                  0
#define ACCEL_TRAPEZOIDAL           1
#define ACCEL_SCURVE                2
#define ACCEL_STEP                  99      // currently unused

#define DEF_TERMINAL_INET_PORT      2016

#define DEF_MOTION_JERK             10      // jerk 10 %
#define DEF_JOG_PERCENT             10
#define DEF_EXJOG_PERCENT           2
#define DEF_MAX_JOINT_SPEED         0.72    //rad/ms
#define DEF_MAX_LINEAR_SPEED        1       //mm/ms(m/s)
#define DEF_MAX_ORIENT_SPEED        0.36    //rad/ms

#define DEF_MAX_MOTOR_SPEED         1.44    //rad/ms

#define DEF_HOME_SPEED              30      // [%]

// accel
#define DEF_ACCEL_PATTERN           ACCEL_SCURVE
#define DEF_ACCEL_TIME              400     // [ms]

// decel
#define DEF_DECEL_PATTERN           ACCEL_SCURVE
#define DEF_DECEL_TIME              400     // [ms]

// error stop
#define DEF_STOP_PATTERN            ACCEL_SCURVE
#define DEF_ERROR_STOP_DEC_TIME     100     // [ms]

// estop
#define DEF_ESTOP_PATTERN           ACCEL_SCURVE
#define DEF_ESTOP_DEC_TIME          300    // [ms]

// touch stop
#define DEF_TSTOP_PATTERN           ACCEL_SCURVE
#define DEF_TSTOP_DEC_TIME          300    // [ms]

// sensor default values
#define DEF_SENSOR_SAMP_TIME        100     // generic sensor sampling time
#define DEF_SENSOR_JOG_RATE         50      // 50 % (half) jog speed compare to normal jog

// application sensor default
#define DEF_APP_SENSOR_SAMP_TIME    50      // default application sensor sampling time

// welding curr/volt adj speed
#define DEF_WELD_CURR_ADJ_SPEED     100     // [A/sec]
#define DEF_WELD_VOLT_ADJ_SPEED     5       // [V/sec]

// sync response expire time (timeout)
#define DEF_SYNC_EXPIRE             200     // [ms]

#define DEF_VOLT_OFFSET_UNIT        0.1     // [V]
#define DEF_CURR_OFFSET_UNIT        10     // [V]

#define DEF_LEFT_VERT_SKIP_BVAR     10
#define DEF_RIGHT_VERT_SKIP_BVAR    20
#define DEF_LEFT_COLLAR_SKIP_BVAR   30
#define DEF_RIGHT_COLLAR_SKIP_BVAR  40
#define DEF_HORIZONTAL_SKIP_BVAR    50
#define DEF_LEFT_BRACKET_SKIP_BVAR  70
#define DEF_RIGHT_BRACKET_SKIP_BVAR 80

// weld output config
#define DEF_VOLT_UPPER_Y            38.0
#define DEF_VOLT_LOWER_Y            20.0
#define DEF_CURR_UPPER_Y            400.0
#define DEF_CURR_LOWER_Y            180.0

#define HYOSUNG_WELD_MAX_VOLT_VAL   45       // 14 ~ 45 [V] // 42?
#define HYOSUNG_WELD_MAX_CURR_VAL   500      // 50 ~ 500 [A]
#define MAX_WELDOUT_VOLTAGE         10 

#define DEF_WELD_SCALE              0.0003052
#define DEF_WELD_OFFSET             0.0

///////////////////////////////////////////////////////////////////////////////
//
//  CONFIGURATION VARIABLES
//

///////////////////////////////////////
// global

extern int g_nReqVersion;
extern int g_nTerminalInetPort;

extern char* g_pszWorkDir;
extern char* g_pszJobDir;
extern char* g_pszSensDir;
extern char* g_pszWireCutJobFileName;
extern char* g_pszNewCreatedJobFileName;
extern char* g_pszConfigDir;
extern char* g_pszEcatConfigDir;
extern char* g_pszLoadedJobFileName;

extern char* g_pszTerminalGreeting;
extern char* g_pszLocale;
extern char* g_pszPrompt;

extern int g_nQNXTimerTick;           // [ms] unit, QNX Timer Tick
extern int g_nQNXTimerRes;            // [us] unit, QNX Timer Resolution
extern int g_nIoTime;                 // [ms] unit, Io Sample Time
extern int g_nTrajUpdateTime;         // [ms] unit, Traj Update Time
extern int g_nServoInterpolationTime; // [ms] unit, Servo Interpolation Time

extern int g_nServoOnBrakeDelayTime;  // [ms] unit, defailt = 70 ms
extern int g_nServoOffBrakeDelayTime; // [ms] unit, defailt = 150 ms

extern int g_nEstopGasOffDelayTime;         // [ms] unit, defailt = 3000 ms
extern int g_nEstopTouchReadyOffDelayTime;  // [ms] unit, defailt = 500 ms

extern int g_rgnEcatMasterResetHours[MAX_MASTER_RESET_CNT_DAY];
extern int g_nEcatMasterResetHoursCount;
extern int g_nEcatMasterResetMin;

extern int g_nSlaveCount;               // Slave Count
extern int g_nWriteOffsetSize;          // BeckHoff output size
extern int g_nReadOffsetSize;           // BeckHoff input size

extern int g_nErrHistorySaveMaxCnt;     // Error History Save Max Count

extern CONFIG_CTRL         g_nControlParam;
extern CONFIG_ECAT         g_nECATParam;
//extern SHM_RM_SYSCONFIG*   g_pShm_SysConfig;

extern double   g_dbAinMaxVolt;
extern double   g_dbADCMaxBit;

extern  int g_fConfigLoadCheck;

///////////////////////////////////////
// robot

extern  int    g_rgfRobotUsed[MAX_ROBOT_COUNT];
extern  char   g_rgszRobotName[MAX_ROBOT_COUNT][ROBOT_NAME_LEN];
extern  int    g_rgnRobotType[MAX_ROBOT_COUNT];

extern  int    g_rgnCommProtocol[MAX_ROBOT_COUNT];     // comm protocol
extern  char   g_rgszCommPort[MAX_ROBOT_COUNT][MAX_COMM_COUNT];

extern  int    g_rgnRobotAxisCount[MAX_ROBOT_COUNT];
extern CONFIG_AXIS g_rgRobotAxis[MAX_ROBOT_COUNT][ROB_AXIS_COUNT];

extern  int    g_rgnRobotDefJogPercent[MAX_ROBOT_COUNT];
extern  int    g_rgnRobotDefExtraJogPercent[MAX_ROBOT_COUNT];

extern  DH_PARAMS g_rgRobotDHParam[MAX_ROBOT_COUNT][ROB_AXIS_COUNT];
extern  CONFIG_ROBOT g_rgRobotMotion[MAX_ROBOT_COUNT];

extern  CONFIG_ROBOT g_rgCoordInfo[MAX_ROBOT_COUNT];

extern  double g_rgdbHomePosVal[MAX_HOME_COUNT][ROB_AXIS_COUNT];    // Unit: deg

extern  int     g_nHomeSpeed[MAX_ROBOT_COUNT];

extern  COORD_EULER    g_CartOffset;

extern  unsigned long  g_rgdwCmdSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwTVarSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwPVarSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwBVarSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwIVarSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwRVarSize[MAX_ROBOT_COUNT];

extern  unsigned long  g_rgdwWeaveSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwSWFSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwMWFSize[MAX_ROBOT_COUNT];
extern  unsigned long  g_rgdwEWFSize[MAX_ROBOT_COUNT];

extern  int    g_rgnRobotWelderCount[MAX_ROBOT_COUNT];
extern  int    g_rgnRobotWelderList[MAX_ROBOT_COUNT][MAX_WELDER_COUNT];

extern CONFIG_ROBOT*       g_pRobot; 
extern CONFIG_WELD_FUNC    g_Weld_Function[MAX_ROBOT_COUNT];

extern  TE_RESTART_PARAM  g_rgRestartParam[MAX_ROBOT_COUNT];
extern  ARCSENSOR_PARAM   g_rgArcSensorParam[MAX_ROBOT_COUNT];

extern  int g_nLeftVertSkipBvar;   
extern  int g_nRightVertSkipBvar;  
extern  int g_nLeftCollarSkipBvar; 
extern  int g_nRightCollarSkipBvar;
extern  int g_nHorizontalSkipBvar; 
extern  int g_nLeftBracketSkipBvar;
extern  int g_nRightBracketSkipBvar;

///////////////////////////////////////
// axis

extern  int    g_rgfAxisUsed[MAX_AXIS_COUNT];
extern  char   g_rgszAxisName[MAX_AXIS_COUNT][AXIS_NAME_LEN];  // axis's name
extern  int    g_rgnAxisType[MAX_AXIS_COUNT];         // axis's type
extern  int    g_rgnAxisIndex[MAX_AXIS_COUNT];        // axis's id

extern  int    g_rgfHwLimitUsed[MAX_AXIS_COUNT][2];    // HW limit used?
extern  double g_rgdbHwLimit[MAX_AXIS_COUNT][2];       // HW limit
extern  int    g_rgfSwLimitUsed[MAX_AXIS_COUNT][2];    // SW limit used?
extern  double g_rgdbSwLimit[MAX_AXIS_COUNT][2];       // SW limit

extern  double g_rgdbGearRatio[MAX_AXIS_COUNT];        // reduction gear ratio
extern  double g_rgdbRotaionDist[MAX_AXIS_COUNT];      // axis's terminal rataion distance
extern  int    g_rgnAxisDirection[MAX_AXIS_COUNT];     // axis's direction
extern  int    g_rgnEncoderResetVal[MAX_AXIS_COUNT];   // INT_MIN = unused
extern  int    g_rgnEcnoderHomeVal[MAX_AXIS_COUNT];    // INT_MIN = unused

extern  int    g_nMotorCount[MAX_AXIS_COUNT];          // Motor count include in axis

extern CONFIG_AXIS*        g_pAxis;

///////////////////////////////////////
// motor

extern  char   g_rgszMotorName[MAX_MOTOR_COUNT][MOTOR_NAME_LEN];  // motor name 
extern  int    g_rgnMotorType[MAX_MOTOR_COUNT];         // motor type
extern  int    g_rgnMotorIndex[MAX_MOTOR_COUNT];         // motor id

extern  int    g_rgnEncoderRes[MAX_MOTOR_COUNT];        // axis's encoder resolution
extern  DWORD  g_rgdwAbsEncoderRes[MAX_MOTOR_COUNT];    // axis's ABS encoder resolution

extern  int    g_rgnEncoderType[MAX_MOTOR_COUNT];       // encoder type (ABS | INC)
extern  int    g_rgnHwHome[MAX_MOTOR_COUNT];            // bit, negtive value is unused

extern  double g_rgdbMotorMaxVel[MAX_MOTOR_COUNT];
extern  double g_rgdbMotorMaxAccel[MAX_MOTOR_COUNT];

extern  CONFIG_MOTOR g_rgMotorConfig[MAX_MOTOR_COUNT];

extern CONFIG_MOTOR*       g_pMotor;

///////////////////////////////////////
// welder

extern  CONFIG_WELDER    g_rgWelderConfig[MAX_WELDER_COUNT];
extern  WELD_PARAM_TUNE  g_rgWeldTuneParam[MAX_WELDER_COUNT];

extern  int g_nWeldTuneInParamApplyIndex;
extern  int g_nWeldTuneOutParamApplyIndex;
 
extern  double  g_dbVoltRealTimeCmdOffsetUnit;
extern  double  g_dbCurrRealTimeCmdOffsetUnit;

extern  double  g_dbUpperBoundY_Volt;
extern  double  g_dbLowerBoundY_Volt;
extern  double  g_dbUpperBoundX_Volt;
extern  double  g_dbLowerBoundX_Volt;
extern  double  g_dbUpperBoundY_Curr;
extern  double  g_dbLowerBoundY_Curr;
extern  double  g_dbUpperBoundX_Curr;
extern  double  g_dbLowerBoundX_Curr;

extern  double  g_dbControllerCmdVolt[MAX_WELDER_COUNT];
extern  double  g_dbWelderMeasureVolt[MAX_WELDER_COUNT];
extern  double  g_dbControllerCmdCurr[MAX_WELDER_COUNT];
extern  double  g_dbWelderMeasureCurr[MAX_WELDER_COUNT];

///////////////////////////////////////////////////////////////////////////////
//
//  CONFIGURATION INTREFACE
//

#define GET_INET_TERMINAL_PORT()    (g_nTerminalInetPort)

#define GET_WORK_DIR()              (g_pszWorkDir)
#define GET_JOB_DIR()               (g_pszJobDir)
#define GET_SENS_DIR()              (g_pszSensDir)

#define GET_TERMINAL_GREETING()     (g_pszTerminalGreeting)
#define GET_SYSTEM_LOCALE()         (g_pszLocale)
#define GET_TERMINAL_PROMPT()       (g_pszPrompt)

///////////////////////////////////////////////////////////////////////////////

extern  int SYSC_LoadConfigGlobal(const char* pszKey, const char* pszValue);
extern  int SYSC_LoadConfigRobot(int nRobot, const char* pszKey, const char* pszValue);
extern  int SYSC_LoadConfigAxis(int nAxis, const char* pszKey, const char* pszValue);
extern  int SYSC_LoadConfigMotor(int nMotor, const char* pszKey, const char* pszValue);
extern  int SYSC_LoadConfigWelder(int nWelder, const char* pszKey, const char* pszValue);
extern  int SYSC_LoadConfigRobotParameter(int nRobot, const char* pszKey, const char* pszValue);
extern  int SYSC_LoadSystemStatistics(int nRobot, const char* pszKey, const char* pszValue);

extern  void    SYSC_ClearConfigGloabl  (void);
extern  void    SYSC_ClearConfigRobot   (void);
extern  void    SYSC_ClearConfigAxis    (void);
extern  void    SYSC_ClearConfigMotor   (void);
extern  void    SYSC_ClearConfigWelder  (void);
extern  void    SYSC_ClearConfig        (void);

extern  int     SYSC_CheckConfigGlobal  (void);
extern  int     SYSC_CheckConfigRobot   (void);
extern  int     SYSC_CheckConfigAxis    (void);
extern  int     SYSC_CheckConfigMotor   (void);
extern  int     SYSC_CheckConfigWelder  (void);

extern  int     SYSC_ArrangeConfigGlobal(void);
extern  int     SYSC_ArrangeConfigRobot (void);
extern  int     SYSC_ArrangeConfigAxis  (void);
extern  int     SYSC_ArrangeConfigMotor (void);
extern  int     SYSC_ArrangeConfigWelder(void);

extern  int     SYSC_LoadSysIoStyle     (CONFIG_IO* pSysIo);
extern  void    SYSC_SetSysIoStyle      (int nStyle);

extern  int     SYSC_LoadConfig         (const char* pszConfigFileName, int nOption);
extern  int     SYSC_LoadConfigDefault  (void);

extern  int     SYSC_ResetRobotInfo     (int nCount,
                                         const char* const rgpszRobotType[],
                                         const int rgnStartAxis[]);
extern  int     SYSC_ResetWelderInfo    (int nCount, const int rgnWelderRobot[],
                                         const char* const rgpszWelderType[],
                                         const int rgnWelderAddr[]);

extern  void    SYSC_SetTrajUpdateTime  (int nTrajTime);
extern  void    SYSC_SetIoUpdateTime    (int nIoTime);

extern  int     SYSC_LoadConfigToShmem  (SHM_RM_SYSCONFIG* pSystemConfig);
extern  int     SYSC_LoadParamToShmem   (SHM_RM_SYSPARAM*  pSystemParam);

// locale
extern  int     SYSC_ExecLocale         (void);

// end of system parameter related

///////////////////////////////////////////////////////////////////////////////

// config file related
extern  char*   PARAM_EncodeURL         (const char* lpszStr, char* lpszURL,
                                         int nLen, int fFullEncode);

extern  int     PARAM_DecodeURL         (const char* lpszURL,
                                         const char** lpszNextURL);
extern  char*   CONF_ParseURLEx         (const char* lpszURL,
                                         char* lpszName, int* lpnNameLen,
                                         char* lpszValue, int* lpnValueLen,
                                         int fDecodeURL, int chStop1, int chStop2);
extern  char*   PARAM_ParseURLAllocEx   (const char* lpszURL,
                                         char** lpszName, char** lpszValue,
                                         int fDecodeURL, int chStop1, int chStop2);
extern  int     PARAM_ReadConfigLine    (FILE* fp,
                                         char* pszBuffer, int nBufferLen,
                                         char* pszKey, int nKeyLen,
                                         char* pszValue, int nValueLen,
                                         int fDecodeURL);
extern  int     CONFIG_FindConfigValue  (const char* pszFileName,
                                         const char* lpszKey,
                                         char* lpszValue, int nValueLen,
                                         int fDecodeURL);

// config type
#define CONFIG_TYPE_ERROR       -1
#define CONFIG_TYPE_NONE        0
#define CONFIG_TYPE_SECTION     1
#define CONFIG_TYPE_KEY         2
#define CONFIG_TYPE_EOF         3
#define CONFIG_TYPE_COMMENT     4

typedef int (*CONFIG_FILE_ENUM_PROC)(int nCallCount, void* pParam, int nConfigType,
                                const char* pszKey, const char* pszValue);
int     CONF_EnumConfigFile     (const char* lpszFileName, int fDecodeURL,
                                 CONFIG_FILE_ENUM_PROC lpProc, void* lpParam);

#define CONFIG_ENUM_OK      0       //
#define CONFIG_ENUM_NO_FILE -1      // config file I/O error
#define CONFIG_ENUM_SYNTAX  -2      // syntax error
#define CONFIG_ENUM_RANGE   -3      // range over
#define CONFIG_ENUM_VALUE   -4      // value over

// just value conversion
#define CONFIG_NUM_ERROR    -1
#define CONFIG_NUM_INT      0
#define CONFIG_NUM_FLOAT    1
#define CONFIG_NUM_OTHER    2

int     PARAM_ConvNum           (const char* pszLiteral, int* pnValue, double* pdbValue);
char*   PARAM_ParseArrayConfig  (const char* pszValue, char* pszBuffer, int nBufferLen);
char*   PARAM_ParseArrayTimeData(const char* pszValue, char* pszBuffer, int nBufferLen);
int     PARAM_ConvArrayOneNum   (const char* pszLiteral, int* pnValue, double* pdbValue,
                                 const char** ppszNext);
int     PARAM_ConvArrayNumInt    (const char* pszArrayStr, int rgn[], int nCount);
int     PARAM_ConvArrayNumFloat  (const char* pszArrayStr, double rgdb[], int nCount);
int     PARAM_ConvArrayNumFloatTime  (const char* pszArrayStr, double rgdb[], int nCount);

// end of config file related

///////////////////////////////////////////////////////////////////////////////

#if defined(_MSC_VER)
#include <crtdbg.h>
#endif

///////////////////////////////////////////////////////////////////////////////

#if defined(_MSC_VER)

#else
extern char** _argv;
extern int _argc;

#define __argv      _argv
#define __argc      _argc
#endif

///////////////////////////////////////////////////////////////////////////////

#define CONFIG_BUFFER_LEN   4096
#define CONFIG_KEY_LEN      256
#define CONFIG_VALUE_LEN    1024

///////////////////////////////////////////////////////////////////////////////

#define IS_WHITE_SPACE(__ch)    ((__ch) == '\t' || (__ch) == ' ' || (__ch) == '\r' || (__ch) == '\n')

///////////////////////////////////////////////////////////////////////////////

int     ConvVersion         (const char* pszVer);
void    MakeVersionString   (int nVer, char* pszVer);
int     FindFullPathName    (const char* pszModuleName, const char* pszSuffix,
                             const char* pszExt, const char* pszDefDirName,
                             char* pszFullName, int nNameLen);

///////////////////////////////////////////////////////////////////////////////


#endif      // end of __CONF_MGR_H__
