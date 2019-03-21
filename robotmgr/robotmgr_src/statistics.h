#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include "service.h"

#define STAT_KEY_TIME_ROBOT_RESET_DATE      "RESET_DATE"
#define STAT_KEY_TIME_ROBOT_RESET_TIME      "RESET_TIME"
#define STAT_KEY_TIME_ROBOT_PWRON           "PWRON_TIME"
#define STAT_KEY_TIME_ROBOT_SRVON           "SRVON_TIME"
#define STAT_KEY_TIME_ROBOT_JOBEXE          "JOBEXE_TIME"
#define STAT_KEY_TIME_ROBOT_ERROR           "ERROR_TIME"
#define STAT_KEY_TIME_ROBOT_STOP            "STOP_TIME"
#define STAT_KEY_TIME_ROBOT_MOVE            "MOVE_TIME"
#define STAT_KEY_TIME_ROBOT_WELD            "WELD_TIME"

#define STAT_KEY_DIST_ROBOT_FILLETWELD      "FILLET_WELD_DIST"
#define STAT_KEY_DIST_ROBOT_WEAVEWELD       "WEAVE_WELD_DIST"

#define STAT_KEY_ERROR_HISTORY_COUNT        "ERR_CNT"
#define STAT_KEY_ERROR_HISTORY_ST_TOP       "ERR_STACK_TOP"
#define STAT_KEY_ERROR_HISTORY_INDEX        "ERR_INDEX"
#define STAT_KEY_ERROR_HISTORY_TIME         "ERR_TIME"
#define STAT_KEY_ERROR_HISTORY_CODE         "ERR_CODE"
#define STAT_KEY_ERROR_HISTORY_DETAIL       "ERR_DETAIL"

#define STAT_FORMAT_DATE                    "%04d-%02d-%02d"
#define STAT_FORMAT_TIME                    "%02d:%02d:%02d"
#define STAT_FORMAT_DIST                    "%.2lf"

#define RCON_STATISTICS_GET         0	// save file to disk & get information
#define RCON_STATISTICS_RESET_ALL   1   // reset all

#define ROBOMAN_SYSTIMER_INTERVAL   1 // 100 [ms]

#define OPT_NULL        0
#define OPT_QUITE       1

typedef enum _RobotStatTime
{
    ROBOT_STAT_TIME_POWERON = 0,
    ROBOT_STAT_TIME_SERVOON,
    ROBOT_STAT_TIME_JOBEXEC,
    ROBOT_STAT_TIME_ERROR,
    ROBOT_STAT_TIME_STOP,
    ROBOT_STAT_TIME_MOVE,
    ROBOT_STAT_TIME_WELD,

    ROBOT_STAT_TIME_COUNT
} RobotStatTime;

#pragma pack(1)
typedef union
{
    unsigned long       rgdwTime[ROBOT_STAT_TIME_COUNT];

    struct
    {
        unsigned long   dwTimePowerOn;  // power on time
        unsigned long   dwTimeServoOn;  // servo enable time
        unsigned long   dwTimeJobExec;  // job execution time
        unsigned long   dwTimeError;    // error or fault time
        unsigned long   dwTimeStop;     // robot stop time by error, fault, estop
        unsigned long   dwTimeMove;     // robot moving time
        unsigned long   dwTimeWeld;     // total welding time
    };    
} ROBOT_STAT_TIME;
#pragma pack()

typedef enum _RobotStatDist
{
    ROBOT_STAT_DIST_FILLETWELD = 0,
    ROBOT_STAT_DIST_WEAVWELD,

    ROBOT_STAT_DIST_COUNT
} RobotStatDist;

#pragma pack(1)
typedef union
{
    double       rgdbDistance[ROBOT_STAT_DIST_COUNT];

    struct
    {
        double   dbDistFilletWeld;  // fillet joint welding distance
        double   dbDistWeavWeld;    // weav welding distance
    };    
} ROBOT_STAT_DIST;
#pragma pack()

#pragma pack(1)
typedef union
{
    FORMAT_DATE   DateReset;    // statistics data reset date
    FORMAT_TIME   TimeReset;    // statistics data reset time
} ROBOT_RESET_TIME;
#pragma pack()

//ROBOT_RESET_TIME        g_RobotStatResetTime;
ROBOT_STAT_TIME         g_dwRobotStatTime;
ROBOT_STAT_DIST         g_dbRobotStatDist;

RMGR_STATISTICS_DATA    g_StatisData;

#endif  // end of __STATISTICS_H__