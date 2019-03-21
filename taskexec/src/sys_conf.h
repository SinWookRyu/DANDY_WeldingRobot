#ifndef SYS_CONF_H__
#define SYS_CONF_H__

// SYS_CONF.H 
// Configuration file for system. 

///// Definitions 

#define PROJECT_NAME            "DANDY2015"

// abbreviation
#define ROBOMAN_ABB_NAME        "RM"
#define TASKEXEC_ABB_NAME       "TE"
#define SERVOCON_ABB_NAME       "SC"

// .exe and/or executable file name
#define ROBOMAN_EXEC_NAME       "robotmgr"
#define TASKEXEC_EXEC_NAME      "taskexec"
#define SERVOCON_EXEC_NAME      "servocon"

// precess name
#define RM_PROCESS_NAME         "RM.Process"
#define TE_PROCESS_NAME         "TE.Process"
#define SC_PROCESS_NAME         "SC.Process"

///////////////////////////////////////
//
// Define Robot Configuration
//
////////////////////////////////////////

#define MAX_ROBOT_COUNT         4
#define MAX_AXIS_COUNT          32
#define ROB_AXIS_COUNT          6
#define MAX_AXIS_MOTOR_COUNT    2

#define MAX_ROBOT_AXIS_COUNT	16    // max axes count for a robot
#define MAX_USER_COORD_COUNT    128   // Max user defined coordination number
#define MAX_COORD_COUNT         MAX_USER_COORD_COUNT
#define MAX_HOME_COUNT          32  // max home info count

///////////////////////////////////////
// Input & Output

#define MAX_MOTOR_COUNT         32
#define MAX_DI_COUNT            128   // max analog input  channel count (DIO)
#define MAX_DO_COUNT            128   // max analog output channel count (DIO)
#define MAX_AI_COUNT            64    // max analog input  channel count (A/D)
#define MAX_AO_COUNT            64    // max analog output channel count (D/A)

#define MAX_AD_COUNT            MAX_AI_COUNT
#define MAX_DA_COUNT            MAX_AO_COUNT

#define ROBOT_DI_SLAVE_COUNT    4
#define SLAVE_DI_PORT_COUNT     8
#define ROBOT_DI_PORT_COUNT     (ROBOT_DI_SLAVE_COUNT * SLAVE_DI_PORT_COUNT)

#define ROBOT_DO_SLAVE_COUNT    3
#define SLAVE_DO_PORT_COUNT     8
#define ROBOT_DO_PORT_COUNT     (ROBOT_DO_SLAVE_COUNT * SLAVE_DO_PORT_COUNT)

#define ROBOT_AI_SLAVE_COUNT    1
#define SLAVE_AI_PORT_COUNT     2
#define ROBOT_AI_PORT_COUNT     (ROBOT_AI_SLAVE_COUNT * SLAVE_AI_PORT_COUNT)

#define ROBOT_AO_SLAVE_COUNT    1
#define SLAVE_AO_PORT_COUNT     2
#define ROBOT_AO_PORT_COUNT     (ROBOT_AO_SLAVE_COUNT * SLAVE_AO_PORT_COUNT)

// WELDER
#define MAX_WELDER_COUNT        32

// COMM
#define MAX_COMM_COUNT          4

// JOB
#define MAX_CALL_COUNT          8   // max job call count

///////////////////////////////////////
// EtherCAT
#define CONFIG_PATH_NAME_LEN    256

////////////////////////////////////////////////////////////////////////////////
// System Control Mode

#define SYS_CTRL_TP             0   // TP control
#define SYS_CTRL_AUTO           1   // automatical operatin
#define SYS_CTRL_GUI            2   // GUI control
#define SYS_CTRL_TERM           3   // Terminal control

////////////////////////////////////////////////////////////////////////////////
// Robot

#define ROBOT_NAME_LEN          32

#define ROBTYPE_NONE            0
#define ROBTYPE_EXCEPT          1   // no axis or exceptional robot
#define ROBTYPE_RECTANGULAR     2   // gantry
#define ROBTYPE_10KG            3   // DSME 10KG welding robot
#define ROBTYPE_DR6             4   // Doosan welding robot
#define ROBTYPE_DANDY           5   // DSME Dandy
#define ROBTYPE_DANDY_II        6   // DSME Dandy-II

// Gap Condition
#define HORZ_GAP_COND           0
#define VERT_LEFT_GAP_COND      1
#define VERT_RIGHT_GAP_COND     2

#define GAP_COND_NONE           0
#define GAP_COND_SMALL          1
#define GAP_COND_BIG            2

////////////////////////////////////////////////////////////////////////////////
// Axis 

#define AXIS_NAME_LEN           32

#define AXISTYPE_NONE           0
#define AXISTYPE_REVOLUTE       1
#define AXISTYPE_PRISMATIC      2

////////////////////////////////////////////////////////////////////////////////
// motor

#define MOTOR_NAME_LEN          32

#define MOTTYPE_NONE            0
#define MOTTYPE_SIGMA_V         1
#define MOTTYPE_MINAS           2

// motor direction
#define MOTDIR_FORWARD          1
#define MOTDIR_REVERSE          -1
#define FOR                     1
#define REV                     -1

// encoder type
#define ENCTYPE_INC             0
#define ENCTYPE_ABS             1

// Coord. Index Definitions
#define COORDINDEX_USER(i_)     (0 + i_)   // 0 ~ 127 are for user world coord
#define COORDINDEX_JOINT        -1     
#define COORDINDEX_WORLD        -2
#define COORDINDEX_BASE         -3
#define COORDINDEX_END          -4
#define COORDINDEX_TOOL         -5
#define COORDINDEX_TCP          -5
#define COORDINDEX_SENSOR       -6

// Axis Definitions
#define AXIS_JNT(i_)            (0 + i_)
#define AXIS_LIN_X              0
#define AXIS_LIN_Y              1
#define AXIS_LIN_Z              2
#define AXIS_ROT_X              3
#define AXIS_ROT_Y              4
#define AXIS_ROT_Z              5

////////////////////////////////////////////////////////////////////////////////
// COMM Port

#define ROBOT_COMM_PORT_LEN     32

#define ROBOT_COMM_NONE         0
#define ROBOT_COMM_RS232        1
#define ROBOT_COMM_RS485        2

////////////////////////////////////////////////////////////////////////////////
// Welder

#define WELDER_NAME_LEN         32

#define WELDER_TYPE_NONE        0
#define WELDER_TYPE_HYOSUNG_UR  1       // Hyosung UR series (UR 351R/501R)
#define WELDER_TYPE_DAIHEN_DM   2       // Daihen DM series (DM 350/500)
#define WELDER_TYPE_ZEUS        3       // Zeus series
#define WELDER_TYPE_GENERIC     99      // generic welder (unensureable)

// welder ability
#define WELDER_ABIL_VOID        0x01
#define WELDER_ABIL_CO2         0x02
#define WELDER_ABIL_MIG         0x04
#define WELDER_ABIL_MAG         0x08
#define WELDER_ABIL_TIG         0x10

#define MAX_WELD_DIN_PORT_COUNT   8
#define MAX_WELD_AIN_PORT_COUNT   2
#define MAX_WELD_DOUT_PORT_COUNT  8
#define MAX_WELD_AOUT_PORT_COUNT  2

////////////////////////////////////////////////////////////////////////////////
// DH parameters
// 1. theta: angle about previous z, from old x to new x
// 2. d: offset along previous z to the common normal
// 3. alpha: angle about common normal, from old z axis to new z axis
// 4. l: length of the common normal. 
//    Assuming a revolute joint, the radius about previous z

#pragma pack(push, 1)
typedef struct
{
    double   th;    // theta
    double   d;     // d
    double   al;    // alpha
    double   l;     // l
} DH_PARAMS;
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////
// Euler Representaion

#pragma pack(push, 1)
typedef struct
{
    double  x;
    double  y;
    double  z;
    double  rol;    // roll
    double  pit;    // pitch
    double  yaw;    // yaw
} COORD_EULER;	
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////
// Controller Configuration

#pragma pack(push, 1)
typedef struct
{
    char szIoName[ROBOT_NAME_LEN];
    int  nMode;     // di, do, ai, ao
    //
    // ..
    //
} CONFIG_IO;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    char szCommName[ROBOT_NAME_LEN];
    int  nMode;     // Serial, Ethernet, ...    
    //
    // ..
    //     
} CONFIG_COMM;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    int     fUsed;                  // 0 = unused, 1 = used
    char    szName[WELDER_NAME_LEN];
    int     nType;                  // WELDER_TYPE_...
    int     nAbility;               // bitwise type (co2/mig/mag/tig)
    int     nWelder;                // welder index
    int     nRobot;                 // attached robot number (0 ~ MAX_ROBOT_COUNT-1)

    // digital input slave no
    int     nDinSlaveNo;

    // digital input port no
    union
    {
        int     rgndin_portno[16];
        struct
        {
            int     nArcOn;         // arc on/off state
            int     nNoGas;         // gas shortage state
            int     nNoWire;        // wire shortage state
            int     nWeldPowerFail; // welder power fail state
            int     nTouchProcess;  // touch process activated state
            int     nTouchSignal;   // touch signal state
            int     nWeld;          // sustainable welding arc on/off (arc confirm bit)
            int     nArcFail;       // arc fail state
        } din_portno;
    };

    // digital input active level
    union
    {
        int     rgndin_actlev[16];
        struct
        {
            int     nArcOn;         // arc on/off state
            int     nNoGas;         // gas shortage state
            int     nNoWire;        // wire shortage state
            int     nWeldPowerFail; // welder power fail state
            int     nTouchProcess;  // touch process activated state
            int     nTouchSignal;   // touch signal state
            int     nWeld;          // sustainable welding arc on/off (arc confirm bit)
            int     nArcFail;       // arc fail state
        } din_actlev;
    };

    // digital output slave no
    int     nDoutSlaveNo;

    // digital output port no
    union
    {
        int     rgndout_portno[16];
        struct
        {
            int     nArcOn;         // arc on/off
            int     nGasOn;         // gas on/off
            int     nInchPos;       // (+) inching
            int     nInchNeg;       // (-) inching
            int     nTouchStart;    // touch start
            int     nTouchReady;    // touch ready (MC state control)
            int     nWireCut;       // wire cut on/off
            int     nWeldPower;     // welding power enabling bit number
        } dout_portno;
    };

    // digital output active level
    union
    {
        int     rgndout_actlev[16];
        struct
        {
            int     nArcOn;         // arc on/off
            int     nGasOn;         // gas on/off
            int     nInchPos;       // (+) inching
            int     nInchNeg;       // (-) inching
            int     nTouchStart;    // touch start
            int     nTouchReady;    // touch ready (MC state control)
            int     nWireCut;       // wire cut on/off
            int     nWeldPower;     // welding power enabling bit number
        } dout_actlev;
    };

    // analog input (A/D)
    int     nCurrentInPortNo;       // welding current A/D channel (-1=invalid)
    int     nVoltageInPortNo;       // welding voltage A/D channel (-1=invalid)

    // analog output (D/A)
    int     nCurrentOutPortNo;      // welding current D/A channel (-1=invalid)
    int     nVoltageOutPortNo;      // welding voltage D/A channel (-1=invalid)
} CONFIG_WELDER;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{
    int nQNXTimerTick;              // [ns] unit, default = 1 ns
    int nQNXTimerRes;               // [us] unit, default = 100 us
    int nIoScanTime;                // [ms] unit, default = 1 ms
    int nTrajUpdateTime;            // [ms] unit, default = 1 ms
    int nServoInterpolationTime;    // [ms] unit, default = 3 ms

    int nServoOnBrakeDelayTime;     // [ms] unit, default = 70 ms
    int nServoOffBrakeDelayTime;    // [ms] unit, default = 150 ms
} CONFIG_CTRL; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{
    // Gap Condition Setting Parameter
    int fGapRefVarUsed;     // Gap Reference B Variables Applied or Not(Def: 0)
    int nGapRefBvar;        // Gap Reference B Variable (Def: B04)
    
    int nLeftWeldBvar;      // Left Weld Reference B Variable  (Def: B10)
    int nRightWeldBvar;     // Right Weld Reference B Variable (Def: B20)

    // CWeav related Parameter
    double dbCWeavTouchUpDis;  // After Touch, Imove Distance
    double dbCWeavHorMargin;   // Distance between Motion End and Collar Plate End
    double dbCWeavWeldLegDis;  // Traj Refer to Weld Leg Distance
} CONFIG_WELD_FUNC; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{
    int nSlaveCount;                       // 
    char szPathName[CONFIG_PATH_NAME_LEN]; // location of ethercat config file

    int nWriteOffsetSize;                  // BeckHoff output size
    int nReadOffsetSize;                   // BeckHoff input size
} CONFIG_ECAT; 
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct
{    
    // Basic
    char    szName[MOTOR_NAME_LEN];    
    int     nMotorType;             // motor type (SIGMA_V)
    int     nMotorIndex;
        
    // Dynamics
    double  acc;                    // [ms] acceleration 
    double  dec;                    // [ms]
    double  jrk;                    // [ms] Must less then Accel value(0:Trapezoidal)

    // Deceleration by event
    double  dec_estop;              // [ms] deceleration by estop        
    double  dec_error;              // [ms] deceleration by error
    
    // Vel Limit
    double  vellim_max;             // [rad/ms] max velocity limit

    // Mechanical
    int     nEncRes;                // pulse per 1 turn ex) 20bit : 1048576        
} CONFIG_MOTOR;
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct
{    
    char    szName[AXIS_NAME_LEN];
    int     nAxisType;              // Axis type (Revolutional / Prismatic)
    int     nAxisIndex;

    // hw limit
    int     fHwLim_min;             // flag for min HW Limit applied or not check
    int     fHwLim_max;             // flag for max HW Limit applied or not check
    double  pos_hwlim_min;          // min HW limit position
    double  pos_hwlim_max;          // max HW limit position   
    
    // sw limit
    int     fSwLim_min;             // flag for min SW Limit applied or not check
    int     fSwLim_max;             // flag for max SW Limit applied or not check
    double  pos_swlim_min;          // min SW limit position
    double  pos_swlim_max;          // max SW limit position  

    // Motors.     
    int     nMotorCount; 
    int     nMotorIndex[MAX_MOTOR_COUNT];
    //CONFIG_MOTOR* motor[MAX_MOTOR_COUNT];
    
    int     dir[MAX_MOTOR_COUNT];   // Direction of motor[i] (Forward / Reverse)
    int     ori[MAX_MOTOR_COUNT];   // Origin of motor[i]
    double  red[MAX_MOTOR_COUNT];   // Reduction of motor[i] (ex)120 : Reduction Ratio 1:120
} CONFIG_AXIS;
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct
{
    char        szRobotName[ROBOT_NAME_LEN];
    int         fUsed;                     
    int         nRobotType;         // ROBOT_TYPE_...
   
    // maximum speed
    double      dbMaxJointSpeed[ROB_AXIS_COUNT];    //[rad/ms]
    double      dbMaxLinearSpeed;   // [mm/ms]
    double      dbMaxOrientSpeed;   // [rad/ms]

    // Acceleration / Deceleration
    double      dbAccel;            // [ms]
    double      dbDecel;            // [ms]    
    double      dbJerk;             // [ms] Must less then Accel value(0:Trapezoidal)
    
    // Deceleration by Event
    double      dbDecel_Error;      // [ms]
    double      dbDecel_Estop;      // [ms]
    double      dbDecel_Touch;      // [ms]
           
    // Axis 
    int         nAxesCount;
    CONFIG_AXIS axis[ROB_AXIS_COUNT]; 

    // DH Params 
    DH_PARAMS   dh[ROB_AXIS_COUNT]; 

    // World Coord Offest wrt BASE
    COORD_EULER       world; 

    // TCP (x-y-z-r-p-y) from end joint
    COORD_EULER       coordTool;

    // World Coord Offset wrt BASE
    COORD_EULER       user[MAX_USER_COORD_COUNT];

    // Home Position
    double      rgdbHomePosVal[MAX_HOME_COUNT][ROB_AXIS_COUNT];

    // Welder
    int         nWelderCount;                     // number of welder
    int         nWelderIndex[MAX_WELDER_COUNT];   // welder index
    //CONFIG_WELDER*      welder[MAX_WELDER_COUNT]; // welder's index list
    CONFIG_WELD_FUNC    weld_func;

    // IO
    int         nDiCount;
    int         nDiIndex[MAX_DI_COUNT];
    //CONFIG_IO*  din[MAX_DI_COUNT];     
    
    int         nDoCount; 
    int         nDoIndex[MAX_DO_COUNT];
    //CONFIG_IO*  dout[MAX_DO_COUNT]; 

    int         nAiCount; 
    int         nAiIndex[MAX_AI_COUNT];
    //CONFIG_IO*  ain[MAX_AI_COUNT]; 

    int         nAoCount; 
    int         nAoIndex[MAX_AO_COUNT];
    //CONFIG_IO*  aout[MAX_AO_COUNT]; 

    // Comm
    int         nCommCount; 
    int         nCommIndex[MAX_COMM_COUNT];
    //CONFIG_COMM* comm[MAX_COMM_COUNT]; 
} CONFIG_ROBOT;
#pragma pack(pop)

///////////////////////////////////////
//
// ROBOT System Configuration
//
#pragma pack(push, 1)
typedef struct
{
    unsigned long   dwLength;

    // EtherCAT Configuration
    CONFIG_ECAT     ecat;

    // Controller Configuration
    CONFIG_CTRL     ctrl; 

    // robots in system
    CONFIG_ROBOT    robot[MAX_ROBOT_COUNT];

    // motors in system
    CONFIG_MOTOR    motor[MAX_MOTOR_COUNT];

    // welders in system
    CONFIG_WELDER   welder[MAX_WELDER_COUNT];

    // IO's in system    
    CONFIG_IO       din[MAX_DI_COUNT]; 
        
    CONFIG_IO       dout[MAX_DO_COUNT]; 

    CONFIG_IO       ain[MAX_AI_COUNT]; 

    CONFIG_IO       aout[MAX_AO_COUNT]; 

    // Comm    
    CONFIG_COMM     comm[MAX_COMM_COUNT]; 
} SHM_RM_SYSCONFIG;
#pragma pack(pop)

#endif  // end of SYS_CONF_H__