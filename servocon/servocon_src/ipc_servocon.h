#ifndef SERVOCON_IPC_H__
#define SERVOCON_IPC_H__

#include "sys_conf.h"

////////////////////////////////////////////////////////////////////////////////
//
// Channel & SHM Name Define
//
#define SC_TIME_CHANNEL      "SC_RUNTIM_CHANNEL"  // runtime channel(TE & SC)
#define SC_CHANNEL_NAME      "CHANNEL_SC"         // general channel
#define SC_SHM_NAME          "SERVOCON_SHM"
#define SC_VERSION           "5.01a"
#define SC_BUILD             "2014.06.30"

/////////////////// Symbol Definition for Service, State //////////////////////

///////////////////////////////////////
//
// RM Service Symbol
//
// Code : Service Name            : Msg Format : Value
//      :                         :            : 0       : 1       : 2         : 3       : 4         : 5     : 6
// 0    : SC_SERV_EXIT            : pulse      :         : 
// 1    : SC_SERV_VERSION         : message    : None    :
// 2    : SC_SERV_INIT            : pulse      :         :
// 3    : SC_SERV_ALARM_RESET     : pulse      :         :
// 4    : SC_SERV_GET_SERVO_ALARM : pulse      : None    :
// 5    : SC_SERV_ECAT_RESTART    : pulse      : None    :
// 6    : SC_SERV_ESTOP           : pulse      : clear   : stop
// 7    : SC_SERV_SCAN_WELD_IO    : pulse      : ALL     : weldDin : weldAin  : weldDout : weldAout ->Use Reply Msg
// 8    : SC_SERV_SCAN_SERVO_IO   : pulse      : ALL     : BrkDin  : BrkOut   : EncClr              ->Use Reply Msg
// 9    : SC_SERV_SCAN_SYS_STATE  : pulse      : ALL     : State   : SysI/O   : Pos      : Pos(Jnt)  : Torque : TrgPos  ->Use Reply Msg
// 10   : SC_SERV_SERVO           : pulse      : OFF     : ON
// 11   : SC_SERV_BRAKE_RELEASE   : pulse      : AxisNo  : 100: All Axes
// 12   : SC_SERV_BRAKE_LOCK      : pulse      : AxisNo  : 100: All Axes
// 15   : SC_SERV_ARCON_OUT       : pulse      : OFF     : ON
// 16   : SC_SERV_GASON_OUT       : pulse      : OFF     : ON
// 17   : SC_SERV_INCHING_POS     : pulse      : OFF     : ON
// 18   : SC_SERV_INCHING_NEG     : pulse      : OFF     : ON
// 19   : SC_SERV_TOUCH_START     : pulse      : OFF     : ON
// 20   : SC_SERV_TOUCH_READY     : pulse      : OFF     : ON
// 21   : SC_SERV_WIRE_CUT        : pulse      : OFF     : ON
// 22   : SC_SERV_VOLT_OUT        : pulse      : Cmd Vaule: -100~100(-10~10Volt), Unit:Vaule1=0.1 Volt
// 23   : SC_SERV_CURR_OUT        : pulse      : Cmd Vaule: -100~100(-10~10Volt), Unit:Vaule1=0.1 Volt
// 28   : SC_SERV_WRITE_DO_PORT   : pulse      : port, outval (Refer to: MAKE_VALUE_OUTPUT)
// 29   : SC_SERV_WRITE_AO_PORT   : pulse/msg  : port, outval (Refer to: MAKE_VALUE_OUTPUT), value: cmd mulplied by 10, msg: opt
// 30   : SC_SERV_SET_POS_ZERO    : pulse      : AxisNo  : 100: All Axes
// 31   : SC_SERV_ABS_ENC_RESET   : pulse      : AxisNo  : 100: All Axes
// 32   : SC_SET_INTERP_PERIOD    : pulse      : Obj Val : 
// 33   : SC_SET_INTERP_INDEX     : pulse      : Obj Val : 
// 34   : SC_SET_QNX_TIMER_REG    : pulse      : SampTime: 
// 35   : SC_CHANGE_AOUT_UNIT     : pulse      : Raw     : Weld   : Inch
// 36   : SC_RELEASE_SHOCKSENSOR  : pulse      : OFF     : ON
// 37   : SC_DOUT_SET_ZERO        : pulse      : Estop   : Stop
// 38   : SC_SERV_ERRORSTOP       : pulse      : None
// 39   : SC_SERV_JOBSTOP_EVENT   : pulse      : OFF     : ON
// 41   : SC_HWLIMIT_MON_ACT      : pulse      : OFF     : ON
// 42   : SC_ARCSENS_FILE_SAVE    : pulse      : ALL     : ArcSens: (Axis) : (I/O)
// 43   : SC_SHM_PARAM_RELOAD     : pulse      : None    :
// 44   : SC_ARCSEN_RDATA_FILELOAD: pulse      : None
// 71   : SC_ACRON_VIRTUAL_PROC   : pulse      : OFF     : ON

// 127  : SC_SERV_ALIVE           : pulse      : None    :
// skip developer service..

//
// Remark: Both msssage and pulse are compatible,
//         But above method recommended

    // system startup & status service
#define SC_SERV_EXIT             0      // terminate SC process
#define SC_SERV_VERSION          1      // check SC version info.
#define SC_SERV_INIT             2      // initialization of SC process

    // system state service
#define SC_SERV_ALARM_RESET      3      // reset error of master
#define SC_SERV_GET_SERVO_ALARM  4      // alarm code of servopack
#define SC_SERV_ECAT_RESTART     5      // EtherCAT master restart
#define SC_SERV_ESTOP            6      // emergency stop value: 0(clear)/1(estop)

    // communication data service (TP)
#define SC_SERV_SCAN_WELD_IO     7      // Value
                                        // 0: All (welder_mon)
                                        // 1: Weld Digital Input(weldDin)
                                        // 2: Weld Analog Intput(weldAin)
                                        // 3: Weld Digital Output(weldDout)
                                        // 4: Weld Analog Output(weldAout)
#define SC_SERV_SCAN_SERVO_IO    8      // Value
                                        // 0: All(servo_mon) excluding absEncstate
                                        // 1: Brake Digital Input(servoin)
                                        // 2: Brake ECAT Output(servoout)
                                        // 3: ABS Encoder Reset Done flag(absEncstate)
#define SC_SERV_SCAN_SYS_STATE   9      // Value
                                        // 0: All (realtime_sysmon)
                                        // 1: System State (sysstate)
                                        // 2: System I/O (sysiostate)
                                        // 3: Actual Position (pos)
                                        // 4: Actual Position (jnt_pos, Joint Coord Value only)
                                        // 5: Actual Torque (%, Torque Value only)
                                        // 6: Target Position(trg_pos)
    // I/O handling service (TP)
#define SC_SERV_SERVO            10     // servo on value: 0(off)/1(on)
#define SC_SERV_BRAKE_RELEASE    11     // Value: Axis No 100: All Axis
#define SC_SERV_BRAKE_LOCK       12     // Value: Axis No 100: All Axis
#define SC_SERV_ARCON_OUT        15     // Arc On/Off  (Value 0: Off, 1: On)
#define SC_SERV_GASON_OUT        16     // Gas On/Off  (Value 0: Off, 1: On)
#define SC_SERV_INCHING_POS      17     // Inching +   (Value 0: Off, 1: On)
#define SC_SERV_INCHING_NEG      18     // Inching -   (Value 0: Off, 1: On)
#define SC_SERV_TOUCH_START      19     // Touch Start (Value 0: Off, 1: On)
#define SC_SERV_TOUCH_READY      20     // Touch Ready (Value 0: Off, 1: On)
#define SC_SERV_WIRE_CUT         21     // Wire Cut    (Value 0: Off, 1: On)
#define SC_SERV_VOLT_OUT         22     // Weld Volt Out
                                        // Value
                                        // Cmd Vaule: -100 ~ 100 (-10 ~ 10 Volt)
                                        // Unit: Vaule 1 = 0.1 Volt
#define SC_SERV_CURR_OUT         23     // Weld Current Out
                                        // Value
                                        // Cmd Vaule: -100 ~ 100 (-10 ~ 10 Volt)
                                        // Unit: Vaule 1 = 0.1 Volt
#define SC_SERV_WRITE_DO_PORT    28     // Value: port, outval (MAKE_VALUE_OUTPUT)
#define SC_SERV_WRITE_AO_PORT    29     // Value: port, outval (MAKE_VALUE_OUTPUT)
                                        // Cmd Value: Unit must be multiplied by 10
                                        // Data: Use 'serv_opt' define aout type
                                        // ex> desired inch speed: 1000mm/s -> cmd: 10000
                                        //     desired weld curr: 500A -> cmd: 5000
                                        //     desired weld volt: 15.2V -> cmd: 152
    // encoder reset service
#define SC_SERV_SET_POS_ZERO     30     // Absolute Encoder Value set to zero
                                        // Value
                                        // 0~5:Axis No
                                        // 100: All Axis
#define SC_SERV_ABS_ENC_RESET    31     // Absolute Encoder Value set to zero
                                        // Value
                                        // 0~5:Axis No
                                        // 100: All Axis

    // servo setup service
#define SC_SET_INTERP_PERIOD     32     // Set Interpolate time period
                                        // Value: Object Value -> 1 ~ 250(def: 125)
                                        // Data: Option nOpt -> quite
#define SC_SET_INTERP_INDEX      33     // Absolute Encoder Value set to zero
                                        // Value: Object Value -> -6 ~ 3 (def: -3)
                                        // Data: Option nOpt -> quite
#define SC_SET_QNX_TIMER_REG     34     // Set QNX Timer Sampling Time
                                        // Value: Sampling Time (1~100)
    // additional H/W control
#define SC_CHANGE_AOUT_UNIT      35     // Change Analog I/O Unit
                                        // Value: 0: raw data(-10~10V),
                                        //        1: Weld Current/Volt,
                                        //        2: Wire Feeding Speed
#define SC_RELEASE_SHOCKSENSOR   36     // Release Shock Sensor E-Stop (Value 0: Off, 1: On)
#define SC_DOUT_SET_ZERO         37     // Dout Port Set Zero 
                                        // Value 0: Smooth Arcoff mode, 1: Urgent mode
#define SC_SERV_ERRORSTOP        38     // Error Stop Event
#define SC_SERV_JOBSTOP_EVENT    39     // TP Stop or RM Job Stop Event Process

    // system support service

#define SC_HWLIMIT_MON_ACT       41     // H/W Limit Monitoring State Activate
#define SC_ARCSENS_FILE_SAVE     42     // Save System Data to File
                                        // Value
                                        // 0: All, 1: ArcSensor, (2: Axis, 3: I/O)
#define SC_SHM_PARAM_RELOAD      43     // Shared Memory Param Reload
#define SC_ARCSEN_RDATA_FILELOAD 44     // ArcSensor Rdata File load to SHM

    // service for developer
#define SC_SET_OPERATEMODE       50     // set modes of operation Value: 1~8
#define SC_SET_SYNCERROR_CNT     51     // set DC mode Sync Error Count Value: 0~15
#define SC_READ_OPERATEMODE      52     // read modes of operation
#define SC_SERV_SERVOON_1AX      54     // servo on  value: 0~5:Axis No
#define SC_SERV_SERVOOFF_1AX     55     // servo off value: 0~5:Axis No
#define SC_CONTROL_WORD_TEST     56     // axis 0 value: control word
#define SC_VGA_DISPLAY_ON_OFF    57     // VGA Display on/off  value: 0: Off, 1: On
#define SC_AXIS_DEBUG            58     // Axis Debug Message on/off  value: 0: Off, 1: On
#define SC_SERV_SET_POSITION     59     // move target position (only test ver.) 
#define SC_SERV_GET_POSITION     60     // get actual position (only test ver.) 
#define SC_IO_TEST_MODE          61     // start I/O test mode, value: 0: Off, 1: On
#define SC_POS_FILE_WRITE        62     // Fiel Write on/off  value: 0: Off, 1: On
#define SC_TEST_IO_ON            63     // test service code that write IO, value: 0(off)/1(on)
#define SC_TEST_IO_OFF           64     // test service code that write IO, value: 0(off)/1(on)
#define SC_SERV_SET_HOME_ZERO    65     // Home offset Value(607Ch) set to zero
                                        // Value
                                        // 0~5:Axis No
                                        // 100: All Axis
#define SC_SERV_ENC_RESET_ONLY   66     // ABS Reset Only
                                        // Value
                                        // 0~5:Axis No
                                        // 100: All Axis
#define SC_SERV_SHOW_LOADED_SHM  67     // Show Loaded shared memory data
#define SC_SERV_ADC_GET_REQ      68     // Set ADC_gathering_request On/Off, value: 0: Off, 1: On
#define SC_ACRON_INPORT_SIGHIGH  70     // Arcon Signal force to High, value: 0: Off, 1: On
#define SC_ACRON_VIRTUAL_PROC    71     // Arcon Virtual Process, value: 0: Off, 1: On

#define SC_SERV_ALIVE            127    // check process alive

// To make the value for SC_SERV_WRITE_DO_PORT, SC_SERV_WRITE_AO_PORT. 
#define MAKE_VALUE_OUTPUT(port_, outval_) \
  ((((port_)   & 0xff)   << 0 )  |        \
   (((outval_) & 0xffff) << 8 ))

#define ANALOG_IO_TYPE_RAW       0  // Controller I/O Level: 0~10 [V]
#define ANALOG_IO_TYPE_WELD      1  // Welding Current/Voltage: 500[A], 45[V]
#define ANALOG_IO_TYPE_INCH      2  // Wire Feeding Speed : 0 ~ 1000[mm/s](Welding Current Port)

////////////////////////////////////////////////////////////////////////////////
//
// Message Define
//

/////////////////////////////////////////////
// Version Data
// - sc_vers : version info. of SC process
// - sc_build : build info. of SC process

#define     SC_VERSION_DATA_LEN      16
#define     SC_BUILD_DATA_LEN        16

#pragma pack(push, 1)
typedef struct t_sc_msg_data_version
{   
    char sc_vers[SC_VERSION_DATA_LEN];
    char sc_build[SC_BUILD_DATA_LEN];
} SC_MSG_DATA_VERSION;
#pragma pack(pop)


/////////////////////////////////////////////
// Service Option
// - option : 1 is quite
// Service Name: SC_SET_INTERP_PERIOD, SC_SET_INTERP_INDEX
//               SC_SET_OPERATEMODE
//
// - option : Define Aout Type (operate same as SC_CHANGE_AOUT_UNIT)
// Service Name: SC_SERV_WRITE_AO_PORT

#pragma pack(push, 1)
typedef struct t_sc_msg_data_serv_opt
{
    int nOpt;
} SC_DATA_SERV_OPT;
#pragma pack(pop)


////////////////////////////////////////////////////////////////////////////////
//
// System Monitoring Data (Use Only TP)
//
// 1. Real-time System State Data
// 2. Servo In/Out Data
// 3. Welder In/Out Data
//

////////////////////////////
//
//  1. Real-time System State Data (Use Only TP)
//
    //  System State Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_system_state
{
    int  fEcatInitState;        // EtherCAT Master init. state 
    int  fEStopState;
    int  fErrorState;           // if 0 is no error, else error

    int  nErrorCode;            // alarm code of servopack
    int  nErrorAxis;            // Axis in errorstate
} SC_DATA_SYSTEM_STATE;
#pragma pack(pop)

    //  System Input Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_sysio_in_state
{
    int fTP_EstopInState;
    int fController_EstopInState;
    int fDeadManSwithInState;
    int fShockSensorInState;
} SC_DATA_SYSIO_IN_STATE;
#pragma pack(pop)

    //  Motor Position Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_motor
{
    int    nCoordIdx;                               // Coordinate Index
	double dbCurrPos[ROB_AXIS_COUNT];               // Current Actual Position(deg)
} SC_DATA_MOTOR;
#pragma pack(pop)

    //  Motor Current Position Data (Joint Coord. value only)
#pragma pack(push, 1)
typedef struct t_sc_msg_data_motor_jointval
{
	double dbCurrJntPos[ROB_AXIS_COUNT];            // Current Actual Position(deg)
} SC_DATA_MOTOR_JOINTVAL;
#pragma pack(pop)

    // Target Position Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_targ_pos              // 48 byte
{
    double dbTargetJntPos[MAX_AXIS_COUNT];         // Target Position(deg)
} SC_DATA_TRG_POS;
#pragma pack(pop)

    //  Motor Actual Torque
#pragma pack(push, 1)
typedef struct t_sc_msg_data_motor_torqueval
{
	double dbActTorque[ROB_AXIS_COUNT];            // Current Actual Torque(%)
} SC_DATA_MOTOR_TORQUEVAL;
#pragma pack(pop)

    // Servo I/O Assem
#pragma pack(push, 1)
typedef struct t_sc_msg_data_realtime_sysio_mon     // 88 byte
{
    SC_DATA_SYSTEM_STATE        sysstate;           // 20 byte
    SC_DATA_SYSIO_IN_STATE      sysiostate;         // 16 byte
    SC_DATA_MOTOR               pos;                // 52 byte
} SC_MSG_DATA_REALTIME_SYSIO_MON;
#pragma pack(pop)

////////////////////////////
//
//  2. Servo In/Out Data (Use Only TP)
//
#define BRAKE_DIN_PORT_COUNT        12 //0~5: Brake Status   6~11: Brake Clear

    // Input Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_servo_in               // 48byte
{
	int nBrakeDInPortVal[BRAKE_DIN_PORT_COUNT];     // Controller Button State
} SC_DATA_SERVO_IN;
#pragma pack(pop)


    // Output Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_servo_out              // 28 byte
{
    int fServoOnOutState;                           // ECAT Commnad (Servo On)
    int fBrakeReleaseOutState[ROB_AXIS_COUNT];      // ECAT Commnad (Brake Release)
} SC_DATA_SERVO_OUT;
#pragma pack(pop)

    // Servo I/O Assem
#pragma pack(push, 1)
typedef struct t_sc_msg_data_servo_io_mon           // 76 byte // must be under 96byte
{
    SC_DATA_SERVO_IN        servoin;                // 48 byte
    SC_DATA_SERVO_OUT       servoout;               // 28 byte
} SC_MSG_DATA_SERVO_IO_MON;
#pragma pack(pop)

    // Servo State Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_abs_enc_state            // 48 byte
{
    int fABSEncResetRequestState[ROB_AXIS_COUNT];   // ABS Encdoer Reset Request flag
    int fABSEncResetDoneState[ROB_AXIS_COUNT];      // ABS Encdoer Reset Done flag
                                                    // Service: SC_SERV_SET_POS_ZERO,
                                                    //          SC_SERV_ABS_ENC_RESET
} SC_DATA_ABS_ENC_STATE;
#pragma pack(pop)

////////////////////////////
//
// 3. Weld In/Out Data (Use Only TP)
//

#define WELD_DIN_PORT_COUNT         6  //0: Arc              1: No Gas
                                       //2: No Wire          3: Welder Power Fail
                                       //4: Touch Process    5: Touch Signal
#define WELD_AIN_PORT_COUNT         2  //0: Volt In          1: Current In


#define WELD_DOUT_PORT_COUNT        7  //0: Arc ON           1: Gas On
                                       //2: Wire Inching +   3: Wrie Inching -
                                       //4: Touch Start      5: Touch Ready(MC On)
                                       //6: Wire Cut
#define WELD_AOUT_PORT_COUNT        2  //0: Volt Out         1: Current Out

    // Input Data
#pragma pack(push, 1)
typedef struct t_sc_data_weld_din 
{
	int nWeldDInPortVal[WELD_DIN_PORT_COUNT];
} SC_DATA_WELD_DIN;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_sc_msg_data_weld_ain
{
	double dbWeldAInPortVal[WELD_AIN_PORT_COUNT];
} SC_DATA_WELD_AIN;
#pragma pack(pop)

    // Output Data
#pragma pack(push, 1)
typedef struct t_sc_msg_data_weld_dout
{
	int nWeldDOutPortVal[WELD_DOUT_PORT_COUNT];
} SC_DATA_WELD_DOUT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_sc_msg_data_weld_aout
{
	double dbWeldAOutPortVal[WELD_AOUT_PORT_COUNT];
} SC_DATA_WELD_AOUT;
#pragma pack(pop)

    // Welder I/O Assem
#pragma pack(push, 1)
typedef struct t_sc_msg_data_welder_io_mon  //80byte // must be under 96byte
{
    SC_DATA_WELD_DIN        weldDin;    //24byte
    SC_DATA_WELD_AIN        weldAin;    //16byte
    SC_DATA_WELD_DOUT       weldDout;   //24byte
    SC_DATA_WELD_AOUT       weldAout;   //16byte
} SC_MSG_DATA_WELDER_IO_MON;
#pragma pack(pop)


////////////////////////////////////////////////////////////////////////////////
// Send & receive Message Format of SC
//
// 1. MSG (REPLY) consists of 
// - code  : Service Code	(Required)
// - value : Service Value  (Required)
// - size  : Size of Data   (Required)
// - data  : Data of MSG    (Optional as Size of Data)

#define SC_PACKET_SIZE               108
#define SC_PACKET_HEAD_SIZE          12
#define SC_PACKET_DATA_SIZE          96

#define SC_REPLY_PACKET_SIZE         108
#define SC_REPLY_PACKET_HEAD_SIZE    12
#define SC_REPLY_PACKET_DATA_SIZE    96

#pragma pack(push, 1)
typedef struct t_sc_msg_packet
{
    // header
    unsigned int code;
    unsigned int value;
    unsigned int size;    // size of body

    // body
    union msg_data
    {
        unsigned char dummy[SC_PACKET_DATA_SIZE];

        // data
        SC_MSG_DATA_VERSION             vers;
        SC_DATA_SERV_OPT                serv_opt;

        //Realtime System I/O Mon (Data for TP Mon)
        SC_MSG_DATA_REALTIME_SYSIO_MON  realtime_sysmon;
        SC_DATA_SYSTEM_STATE            sysstate;
        SC_DATA_SYSIO_IN_STATE          sysiostate;
        SC_DATA_MOTOR                   pos;
        SC_DATA_MOTOR_JOINTVAL          jnt_pos;
        SC_DATA_MOTOR_TORQUEVAL         jnt_torque;
        
        //Servo I/O Mon (Data for TP Mon)
        SC_MSG_DATA_SERVO_IO_MON        servo_mon;
        SC_DATA_SERVO_IN                servoin;
        SC_DATA_SERVO_OUT               servoout;
        SC_DATA_TRG_POS                 trg_pos;
        SC_DATA_ABS_ENC_STATE           absEncstate;

        // Welder I/O Mon (Data for TP Mon)
        SC_MSG_DATA_WELDER_IO_MON       welder_mon;
        SC_DATA_WELD_DIN                weldDin;
        SC_DATA_WELD_AIN                weldAin;
        SC_DATA_WELD_DOUT               weldDout;
        SC_DATA_WELD_AOUT               weldAout;
    } data;
} SC_MSG, SC_REPLY;
#pragma pack(pop)


////////////////////////////////////////////////////////////////////////////////
// Shared Memory System Control
//
#define ERROR_MESSAGE_BUFFER_SIZE           256

#pragma pack(push, 1)
typedef struct t_system_state
{
    int  fEcatInitState;        // EtherCAT Master init. state 
    int  fEStopState;
    int  fErrorState;           // if 0 is no error, else error
    
    int  nErrorCode;            // alarm code of servopack & ecat
    int  nErrorAxis;            // Axis in errorstate

    int  nAoutMappingType[ROBOT_AO_PORT_COUNT];
                                // Mapping type define
                                // 0: raw data(-10~10V), 1: Weld Current/Volt, 2: Wire Feeding Speed
    char g_szErrorDescription[ERROR_MESSAGE_BUFFER_SIZE]; // Error Description
} SYSTEM_STATE;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_output_cmd
{
    // Boolean Output (Axis)                   // true : on, false : off
    int fServoOnOutCmd;                        //ECAT Commnad
    int fBrakeReleaseOutCmd[ROB_AXIS_COUNT];   //ECAT Commnad

    // Digital output (Controller Lamp)
    int fLampControllerReadyCmd;               //LampDoutport: 0
    int fLampUnderOperatingCmd;                //LampDoutport: 1
    int fLampServoOnCmd;                       //LampDoutport: 2
    int fLampEtherCATRunCmd;                   //LampDoutport: 3
    int fLampErrorCmd;
    int fCartLampAlarmCmd;
    int fCartLampRunCmd;
    int fCartJobStartConfirmCmd;
    int fCartDoutSpareCmd;

    double dbTrgPos[MAX_AXIS_COUNT];           // target position(rad)

    double  dbVoltRealTimeCmdOffset;
    double  dbCurrRealTimeCmdOffset;

    int     nDoutPortCmd[ROBOT_DO_PORT_COUNT];
    double  dbAoutPortCmd[ROBOT_AO_PORT_COUNT];
} OUTPUT_CMD;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_output_state
{
    // Boolean Output (Axis)                   // true : on, false : off
    int fServoOnOutState;                      //ECAT Commnad
    int fBrakeReleaseOutState[ROB_AXIS_COUNT]; //ECAT Commnad

    // Digital output (Controller Lamp)
    int fLampControllerReadyState;             //LampDoutport: 0
    int fLampUnderOperatingState;              //LampDoutport: 1
    int fLampServoOnState;                     //LampDoutport: 2
    int fLampEtherCATRunState;                 //LampDoutport: 3
    int fLampErrorState;
    int fCartLampAlarmState;
    int fCartLampRunState;
    int fCartJobStartConfirmState;
    int fCartDoutSpareState;

    // Digital Output (Weld)
    int fArcOnOutState;                        //WeldDoutPort: 0
    int fGasOnOutState;                        //WeldDoutPort: 1
    int fInchingPosOutState;                   //WeldDoutPort: 2
    int fInchingNegOutState;                   //WeldDoutPort: 3
    int fTouchStartOutState;                   //WeldDoutPort: 4
    int fTouchReadyOutState;                   //WeldDoutPort: 5
    int fWireCutOutState;                      //WeldDoutPort: 6

    // Analog Output (Weld)
    double dbWeldVoltOutVal;                   //WeldAoutPort: 0
    double dbWeldCurrOutVal;                   //WeldAoutPort: 1
#if 0
    SC_DATA_WELD_DOUT              weldDoutState;   //0: Arc ON          1: Gas On       2: Wire Inching(+)
                                                    //3: Wrie Inching(-) 4: Touch Start  5: Touch Ready(MC On)
                                                    //6: Wire Cut
    SC_DATA_WELD_AOUT              weldAoutState;   //0: Volt Out        1: Current Out
#endif
    int     nDoutPortVal[ROBOT_DO_PORT_COUNT];
    double  dbAoutPortVal[ROBOT_AO_PORT_COUNT];
} OUTPUT_STATE;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct t_input_state
{
    // Digital Input (Axis)
    int fBrakeStatusInState[ROB_AXIS_COUNT];   //AxisDinPort: 0~5
    int fBrakeClearInState[ROB_AXIS_COUNT];    //AxisDinPort: 6~11
    
    // Digital Input (System)
    int fTP_EstopInState;                      //SysDinPort: 0
    int fController_EstopInState;              //SysDinPort: 1
    int fDeadManSwithInState;                  //SysDinPort: 2
    int fShockSensorInState;                   //SysDinPort: 3
    int fController_ResetButton;
    int fCart_EstopInState;
    int fCartJobStartButton;
    int fCartCylinderState;

    // Digital Input (Weld)
    int fArcOnInState;                         //WeldDinPort: 0
    int fNoGasInState;                         //WeldDinPort: 1
    int fNoWireInState;                        //WeldDinPort: 2
    int fWeldPowerFailInState;                 //WeldDinPort: 3
    int fTouchProcessInState;                  //WeldDinPort: 4
    int fTouchSignalInState;                   //WeldDinPort: 5

    // Analog Input (Weld)
    double dbWeldVoltInVal;                    //WeldAinPort: 0
    double dbWeldCurrInVal;                    //WeldAinPort: 1
    
    // Position Data
    double dbActPos[MAX_AXIS_COUNT];           // actual position(rad)
#if 0
    int    nCoordIdx;                          // coordinate inform
    double dbCurrPos[MAX_AXIS_COUNT];          // position monitoring data

    SC_DATA_WELD_DIN               weldDinState;    //0: Arc                 1: No Gas           2: No Wire
                                                    //3: Welder Power Fail   4: Touch Process    5: Touch Signal
    SC_DATA_WELD_AIN               weldAinState;    //0: Volt In             1: Current In
#endif
    int     nDinPortVal[ROBOT_DI_PORT_COUNT];
    double  dbAinPortVal[ROBOT_AI_PORT_COUNT];
} INPUT_STATE;
#pragma pack(pop)


////////////////////////////////////////////////////////////////////////////////
// Shared Memory Body
//

#pragma pack(push, 1)
typedef struct t_shm_sc_system
{
    int  nsize;                         // size of shared memory
    
    /* System State */
    SYSTEM_STATE        sysstate;

    /* Output Command */
    OUTPUT_CMD          outputcmd;

    /* Output State Data */
    OUTPUT_STATE        outputstate;

    /* Input State Data */
    INPUT_STATE         inputstate;

} SHM_SC_SYSTEM;
#pragma pack(pop)


#endif  // SERVOCON_IPC_H__ 
