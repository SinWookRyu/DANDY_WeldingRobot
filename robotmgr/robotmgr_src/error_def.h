#ifndef __ERROR_DEF_H__
#define __ERROR_DEF_H__

#include "ipc_robotmgr.h"

#define     MAX_ERROR_STACK_SIZE        2048

///////////////////////////////////////
//
// Estop Code Symbol
//

#define SYS_ESTOP_NONE                        0Xf300
#define SYS_ESTOP_TP                          0Xf301
#define SYS_ESTOP_SERV                        0Xf302
#define SYS_ESTOP_SHOCKSENSOR                 0Xf303
#define SYS_ESTOP_CONTROLBOX                  0Xf304
#define SYS_ESTOP_CART                        0Xf305
#define SYS_ESTOP_HWLIMIT_ACT                 0Xf306

///////////////////////////////////////
//
// ERROR Code Symbol
//

#define SYS_ERR_OK                            0x0000

/////////////////////////////////////////////////
//  Error Code Header Define: 0xffffff
//
//  0xf000000: error owner in error state
//  0x0ff0000: error module in error state
/////////////////////////////////////////////////
typedef enum _ErrCodeHeader
{
    // Owner
    ERR_OWNER_FROM_RM                       = 0X1000000,
    ERR_OWNER_FROM_TE                       = 0X2000000,
    ERR_OWNER_FROM_SC                       = 0X3000000,

    // Module
        // Owner: SC
    ERR_MOD_FROM_AXIS1                      = 0X0010000, //Axis1 ~ Axis6
    ERR_MOD_FROM_AXIS2                      = 0X0020000,
    ERR_MOD_FROM_AXIS3                      = 0X0030000,
    ERR_MOD_FROM_AXIS4                      = 0X0040000,
    ERR_MOD_FROM_AXIS5                      = 0X0050000,
    ERR_MOD_FROM_AXIS6                      = 0X0060000,
    ERR_MOD_FROM_NOTAXIS                    = 0X00f0000,
        
        // Owner: RM
    ERR_MOD_CONF_RELOAD                     = 0X0100000,
    ERR_MOD_JOB_COMPILE                     = 0X0200000,
    ERR_MOD_PARAM_VALIDCHECK                = 0X0300000,
    ERR_MOD_PARAM_EDIT                      = 0X0400000,
    ERR_MOD_SYSTEM                          = 0X0500000,
    ERR_MOD_FILE                            = 0X0600000,
    ERR_MOD_JOB_EXEC                        = 0X0700000,
} ErrCodeHeader;

/////////////////////////////////////////////////
//  Error Code Header Define: 0xfffffff
//
//  0x000ffff: error code data
/////////////////////////////////////////////////

/* -------------------------------------------------------- */
    //system error (RM)
typedef enum _SystemError
{
    SYS_ERR_INIT_RM                         = 0X0111,
    SYS_ERR_INIT_TE                         = 0X0112,
    SYS_ERR_INIT_SC                         = 0X0113,
    SYS_ERR_INIT_CONF_LOAD                  = 0X0114,
    SYS_ERR_FINALIZE_RM                     = 0X0115,
    SYS_ERR_FINALIZE_TE                     = 0X0116,
    SYS_ERR_FINALIZE_SC                     = 0X0117,
    SYS_ERR_PROC_ALIVE_TE                   = 0X0118,
    SYS_ERR_PROC_ALIVE_SC                   = 0X0119,
} SystemError;

    //config file load error (RM)
typedef enum _ConfLoadError
{
    SYS_ERR_SYNTAX_GLOBAL_PARAM             = 0X0150,
    SYS_ERR_SYNTAX_ROBOT_PARAM              = 0X0151,
    SYS_ERR_SYNTAX_AXIS_PARAM               = 0X0152,
    SYS_ERR_SYNTAX_MOTOR_PARAM              = 0X0153,
    SYS_ERR_SYNTAX_WELDER_PARAM             = 0X0154,
    SYS_ERR_SYNTAX_USER_PARAM               = 0X0155,
    SYS_ERR_OPEN_RESTART_PARAM              = 0X0156,
    SYS_ERR_SYNTAX_STATISTICS_PARAM         = 0X0157,
    SYS_ERR_OPEN_STATISTICS_PARAM           = 0X0158,
    SYS_ERR_OPEN_CONST_VAR                  = 0X0159,
    //SVC_ERR_CONF_RELOAD                     = 0X0160, //replaced by module inform
} ConfLoadError;

    //file handling error (RM)
typedef enum _FileHandlingError
{
    SVC_ERR_USER_PARAM_SAVING               = 0X0200,
    SVC_ERR_RESTART_PARAM_SAVING            = 0X0201,
    SVC_ERR_STATISTICS_DATA_SAVING          = 0X0202,
    SVC_ERR_GET_STATISTICS_DATA             = 0X0203,
    SVC_ERR_CONST_VAR_SAVING                = 0X0204,
    SVC_ERR_BLANK_JOB_OPEN                  = 0X0205,
} FileHandlingError;

    //job executing error (RM)
typedef enum _JobExecutingError
{
    SVC_ERR_JOBCMD_PROC                     = 0X0250,
    SVC_ERR_SERVO_OFF                       = 0X0251,
    SVC_ERR_JOBDATA_COMM                    = 0X0252,
    SVC_ERR_JOB_MON                         = 0X0253,
    SVC_ERR_JOB_EDIT                        = 0X0254,
    SVC_ERR_JOB_SAVE                        = 0X0255,
    SVC_ERR_JOB_ALREADY_EXEC                = 0X0256,
    SVC_ERR_JOBEXE_TE_REACT                 = 0X0257,
    SVC_ERR_NOT_READY_RESTART               = 0X0258,
    SVC_ERR_JOBEXECAUTO                     = 0X0260,
    SVC_ERR_JOBEXECDRY                      = 0X0261,
    SVC_ERR_JOBSTOP                         = 0X0262,
    SVC_ERR_JOBEXECSTEP                     = 0X0263,
    SVC_ERR_JOB_EXECRESTART                 = 0X0264,
    SVC_ERR_CART_CYLINDER_STATE             = 0X0265,
} JobExecutingError;

    //service error (RM)
typedef enum _ServiceError
{
    SVC_ERR_JOB_SNDHEADER                   = 0X0300,
    SVC_ERR_JOB_SNDBODY                     = 0X0301,
    SVC_ERR_JOB_RCVHEADER                   = 0X0302,
    SVC_ERR_JOB_RCVBODY                     = 0X0303,
    SVC_ERR_HOME_MOVE                       = 0X0304,
    SVC_ERR_WIRECUT_JOB_CALL                = 0X0305,
    SVC_ERR_ERRHIST_STACK_FULL              = 0X0306,
    SVC_ERR_ZIPFILE_UNCOMPRESS              = 0X0307,
} ServiceError;

    //parameter edit error (RM)
typedef enum _ParamEditError
{
    SVC_ERR_HOMEPOS_EDIT                    = 0X0350,
    SVC_ERR_USRCRD_EDIT                     = 0X0351,
    SVC_ERR_TCP_EDIT                        = 0X0352,
    SVC_ERR_WORLD_EDIT                      = 0X0353,
    SVC_ERR_WELDIN_TUNE_EDIT                = 0X0354,
    SVC_ERR_WELDOUT_TUNE_EDIT               = 0X0355,
    SVC_ERR_RESTART_PARAM_EDIT              = 0X0356,
    SVC_ERR_ARCSENS_PARAM_EDIT              = 0X0357,
    SVC_ERR_GAP_COND_SET                    = 0X0358,
    SVC_ERR_CONST_VAR_EDIT                  = 0X0359,
} ParamEditError;

    //parameter validation error (RM)
typedef enum _ParamValidError
{
    ERR_WOUT_VOLT_CALIB_PARAM_INVALID         = 0X0400,
    ERR_VOLT_PARAM_MAX_DISCRIMINANT_IMAGINARY = 0X0401,
    ERR_VOLT_PARAM_MIN_DISCRIMINANT_IMAGINARY = 0X0402,
    ERR_VOLT_PARAM_C_VALUE_LOW                = 0X0403,
    ERR_VOLT_PARAM_C_VALUE_HIGH               = 0X0404,
    ERR_WOUT_CURR_CALIB_PARAM_INVALID         = 0X0405,
    ERR_CURR_PARAM_MAX_DISCRIMINANT_IMAGINARY = 0X0406,
    ERR_CURR_PARAM_MIN_DISCRIMINANT_IMAGINARY = 0X0407,
    ERR_CURR_PARAM_C_VALUE_LOW                = 0X0408,
    ERR_CURR_PARAM_C_VALUE_HIGH               = 0X0409,
    ERR_NOT_VALID_MATRIX_DIMENSION            = 0X0410,
    ERR_SOVLE_INVERSE_MATRIX                  = 0X0411,
    ERR_VOLT_RATIO_MEASURED_AND_CALCULATED    = 0X0412,
    ERR_CURR_RATIO_MEASURED_AND_CALCULATED    = 0X0413,
} ParamValidError;

/* -------------------------------------------------------- */
    //ethercat error (SC)
typedef enum _EcatError
{
    ECAT_ERR_INIT_ECAT_FAIL                 = 0X0130,
    ECAT_ERR_CHANGE_OP_MODE                 = 0X0131,
    ECAT_ERR_LOAD_XML_CONFIG                = 0X0132,
} EcatError;

    //control service error (SC)
typedef enum _ControlServiceError
{
    SVC_ERR_SENSDATA_SAVING                 = 0X0142,
    SVC_ERR_ABS_ENC_RESET                   = 0X0143,
    SVC_ERR_SET_ZERO_POSITION               = 0X0144,
    SVC_ERR_POS_DATA_SAVING                 = 0X0145,
} ControlServiceError;

    //I/O error (SC)
typedef enum _IOError
{
    IO_ERR_HW_LIMIT_DETECT                  = 0X0200,   //140326
} IOError;


/////////////////////////////////////////////////////////////////////////////
//
//  assemble error list
/* Defined in 'dandy_jobasm.h' */       //module: ERR_MOD_JOB_COMPILE

#if 0
#define JOBASM_ERR_NO_ERROR                 0
#define JOBASM_NO_ERROR                     JOBASM_ERR_NO_ERROR
#define JOBASM_OK                           JOBASM_ERR_NO_ERROR

#define JOBASM_ERR_INTERNAL                 1   // assembler internal error
#define JOBASM_ERR_ASSEMBLER                2   // assembler handle invalid
#define JOBASM_ERR_SYNTAX                   3   // syntax error
#define JOBASM_ERR_UNSUPPORTED              4
#define JOBASM_ERR_TYPE_CONSTANT            5   // argument type constant cannot be assigne
#define JOBASM_ERR_TYPE_MISMATCH            6   // argument type mismatch
#define JOBASM_ERR_SYM_RESERVED             7   // reserved symbol
#define JOBASM_ERR_SYM_NAME                 8   // symbol value invalid
#define JOBASM_ERR_SYM_DUPLICATE            9   // duplicated symbol
#define JOBASM_ERR_SYM_MISSING_VAL          10  // symbol value is not defined
#define JOBASM_ERR_SYM_INVALID              11  // symbol value invalid
#define JOBASM_ERR_SYM_UNDEF                12  // local symbol not found
#define JOBASM_ERR_SYM_UNLINK               13  // global symbol not found
#define JOBASM_ERR_SYM_CROSS                14  // symbol cross referenced
#define JOBASM_ERR_FILE_BROKEN              15  // encounter file error
#define JOBASM_ERR_FILE_SEEK                16  // file seek error
#define JOBASM_ERR_NO_SOURCE                17  // no source to assemble
#define JOBASM_ERR_TOO_MANY_SOURCE          18  // too many source file to assemble
#define JOBASM_ERR_UNKNOWN_CMD              19
#define JOBASM_ERR_INVALID_CMD              20
#define JOBASM_ERR_INVALID_POS              21
#define JOBASM_ERR_INVALID_TARGET           22
#define JOBASM_ERR_INVALID_VIAWAY           23
#define JOBASM_ERR_INVALID_VELOCITY         24
#define JOBASM_ERR_INVALID_TIME             25
#define JOBASM_ERR_INVALID_WEAV             26
#define JOBASM_ERR_INVALID_WEAV_EXPR        27
#define JOBASM_ERR_INVALID_WELD             28
#define JOBASM_ERR_INVALID_SWF              29
#define JOBASM_ERR_INVALID_MWF              30
#define JOBASM_ERR_INVALID_EWF              31
#define JOBASM_ERR_INVALID_WELD_EXPR        32
#define JOBASM_ERR_INVALID_VAR              33  // invalid variable
#define JOBASM_ERR_INVALID_ARG              34  // invalid argument
#define JOBASM_ERR_INVALID_COORD            35
#define JOBASM_ERR_MISSING_OBJECT           36  // missing sensor object number
#define JOBASM_ERR_BRANCH_RANGE             37
#define JOBASM_ERR_EOF                      38  // not error, just end of file
#define JOBASM_ERR_MEMORY_EXHAUST           39  //
#define JOBASM_ERR_PATH_NAME_TOO_LONG       40
#define JOBASM_ERR_FILE_NAME_TOO_LONG       41
#define JOBASM_ERR_FILE_OPEN                42
#define JOBASM_ERR_FILE_INCLUDE             43  // internal only, not error but for include
#endif

typedef enum _ServoPackError
{
    SERVO_ERR_OK                                                    = 0x0000,
    SERVO_ERR_PARAMETER_CHECKSUM_1                                  = 0x0020,
    SERVO_ERR_PARAMETER_FORMAT_1                                    = 0x0021,
    SERVO_ERR_SYSTEM_CHECKSUM_1                                     = 0x0022,
    SERVO_ERR_MAIN_CIRCUIT_DECTECTOR_1                              = 0x0030,
    SERVO_ERR_PARAMETER_SETTING_1                                   = 0x0040,
    SERVO_ERR_ENCODER_OUTPULSE_SETTING                              = 0x0041,
    SERVO_ERR_PARAMETER_COMBINATION                                 = 0x0042,
    SERVO_ERR_SEMICLOSED_FULLCLOSED_CONTROL_PARAMETER_SETTING       = 0x0044,
    SERVO_ERR_COMBINATION                                           = 0x0050,
    SERVO_ERR_UNSUPPORTED_DEVICE_ALARM                              = 0x0051,
    SERVO_ERR_CANCELLED_SERVOON_CMD_ALARM                           = 0x00b0,
    SERVO_ERR_OVERCURRENT_HEATSINK_OVERHEATED                       = 0x0100,
    SERVO_ERR_REGENERATION                                          = 0x0300,
    SERVO_ERR_REGENERATIVE_OVERLOAD                                 = 0x0320,
    SERVO_ERR_MAIN_CIRCUIT_PWR_SUPPLY_WIRING                        = 0x0330,
    SERVO_ERR_OVERVOLTAGE                                           = 0X0400,
    SERVO_ERR_UNDERVOLTAGE                                          = 0X0410,
    SERVO_ERR_MAIN_CIRCUIT_CAPACITOR_OVERVOLTAGE                    = 0X0450,
    SERVO_ERR_OVERSPEED                                             = 0X0510,
    SERVO_ERR_OVERSPEED_ENCODER_OUTPUT_PULSE_RATE                   = 0X0511,
    SERVO_ERR_VIBRATION_ALARM                                       = 0X0520,
    SERVO_ERR_AUTOTUNING_ALARM                                      = 0X0521,
    SERVO_ERR_OVERLOAD_HIGH_LOAD                                    = 0X0710,
    SERVO_ERR_OVERLOAD_LOW_LOAD                                     = 0X0720,
    SERVO_ERR_DYNAMIC_BRAKE_OVERLOAD1                               = 0X0730,
    SERVO_ERR_DYNAMIC_BRAKE_OVERLOAD2                               = 0X0731,
    SERVO_ERR_OVERLOAD_SURGE_CURRENT_LIMIT_RESISTOR                 = 0X0740,
    SERVO_ERR_HEATSINK_OVERHEATED                                   = 0X07A0,
    SERVO_ERR_BUILT_IN_FAN_STOPPED                                  = 0X07AB,
    SERVO_ERR_ENCODER_BACKUP                                        = 0X0810,
    SERVO_ERR_ENCODER_CHECKSUM                                      = 0X0820,
    SERVO_ERR_ABS_ENCODER_BATTERY                                   = 0X0830,
    SERVO_ERR_ENCODER_DATA                                          = 0X0840,
    SERVO_ERR_ENCODER_OVER_SPEED                                    = 0X0850,
    SERVO_ERR_ENCODER_OVER_HEATED                                   = 0X0860,
    SERVO_ERR_CURRENT_DETECTION_1_PHASE_U                           = 0X0b31,
    SERVO_ERR_CURRENT_DETECTION_2_PHASE_V                           = 0X0b32,
    SERVO_ERR_CURRENT_DETECTION_3_CURRENT_DETECTOR                  = 0X0b33,
    SERVO_ERR_SYSTEM_ALARM_0                                        = 0X0bF0,
    SERVO_ERR_SYSTEM_ALARM_1                                        = 0X0bF1,
    SERVO_ERR_SYSTEM_ALARM_2                                        = 0X0bF2,
    SERVO_ERR_SYSTEM_ALARM_3                                        = 0X0bF3,
    SERVO_ERR_SYSTEM_ALARM_4                                        = 0X0bF4,
    SERVO_ERR_OVERRUN_DETECTED                                      = 0X0C10,
    SERVO_ERR_ABS_ENCODER_CLEAR_AND_MULTITURN_LIMIT_SET             = 0X0C80,
    SERVO_ERR_ENCODER_COMMUNICATION                                 = 0X0C90,
    SERVO_ERR_ENCODER_COMMUNICATION_POSITION_DATA                   = 0X0C91,
    SERVO_ERR_ENCODER_COMMUNICATION_TIMER                           = 0X0C92,
    SERVO_ERR_ENCODER_PARAMETER                                     = 0X0CA0,
    SERVO_ERR_ENCODER_ECHOBACK                                      = 0X0Cb0,
    SERVO_ERR_MULTI_TURN_LIMIT_DISAGREEMENT                         = 0X0CC0,
    SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW                           = 0X0d00,
    SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW_ALARM_AT_SERVOON          = 0X0d01,
    SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW_ALARM_BY_SPEEDLIMIT       = 0X0d02,
    SERVO_ERR_POSITION_DATA_OVERFLOW                                = 0X0d30,
    SERVO_ERR_CMD_OPTION_MODULE_IF_INIT_TIMEOUT                     = 0X0E00,
    SERVO_ERR_CMD_OPTION_MODULE_IF_SYNC                             = 0X0E02,
    SERVO_ERR_CMD_OPTION_MODULE_IF_COMMUNICATION_DATA               = 0X0E03,
    SERVO_ERR_CMD_OPTION_MODULE_DETECTION_FAIL_ALARM                = 0X0E70,
    SERVO_ERR_SAFETY_OPTION_MODULE_DETECTION_FAIL_ALARM             = 0X0E71,
    SERVO_ERR_UNSUPPORTED_CMD_OPTION_MODULE_DETECTION_FAIL_ALARM    = 0X0E73,
    SERVO_ERR_UNSUPPORTED_SAFETY_OPTION_MODULE_DETECTION_FAIL_ALARM = 0X0E74,
    SERVO_ERR_CMD_OPTION_MODULE_DETECTION_DISAGREE_ALARM            = 0X0E80,
    SERVO_ERR_SAFETY_DEVICE_SIGNAL_INPUT_TIMING                     = 0X0Eb1,
    SERVO_ERR_MAIN_CIRCUIT_CABLE_OPEN_PHASE                         = 0X0F10,
} ServoPackError;


typedef enum _ServoPackWarning
{
    SERVO_WARN_POSITION_ERR_PULSE_OVERFLOW                          = 0X0900,
    SERVO_WARN_POSITION_ERR_PULSE_OVERFLOW_ALARM_AT_SERVOON         = 0X0901,
    SERVO_WARN_OVERLOAD                                             = 0X0910,
    SERVO_WARN_VIBRATION                                            = 0X0911,
    SERVO_WARN_REGENERATIVE_OVERLOAD                                = 0X0920,
    SERVO_WARN_DYNAMIC_BRAKE_OVERLOAD                               = 0X0921,
    SERVO_WARN_ABS_ENCODER_BATTERY                                  = 0X0930,
    SERVO_WARN_UNDERVOLTAGE                                         = 0X0971,
} ServoPackWarning;


typedef enum _ServoEtherCATError
{
    ECAT_ERR_CMD_OPTION_IF_SERVO_UNIT_INIT                          = 0X0EA0,
    ECAT_ERR_CMD_OPTION_IF_MEMORY_CHECK                             = 0X0EA1,
    ECAT_ERR_CMD_OPTION_IF_SERVO_SYNC                               = 0X0EA2,
    ECAT_ERR_CMD_OPTION_IF_SERVO_DATA                               = 0X0EA3,
    ECAT_ERR_ECAT_DC_SYNC                                           = 0X0A10,
    ECAT_ERR_ECAT_STATE                                             = 0X0A11,
    ECAT_ERR_ECAT_OUTPUT_DATA_SYNC                                  = 0X0A12,
    ECAT_ERR_PARAMETER_SETTING                                      = 0X0A20,
    ECAT_ERR_SYSTEM_INIT                                            = 0X0A40,
    ECAT_ERR_COMMUNICATION_DEVICE_INIT                              = 0X0A41,
    ECAT_ERR_LOADING_SERVO_INFORM                                   = 0X0A47,
    ECAT_ERR_EEPROM_PARAMETER_DATA                                  = 0X0A48,
    ECAT_ERR_SLAVE_CONNECTION_CLOSED                                = 0X0A50,
} ServoEtherCATError;


/* Defined in 'ecattypes.h' */
/*! @brief Constants that represent various types of failures and successes.
    @anchor ECAT_RESULT
    Constants starting with "ECAT_S_" mean various types of successful results.
    Constants starting with "ECAT_E_" mean various types of failures.
    When masked with 0x8000 success-constants give 0x0000 and
    failure-constants give 0x8000.
    @ingroup master_sys_msg
    @see ECAT_SUCCEEDED, ECAT_FAILED, EcatGetErrorMessage */
#if 0 
typedef enum _EcatMasterError
{
    ECAT_S_OK                           = 0x0000,   /*!< No error. */
    ECAT_S_SIZE_LIMIT                   = 0x0001,   /*!< Buffer size limit for current call. Require more calls. */
    ECAT_S_NOITEMS                      = 0x0002,   /*!< No more items (for get next functions). */
    ECAT_S_PI_ALREADY_SYNC              = 0x0003,   /*!< Process Image double buffer is already synchronized. */
    ECAT_S_NOITEMS_MAILBOX_WAIT         = 0x0004,   /*!< No more items, but wait for new events(mailbox). */
    ECAT_S_TIMEOUT                      = 0x0005,   /*!< Timeout. */
    ECAT_S_THREAD_EXIT                  = 0x0006,   /*!< Thread exit code. */
    ECAT_S_SYNC_CALL                    = 0x0007,   /*!< Call need synchronization (internal). */
    ECAT_S_REDUNDANCY                   = 0x0008,   /*!< Indicate network redundancy state. */
    ECAT_S_AGAIN                        = 0x0009,   /*!< Try again. */

    ECAT_S_INVALID_LICENSE              = 0x0100,   /*!< Invalid license. */
    ECAT_S_DEMO_TIME_EXPIRED            = 0x0101,   /*!< Demo time expired. */

    ECAT_E_OUTOFMEMORY                  = 0x8001,   /*!< Memory limit(usually can not allocate new memory). */
    ECAT_E_INVALIDARG                   = 0x8002,   /*!< Function calls with invalid arguments. */
    ECAT_E_FAIL                         = 0x8003,   /*!< General error. */
    ECAT_E_NOTIMPL                      = 0x8004,   /*!< Function not implemented. */
    ECAT_E_XML_OPEN                     = 0x8005,   /*!< Can't open xml file. */
    ECAT_E_XML_PARSE                    = 0x8006,   /*!< Xml configuration file contains errors. */
    ECAT_E_MASTER_NOT_CONFIGURED        = 0x8007,   /*!< Master not configured. You mast configure master before call. */
    ECAT_E_MASTER_NOT_CONNECTED         = 0x8008,   /*!< Master not connected. You mast connect master before call. */
    ECAT_E_MASTER_ALREADY_CONNECTED     = 0x8009,   /*!< Master already connected. */
    ECAT_E_INVALID_SLAVE_INDEX          = 0x800A,   /*!< Invalid slave index. */
    ECAT_E_INVALID_TRANSITION           = 0x800B,   /*!< Invalid transition. Can't transit master to request state. */
    ECAT_E_DRIVER_LOAD                  = 0x800C,   /*!< Can't load driver. */
    ECAT_E_DRIVER_INIT                  = 0x800D,   /*!< Can't initialize driver. */
    ECAT_E_DRIVER_NOT_SUPPORTED         = 0x800E,   /*!< Driver not supported. */
    ECAT_E_DRIVER_NOT_LOADED            = 0x800F,   /*!< Driver not loaded. */
    ECAT_E_SEND_FRAME_FAILED            = 0x8010,   /*!< Can't send frame. */
    ECAT_E_RECV_FRAME_FAILED            = 0x8011,   /*!< Can't receive frame. */
    ECAT_E_TRACE_BUFFER_OVERFLOW        = 0x8012,   /*!< Trace buffer overflow. */
    ECAT_E_NOTAVIABLE_TRANSITION        = 0x8013,   /*!< Incorrect transition for current master state. */
    ECAT_E_TRANSITION_ERROR             = 0x8014,   /*!< Transition error. */
    ECAT_E_INVALID_POINTER              = 0x8015,   /*!< Invalid pointer. */
    ECAT_E_INVALID_COMMAND_TYPE         = 0x8016,   /*!< Invalid command type. */
    ECAT_E_TRANSITION_FORBIDDEN         = 0x8017,   /*!< Transition forbidden (can't send frames to the network). */
    ECAT_E_SYNC_CALL_TIMEOUT            = 0x8018,   /*!< Call timeout. */
    ECAT_E_MB_COE_TRANSITION_ABORTED    = 0x8019,   /*!< Mailbox CanOpen transition aborted. */
    ECAT_E_INVALID_WC                   = 0x801A,   /*!< Invalid working counter received. */
    ECAT_E_INVALID_SLAVE_STATE          = 0x801B,   /*!< Invalid slave state. */
    ECAT_E_SNAPSHOT_DATA_LOSE           = 0x801C,   /*!< Data lose at snapshot call. */
    ECAT_E_XML_LICENSE_OPEN             = 0x801D,   /*!< Can't open license xml file. */
    ECAT_E_XML_LICENSE_PARSE            = 0x801E,   /*!< License Xml configuration file contains errors. */
    ECAT_E_XML_LICENSE_PRODUCT_NAME     = 0x801F,   /*!< License Xml configuration file doesn't contain Product tag. */
    ECAT_E_XML_LICENSE_LICENSED_TO      = 0x8020,   /*!< License Xml configuration file doesn't contain LicensedTo tag. */
    ECAT_E_XML_LICENSE_EXP_DATE         = 0x8021,   /*!< License Xml configuration file doesn't contain ExpirationDate tag. */
    ECAT_E_XML_LICENSE_DEMO_MODE        = 0x8022,   /*!< License Xml configuration file doesn't contain DemoMode tag. */
    ECAT_E_XML_LICENSE_HW_BINDING       = 0x8023,   /*!< License Xml configuration file doesn't contain HardwareBinding tag. */
    ECAT_E_XML_LICENSE_PRODUCT_KEY      = 0x8024,   /*!< License Xml configuration file doesn't contain ProductKey tag. */
    ECAT_E_CMD_ACYCLIC_SET_DATA         = 0x8025,   /*!< Acyclic cmd data is not set. */
    ECAT_E_CMD_CONTAINER_ADD_RES_CMD    = 0x8026,   /*!< Can't add reserved cmd to command container. */
    ECAT_E_BUFFER_SIZE_LIMIT            = 0x8027,   /*!< Buffer size limit. */
    ECAT_E_TELEGRAM_BUILD               = 0x8028,   /*!< Can't build ecat telegram. */
    ECAT_E_FRAME_BUILD                  = 0x8029,   /*!< Can't build ecat frame. */
    ECAT_E_LOAD_FORBIDDEN               = 0x802A,   /*!< Can't load configuration at current master state. */
    ECAT_E_INVALID_CONFIGURATION        = 0x802B,   /*!< Can't load configuration because verification errors. */
    ECAT_E_READ_SLAVE_STATES            = 0x802C,   /*!< Can't read slave states from network. */
    ECAT_E_TRANSIT_SLAVES               = 0x802D,   /*!< Can't transit slave to request state. */
    ECAT_E_SEND_MASTER_CMDS             = 0x802E,   /*!< Can't send master init commands. */
    ECAT_E_MASTER_ALREADY_STARTED       = 0x802F,   /*!< Master already started. */
    ECAT_E_THREAD_INITIALISE            = 0x8030,   /*!< Can't initialize thread. */
    ECAT_E_READ_EEPROM                  = 0x8031,   /*!< Error occurred while reading EEPROM. */
    ECAT_E_MB_NOT_SUPPORTED             = 0x8032,   /*!< Mailbox not supported. */
    ECAT_E_MB_COE_NOT_SUPPORTED         = 0x8033,   /*!< Mailbox CoE not supported. */
    ECAT_E_PI_ALREADY_UNLOCKED          = 0x8034,   /*!< Process Image already unlocked. */

    ECAT_E_INVALID_FRAME_SIZE           = 0x8035,   /*!< Process frame error: invalid frame size. */
    ECAT_E_INVALID_FRAME_TYPE           = 0x8036,   /*!< Process frame error: unsupported EtherCAT?frame type. */
    ECAT_E_INVALID_COMMAND_SIZE         = 0x8037,   /*!< Process frame error: invalid command length. */
    ECAT_E_INVALID_COMMAND_DATA         = 0x8038,   /*!< Process frame error: invalid internal data. */

    ECAT_E_WRITE_EEPROM                 = 0x8039,   /*!< Error occurred while writing EEPROM. */
    ECAT_E_MASTER_NOT_STARTED           = 0x803A,   /*!< Master not started. You mast start master before call. */

    ECAT_E_ARRAY_BOUNDS_EXCEEDED        = 0x803B,   /*!< Array bounds error exceeded. */
    ECAT_E_INVALID_LICENSE              = 0x803C,   /*!< Invalid license. */

    ECAT_E_RECV_FRAME_TIMEOUT           = 0x803D,   /*!< Frame receive time-out. */

    ECAT_E_EEPROM_ACCESS_DENIED         = 0x803E,   /*!< Access to EEPROM temporarily denied. */
    ECAT_E_EEPROM_ACCESS_DENIDED        = 0x803E,   /*!< Left for compatibility.
                                                         Please use #ECAT_E_EEPROM_ACCESS_DENIED
                                                         instead of this symbol. */

    ECAT_E_BUSY                         = 0x803F,   /*!< Busy. */

    ECAT_E_OPEN_FILE                    = 0x8050,   /*!< Can't open file. */
    ECAT_E_WRITE_FILE                   = 0x8051,   /*!< Can't write to file. */
    ECAT_E_READ_FILE                    = 0x8052,   /*!< Can't read from file. */
    ECAT_E_CANT_GET_HW_KEY              = 0x8053,   /*!< Can't obtain hardware key. */
    ECAT_E_CANT_ENCODE_DATA             = 0x8054,   /*!< Can't encode data. */
    ECAT_E_CANT_DECODE_DATA             = 0x8055,   /*!< Can't decode data. */

    ECAT_E_MB_COE_UNSUPPORTED_DATA_TYPE = 0x8100,   /*!< CoE: Unsupported CANOpen data type. */
    ECAT_E_MB_COE_OBJECT_IS_READ_ONLY   = 0x8101,   /*!< CoE: Access to read only object. */
    ECAT_E_MB_COE_OBJECT_IS_WRITE_ONLY  = 0x8102,   /*!< CoE: Access to write only object. */
    ECAT_E_MB_COE_INCOMPLETE_ENTRY_DESC = 0x8103,   /*!< CoE: Incomplete object entry description. */
    ECAT_E_MB_COE_SDOINFO_ERROR         = 0x8104,   /*!< CoE: SDO information error response. */

    ECAT_E_MB_VOE_NOT_SUPPORTED         = 0x8150,   /*!< Mailbox VoE not supported. */
    ECAT_E_MASTER_IS_LOCKED             = 0x8200,   /*!< Master is already locked. */
    ECAT_E_MASTER_NO_RIGHTS             = 0x8201,   /*!< No rights to complete this operation. */
    ECAT_E_MASTER_USER_NOT_FOUND        = 0x8202,   /*!< User not found in the configuration. */

    ECAT_E_AOE_ERROR_CODE               = 0x8300,   /*!< Error code in AoE header is received. */
    ECAT_E_AOE_CMD_ERROR                = 0x8301,   /*!< Result in AoE command response not 0. */
    ECAT_E_AOE_INVALID_HEADER           = 0x8302,   /*!< Failed to build/parse AoE header. */
    ECAT_E_MB_AOE_NOT_SUPPORTED         = 0x8303,   /*!< Mailbox AoE not supported. */
    ECAT_E_AOE_INVALID_ROUTE            = 0x8304,   /*!< Route not found. */

    ECAT_E_MB_SOE_NOT_SUPPORTED         = 0x8310,   /*!< Mailbox SoE not supported. */
    ECAT_E_SOE_ERROR_CODE               = 0x8311,   /*!< Error bit in SoE response is true. See error code for more information. */
    ECAT_E_SOE_INVALID_HEADER           = 0x8312,   /*!< Failed to build/parse SoE header. */
    ECAT_E_SOE_FRAGMENT_LEFT            = 0x8313,   /*!< SoE Fragment is left. */


    ECAT_E_OD_OBJECT_NOTFOUND                   = 0x8400, /*!< No object in OD. */
    ECAT_E_OD_OBJECT_ALREADY_EXISTS             = 0x8401, /*!< Object is already in OD. */
    ECAT_E_OD_ENTRY_DESCRIPTION_ALREADY_EXISTS  = 0x8402, /*!< OD entry description already exists. */
    ECAT_E_OD_ENTRY_DESCRIPTION_FAILED          = 0x8403, /*!< Failed to create OD entry description. */
    ECAT_E_OD_INVALID_OBJECT_TYPE               = 0x8404, /*!< OD object type is invalid. */
    ECAT_E_OD_INVALID_ACCESS_TYPE               = 0x8405, /*!< OD access type is invalid.  */
    ECAT_E_OD_INVALID_DATA_TYPE                 = 0x8406, /*!< Invalid OD data type.  */
    ECAT_E_OD_ACCESS_DENIED                     = 0x8407, /*!< Access to OD denied. */
    ECAT_E_OD_NOT_CREATED                       = 0x8408, /*!< OD has not been created. */

    ECAT_E_OD_SDO_SERVICE_NOT_SUPORTED          = 0x8409, /*!< Mailbox error: not supported CoE service in CoE header. */
    ECAT_E_OD_SDO_SIZE_INVALID_HEADER           = 0x840A, /*!< Mailbox error: invalid CoE SDO header. */
    ECAT_E_OD_SDO_SIZE_TOO_SHORT                = 0x840B, /*!< Mailbox error: CoE SDO service with Len < 10. */
    ECAT_E_OD_SDO_INVALID_SIZE                  = 0x840C, /*!< Mailbox error: invalid size. */

    ECAT_E_OD_SUB_OBJ_NOTFOUND                  = 0x840D, /*!< Sub-object not found in object. */
    ECAT_E_OD_MORE_MAXIMUM_VALUE                = 0x840E, /*!< Entry value is more than maximum. */
    ECAT_E_OD_LESS_MINIMUM_VALUE                = 0x840F, /*!< Entry value is less than minimum. */

	// licensig errors
	ECAT_E_LICENSE_INIT							= 0x8502,   /*!< Can't initialize licensing. Check licensing configuration */
	ECAT_E_LICENSE_LOAD							= 0x8503,   /*!< License load error (invalid(not exixting) license file, inconsistent content etc. ). */
	ECAT_E_LICENSE_INVALID_TARGET				= 0x8504,   /*!< Invalid licensing target (product, OS, class etc.)*/
	ECAT_E_LICENSE_EXPIRED						= 0x8505,   /*!< License time expired. */
	ECAT_E_LICENSE_INVALID_HW					= 0x8506,   /*!< Valid hardware is not detected. */
} EcatMasterError;
#endif


///////////////////////////////////////
//
// ERROR Code Stack Define
//  - szErrStackSysTime[][]: Error Time Stack Value
//  - nErrStack[]          : Error Code Stack Value
//  - nTop                 : Stack Index

#pragma pack(push, 1)
typedef struct t_error_code_stack
{
    int   nErrCnt;
    int   nErrHistoryIdx;
    FORMAT_DATE             ErrDate[MAX_ERROR_STACK_SIZE];
    FORMAT_TIME             ErrTime[MAX_ERROR_STACK_SIZE];
    int   nTop;                                                      // index of stack
    char  szErrStackSysTime[MAX_ERROR_STACK_SIZE][SYSTIME_DATA_LEN]; // error time stack
    int   nErrStack[MAX_ERROR_STACK_SIZE];                           // error history stack
    short fErrorActiveState[MAX_ERROR_STACK_SIZE];                   // error active state
    char  g_szErrContent[MAX_ERROR_STACK_SIZE][ERROR_NAME_LEN];
} ERROR_CODE_STACK;
#pragma pack(pop)

#endif