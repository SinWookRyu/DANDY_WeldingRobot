#ifndef ECATDEF_H__
#define ECATDEF_H__

///////////////////////////////////////////////////////////////////////
//
// Master configuration
//

#define DEF_ECAT_CONFIG_FILE_NAME        "/root/ethercat/dandy_master_131126_Kon.xml"

/* cable redundancy mode */
//#define REDUNDANCY_MODE                  1

// EtherCAT Sync mode

/* define sync mode */
//#define ECAT_MODE_SYNC2A                 1
//#define ECAT_MODE_SYNC2B                 1
#define ECAT_MODE_ASYNC                   1
 
/* sync mode cycle time */
#if defined (ECAT_MODE_SYNC2A) || (ECAT_MODE_SYNC2B) && (!ECAT_MODE_ASYNC)
#define ECAT_MASTER_CYCLE_TIMEOUT_US     0
#define ECAT_MASTER_SUBCYCLE_TIMEOUT_US  4000
#endif
 
/* async mode cycle time */
#if defined (ECAT_MODE_ASYNC) && (!ECAT_MODE_SYNC2A) && (!ECAT_MODE_SYNC2B)
#define ECAT_MASTER_CYCLE_TIMEOUT_US     1000
#define ECAT_MASTER_SUBCYCLE_TIMEOUT_US  10000
#endif

#define ECAT_LOAD_CONFIG_BUF_SIZE        1024
#define ECAT_AUTORECOVERY_TIMEOUT_MS     100
#define ECAT_MASTER_PRIORITY             160


///////////////////////////////////////////////
//
// Slave Offset Configuration (write/output)
//

#define DEF_WRITE_INIT_OFFSET_SIZE      0x00    //0x28
#define DEF_WRITE_ECAT_SRVDATA_SIZE     0x50    // size of output variable
#define WRITE_CONTROLWORD_SIZE          0X10
#define WRITE_POSITION_SIZE             0X20
#define WRITE_PHYSIC_OUTPUT_SIZE        0X20
#define DEF_WRITE_OFFSET_CONTROLWORD    DEF_WRITE_INIT_OFFSET_SIZE  //0x28 // start address of control word
#define DEF_WRITE_OFFSET_POSITION      ((DEF_WRITE_INIT_OFFSET_SIZE + \
                                        WRITE_CONTROLWORD_SIZE))     //0x38 // start address of target position
#define DEF_WRITE_OFFSET_OUTPUT        ((DEF_WRITE_INIT_OFFSET_SIZE + \
                                        WRITE_CONTROLWORD_SIZE + \
                                        WRITE_POSITION_SIZE))        //0x38 // start address of target position

// Servo Driver offset configuration (read/input)
#define DEF_READ_INIT_OFFSET_SIZE       0x00    //0x38
#define DEF_READ_ECAT_SRVDATA_SIZE      0x40    // size of input variable
#define READ_STATUS_SIZE                0X10
#define READ_POSITION_SIZE              0X20
#define READ_ERRORCODE_SIZE             0X10
#define DEF_READ_OFFSET_STATUS          DEF_READ_INIT_OFFSET_SIZE  //0x38  // start address of status word
#define DEF_READ_OFFSET_POSITION       ((DEF_READ_INIT_OFFSET_SIZE + \
                                        READ_STATUS_SIZE)) //0x48  // start address of position actual value(org: 0x58)
#define DEF_READ_OFFSET_ERROR          ((DEF_READ_INIT_OFFSET_SIZE + \
                                        READ_STATUS_SIZE + \
                                        READ_POSITION_SIZE)) //0x68  // start address of error code


///////////////////////////////////////////////////////////////////////
//
// I/O Device Structure
//  Servo 1~6 –> 1088(Brake State) –> 1088(Brake Clear) –> 4132(Weld Aout)
//  –> 3102(Weld Ain) -> 1088(Weld Din) –> 2088(Weld Dout) –> 2088(Lamp Dout)
//  -> 1088(Cart Din) -> 2088(Cart Dout)
//      - Input:  Brake State Din - Brake Clear Din - Weld Ain - Weld Din
//      - Output: Weld Aout - Weld Dout - Lamp Dout

// AO offset configuration (write/output)
#define ECAT_WRITE_AO_PORT_SIZE          0x10
#define ECAT_WRITE_AO_SLAVE_SIZE         (ECAT_WRITE_AO_PORT_SIZE * SLAVE_AO_PORT_COUNT)
#define ECAT_WRITE_AO_OFFSET             (ECAT_WRITE_DO_OFFSET + \
                                         ROBOT_DO_SLAVE_COUNT * ECAT_WRITE_DO_SLAVE_SIZE)  //0x1F8

// DO offset configuration (write/output)
#define ECAT_WRITE_DO_PORT_SIZE          0x01
#define ECAT_WRITE_DO_SLAVE_SIZE         (ECAT_WRITE_DO_PORT_SIZE * SLAVE_DO_PORT_COUNT)
#define ECAT_WRITE_DO_OFFSET             (DEF_AXIS_COUNT * DEF_WRITE_ECAT_SRVDATA_SIZE)    //0x1E0, 0x1E8, 0x1F0

// Cart DO offset configuration (write/output)
#define CONTROLLER_UIO_SIZE             (ECAT_WRITE_DO_OFFSET + 2 * ECAT_WRITE_DO_SLAVE_SIZE)
#define ECAT_WRITE_CART_DO_OFFSET       CONTROLLER_UIO_SIZE   //0x210

// DI offset configuration (read/input)
#define ECAT_READ_DI_PORT_SIZE          0x01
#define ECAT_READ_DI_SLAVE_SIZE         (ECAT_READ_DI_PORT_SIZE * SLAVE_DI_PORT_COUNT)
#define ECAT_READ_DI_OFFSET             DEF_AXIS_COUNT * DEF_READ_ECAT_SRVDATA_SIZE //0x180, 0x188, 0x1C0

// AI offset configuration (read/input)
#define ECAT_READ_AI_PORT_SIZE          0x18
#define ECAT_READ_AI_STATUS_SIZE        0x08
#define ECAT_READ_AI_VALUE_SIZE         0x10
#define ECAT_READ_AI_SALVE_SIZE         (ECAT_READ_AI_PORT_SIZE * SLAVE_AI_PORT_COUNT)
#define ECAT_READ_AI_OFFSET             (ECAT_READ_DI_OFFSET + \
                                       (ECAT_READ_DI_SLAVE_SIZE * 2))   //0x190

// Cart DI offset configuration (read/input)
#define ECAT_READ_CART_DI_OFFSET        (ECAT_READ_DI_OFFSET + 2 * ECAT_READ_DI_SLAVE_SIZE \
                                        + ECAT_READ_AI_SALVE_SIZE + ECAT_READ_DI_SLAVE_SIZE)  //0x1C8

///////////////////////////////////////////////////////////
//
// Port No Value Define
//
    // D-in slave
#define BRAKESTATE_DI_SLAVE_NO          0
#define SYSTEMIO_DI_SLAVE_NO            0   // 6,7 port
#define BRAKECLEAR_DI_SLAVE_NO          1
#define WELD_DI_SLAVE_NO                2
#define CART_DI_SLAVE_NO                3

    // D-in port
// slave 0: brake in 0~5
#define CONT_RESET_DI_PORT_NO           6
#define CONT_ESTOP_DI_PORT_NO           7

// slave 1: clear in 0~5

// slave 2: weld in
#define ARCON_DI_PORT_NO                0
#define NOGAS_DI_PORT_NO                1
#define NOWIRE_DI_PORT_NO               2
#define WELDERPWRFAIL_DI_PORT_NO        3
#define TOUCHPROCESS_DI_PORT_NO         4
#define TOUCHSIGNAL_DI_PORT_NO          5

// slave 3: cart in
#define TP_ESTOP_DI_PORT_NO             0
#define DEADMAN_DI_PORT_NO              1
#define SHOCKSENSOR_DI_PORT_NO          2
#define JOBSTART_DI_PORT_NO             3
#define CART_ESTOP_DI_PORT_NO           4
#define CART_CYLIND_DI_PORT_NO          5   // To be confirmed

    // A-in slave & port
#define WELD_AI_SLAVE_NO                0
#define VOLT_AI_PORT_NO                 0
#define CURR_AI_PORT_NO                 1

    // D-out slave
#define WELD_DO_SLAVE_NO                0
#define LAMP_DO_SLAVE_NO                1
#define LAMP_DO_SLAVE_OFFSET            (LAMP_DO_SLAVE_NO * SLAVE_DO_PORT_COUNT)
#define CART_DO_SLAVE_NO                2
#define CART_DO_SLAVE_OFFSET            (CART_DO_SLAVE_NO * SLAVE_DO_PORT_COUNT)

    // D-out cmd port
// slave 0: weld
#define ARCON_DO_PORT_NO                0
#define GASON_DO_PORT_NO                1
#define INCHPOS_DO_PORT_NO              2
#define INCHNEG_DO_PORT_NO              3
#define TOUCHSTART_DO_PORT_NO           4
#define TOUCHREADY_DO_PORT_NO           5
#define WIRECUT_SEND_PORT_NO            6

// slave 1: lamp
#define LAMP_CONT_READY_DO_PORT_NO      0
#define LAMP_OPERATING_DO_PORT_NO       1
#define LAMP_SERVOON_DO_PORT_NO         2
#define LAMP_ECAT_RUN_DO_PORT_NO        3
#define LAMP_ERROR_DO_PORT_NO           4

// slave 2: cart
#define CART_LAMP_ALARM_DO_PORT_NO      0
#define CART_LAMP_RUN_DO_PORT_NO        1
#define CART_WIRECUT_DO_PORT_NO         2
#define CART_JOBSTARTCON_DO_PORT_NO     3
#define CART_SPARE_DO_PORT_NO           5


    // A-out slave & port
#define WELD_AO_SLAVE_NO                0

#define VOLT_AO_PORT_NO                 0
#define CURR_AO_PORT_NO                 1

#if 0
    // D-out state port
#define ARCON_DO_STATE_NO               0
#define GASON_DO_STATE_NO               1
#define INCHPOS_DO_STATE_NO             2
#define INCHNEG_DO_STATE_NO             3
#define TOUCHSTART_DO_STATE_NO          4
#define TOUCHREADY_DO_STATE_NO          5

#define VOLT_AO_STATE_NO                0
#define CURR_AO_STATE_NO                1
#endif

#endif //ECATDEF_H__