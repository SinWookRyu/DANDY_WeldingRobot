/************************************************************************
        KPA EtherCAT Master

    Copyright (c) 2004-2012
    koenig-pa GmbH.
    Visutech System Ltd.
    All rights reserved.
*************************************************************************
    QNX KPA EtherCAT Master Sample
*************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_


#define KPA_SERVER_PORT_NUMBER              5000
#define KPA_MASTER_CONFIGURATION_FILE       "/tmp/test.xml"
#define KPA_MASTER_AUTORECOVERY_TIMEOUT_MS  100
#define KPA_MASTER_CYCLE_TIME_US            1000
#define KPA_MASTER_SUBCYCLE_TIME_US         5000
#define KPA_MASTER_PRIORITY                 60
#define KPA_MASTER_TRACE_PRIORITY           20
#define KPA_DEFAULT_ADAPTER_INDEX           0
#define KPA_INPUT_VARIABLE_BIT_OFFSET       0
#define KPA_INPUT_VARIABLE_BIT_SIZE         4
#define KPA_OUTPUT_VARIABLE_BIT_OFFSET      2
#define KPA_OUTPUT_VARIABLE_BIT_SIZE        4

#define KPA_SYNC_CHANNEL_POINT 				"ecat_sync_channel"
#define KPA_SYNC_PROCESS_TASK_LIBRARY		"/tmp/mkpasync.so"


/*
#define DEFAULT_ADAPTER_INDEX				0
#define MASTER_CONFIGURATION_FILE			"/tmp/test.xml"
#define MASTER_CYCLE_TIME_US				1000
#define MASTER_SUBCYCLE_TIME_US				5000
#define MASTER_AUTORECOVERY_TIMEOUT_MS		100



// synchronization process task settings
#define SYNC_CHANNEL_POINT 			"ecat_sync_channel"
#define SYNC_PROCESS_TASK_LIBRARY	"/tmp/pt_sync.so"


#define UPDATE_THREAD_PRIORITY		60
#define SYNC_THREAD_PRIORITY		60
#define TRACE_THREAD_PRIORITY		10*/


#endif /*CONFIG_H_*/
