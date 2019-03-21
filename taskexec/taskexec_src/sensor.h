#ifndef SENSOR_H_
#define SENSOR_H_

#include "ipc_taskexec.h"

#if defined(__cplusplus)
extern "C" {
#endif
    
#if 0
#pragma pack(push, 1)
typedef struct
{
    unsigned int f_active;      // 0:arc sensor off, 1:arc sensor on
    double       V_main; 
    double       I_main; 
} ARC_SENSOR_INFO; 
#pragma pack(pop)
#endif 

#define CORRECTION_LIMIT 0.1

#pragma pack(push, 1)
typedef struct
{
    int     w_flag;     // WD Flag : 1(Normal Node), 0(WD Node)
    uint    wd_node;
    uint    node;
    double  speed;
} weave_data_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{		
	double volt;			// Main 	[V]
	double curr;			// Main 	[A]
} main_cond_t;
#pragma pack(pop)

// Arc Sensor Info. 
#pragma pack(push, 1)
typedef struct
{
	byte    arc_sensor;	// sensor running condition.ON at ARC_ON, OFF at ARC_OFF
    double  V_main;     // No Effective 
    double  I_main;     // No Effective 
} weld_idx_t;
#pragma pack(pop)


// set on at arc-on & main welding cond. set off at arc-off
extern weld_idx_t   weld_idx; 
#if 0
extern main_cond_t  main_cond; 
#endif

extern volatile shm_mmi_task_t*         shm_mmi; 
extern volatile shm_task_servo_t*       shm_servo; 

extern uint arc_save_num;
extern double ref_current; 
extern double ref_weight; 
extern double Delta_T[NUM_OF_NODE];    // dir of yw
extern double Delta_Z[NUM_OF_NODE];    // dir of zw(upper from workpiece is '+')
extern double Mean_Moving_Ampere[NUM_OF_NODE];
extern double Mean_Moving_Weight[NUM_OF_NODE];

// calculates current difference value. 
// call at node-planning of weaving
// - dt : correction value wrt Even 'yw' dir.  
// - dz : correction value wrt Current value. 
void get_correct_value(double *dt, double *dz, weave_data_t *wv);

#if defined(__cplusplus)
}
#endif

#endif