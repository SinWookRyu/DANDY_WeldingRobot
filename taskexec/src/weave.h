// WEAVE is the weaving motion program. 
// 2013.12.04 mrch0
// 'WV' represents WEAVE. 

// Weaving plane is the plane the weaving motion is on. [x_w, y_w]. 
// y_w means Right. 

// Weaving is done on the weaving plane(xw & wy) in order of 
//   EVEN_MOTION -> EVEN_DWELL -> ODD(CENT)_MOTION -> CENT_DWELL -> ODD_MOTION -> ODD_DWELL... 
//   Center Dwell is 0, No Center Motion & Center Dwell. 
//   END_WEAV : If 1 weave-dist is larger than remained weave-dist. 
//   Even Dir == yw Direction

// 2 Weaving Conditions : Start, Main Weaving Conditions
//   START_WEAVE during Start-Weaving-Dist
//   MAIN_WEAVE After Start-Weaving-Dist

// Angle Transient Distance(ATD) 
//   Start Rotation Changes to Final Rotation in ATD. 
//   After ATD, Constant Rotation Motion
//   If ATD = 0, ATD = Work Dist

/*
_________________________________
|                               |
|                               |
|            |---|              |
|            |   |              |
|  --|   |---|   |---|	 |-->xw |
|    |   |      	 |   |      |
|    |---|       	 |---|	    |
|yw	 w0	 w1  	 w2      w3     |
|      dwe dwc dwo dwc dwe      |
---------------------------------
*/

////////////////////////////////////////////////////////////////////////////////

#ifndef WEAVE_H_2013_09_04_
#define WEAVE_H_2013_09_04_

#include "trajectory.h"
#if 0
#include "sensor.h"
#endif

#define WV_MODE_NONE        0
#define WV_MODE_MOT    1
#define WV_MODE_DWL    2
//#define WV_MODE_EVEN_MOT    1
//#define WV_MODE_EVEN_DWL    2
//#define WV_MODE_ODD_MOT     3
//#define WV_MODE_ODD_DWL     4
#define WV_MODE_FIN_MOT     5

#define WV_SEG_START        0
#define WV_SEG_MAIN         1
#define WV_SEG_FINAL        2

#define WV_ERR_NONE         0
#define WV_ERR_NULL_PTR     1
#define WV_ERR_INV_PARAM    2
#define WV_ERR_NO_WX_DIR    3
#if 0 // error 
#define WV_ERR_TRAJ         4
#endif 
#define WV_ERR_SHORT_DIST   5
#define WV_ERR_UNDEF_MODE   6
#define WV_INV_BASE_PLANE_OR_WV_DIR 7
#define WV_ERR_LONG_START_END_DIST 8
#define WV_ERR_LONG_ANGLE_DIST 9

#if defined(__cplusplus)
extern "C" {
#endif
    
#pragma pack(push, 1)
typedef struct
{   
    // Traj for Motion
    TRAJ*            traj;     
#if 0
    ARC_SENSOR_INFO* arc_sensor; 
#elif 0
    UNT              f_sensor;  // run flag for arc sensor on. 
#endif 

    // Weaving Frame
    POS pw;         // weaving ref(start) pos wrt BASE
    ROT Rs;         // Start Pose wrt BASE 
    POS xw;         // weaving 'x' dir wrt BASE
    POS yw;         // weaving 'y' dir wrt BASE 
    POS zw;         // weaving 'z' dir wrt BASE. upper from workpiece & ortho to xw & yw. 
    POS sr;         // Rotational Motion Vector

    // Unit Motion Param
    double pitch;   
    double width;     
    double dwl_even;// even dwell time [ms]
    double dwl_odd; // odd  dwell time [ms]
    double dwl_cent;// cent dwell time [ms]. if 0 no center dwell
    double angle;   // Rotational Motion Angle at Each Node [rad]
    double v_pos;   // vel for pos motion [../ms]
#if 0
    double v_rot;   // vel for rot Motion [rad/ms]
#endif
    
    // Start Weaving Condition 
    double pitch_s;   
    double width_s;     
    double v_pos_s; 
    
    // Total Displacement
    double d_total; // total pos displacement    
    double d_trg;   // target pos displacement of i-th motion from start    
    double d_act;   // actual working distance in the direction of 'xw'.
#if 0
    double d_start; // start weaving distance
#endif
    double d_final; // final dist from end. 
    double d_angle; // angle dist. if <=0 no rotation. 
    double r_total; // total rot displacement    

    unsigned config;// Robot Pose Config 

    // Weaving Control Var's 
    int      mode;  // WV_MODE_XXX
    int      f_stop;// Stop Flag
    unsigned error; // Error. 0th byte : error code, 1st : error source. Recommend to use Wv_IsTrajErr(..)
    unsigned i_mot; // Actual motion index. 0,1,2... Even No:Right, Odd No:Left
    unsigned n_start;//Start Node Count
    unsigned f_cent;// Flag which Actual motion is to center. 
    unsigned n_seg; // Weaving Segment. WV_SEG_START/MAIN/FINAL    
    unsigned f_seg_change; // Flag for Weave Seg. Changed. Act during 1 sample.
    double   t_dwl; // Dwell Time [ms]    
    double   t_mode;// mode elapse time[ms]
        
} WEAVE; 
#pragma pack(pop)

UNT Wv_IsTrajErr(unsigned err); 

// returns ON if 'wv' is running or OFF. 
UNT Wv_IsRunning(WEAVE* wv); 

// The flag for Arc-sensing is 'weld_idx::arc_sensor'.
int Wv_Set(WEAVE*       wv,             // 
           TRAJ*        traj,           // Basic Init of 'traj' is prior to this
#if 0
           ARC_SENSOR_INFO* arc_sensor, // Arc Sensor Info's
#elif 0
           unsigned     f_sensor,       // run request of arc sensing
#endif
           const double th_s[6],        // Actual Joint Value [rad]           
           const TRANS* Tf,             // Final Pos & Rot wrt BASE
           unsigned     config,         // Robot Pose Configuration                      
           const POS*   s_plane,        // Weaving Plane(Width) Direction
           double       ang_plane,      // [rad] Angle of Weaving Plane
           double       pitch,          // [..] if 0, Hover weaving 
           double       width,          // [..]
           double       speed,          // [../ms]           
           double       pitch_s,        // Start Pitch
           double       width_s,        // Start Width
           double       speed_s,        // Start Speed
           double       dwl_e,          // [ms] even dwell time
           double       dwl_o,          // [ms] odd dwell time                      
           double       dwl_c,          // [ms] center dwell time. if 0 no center dwell. 
           double       d_angle,        // angle varying distance. if 0, Rot Mot is over all WV Dist.
           double       d_start,        // start weaving distance. if 0, Start with Main                                 
           double       d_final);       // final distance from end. 
      
int Wv_Update(WEAVE* wv, const double* th_a, const TRANS* bTt_a); 

void Wv_Stop(WEAVE* wv, int f_fast); 

// Error Code String is returned. 
char* Wv_ErrorStr(char err); 

// returns weaving start distance
double Wv_StartDist(WEAVE* wv); 

// returns distance of 'pos' from weave ref. pos 'wv->pw' to the weaving dir. 
// pos : intereting position wrt base 
double Wv_Dist(const WEAVE* wv, const POS* pos);

// Outputs Final Pos & Rot. 
// Final Pos is not const but can be change from Init as Sensor acts. 
void Wv_FinalTrans(const WEAVE* wv, POS* p_f, ROT* R_f); 

char* Wv_Print(const WEAVE* wv); 

#if defined(__cplusplus)
}
#endif

#endif