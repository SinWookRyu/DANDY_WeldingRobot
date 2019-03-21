#ifndef TRAJECTORY_H_20130701_
#define TRAJECTORY_H_20130701_

#include "profile7.h"
#include "motion.h"

#define TRAJ_TYPE_NORMAL        (0)     // no motion
#define TRAJ_TYPE_JOINT         (101)
#define TRAJ_TYPE_LINEAR        (102)
#define TRAJ_TYPE_CIRCLE        (103)

// Error. Traj Error must not be '-'. 
#define TRAJ_ERR_NONE           (0x00)
#define TRAJ_ERR_WRONG_PARAM    (0x01)
#define TRAJ_ERR_PROFILE_FAIL   (0x02)
#define TRAJ_ERR_MOTION_FAIL    (0x03)
#define TRAJ_ERR_ZERO_VEL       (0x04)
#define TRAJ_ERR_LIMIT_POS      (0x05)
#define TRAJ_ERR_LIMIT_VEL      (0x06)
#define TRAJ_ERR_CONF_MISMATCH  (0x07)
#define TRAJ_ERR_UNDEF_CIRCLE   (0x08)
#define TRAJ_ERR_NEAR_POS       (0x09)  // 
#define TRAJ_ERR_UNDEF_INVERSE  (0x10)  // Undefined Inverse Error 
#define TRAJ_ERR_INVERSE_Z0     (0x11)  // Inverse Error : Target Near Z0
#define TRAJ_ERR_INVERSE_TH5    (0x12)  // Inverse Error : th5 singular
#define TRAJ_ERR_INVERSE_UNREACH (0x13) // Inverse Error : Unreachable
#define TRAJ_ERR_INVERSE_LARGE_V (0x14) // Inverse Error : Unknown but big vel. 
#define TRAJ_ERR_LIMIT_POS_0     (0x20) // pos lim err. up ver of TRAJ_ERR_LIMIT_POS
#define TRAJ_ERR_LIMIT_POS_1     (0x21)
#define TRAJ_ERR_LIMIT_POS_2     (0x22)
#define TRAJ_ERR_LIMIT_POS_3     (0x23)
#define TRAJ_ERR_LIMIT_POS_4     (0x24)
#define TRAJ_ERR_LIMIT_POS_5     (0x25)
#define TRAJ_ERR_LIMIT_VEL_0     (0x30) // vel lim err. up ver of TRAJ_ERR_LIMIT_VEL
#define TRAJ_ERR_LIMIT_VEL_1     (0x31)
#define TRAJ_ERR_LIMIT_VEL_2     (0x32)
#define TRAJ_ERR_LIMIT_VEL_3     (0x33)
#define TRAJ_ERR_LIMIT_VEL_4     (0x34)
#define TRAJ_ERR_LIMIT_VEL_5     (0x35)


#pragma pack(push, 1)
typedef struct t_traj
{   
    int     type;           // TRAJ_TYPE_JOINT / TRAJ_TYPE_LINEAR / TRAJ_TYPE_CIRLE
    double  t_start;        // start time
    double  t_samp;         // Sampling Time
#if 0
    PROFILE3 profile[6];    // For Linear Motion     
#else
    PROFILE7 profile;       // For Linear Motion     
#endif

    union
    {
        JMOT    jmot[6];    // Joint Motion
        LMOT    lmot;       // Liniear Motion
        CMOT    cmot;       // Circular Motion 
    };  

    unsigned config;        // robot pose configuration

    // dynamics.
    double jerk_joint[6]; 
    double jerk_pos; 
    double jerk_rot; 

    // General Motion Acc &  Dec
    double acc_joint[6]; 
    double acc_pos;         // translation motion acc
    double acc_rot;         // orientation motion acc

    // Stop Decel
    double dec_stop_joint[6]; 
    double dec_stop_pos;         // translation motion dec [mm/mm^2]
    double dec_stop_rot;         // orientatoin motion dec [rad/mm^2]

    // Fast Stop Decel
    double dec_fast_joint[6]; 
    double dec_fast_pos; 
    double dec_fast_rot;

    int  (*Inverse)(double th[6], const TRANS* bTe,  unsigned conf, const DH dh[6], const double* th_prev, unsigned f_running); 
    void (*Forward)(TRANS* bTe, const double th[6], const DH dh[6]); 
    unsigned (*Config)(const double* th, const DH *dh); 
    DH   dh[6]; 
    TRANS eTt;              // END->TCP

    // limits 
    double lim_max[6];      // joint max pos
    double lim_min[6];      // joint min pos
    double lim_vel[6];      // joint vel lim
    double lim_v_lin;       // linear v lim
    double lim_v_rot;       // rotaion v lim
    double lim_near;        // So near 3pt's cannot make a circle.

    // Last Error. Reset at Init. Write 1st Error only, No Re-Write    
    unsigned error; 
    double err_vel[6];  // Error Occurred Velocity 
    double err_pos[6];  // Error Occureed Position
} TRAJ; 
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////                                 

// Trajectory Dynamics Setting 
int Traj_Set_Dynamics(TRAJ* traj,           
                       double t_samp,       // [ms] sampleing time
                       double j_axis[6], 
                       double a_axis[6], 
                       double d_axis_stop[6], 
                       double d_axis_fast[6], 
                       double j_pos, 
                       double a_pos, 
                       double d_pos_stop, 
                       double d_pos_fast,     
                       double j_rot, 
                       double a_rot,                
                       double d_rot_stop,   
                       double d_rot_fast,                          
                       int  (*Inverse)(double th[6], const TRANS* bTe,  unsigned conf, const DH dh[6], const double* th_prev, unsigned f_running), 
                       void (*Forward)(TRANS* bTe, const double th[6], const DH dh[6]),                    
                       unsigned (*Config)(const double* th, const DH *dh), 
                       const DH dh[6],  
                       const TRANS* eTt);       // END->TCP, If NULL, Eye used.

// Trajectory Limit Setting. 
// Traj_Set_Dynamics -> Traj_Set_Limit -> Traj_Set_Axis(or Lmot)
// No use to call each time of TRAJ init. Call Once If Limit Info's. Change 
int Traj_Set_Limit(TRAJ* traj,                        
                   double lim_max[6],   // position limit to max(+)
                   double lim_min[6],   // position limit to min(-)
                   double lim_vel[6],   // velocity limit (>0)
                   double lim_v_lin,    // translation motion limit [.../ms]
                   double lim_v_rot,    // Orientation motion limit [rad/ms]
                   double lim_near);     // so near 3pt's cannot make a circle. 

#if 0
// Trajectory setting for Joint Motion
int Traj_Set_Jmot( TRAJ* traj,                                       
                   const double p_start[6],     // start pos
                   double p_final[6],           // final pos
                   double v_trg[6], 
                   double t_start);                    

// Trajectory setting for Linear Motion
// If conf_trg != conf_act, returns Conf. Error.
// To Set LIN(ROT) Mot Fix, Set Norm of 'dir_pos'('v_rot') 0
// To sync LIN(ROT) Mot to Rot(Lin), Set 'v_pos'('v_rot') 0
int Traj_Set_Lmot( TRAJ* traj,                  // output
                   const double axis_act[6],    // start axis pos
                   const POS* dir_lin,          // Unit vector to Target Pos
                   double dist_lin,                    
                   double v_lin,                    
                   const POS* dir_rot,          // Unit vector of Rotation
                   double dist_rot,             // [rad]
                   double v_rot,                
                   unsigned conf_trg,           // robot pose configuration
                   double t_start); 
                   
// TRAJ_IsRunning returns if Trajectory at 'time' is over or not. 
// Returns Running(1) Else(0)
int Traj_IsRunning(const TRAJ* traj, double time);    

// Gets Target Pos of 1Axis Trajectory.
// Returns SUCCESS(0), TRAJ_ERR_WRONG_PARAM, TRAJ_ERR_INVERSE_FAIL, TRAJ_ERR_INV_FAIL_AXIS_FAIL
int Traj_GetTarget(TRAJ* traj, 
                   double target[6], 
                   double u[6], 
                   const double p_prev[6], 
                   const double v_prev[6], 
                   double time); 

// time : Stop Start Time. Usully use 'time_act - sampling' (>0)
void Traj_Stop(TRAJ* traj, double time); 

// time : Stop Start Time. Usully use 'time_act - sampling' (>0)
void Traj_StopFast(TRAJ* traj, double time);
#else

// Trajectory setting for Joint Motion with Joint Vel. 
int Traj_Set_Jmot( TRAJ* traj,                                       
                   const double p_start[6],     // start pos
                   double p_final[6],           // final pos
                   double v_trg[6]);      

// Trajectory setting for Joint Motion with rational velocity
int Traj_Set_Jmot2(TRAJ* traj,                                       
                   const double p_start[6],     // start pos
                   double p_final[6],           // final pos
                   double vel_r);               // rational vel (0 < vel_r <= 1)

// Trajectory setting for Linear Motion
// Parameters are described wrt BASE
// If conf_trg != conf_act, returns Conf. Error.
int Traj_Set_Lmot( TRAJ* traj,                  // output
                   const double axis_act[6],    // start axis pos
                   const POS* dir_lin,          // Unit vector to Target Pos
                   double dist_lin,                    
                   double v_lin,                    
                   const POS* dir_rot,          // Unit vector of Rotation
                   double dist_rot,             // [rad]
                   double v_rot,                
                   unsigned conf_trg);          // robot pose configuration                   


// Trajectory setting for Linear Motion with Final Pose
// If conf_trg != conf_act, returns Conf. Error.
int Traj_Lmot_Set2(TRAJ* traj,                  // output
                   const double jnt_s[6],       // start joint
                   const TRANS* Tf,             // Final TCP wrt BASE 
                   unsigned conf_f,             // Final Robot pose configure                   
                   double v_pos, 
                   double v_rot);

// Trajectory setting for Circular Motion Basically
// Parameters are described wrt BASE
// If conf_trg != conf_act, returns Conf. Error.
int Traj_Set_Cmot( TRAJ*        traj,           // output
                   const double jnt_s[6],       // start axis pos
                   const POS*   cent,           // Circle Center
                   const POS*   s_p,            // Dir Vector for Pos Motion 
                   double       arc,            // [mm] Pos Motion Distance         
                   double       v_p,            // [mm/ms] Pos Motion Vel(>0)        
                   const POS*   s_r,            // Unit vector of Rotation
                   double       rot,            // [rad] Rot Motion Orientation
                   double       v_r,            // [rad/ms] Rot Motion Vel(>0)
                   unsigned     conf_f);        // robot pose configuration

// Trajectory setting for Circular Motion with 3 Points
// 3 Points are Start, Viaway and Final Point. 
// If conf_trg != conf_act, returns Conf. Error.
int Traj_Set_Cmot_3Pt(TRAJ* traj,              // output
                      const double jnt_s[6],   // start joint
                      const POS*   Pv,         // Via Pos
                      const TRANS* Tf,         // Final Pose                   
                      unsigned conf_v,         // Via Pose Config           
                      unsigned conf_f,         // Final Pose Config
                      double v_p,              // [mm/ms]  Vel of Pos Mot (>0)
                      double v_r);             // [mm/rad] Vel of Rot Mot (>0) 

// TRAJ_IsRunning returns if Trajectory at 'time' is over or not. 
// Returns Running(1) Else(0)
int Traj_IsRunning(const TRAJ* traj); 

// TRAJ_IsRunning returns if Trajectory at 'time' is over or not. 
// Returns Running(1) Else(0)
int Traj_Update(TRAJ* traj); 

// returns Profile
double Traj_GetProf(TRAJ* traj); 

// Gets Target Pos of 1Axis Trajectory.
// Returns SUCCESS(0), TRAJ_ERR_WRONG_PARAM, TRAJ_ERR_INVERSE_FAIL, TRAJ_ERR_INV_FAIL_AXIS_FAIL
int Traj_GetTarget(TRAJ* traj, 
                   double target[6], 
#if 0
                   double u[6], 
#else
                   double* u, 
#endif
                   const double p_prev[6], 
                   const double v_prev[6]); 

#if 0
// time : Stop Start Time. Usully use 'time_act - sampling' (>0)
void Traj_Stop(TRAJ* traj); 

// time : Stop Start Time. Usully use 'time_act - sampling' (>0)
void Traj_StopFast(TRAJ* traj); 
#else
// Stops Running Traj. 
// f_estop : flag for fast stop. 
void Traj_Stop(TRAJ* traj, int f_estop);
#endif
#endif

// Stops Running Traj in the decel time 't_dec'. 
void Traj_Stop_Time(TRAJ* traj, double t_dec); 

// Returns Error String of TRAJ. 
char* Traj_ErrorStr(char err); 

// Prints 'traj' Info's into String. 
char* Traj_Print(TRAJ* traj); 

char* Traj_TypeStr(int type); 

#endif
