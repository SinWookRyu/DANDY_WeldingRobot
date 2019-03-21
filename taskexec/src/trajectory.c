// NEW_SYNC_TEST 
// To correct to keep the vel of MOVL. 
// So small orientation didn't make MOVL vel properly. 
// So small orientation made so small linear velocity. 
#define NEW_SYNC_TEST

#include "stdio.h"
#include "float.h"
#include "memory.h"
#include "string.h"
#include "math.h"
#include "trajectory.h"
#include "utility.h"
#include "kinematics.h"
#include "CRT.h"

// For Profile Indexing 
#define PROF_POS   0
#define PROF_ROT   1

// Calculates Vel for Lin(Rot) Which is compatible to the Vel for Rot(Lin). 
#define SYNC_POS_VEL(traj_, vr_) ((traj_)->acc_pos * (vr_) / (traj_)->acc_rot)
#define SYNC_ROT_VEL(traj_, vp_) ((traj_)->acc_rot * (vp_) / (traj_)->acc_pos)

// Calc Target 
static int TargetCalc(const TRAJ* traj,         //[i]
                      double trg[6],            //[o]
                      double* u,                //[o]
                      const double prev[6]);    //[i]                                             

// Converts Joint Mode & Set Stops. Returns Traj Error Code. 
static int JointStop(TRAJ* traj, 
                     const double p_prev[6], 
                     const double v_prev[6]); 

// Returns  No Limit(0), Limit Axis(-1), Limit Vel(-2)
static int LimitCheck(TRAJ* traj, 
                      const double trg[6], 
                      const double prev[6]); 

static int LimitErrAssign_Pos(int i_axis); 
static int LimitErrAssign_Vel(int i_axis); 

static void TrajErrorReset(TRAJ* traj)
{
    if(traj)
    {
        traj->error = 0; 
        memset(traj->err_pos, 0, sizeof(traj->err_pos)); 
        memset(traj->err_vel, 0, sizeof(traj->err_vel)); 
    }
}


////////////////////////////////////////////////////////////////////////////////

// Trajectory Dynamics Setting. 
// Traj_Set_Dynamics -> Traj_Set_Axis(or Lmot)
// No use to call each time of TRAJ init. 
// Call Once If Limit Info's. Change 
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
                      const TRANS* eTt) // END->TCP                      
{
    if(!traj    || !a_axis  || !d_axis_stop  || !d_axis_fast || 
       !Inverse || !Forward || !dh           || !eTt)
    {        
        return TRAJ_ERR_WRONG_PARAM; 
    }

    memcpy(traj->jerk_joint,j_axis, sizeof(traj->jerk_joint)); 
    memcpy(traj->acc_joint, a_axis, sizeof(traj->acc_joint)); 
    memcpy(traj->dec_stop_joint, d_axis_stop, sizeof(traj->dec_stop_joint)); 
    memcpy(traj->dec_fast_joint, d_axis_fast, sizeof(traj->dec_fast_joint)); 
    
    traj->jerk_pos= j_pos; 
    traj->acc_pos = a_pos; 
    traj->dec_stop_pos = d_pos_stop; 
    traj->dec_fast_pos = d_pos_fast; 

    traj->jerk_rot= j_rot; 
    traj->acc_rot = a_rot; 
    traj->dec_stop_rot = d_rot_stop;
    traj->dec_fast_rot = d_rot_fast; 

    traj->t_samp = (t_samp < 0)? 0 : t_samp; 

    traj->eTt = *eTt; 
    traj->Inverse = Inverse; 
    traj->Forward = Forward;     
    traj->Config  = Config; 
    memcpy(traj->dh, dh, sizeof(traj->dh));  

    return TRAJ_ERR_NONE; 
}

// Trajectory Limit Setting. 
// Traj_Set_Dynamics -> Traj_Set_Limit -> Traj_Set_Axis(or Lmot)
// No use to call each time of TRAJ init. Call Once If Limit Info's. Change 
int Traj_Set_Limit(TRAJ* traj,                        
                   double lim_max[6],   // position limit to max(+)
                   double lim_min[6],   // position limit to min(-)
                   double lim_vel[6],   // velocity limit (>0)
                   double lim_v_lin,    // translation motion limit [.../ms]
                   double lim_v_rot,    // Orientation motion limit [rad/ms] 
                   double lim_near)     // So near 3pt's cannot make circle. 
{
    if(!traj || !lim_max || !lim_min || !lim_vel)
    {        
        return TRAJ_ERR_WRONG_PARAM; 
    }

    memcpy(traj->lim_max, lim_max, sizeof(traj->lim_max)); 
    memcpy(traj->lim_min, lim_min, sizeof(traj->lim_min)); 
    memcpy(traj->lim_vel, lim_vel, sizeof(traj->lim_vel)); 
    traj->lim_v_lin = lim_v_lin; 
    traj->lim_v_rot = lim_v_rot; 
    traj->lim_near  = lim_near; 

    return TRAJ_ERR_NONE; 
}

int Traj_Update(TRAJ* traj)
{   
    ASSERT_RETURN(traj, -1);

    if(traj->type == TRAJ_TYPE_NORMAL)
    {
        return -1; 
    }

    traj->t_start += traj->t_samp;

    // Excluded '=' : calculated the final point 
    if(traj->t_start < traj->profile.t_2d + traj->t_samp)
    {
        return 0; 
    }

    // Nothing is running, Traj is stopped. 
    traj->type = TRAJ_TYPE_NORMAL; 
    return -1; 
}

// TRAJ_IsRunning returns if Trajectory at 'time' is over or not. 
// Returns Running(1) Else(0)
// 
int Traj_IsRunning(const TRAJ* traj)
{   
    ASSERT_RETURN(traj, 0); 
    return (traj->type == TRAJ_TYPE_NORMAL)? 0 : 1; 
}

// Trajectory setting for Joint Motion
int Traj_Set_Jmot( TRAJ* traj,                                       
                   const double p_start[6],     // [rad] start pos
                   double p_final[6],           // [rad] final pos
                   double v_trg[6])             // [rad/ms] vel(>0)
{      
    int i;  
    PROFILE7 temp_prof; 

    traj->error = 0; 

    // data validation
    if(traj  == NULL || p_start == NULL || p_final == NULL || v_trg == NULL)
    {   
        return TRAJ_ERR_WRONG_PARAM; 
    }

    // Motion & Profile Setting     
    for(traj->profile.t_2d = 0., i=0 ; i<6 ; i++)
    {
        // JMOT Setting        
        if(Jmot_Set(&traj->jmot[i], p_start[i], p_final[i], v_trg[i]))
        {        
            return TRAJ_ERR_WRONG_PARAM;
        }
        // PROFILE Setting
        if(Prof7_Set(&temp_prof, traj->jmot[i].dbJointD, v_trg[i], 
                    traj->acc_joint[i], traj->jerk_joint[i]))
        {                 
            return TRAJ_ERR_PROFILE_FAIL; 
        }    
        // Sychronization of 6-axis 
        traj->profile = Prof7_GetSync(&traj->profile, &temp_prof); 
    }
    
    // set TRAJ                
    traj->type    = TRAJ_TYPE_JOINT; 
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 

    // Reset Error
    TrajErrorReset(traj); 
    return 0;
}

// Trajectory setting for Joint Motion
int Traj_Set_Jmot2(TRAJ* traj,                                       
                   const double p_start[6],     // start pos
                   double p_final[6],           // final pos
                   double vel_r)                // rational vel (0 < vel_r <= 1)
{      
    int i;   
    double vel; 
    PROFILE7 temp_prof;   

    traj->error = 0; 

    // data validation
    if(traj  == NULL || p_start == NULL || p_final == NULL || 1.< vel_r)
    {   
        return TRAJ_ERR_WRONG_PARAM; 
    }

    // Motion & Profile Setting    
    for(traj->profile.t_2d=0., i=0 ; i<6 ; i++)
    {
        vel = traj->lim_vel[i]*vel_r; 

        // JMOT Setting        
        if(Jmot_Set(&traj->jmot[i], p_start[i], p_final[i], vel))
        {        
            return TRAJ_ERR_MOTION_FAIL;
        }
        // PROFILE Setting
        if(Prof7_Set(&temp_prof, traj->jmot[i].dbJointD, vel, /*traj->lim_vel[i]*vel_r,*/
                    traj->acc_joint[i], traj->jerk_joint[i]))
        {                 
            return TRAJ_ERR_PROFILE_FAIL; 
        }  
        // Sychronization of 6-axis 
        traj->profile = Prof7_GetSync(&traj->profile, &temp_prof);   
    }

    // set TRAJ                
    traj->type    = TRAJ_TYPE_JOINT; 
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 

    TrajErrorReset(traj); 
    return 0;
}

// Trajectory setting for Linear Motion
// If conf_trg != conf_act, returns Conf. Error.
// To Set LIN(ROT) Mot Fix, Set Norm of 'dir_pos'('v_rot') 0
// To sync LIN(ROT) Mot to Rot(Lin), Set 'v_pos'('v_rot') 0
int Traj_Set_Lmot( TRAJ* traj,                  // output
                   const double axis_act[6],    // start axis pos
                   const POS* dir_pos,          // Unit vector to Target Pos
                   double dist_pos,                    
                   double v_pos,                    
                   const POS* dir_rot,          // Unit vector of Rotation
                   double dist_rot,             // [rad]
                   double v_rot,                
                   unsigned conf_f)           // target robot pose configure                   
{      
    TRANS bTe, bTt; 
    unsigned conf_s; 
    PROFILE7 prof_pos, prof_rot; 

    // data validation
    if(traj  == NULL    || axis_act == NULL || 
       dir_pos == NULL  || dir_rot == NULL )       
    {
        return TRAJ_ERR_WRONG_PARAM; 
    }

#ifdef NEW_SYNC_TEST
    if(v_pos < EPS_P && v_rot < EPS_O)
    {
        return TRAJ_ERR_ZERO_VEL; 
    }
#endif 

    // Configuration    
    conf_s = traj->Config(axis_act, traj->dh);    
    if(conf_s != conf_f)
    {
#if 0 // PRINT
        printf("[A] Conf_s : %x, Conf_t : %x \n", conf_s, conf_f); 
        printf("Joint_a : %f %f %f %f %f %f \n", 
            axis_act[0], axis_act[1], axis_act[2], 
            traj->dh[0].d, traj->dh[1].d, traj->dh[2].d); 
#endif
        return TRAJ_ERR_CONF_MISMATCH; 
    }    
    traj->config = conf_s; 

    // Linear Motion Setting 
    traj->Forward(&bTe, axis_act, traj->dh);     
    bTt = TRANS_Multi_TRANS(bTe, traj->eTt); 
    
#ifndef NEW_SYNC_TEST
    // sync vel for pos & rot    
    v_pos = (v_pos < EPS_P)? SYNC_POS_VEL(traj, v_rot) : v_pos; 
    v_rot = (v_rot < EPS_R)? SYNC_ROT_VEL(traj, v_pos) : v_rot; 
#endif

    // Setting Lin Mot uses TCP wrt BASE 
    if(Lmot_Set(&traj->lmot, 
                &bTt.p, dir_pos, dist_pos, v_pos, 
                &bTt.R, dir_rot, dist_rot, v_rot))
    {        
        return TRAJ_ERR_MOTION_FAIL; 
    }
    
    // Linear Motion Profile Setting 
    if(EPS_P < v_pos)
    {
        if(Prof7_Set(&prof_pos, dist_pos, v_pos, 
                    traj->acc_pos, traj->jerk_pos))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

    if(EPS_O < v_rot)
    {
        // Rotation Motion Profile Setting 
        if(Prof7_Set(&prof_rot, dist_rot, v_rot, 
                    traj->acc_rot, traj->jerk_rot))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

#ifdef NEW_SYNC_TEST
    // Switch to the another Profile if one vel is 0
    prof_pos = (EPS_P < v_pos)? prof_pos : prof_rot; 
    prof_rot = (EPS_O < v_rot)? prof_rot : prof_pos; 
#endif 

    // Synchronization
    traj->profile = Prof7_GetSync(&prof_pos, &prof_rot);     
    
    // set TRAJ                
    traj->type    = TRAJ_TYPE_LINEAR; 
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 

    // Reset Error 
    TrajErrorReset(traj); 
    return 0;
}

// Trajectory setting for Linear Motion with Final Pose
// If conf_trg != conf_act, returns Conf. Error.
// If v_p or v_r is 0, 0-vel is synchronized to the other. 
int Traj_Lmot_Set2(TRAJ* traj,                  // output
                   const double jnt_s[6],       // start joint
                   const TRANS* Tf,             // Final TCP wrt BASE 
                   unsigned conf_f,             // Final Robot pose configure                   
                   double v_p,                  // 
                   double v_r)                  // 
{      
    double pos, rot; 
    TRANS  Ts; 
    POS    s_p, s_r;     
    unsigned conf_s; 
    PROFILE7 prof_pos, prof_rot; 

    // data validation
    if(traj  == NULL || jnt_s == NULL || Tf == NULL)
    {
        return TRAJ_ERR_WRONG_PARAM; 
    }
#ifdef NEW_SYNC_TEST
    if(v_p < EPS_P && v_r < EPS_O)
    {
        return TRAJ_ERR_ZERO_VEL; 
    }
#endif 

    // Configuration 
    conf_s = traj->Config(jnt_s, traj->dh); 
    if(conf_s != conf_f)
    {
        return TRAJ_ERR_CONF_MISMATCH; 
    }    
    traj->config = conf_s; 

    // Start bTt wrt BASE
    traj->Forward(&Ts, jnt_s, traj->dh);     
    Ts = TRANS_Multi_TRANS(Ts, traj->eTt); 

    // Translation Motion Info's    
    s_p = POS_Minus_POS(Tf->p, Ts.p); 
    pos = POS_Norm(s_p); 
    s_p = POS_SafeUnit(&s_p);     
    
    // Rf = R*Rs, R = Rf*Rs'
    ROT_GetScrewAxis(&s_r, &rot, ROT_Multi_ROT(Tf->R, ROT_Inv(Ts.R))); 
    s_r = POS_SafeUnit(&s_r);  

#ifndef NEW_SYNC_TEST
    // sync vel for pos & rot    
    v_p = (v_p < EPS_P)? SYNC_POS_VEL(traj, v_r) : v_p; 
    v_r = (v_r < EPS_O)? SYNC_ROT_VEL(traj, v_p) : v_r; 
#endif 

    // Setting Lin Mot uses TCP wrt BASE 
    if(Lmot_Set(&traj->lmot, 
                &Ts.p, &s_p, pos, v_p, 
                &Ts.R, &s_r, rot, v_r))
    {        
        return TRAJ_ERR_MOTION_FAIL; 
    }
   
    // Linear Motion Profile Setting 
    if(EPS_P < v_p)
    {
        if(Prof7_Set(&prof_pos, pos, v_p, 
                    traj->acc_pos, traj->jerk_pos))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

    // Rotation Motion Profile Setting 
    if(EPS_O < v_r)
    {
        if(Prof7_Set(&prof_rot, rot, v_r, 
                traj->acc_rot, traj->jerk_rot))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

#ifdef NEW_SYNC_TEST
    // Switch to the another Profile if one vel is 0
    prof_pos = (EPS_P < v_p)? prof_pos : prof_rot; 
    prof_rot = (EPS_O < v_r)? prof_rot : prof_pos; 
#endif 

    // Synchronization
    traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
   
    // set TRAJ                
    traj->type    = TRAJ_TYPE_LINEAR; 
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 

    // Reset Error 
    TrajErrorReset(traj); 
    return 0;
}

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
                      double v_r)              // [mm/rad] Vel of Rot Mot (>0) 
{    
    TRANS Ts; 
    POS cent; 
    POS s_p, s_r; 
    DBL arc, ang_arc, ang_rot; 
    DBL rad; 
    UNT conf_s; 
    PROFILE7 prof_pos, prof_rot; 

    // data validation
    if(traj == NULL || jnt_s == NULL || Pv   == NULL || Tf    == NULL )       
    {
        return TRAJ_ERR_WRONG_PARAM; 
    }

#ifdef NEW_SYNC_TEST
    if(v_p < EPS_P && v_r < EPS_O)
    {
        return TRAJ_ERR_ZERO_VEL; 
    }
#endif 

    // Configuration   
    conf_s = traj->Config(jnt_s, traj->dh); 
    if(conf_s != conf_v || conf_v != conf_f)
    {
        return TRAJ_ERR_CONF_MISMATCH; 
    }       
    traj->config  = conf_s; 

    // bTf_s
    traj->Forward(&Ts, jnt_s, traj->dh);     
    Ts = TRANS_Multi_TRANS(Ts, traj->eTt); 

    // Check 3Pt's are so near
    // Checking 3 Points are so near
    if( POS_Norm(POS_Minus_POS(Ts.p, *Pv)) < traj->lim_near || 
        POS_Norm(POS_Minus_POS(Tf->p, *Pv)) < traj->lim_near || 
        POS_Norm(POS_Minus_POS(Ts.p, Tf->p)) < traj->lim_near)
    {
        return TRAJ_ERR_NEAR_POS; 
    }

    // Translation Info's
    if(CIR_GetCircle(&cent, &s_p, &ang_arc, &rad, Ts.p, Tf->p, *Pv))
    {
        return TRAJ_ERR_UNDEF_CIRCLE; 
    }
    arc = rad * ang_arc; 

    // Orientation Info's        
    if(ROT_GetScrewAxis(&s_r, &ang_rot, ROT_Multi_ROT(Tf->R, ROT_Inv(Ts.R))))
    {        
        s_r = Ts.R.w;
        ang_rot = 0.0;
    }
    else
    {
        POS_Unit(&s_r);
    }
    
#ifndef NEW_SYNC_TEST
    // sync vel for pos & rot    
    v_p = (v_p < EPS_P)? SYNC_POS_VEL(traj, v_r) : v_p; 
    v_r = (v_r < EPS_R)? SYNC_ROT_VEL(traj, v_p) : v_r; 
#endif 

    // Circular Motion Setting
    if(Cmot_Set(&traj->cmot, &cent,  
                &Ts.p, &s_p, arc, v_p, 
                &Ts.R, &s_r, ang_rot, v_r))
    {        
        return TRAJ_ERR_MOTION_FAIL; 
    }

    // Translation Motion Profile Setting 
    if(EPS_P < v_p)
    {
        if(Prof7_Set(&prof_pos, arc, v_p, 
                    traj->acc_pos, traj->jerk_pos))
        {        
            return TRAJ_ERR_PROFILE_FAIL;
        }
    }

    // Orientation Motion Profile Setting 
    if(EPS_O < v_r)
    {
        if(Prof7_Set(&prof_rot, ang_rot, v_r, 
                    traj->acc_rot, traj->jerk_rot))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

#ifdef NEW_SYNC_TEST
    // Switch to the another Profile if one vel is 0
    prof_pos = (EPS_P < v_p)? prof_pos : prof_rot; 
    prof_rot = (EPS_O < v_r)? prof_rot : prof_pos; 
#endif 
    // Synchronization
    traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
    
    // set TRAJ          
    traj->type    = TRAJ_TYPE_CIRCLE;     
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 


    // Reset Error 
    TrajErrorReset(traj); 
    return 0;
}

// Trajectory setting for Circular Motion
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
                   unsigned     conf_f)         // robot pose configuration
{
    TRANS bTe, bTt; 
    unsigned conf_s; 
    PROFILE7 prof_pos, prof_rot; 

    // data validation
    if(traj == NULL || jnt_s == NULL || 
       cent == NULL || s_p == NULL   || s_r == NULL )
    {
        return TRAJ_ERR_WRONG_PARAM; 
    }

#ifdef NEW_SYNC_TEST
    if(v_p < EPS_P && v_r < EPS_O)
    {
        return TRAJ_ERR_ZERO_VEL; 
    }
#endif 


#if 0 // PRINT
        printf("[D] Conf_s : %x, Conf_t : %x \n", conf_s, conf_f); 
        printf("Joint_a : %f %f %f  \n", 
            jnt_s[0], jnt_s[1], jnt_s[2]); 
#endif

    // Forward Kinematics     
    conf_s = traj->Config(jnt_s, traj->dh); 
    if(conf_f != conf_s)
    {
        return TRAJ_ERR_CONF_MISMATCH; 
    }
    traj->config  = conf_s; 

    // Start TRANS
    traj->Forward(&bTe, jnt_s, traj->dh);     
    bTt = TRANS_Multi_TRANS(bTe, traj->eTt); 
    
#ifndef NEW_SYNC_TEST
    // sync vel for pos & rot    
    v_p = (v_p < EPS_P)? SYNC_POS_VEL(traj, v_r) : v_p; 
    v_r = (v_r < EPS_R)? SYNC_ROT_VEL(traj, v_p) : v_r; 
#endif 

    // Setting Circular Motion
    if(Cmot_Set(&traj->cmot, cent, 
                &bTt.p, s_p, arc, v_p, 
                &bTt.R, s_r, rot, v_r))    
    {        
        return TRAJ_ERR_MOTION_FAIL; 
    }

    // Linear Motion Profile Setting 
    if(EPS_P < v_p)
    {
        if(Prof7_Set(&prof_pos, arc, v_p, 
                    traj->acc_pos, traj->jerk_pos))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }
    }

    // Rotation Motion Profile Setting 
    if(EPS_O < v_r)
    {
        if(Prof7_Set(&prof_rot, rot, v_r, 
                    traj->acc_rot, traj->jerk_rot))
        {        
            return TRAJ_ERR_PROFILE_FAIL; 
        }    
    }

#ifdef NEW_SYNC_TEST
    // Switch to the another Profile if one vel is 0
    prof_pos = (EPS_P < v_p)? prof_pos : prof_rot; 
    prof_rot = (EPS_O < v_r)? prof_rot : prof_pos; 
#endif 

    // Synchronization
    traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 

    // set TRAJ                
    traj->type    = TRAJ_TYPE_CIRCLE; 
    traj->t_start = traj->t_samp;       // 1st target iteration is 1. 

    // Reset Error 
    traj->error = 0; 
    memset(traj->err_pos, 0, sizeof(traj->err_pos)); 
    memset(traj->err_vel, 0, sizeof(traj->err_vel)); 
    return 0;
}

double Traj_GetProf(TRAJ* traj)
{
    ASSERT_RETURN(traj, TRAJ_ERR_WRONG_PARAM); 
    return Prof7_Get(&traj->profile, NULL, NULL, traj->t_start);  
}

// Gets Target Pos of 1Axis Trajectory.
// Returns SUCCESS(0), TRAJ_ERR_WRONG_PARAM, TRAJ_ERR_INVERSE_FAIL, TRAJ_ERR_INV_FAIL_AXIS_FAIL
int Traj_GetTarget(TRAJ* traj, 
                   double target[6], 
                   double* u, 
                   const double p_prev[6], 
                   const double v_prev[6])
{    
    int ret = 0;         
        
    ASSERT_RETURN(traj, TRAJ_ERR_WRONG_PARAM); 
   
    if(traj->type == TRAJ_TYPE_NORMAL) 
    {
        return 0;
    }
    
    // 1st Target Calculation & Inv Error Check  
    ret = TargetCalc(traj, target, u, p_prev); 
    if(ret != TRAJ_ERR_NONE)      
    {
        // Inverse Error 
        JointStop(traj, p_prev, v_prev); 
        TargetCalc(traj, target, u, p_prev);   

        // Error Set
        if(!traj->error)
        {
            memcpy(traj->err_pos, target, sizeof(traj->err_pos)); 
            memcpy(traj->err_vel, v_prev, sizeof(traj->err_vel)); 
        }

        SET_ERROR(&traj->error, ret); 
        return ret; 
    }

    // What is this module meaning? I think useless module. ////////////////////
    if(ret != TRAJ_ERR_NONE)
    {    
        // Error Set
        if(!traj->error)
        {
            memcpy(traj->err_pos, target, sizeof(traj->err_pos)); 
            memcpy(traj->err_vel, v_prev, sizeof(traj->err_vel)); 
        }
        SET_ERROR(&traj->error, ret); 
        return ret; 
    }
    // What is this module meaning? I think useless module. ////////////////////

    // Limit Error Check 
    ret = LimitCheck(traj, target, p_prev); 
    if(ret)
    {
        double v_temp; 
        int i; 
  
        Traj_Stop(traj, 1);                    
        TargetCalc(traj, target, u, p_prev); 

        ////////////////////////////////////////////////////////////////////////
        // Check Increase of Vel After Stopping, In case of Near of Inv. Error. 

        for(i=0 ; i<6 ; i++)
        {
            // Error & Decel But Velocity is not decreased.             
            v_temp = target[i] - p_prev[i];
            
            if(v_prev[i]*v_prev[i] < v_temp*v_temp)
            {               
                // Nearby Inverse Error 
                JointStop(traj, p_prev, v_prev); 
                TargetCalc(traj, target, u, p_prev);   
                return ret; 
            }
        }
        return ret; 
    }
    if(ret != TRAJ_ERR_NONE)
    {        
        return ret; 
    }

    return ret; 
}

// Stops Running Traj. 
// f_estop : flag for fast stop... 
void Traj_Stop(TRAJ* traj, int f_estop)
{
    double dec; 
    double t_stop;
    int i; 
    PROFILE7 prof_pos, prof_rot; 

    ASSERT_RETURN(traj, ;); 
    
    // Setting Stop Time(Prev. Sampling, >0)
    t_stop = traj->t_start - traj->t_samp; 
    t_stop = (t_stop < 0)? 0 : t_stop; 

    switch(traj->type)
    {    
    case TRAJ_TYPE_JOINT:  
        for(prof_pos.t_2d=0, i=0 ; i<6 ; i++)
        {    
            dec = (f_estop)? traj->dec_fast_joint[i] : traj->dec_stop_joint[i]; 

            // stop profile of each joint            
            prof_rot = traj->profile;   
            Prof7_Stop(&prof_rot, traj->jmot[i].dbJointD, dec, t_stop);             
            prof_pos = Prof7_GetSync(&prof_pos, &prof_rot); 
        }     
        // synchronize profile with max stop time profile 
        traj->profile = prof_pos; 
        break; 
    
    case TRAJ_TYPE_LINEAR:

        // pos profile
        prof_pos = traj->profile; 
        dec = (f_estop)? traj->dec_fast_pos : traj->dec_stop_pos; 
        Prof7_Stop(&prof_pos, traj->lmot.dist, dec, t_stop);        

        // rot profile
        prof_rot = traj->profile; 
        dec = (f_estop)? traj->dec_fast_rot : traj->dec_stop_rot; 
        Prof7_Stop(&prof_rot, traj->lmot.ang, dec, t_stop);        

        // sync pos & rot profile
        traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
        break; 

    case TRAJ_TYPE_CIRCLE:
        
        // pos profile
        prof_pos = traj->profile; 
        dec = (f_estop)? traj->dec_fast_pos : traj->dec_stop_pos; 
        Prof7_Stop(&prof_pos, traj->cmot.arc, dec, t_stop);

        // rot profile
        prof_rot = traj->profile; 
        dec = (f_estop)? traj->dec_fast_rot : traj->dec_stop_rot; 
        Prof7_Stop(&prof_rot, traj->cmot.ang, dec, t_stop);

        // sync pos & rot profile
        traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
        break; 

    case TRAJ_TYPE_NORMAL:
    default:
        Prof7_Stop(&traj->profile, 0, LARGE_NUM, t_stop);                     
        break; 
    }
    return; 
}

// Stops Running Traj in the decel time 't_dec'. 
void Traj_Stop_Time(TRAJ* traj, DBL t_dec)
{
    double dec; 
    double t_stop;
    int i; 

    PROFILE7 prof_pos, prof_rot; 

    ASSERT_RETURN(traj, ;); 
    t_dec = (t_dec < EPS_T)? EPS_T : t_dec; 
    
    // Setting Stop Time(Prev. Sampling, >0)
    t_stop = traj->t_start - traj->t_samp; 
    t_stop = (t_stop < 0)? 0 : t_stop; 

    switch(traj->type)
    {    
    case TRAJ_TYPE_JOINT:  
        for(prof_pos.t_2d=0, i=0 ; i<6 ; i++)
        {    
            dec = traj->lim_vel[i] / t_dec;
            // stop profile of each joint            
            prof_rot = traj->profile;   
            Prof7_Stop(&prof_rot, traj->jmot[i].dbJointD, dec, t_stop);             
            prof_pos = Prof7_GetSync(&prof_pos, &prof_rot); 
        }     
        // synchronize profile with max stop time profile 
        traj->profile = prof_pos; 
        break; 
    
    case TRAJ_TYPE_LINEAR:

        // pos profile
        prof_pos = traj->profile; 
        dec = traj->lim_v_lin / t_dec; 
        Prof7_Stop(&prof_pos, traj->lmot.dist, dec, t_stop);        

        // rot profile
        prof_rot = traj->profile; 
        dec = traj->lim_v_rot / t_dec; 
        Prof7_Stop(&prof_rot, traj->lmot.ang, dec, t_stop);        

        // sync pos & rot profile
        traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
        break; 

    case TRAJ_TYPE_CIRCLE:
        
        // pos profile
        prof_pos = traj->profile; 
        dec = traj->lim_v_lin / t_dec; 
        Prof7_Stop(&prof_pos, traj->cmot.arc, dec, t_stop);

        // rot profile
        prof_rot = traj->profile; 
        dec = traj->lim_v_rot / t_dec; 
        Prof7_Stop(&prof_rot, traj->cmot.ang, dec, t_stop);

        // sync pos & rot profile
        traj->profile = Prof7_GetSync(&prof_pos, &prof_rot); 
        break; 

    case TRAJ_TYPE_NORMAL:
    default:

        Prof7_Stop(&traj->profile, 0, LARGE_NUM, t_stop);                     
        break; 
    }
    return; 
}

// Error Code String is returned. 
char* Traj_ErrorStr(char err)
{
    switch(err)
    {
    case TRAJ_ERR_NONE:
        return "None"; 
    case TRAJ_ERR_LIMIT_POS:
        return "Pos Limit"; 
    case TRAJ_ERR_LIMIT_POS_0:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_POS_1:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_POS_2:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_POS_3:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_POS_4:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_POS_5:
        return "Pos Limit 0"; 
    case TRAJ_ERR_LIMIT_VEL:
        return "Vel Limit"; 
    case TRAJ_ERR_LIMIT_VEL_0:
        return "Vel Limit 0"; 
    case TRAJ_ERR_LIMIT_VEL_1:
        return "Vel Limit 1"; 
    case TRAJ_ERR_LIMIT_VEL_2:
        return "Vel Limit 2"; 
    case TRAJ_ERR_LIMIT_VEL_3:
        return "Vel Limit 3"; 
    case TRAJ_ERR_LIMIT_VEL_4:
        return "Vel Limit 4"; 
    case TRAJ_ERR_LIMIT_VEL_5:
        return "Vel Limit 5"; 
    case TRAJ_ERR_CONF_MISMATCH:
        return "Pose Config Mismatch"; 
    case TRAJ_ERR_UNDEF_CIRCLE:
        return "No Circle Found"; 
    case TRAJ_ERR_NEAR_POS:
        return "So Near Poses Used"; 
    case TRAJ_ERR_WRONG_PARAM:
        return "Wrong Param Used"; 
    case TRAJ_ERR_PROFILE_FAIL:
        return "Failed to Get Vel-profile"; 
    case TRAJ_ERR_ZERO_VEL:
        return "Near 0 Vel"; 
    case TRAJ_ERR_MOTION_FAIL:
        return "Failed to Get MOT"; 
    case TRAJ_ERR_UNDEF_INVERSE: 
        return "Inverse Error (Undef)"; 
    case TRAJ_ERR_INVERSE_Z0:
        return "Singular, Target Near Z0"; 
    case TRAJ_ERR_INVERSE_TH5:
        return "Singular, th5 0 or PI"; 
    case TRAJ_ERR_INVERSE_UNREACH:
        return "Invserse Err, Unreachable";  
    case TRAJ_ERR_INVERSE_LARGE_V:
        return "Inverse Err. Large Vel"; 
    default:
        return "Undef"; 
    }
}

// Returns Pointer to the string of TRAJ Info's.
// Units : [rad, rad/s, rad/s^2]
char* Traj_Print(TRAJ* traj)
{
    int i, n; 
    static char str[80*64]; 

    str[0] = 0; 
    n = 0; 
    n += CRT_sprintf(str+n, sizeof(str)-n, "Traj Err(%d) : %s\n", traj->error, Traj_ErrorStr(traj->error));
    n += CRT_sprintf(str+n, sizeof(str)-n, "      <Jrk>  <Acc>  <Dec> <E-Dec>  <L_Max>     <L_Min> <L_Vel> <p_err> <v_err> \n"); 
            
    // Joint Dynamics
    for(i=0 ; i<6 ; i++)
    {
        n += CRT_sprintf(str+n, sizeof(str)-n, 
            " [%d] %5.2e %5.2f  %5.2f  %5.2f  %5.2e  %5.2e  %5.2f  %5.2f  %5.2f\n",
            i, traj->jerk_joint[i]*1000000000, traj->acc_joint[i] * 1000000, traj->dec_stop_joint[i] * 1000000, traj->dec_fast_joint[i] * 1000000, 
               traj->lim_max[i],   traj->lim_min[i],   traj->lim_vel[i]*1000, 
               traj->err_pos[i],   traj->err_vel[i]*1000); 
    }
    return str; 
}

char* Traj_TypeStr(int type)
{
    switch(type)
    {
    case TRAJ_TYPE_JOINT:
        return "JNT"; 
    case TRAJ_TYPE_LINEAR:
        return "LIN"; 
    case TRAJ_TYPE_CIRCLE:
        return "CIR"; 
    default:
        return "NON"; 
    }
}

////////////////////////////////////////////////////////////////////////////////

// Calculates Target of 'time' as Mode of 'traj'. 
// Return Error Code of TRAJ
static int TargetCalc(const TRAJ* traj,         //[i]
                      double trg[6],            //[o]
                      double* u,                //[o]
                      const double prev[6])     //[i]                                             
{
    double t; 
    int i, ret; 
    TRANS bTt, bTe; 

    ASSERT_RETURN((traj && prev && trg && u), TRAJ_ERR_WRONG_PARAM); 
    
    t = traj->t_start; 

    switch(traj->type)
    {    
    case TRAJ_TYPE_JOINT:
        // get parameter 'u' and Joint with jmot
        *u = Prof7_Get(&traj->profile, NULL, NULL, t);                         
        for(i=0 ; i<6 ; i++)
        {   
            // Target Joint Value
            trg[i] = Jmot_JointOnParam(&traj->jmot[i], *u); 
        }
        return 0; 
    
    case TRAJ_TYPE_LINEAR:        

        // Get Postion 
        *u = Prof7_Get(&traj->profile, NULL, NULL, t); 
       
        // Target Pose
        Lmot_PosOnParam(&bTt.p, &traj->lmot, *u);            
        Lmot_RotOnParam(&bTt.R, &traj->lmot, *u);

        // Target Joint
        bTe = TRANS_Multi_TRANS(bTt, TRANS_Inv(traj->eTt)); // bTe = bTt * tTe

#if 0 // PRINT        
    if(isnan(bTe.p.x))
    {
        printf("bTt.x:%f y:%f, bTe.x:%f y:%f d:%f u:%f\n", bTt.p.x, bTt.p.y, bTe.p.x, bTe.p.y, traj->lmot.dist, *u);
        TRANS_Print4(bTt, "bTt"); 
        TRANS_Print4(traj->eTt, "eTt"); 
    }
#endif 
        ret = (traj->Inverse)(trg, &bTe, traj->config, traj->dh, prev, 1);

        return (ret == 0)?                    0                    :  
               (ret == ERR_INVALID_PARAM)?    TRAJ_ERR_WRONG_PARAM :               
               (ret == ERR_SINGULAR_NEAR_Z0)? TRAJ_ERR_INVERSE_Z0  : 
               (ret == ERR_SINGULAR_TH5)?     TRAJ_ERR_INVERSE_TH5 : 
               (ret == ERR_UNREACHABLE)?      TRAJ_ERR_INVERSE_UNREACH : 
               (ret == ERR_LARGE_VEL)?        TRAJ_ERR_INVERSE_LARGE_V : 
                                              TRAJ_ERR_UNDEF_INVERSE; 
        
    case TRAJ_TYPE_CIRCLE:       

        *u = Prof7_Get(&traj->profile, NULL, NULL, t); 
        
        // Target Pose     
        Cmot_PosOnParam(&bTt.p, &traj->cmot, *u);            
        Cmot_RotOnParam(&bTt.R, &traj->cmot, *u);

        // Target Joint Value
        bTe = TRANS_Multi_TRANS(bTt, TRANS_Inv(traj->eTt)); // bTe = bTt * tTe
        ret = (traj->Inverse)(trg, &bTe, traj->config, traj->dh, prev, 1);


        // Ret Suc(0), INVALID_PARAM, SINGULAR_NEAR_Z0, SINGULAR_TH5, UNREACHABLE
        // ret = 0; //Cmot_JointOnParam(trg, &traj->cmot, u[PROF_POS], u[PROF_ROT], prev);   
        
        return (ret == 0)?                    0                    :  
               (ret == ERR_INVALID_PARAM)?    TRAJ_ERR_WRONG_PARAM :               
               (ret == ERR_SINGULAR_NEAR_Z0)? TRAJ_ERR_INVERSE_Z0  : 
               (ret == ERR_SINGULAR_TH5)?     TRAJ_ERR_INVERSE_TH5 : 
               (ret == ERR_UNREACHABLE)?      TRAJ_ERR_INVERSE_UNREACH : TRAJ_ERR_UNDEF_INVERSE; 
        
    case TRAJ_TYPE_NORMAL:
    default:
        return 0; 
    }
}

// Converts Joint Mode & Set Stops. 
// Returns Traj Error Code. 
static int JointStop(TRAJ* traj, const double p_prev[6], const double v_prev[6])
{
    int i; 
    double d_dec, td; 
    PROFILE7 prof; 

    ASSERT_RETURN(traj, TRAJ_ERR_WRONG_PARAM);

    // Motion & Profile Setting    
    td = traj->lim_vel[0] / traj->dec_fast_joint[0]; 
    for(traj->profile.t_2d = 0, i=0 ; i<6 ; i++)
    {
        // PROFILE Setting
        if(Prof7_Set_Dec(&prof, &d_dec, v_prev[i], 
                         traj->dec_fast_joint[i], 
                         traj->t_start-traj->t_samp))
        {            
            return TRAJ_ERR_PROFILE_FAIL; 
        }
        // New Joint Motion Value
        if(Jmot_Set(&traj->jmot[i], p_prev[i], p_prev[i] + d_dec, v_prev[i]))            
        {            
            return TRAJ_ERR_MOTION_FAIL; 
        }
        // Sync        
        traj->profile = Prof7_GetSync(&traj->profile, &prof);         
    }    

    // set TRAJ Joint Mode.                
    traj->type    = TRAJ_TYPE_JOINT;         
    return TRAJ_ERR_NONE;         
}

// Returns  'target' is No Limit(0), TRAJ_ERR_LIMIT_POS, TRAJ_ERR_LIMIT_VEL
static int LimitCheck(TRAJ* traj, const double target[6], const double prev[6])
{
    int i; 
    int ret; 
    double v[6]; 

    // validation of param's
    if(!traj || !target || !prev)
    {
        return TRAJ_ERR_WRONG_PARAM; 
    }

    // v
    for(i=0 ; i<6 ; i++)
    {
        v[i] = fabs((target[i]-prev[i])/traj->t_samp); 
    }

    // check 
    for(i=0, ret=0; i<6 ; i++)
    {
        // Max(+) Limit Check 
        if(traj->lim_max[i] <= target[i] && prev[i] < target[i])
        {         
            // ret = TRAJ_ERR_LIMIT_POS;             
            ret = LimitErrAssign_Pos(i); 
            break; 
        }
        
        // Min(-) Limit Check 
        if(target[i] <= traj->lim_min[i] && target[i] < prev[i])
        {   
            // ret = TRAJ_ERR_LIMIT_POS; 
            ret = LimitErrAssign_Pos(i); 
            break;
        }
        
        // Velocity Check         
        if(traj->lim_vel[i] <= v[i] - 0.000000001)  // 0.000000001 rad/ms, Eps of vel of ori 
        {   
            // ret = TRAJ_ERR_LIMIT_VEL;
            ret = LimitErrAssign_Vel(i); 
            break; 
        }
    } 
    if(ret)
    {
        // Error Set
        if(!traj->error)
        {
            memcpy(traj->err_pos, target, sizeof(traj->err_pos));
            memcpy(traj->err_vel, v,      sizeof(traj->err_vel)); 
        }
        SET_ERROR(&traj->error, ret); 
    }
    return ret; 
}

static int LimitErrAssign_Pos(int i_axis)
{
    return (i_axis == 0)? TRAJ_ERR_LIMIT_POS_0 : 
           (i_axis == 1)? TRAJ_ERR_LIMIT_POS_1 : 
           (i_axis == 2)? TRAJ_ERR_LIMIT_POS_2 : 
           (i_axis == 3)? TRAJ_ERR_LIMIT_POS_3 : 
           (i_axis == 4)? TRAJ_ERR_LIMIT_POS_4 : 
           (i_axis == 5)? TRAJ_ERR_LIMIT_POS_5 : TRAJ_ERR_LIMIT_POS; 
}

static int LimitErrAssign_Vel(int i_axis)
{
    return (i_axis == 0)? TRAJ_ERR_LIMIT_VEL_0 : 
           (i_axis == 1)? TRAJ_ERR_LIMIT_VEL_1 : 
           (i_axis == 2)? TRAJ_ERR_LIMIT_VEL_2 : 
           (i_axis == 3)? TRAJ_ERR_LIMIT_VEL_3 : 
           (i_axis == 4)? TRAJ_ERR_LIMIT_VEL_4 : 
           (i_axis == 5)? TRAJ_ERR_LIMIT_VEL_5 : TRAJ_ERR_LIMIT_VEL; 
}
