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

#include "memory.h"
#include "float.h"
#include "utility.h"
#include "math.h"
#include "geometry.h"
#include "profile7.h"
#include "trajectory.h"
#include "weave.h"
#include "sensor.h"
#include "CRT.h"

#define ERRMASK_TRAJ 0x0100

#if 0
#define TEST_   // ARC Sensor Center Start Test
#endif 

#ifdef TEST_

#include "geometry.h"
#include "dandy_job.h"

void For_Dandy2(TRANS *bTe,             //[out]trans of base->6
                const double *th,       //[in] physical joint.
                const DH *dh);          //[in] dh-param

extern DANDY_JOB_POS* g_varP; 
extern TRAJ g_traj; 
extern TRANS g_eTt; 

TRANS bTe, bTt; 
POS xw_1st;                     // xw before 12th node of main weaving. 
POS xw_ori;                 // Since 12th node of main the original xw is used. 

#endif 
////////////////////////////////////////////////////////////////////////////////

// Plans Weaving Motion as Current Mode is EVEN_DWL or ODD_DWL. 
// At Start & Final this must not be called
// - wv : WEAVE
// - th_a : Actual Joint Value
// - i : Motion Index. (0)Start, (-1)Final, (Else)Normal
static int MotionPlan(WEAVE* wv, const double* th_a, int i);

// i_mot : dwell of i_th motion. 
static int DwellPlan(WEAVE* wv, int i_mot);

static unsigned TrajErrAssign(unsigned char err)
{
    return ERRMASK_TRAJ | err; 
}

////////////////////////////////////////////////////////////////////////////////

UNT Wv_IsTrajErr(unsigned err)
{
    return ERRMASK_TRAJ & err; 
}

// returns ON if 'wv' is running or OFF. 
UNT Wv_IsRunning(WEAVE* wv)
{
    return (wv->mode != WV_MODE_NONE)? ON : OFF; 
}

// The flag for the arc-sensing is 'weld_idx::arc_sensor'. 
int Wv_Set(WEAVE*       wv,             // 
           TRAJ*        traj,           // Basic Init of 'traj' is prior to this
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
           double       d_angle,        // angle varying distance. if 0, Rot Mot is all over WV dist.
           double       d_start,        // start weaving distance. if 0, Start with Main                                            
           double       d_final)        // final distance from end.      
{    
    TRANS Te, Ts; 
    POS s_upper; 
    int ret; 
  
    // pitch==0.0 : Hover weaving
    if( !wv             || !traj        || // !arc_sensor      ||
        !th_s           || !Tf          || 
        pitch < 0.0     || width <= 0.0 || speed <= EPS_P ||
        dwl_e < 0.0  || dwl_o < 0.0 ) 
    {   
        wv->error = WV_ERR_INV_PARAM; 
        return WV_ERR_INV_PARAM;
    }

    // Case of Start Weaving Exist
    if((0. < d_start) && (pitch_s < 0.0 || width_s <= 0. || speed_s <= EPS_P))
    {
        wv->error = WV_ERR_INV_PARAM; 
        return WV_ERR_INV_PARAM;
    }
    
    // traj setting
    wv->traj = traj; 
#if 0
    wv->arc_sensor = arc_sensor; 
#elif 0
    wv->f_sensor = f_sensor;
#endif
    wv->config = config;
    
    // Ts
    traj->Forward(&Te, th_s, traj->dh); 
    Ts = TRANS_Multi_TRANS(Te, traj->eTt); 
    wv->pw = Ts.p; 
    wv->Rs = Ts.R; 
    
    // xw
#ifdef TEST_
    // xw_1st is used before 12th node of main weaving, 
    // xw_ori is used since 12th node of main weaving.     
    {
        double jnt[6] = {
            g_varP[50].pos[0]*PI/180., g_varP[50].pos[1]*PI/180., 
            g_varP[50].pos[2]*PI/180., g_varP[50].pos[3]*PI/180., 
            g_varP[50].pos[4]*PI/180., g_varP[50].pos[5]*PI/180.};             
        For_Dandy2(&bTe, jnt, g_traj.dh); 
        bTt = TRANS_Multi_TRANS(bTe, g_eTt); 
    }    

    xw_1st = POS_Minus_POS(bTt.p, Ts.p);
    xw_ori = POS_Minus_POS(Tf->p, Ts.p);     
    
    wv->d_total = POS_Norm(xw_ori);     
    
    POS_Unit(&xw_1st); 
    POS_Unit(&xw_ori);     
    wv->xw = xw_1st; 
    
#else
    wv->xw = POS_Minus_POS(Tf->p, Ts.p); 
    wv->d_total = POS_Norm(wv->xw);     
#endif 

#if 0
    if(wv->d_total < 0.25*((0. < d_start)? pitch_s:pitch) )
    {
        wv->error = WV_ERR_SHORT_DIST; 
        return WV_ERR_SHORT_DIST; 
    }
#endif
    if(POS_Unit(&wv->xw))
    {
        wv->error = WV_ERR_NO_WX_DIR;
        return WV_ERR_NO_WX_DIR;
    }
    if(wv->d_total + EPS_P < d_start + d_final)
    {
        wv->error = WV_ERR_LONG_START_END_DIST;
        return WV_ERR_LONG_START_END_DIST; 
    }
    if(wv->d_total + EPS_P < d_angle)
    {
        return WV_ERR_LONG_ANGLE_DIST;
    }
    
    wv->yw = ROT_Multi_POS(ROT_Screw(wv->xw, ang_plane), *s_plane);     
    wv->d_trg = 0.; 
    wv->d_act = 0; 

    // zw : dir to the workpiece & orthogonal to xw, yw. 
    s_upper = POS_Minus_POS(Te.p, Ts.p); // Ts:Start TCP, Te:Start END(Wrist)
    wv->zw  = POS_Cross(wv->xw, wv->yw);
    if(POS_Inner(s_upper, wv->zw) < 0.)
    {
        // If oposite to the dir of wall, Set Oposite dir. 
        wv->zw = POS_Multi_S(wv->zw, -1.); 
    }    

    // Translation Motion Parameter Setting
    wv->v_pos = speed; 
    wv->pitch = pitch;      // pitch == 0, Hover weaving
    wv->width = width;     

    // Start 
    wv->v_pos_s = speed_s; 
    wv->pitch_s = pitch_s;  // pitch == 0, Hover weaving
    wv->width_s = width_s;     

#if 0
    wv->dwl_even = dwl_e; 
    wv->dwl_odd  = dwl_o;     
    wv->dwl_cent = dwl_c; 
    
#else
    // !!!!!! CAUTION !!!!!!
    // For arc sensor, Min 0.1ms dwell time needs for proper gethering ADC data. 
    // Min 0.1ms must not be applied to the center dwell. 
    wv->dwl_even = (weld_idx.arc_sensor && dwl_e < 100)? 100 : dwl_e;  
    wv->dwl_odd  = (weld_idx.arc_sensor && dwl_o < 100)? 100 : dwl_o;     
    // !!!!!! CAUTION !!!!!!    
#endif 

    wv->f_cent = 0; 

    // distance setting 
    // a little length is counted 1 node.     
    // + 1, - 0.25*pitch_s : start node pitch
    // 0.5*pitch_s - DBL_EPSILON : regarding truncation
#if 0
    wv->n_start = (unsigned)( (d_start < EPSILON || pitch_s < EPSILON)? 
             0.:( (d_start - 0.25*pitch_s + 0.5*pitch_s - DBL_EPSILON) / (0.5*pitch_s) ) + 1); 
#endif 
    wv->n_start = (unsigned)( (d_start < EPS_P || pitch_s < EPS_P)? 
             0.:( (d_start - 0.25*pitch_s + 0.5*pitch_s - EPS_P) / (0.5*pitch_s) ) + 1); 

    wv->d_final = d_final; 
    wv->d_angle = d_angle; 

    // Segment setting 
    wv->n_seg = (wv->n_start <= 0.)? WV_SEG_MAIN : WV_SEG_START; 
    wv->f_seg_change = ON; 
    
#if 0
    // Orientation Motion Parameter Setting     
    // d_angle is the rotational motion distance. 
    // if d_angle == 0, Rotation is all over weave distance.     
    // if d_angle is smaller than 0-th mot dist, d_angle is 0-th mot dist. 
    wv->d_angle = (d_angle <= 0.0)?         wv->d_total : 
                  (0. < d_start && d_angle <= 0.25*pitch_s)? 0.25*pitch_s : 
                  (0. >=d_start && d_angle <= 0.25*pitch  )? 0.25*pitch   : d_angle; 
#else
    // Orientation Motion Parameter Setting     
    // d_angle is the rotational motion distance. 
    // if d_angle == 0, Rotation is all over weave distance.         
    wv->d_angle = (d_angle <= 0.0)? wv->d_total : d_angle;                   
#endif
    
    // rotational info's from 'bTt_s' & 'bTt_f'
    // R_f = R * R_s -> R = R_f * R_s'
    if(ROT_GetScrewAxis(&wv->sr, &wv->r_total, ROT_Multi_ROT(Tf->R, ROT_Inv(Ts.R))))
    {
        wv->sr = Ts.R.w; 
        wv->r_total = 0.0; 
    }
    
    // stop flag setting 
    wv->f_stop = 0; 
    
    // 1st Motion Plan    
    ret = MotionPlan(wv, th_s, 0);     
    if(ret)
    {
        wv->error = ret; 
        wv->mode = WV_MODE_NONE; 
        return ret; 
    }

    if(weld_idx.arc_sensor)
    {
        arc_save_num = 0; 
        ref_current  = 0; 
        ref_weight   = 0; 
        memset(Delta_Z, 0, sizeof(Delta_Z)); 
        memset(Delta_T, 0, sizeof(Delta_T)); 
        memset(Mean_Moving_Ampere, 0, sizeof(Mean_Moving_Ampere)); 
        memset(Mean_Moving_Weight, 0, sizeof(Mean_Moving_Weight)); 
    }

    wv->error = 0; 
    wv->i_mot = 0;    
    return 0;    
}

int Wv_Update(WEAVE* wv, const double* th_a, const TRANS* bTt_a)
{   
    int ret; 
    unsigned n_seg; 

    ASSERT_RETURN(wv, -1); 
        
    // None Running Case 
    if(wv->mode == WV_MODE_NONE)
    {
        return -1; 
    }

    // mode time update
    wv->t_mode += wv->traj->t_samp; 

    // Resets Weaving Seg Change Flag
    // 1st iteration is not included to use the change of START_SEG
    if(wv->d_act > 0)
    {
        wv->f_seg_change = OFF; 
    }

    // Work Distance     
    wv->d_act = POS_GetDist(POS_Minus_POS(bTt_a->p, wv->pw), wv->xw);     

    ////////////////////////////////////////////////////////////////////////////
    // End of Weaving : Final Weaving End
    if(wv->mode == WV_MODE_FIN_MOT)
    {
        if(!Traj_Update(wv->traj))       
        {            
            return 0; 
        }
        wv->mode = WV_MODE_NONE; 
        return -1; 
    }

    // Motion Case 
    if(wv->mode == WV_MODE_MOT)
    {       
        if(!Traj_Update(wv->traj))       
        {            
            return 0; 
        }

        ////////////////////////////////////////////////////////////////////////
        // End of Motion. Dwell Planning

        // Check Error of Traj
        if(wv->error)
        {
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }    

        // Traj Error Check 
        if(wv->traj->error)
        {
#if 0 // error 
            wv->error = WV_ERR_TRAJ; 
#else
            wv->error = TrajErrAssign(wv->traj->error); 
#endif 
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }

        // Check Stop Condition
        if(wv->f_stop)
        {
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }
        // Dwell Planning
        ret = DwellPlan(wv, wv->i_mot); 
        if(ret)
        {
            wv->error = ret;
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }
        
        // At Motion End, Arc-sensor data gethering Stop Request. 
        // During Center-Dwell, Gethering goes on.         
        if(!wv->f_cent)
        {
            shm_servo->ADC_gathering_request = OFF; 
        }

        wv->mode = WV_MODE_DWL; 
        wv->t_mode = wv->traj->t_samp;
        return 0; 
    }

    // Dwell Case 
    if(wv->mode == WV_MODE_DWL)
    {   
        // Check Error of Traj
        if(wv->error)
        {
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }        
        // Check Stop Condition
        if(wv->f_stop)
        {
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }

         // Check Dwell Time Done      
        if(wv->t_mode < (wv->t_dwl + wv->traj->t_samp))
        {
            return 0;
        }         

        ////////////////////////////////////////////////////////////////////////
        // End of Dwell 
         
        // motion index update
        // if motion is the center motion, motion index is not counted. 
        if(wv->f_cent)
        {            
            wv->f_cent = 0; 
        }
        else
        {
            wv->f_cent = (0 < wv->dwl_cent)? 1 : 0; // Center Dwell runs if dwl_cent > 0. 
            wv->i_mot++;
        }

        // Weaving Segment Setting 
        // This must be before 'MotionPlan' which upgrades 'wv->d_trg'
#if 1
#if 0
        wv->n_seg = (wv->d_trg <= wv->d_start)?                 WV_SEG_START : 
                    (wv->d_total - wv->d_trg <= wv->d_final)?   WV_SEG_FINAL :
                                                                WV_SEG_MAIN; 
#else
        // WARNING !! 
        // Presently i_mot is updated. but d_trg is before updated. 
        // Judge Priority Order : START->MAIN->FINAL
        n_seg = (wv->i_mot < wv->n_start)?                  WV_SEG_START: 
                  (wv->d_total - wv->d_trg > wv->d_final)?  WV_SEG_MAIN :
                                                            WV_SEG_FINAL; 
        if(n_seg != wv->n_seg)
        {
            wv->f_seg_change = ON; 
        }
        wv->n_seg = n_seg; 
#endif

#ifdef TEST_
        if(wv->i_mot >= 12 + wv->n_start)
        {
            wv->xw = xw_ori; 
        }
#endif 

        // motion planning 
        ret = MotionPlan(wv, th_a, wv->i_mot); 
        if(ret)
        {
            wv->error = ret; 
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }
#else   
        // Presently i_mot is updated. but d_trg is before updated. 
        n_seg = (wv->i_mot < wv->n_start)? WV_SEG_START : WV_SEG_MAIN; 
        wv->f_seg_change = (wv->n_seg != n_seg)? ON : OFF; 
        wv->n_seg = n_seg; 

        // motion planning, wv->d_trg is updated
        ret = MotionPlan(wv, th_a, wv->i_mot); 
        if(ret)
        {
            wv->error = ret; 
            wv->mode = WV_MODE_NONE; 
            return -1; 
        }
        
        // weaving seg 1 more update. 
        n_seg = (wv->d_total - wv->d_trg <= wv->d_final)? WV_SEG_FINAL : wv->n_seg; 
        if(wv->n_seg == WV_SEG_MAIN && wv->d_total - wv->d_trg <= wv->d_final)
        {
            wv->n_seg = WV_SEG_FINAL; 
            wv->f_seg_change = (wv->d_total - wv->d_trg <= wv->pitch)? ON : OFF; 
        }

#endif

        // At Motion Start, ADC Data Gethering Reset & Start Request 
        // No Center-dwell Weaving, Every Motoin Start, Gethering is Requested. 
        // Center-dwell Weaving, Every Center Motion Start, Gethering is Requested. 
        if(wv->dwl_cent <= 0 || wv->f_cent )
        {
            shm_servo->ADC_gathering_request = ON; 
        }

        wv->t_mode = wv->traj->t_samp;
        return 0; 
    }

    wv->error = WV_ERR_UNDEF_MODE; 
    wv->mode = WV_MODE_NONE; 
    return -1; 
}

void Wv_Stop(WEAVE* wv, int f_fast)
{
    wv->f_stop = 1; 
    Traj_Stop(wv->traj, f_fast); 
}

// Error Code String is returned. 
char* Wv_ErrorStr(char err)
{
    switch(err)
    {        
    case WV_ERR_NONE         :
        return "None"; 
        
    case WV_ERR_NULL_PTR    :
        return "Null Ptr"; 
    case WV_ERR_INV_PARAM:
        return "Invalid Param"; 
    case WV_ERR_NO_WX_DIR:
        return "No Wx Dir";
#if 0 // error 
    case WV_ERR_TRAJ:
        return "Traj Setting Fail"; 
#endif
    case WV_ERR_SHORT_DIST:
        return "Too Short Dist(0.25 Pitch)"; 
    case WV_ERR_UNDEF_MODE:
        return "Undefined Weaving Mode";
    case WV_INV_BASE_PLANE_OR_WV_DIR:
        return "Invalid Wx or Wy"; 
    case WV_ERR_LONG_START_END_DIST:
        return "Too Long Start/End Dist";
    case WV_ERR_LONG_ANGLE_DIST:
        return "Too Long Angle Dist"; 
    default:
        return "Undef"; 
    }
}

// returns weaving start distance
double Wv_StartDist(WEAVE* wv)
{
    if(!wv)
    {
        return 0; 
    }
    if(0 < wv->n_start)
    {
        return wv->n_start * 0.5*wv->pitch_s - 0.25*wv->pitch_s; 
    }
    else
    {
        return 0; 
    }
}

// returns distance of 'pos' from weave ref. pos 'wv->pw' to the weaving dir. 
// pos : intereting position wrt base 
double Wv_Dist(const WEAVE* wv, const POS* pos)
{
    POS p;     
    if(wv && pos)
    {
        p = POS_Minus_POS(*pos, wv->pw); 
        return POS_GetDist(p, wv->xw);     
    }
    else
    {
        return 0;
    }
}

// Outputs Final Pos & Rot. 
// Final Pos is not const but can be changed from Init as Sensor acts. 
void Wv_FinalTrans(const WEAVE* wv, POS* p_f, ROT* R_f)
{
    if(wv == 0)
    {
        return; 
    }    
    if(p_f)
    {
        *p_f = POS_Plus_POS(wv->pw, POS_Multi_S(wv->xw, wv->d_total));     
    }
    if(R_f)
    {
        // R_target = Rscrew * R_start.
        *R_f = ROT_Multi_ROT(ROT_Screw(wv->sr, wv->r_total), wv->Rs);   
    }
}

char* Wv_Print(const WEAVE* wv)
{   
    static char str[80*64]; 
    int n; 

    str[0] = 0; 
    n = 0; 
    n += CRT_sprintf(str+n, sizeof(str)-n, "Weaving Info's. Unit:[rad], [rad/s], [rad/s^2]\n");
    n += CRT_sprintf(str+n, sizeof(str)-n, "  ps:%.2f %.2f %.2f, xw:%.2f %.2f %.2f, yw:%.2f %.2f %.2f\n", 
        wv->pw.x, wv->pw.y, wv->pw.z, wv->xw.x, wv->xw.y, wv->xw.z, wv->yw.x, wv->yw.y, wv->yw.z); 
    n += CRT_sprintf(str+n, sizeof(str)-n, "  pit  :%.2f wid  :%.2f vel  :%.2f pit_s:%.2f wid_s:%.2f vel_s:%.2f\n", 
        wv->pitch, wv->width, wv->v_pos, wv->pitch_s, wv->width_s, wv->v_pos_s);     
    n += CRT_sprintf(str+n, sizeof(str)-n, "  dwl_e:%.2f dwl_o:%.2f dwl_c:%.2f d_tot:%.2f d_end:%.2f d_ang:%.2f\n", 
        wv->dwl_even, wv->dwl_odd, wv->dwl_cent, wv->d_total, wv->d_final, wv->d_angle);     
    n += CRT_sprintf(str+n, sizeof(str)-n, "  r_tot:%.2f n_wd :%03d  error:%d  \n", wv->r_total, wv->n_start, wv->error); 
    return str;
}

////////////////////////////////////////////////////////////////////////////////

// Gets Remained Working Distance to 'xw' Dir When n-th motion is ended. 
static double RemainDist(const WEAVE* wv) 
{   
    ASSERT_RETURN(wv, 0.0); 
    return wv->d_total - wv->d_trg; 
}

// Plans Weaving Motion as Current Mode is EVEN_DWL or ODD_DWL. 
// At Start & Final this must not be called
// - wv : WEAVE
// - th_a : Actual Joint Value
// - i : Motion Index. (0)Start, (-1)Final, (Else)Normal
// 
static int MotionPlan(WEAVE* wv, const double* th_a, int i)
{   
    POS    s_p;                     // progressive unit vector
    double pos, rot;                // progressive amplitude.    
    double dir;   
    double d_rem; 
    double d_r; 
    double x, y, z;
    double pitch; 
    double width; 
    double vel; 
    int    mode; 
    int    ret; 

    ASSERT_RETURN(wv, -1);  
    ASSERT_RETURN(th_a, -1);     
        
    // start condition setting

    pitch = (wv->n_seg == WV_SEG_START)? wv->pitch_s : wv->pitch; 
    width = (wv->n_seg == WV_SEG_START)? wv->width_s : wv->width; 
    vel   = (wv->n_seg == WV_SEG_START)? wv->v_pos_s : wv->v_pos; 

    // 
    d_rem = RemainDist(wv); 
    
    ///// Normal Weaving (No Center Dwell) /////
    if(wv->dwl_cent <= 0)
    {
        // start motion
        if(i==0) 
        {   
            x = pitch * 0.25; 
            y = width * 0.5; 
            mode = WV_MODE_MOT; 
            
            // If Final Motion, Goes to the End Pos
            if(d_rem <= x)
            {
                x = d_rem; 
                y = 0.; 
                mode = WV_MODE_FIN_MOT; 
            }
        }
        // just start->main weaving mode
        else if(wv->i_mot == wv->n_start)
        {
            x = (wv->pitch + wv->pitch_s) * 0.25; 
            y = (wv->width + wv->width_s) * 0.50; 
            mode = WV_MODE_MOT; 

            if(d_rem <= x)
            {        
                x = d_rem; 
                y = wv->width_s; 
                mode = WV_MODE_FIN_MOT; 
            }
        }
        // middle motion
        else
        {
            x = pitch * 0.5; 
            y = width; 
            mode = WV_MODE_MOT; 

            // final motion 
            if(d_rem <= x)
            {        
                x = d_rem; 
                y = width * 0.5; 
                mode = WV_MODE_FIN_MOT;                 
            }
        }
    }
    ///// Center Dwell Weaving /////
    else
    {
        // start motion or From Center Motion(f_cent: To Center Motion)
        if(i==0)
        {   
            x = pitch * 0.25; 
            y = width * 0.5; 
            mode = WV_MODE_MOT; 
            
            // If Final Motion, Goes to the End Pos
            if(d_rem <= x)
            {
                x = d_rem; 
                y = 0.; 
                mode = WV_MODE_FIN_MOT; 
            }
        }
        // Transient & To-Center Node
        else if(wv->i_mot == wv->n_start && wv->f_cent)
        {
            x = 0.25*wv->pitch_s; 
            y = 0.50*wv->width_s; 
            mode = WV_MODE_MOT; 

            if(d_rem <= x)
            {
                x = d_rem; 
                y = 0.50*wv->width_s;
                mode = WV_MODE_FIN_MOT; 
            }
        }        
        // middle (to-center) motion
        else
        {
            x = pitch * 0.25; 
            y = width * 0.5; 
            mode = WV_MODE_MOT; 

            // final motion planning 
            if(d_rem <= x)
            {   
                x = d_rem; 
                y = (wv->f_cent)? width * 0.5 : 0.0; 
                mode = WV_MODE_FIN_MOT;                 
            }
        }
    }

    dir = (i % 2 == 0)? 1. : -1.; // direction
    y *= dir; 
    z = 0;    
    
    // Arc Sensor Modification
    // !! CAUTION !!
    // Motion Index is Index to Move. Sensor Index is Index Sensed. 
    // -> Sensor Index = Motion Index - 1
    // No Arc Sensing for Center Dwell Weaving. 
    // No proper seam tracking with Center Dwell. 
    if(weld_idx.arc_sensor && 1 <= i && wv->dwl_cent <= 0)
    {
        double dt = 0., dz = 0.; 
        weave_data_t weav;   
        POS p_d; 

        weav.node = i-1; 
        weav.speed = 0.; // speed is not used. 
        weav.wd_node = wv->n_start; 
        weav.w_flag = (wv->n_seg == WV_SEG_MAIN)? 1 : 0; 
                
        // correction value
        get_correct_value(&dt, &dz, &weav);
        y += dz; // dz : yw directional compensation value
        z -= dt; // dt : current compensation value. current is negative to the zw(upper). 

        // Weaving Ref Pos Correction
        p_d = POS_Plus_POS(POS_Multi_S(wv->yw, dz), POS_Multi_S(wv->zw, -dt)); 
        wv->pw = POS_Plus_POS(wv->pw, p_d);
    }
    s_p = POS_Plus_POS(POS_Multi_S(wv->xw, x), POS_Multi_S(wv->yw, y));
    s_p = POS_Plus_POS(POS_Multi_S(wv->zw, z), s_p);

    pos = POS_Norm(s_p); 
    POS_Unit(&s_p); 

    // Orientation Params. This must be before 'wv->d_trg += x'. 
    // 'd_r' rot existing dist at this motion. 
    // if the target dist exceeds the d_angle, d_r = d_angle - d_prev. 
    // d_r / d_angle = rot / r_total,    
    if(wv->d_angle <= wv->d_trg) // rotation over 
    {        
        d_r = 0.; 
    }
    else if(wv->d_angle <= wv->d_trg + x)   // last rotation
    {
        d_r = wv->d_angle - wv->d_trg; 
    }
    else 
    {
        d_r = x; 
    }    
    rot = wv->r_total * d_r / wv->d_angle;     

    // Saving Target of Main Weaving 
    if(i - wv->n_start >= 0)
    {
        shm_servo->d_xw[i-wv->n_start] = wv->pw.x; 
        shm_servo->d_yw[i-wv->n_start] = wv->pw.y;  
        shm_servo->d_zw[i-wv->n_start] = wv->pw.z; 
    }

    // Traj Setting 
    wv->d_trg += x; 
    wv->mode = mode;
    ret = Traj_Set_Lmot(wv->traj, th_a, &s_p, pos,  wv->v_pos, 
                        &wv->sr, rot, 0., wv->config); 
#if 0 // error 
    return (ret)? WV_ERR_TRAJ : 0; 
#else
    return (ret)? TrajErrAssign(ret) : 0; 
#endif
}

// i_mot : dwell of i_th motion. 
static int DwellPlan(WEAVE* wv, int i_mot)
{      
    // Present Center Motion. 
    // This is set at starting of center move. 
    // & Reset Center Dwell End
    if(wv->f_cent)
    {
        wv->t_dwl = wv->dwl_cent;             
        return 0; 
    }
    // Odd Dwell Plan     
    else if(i_mot % 2)
    {
        wv->t_dwl = wv->dwl_odd;             
        return 0; 
    }    
    // Even Dwell Plan
    else
    {        
        wv->t_dwl = wv->dwl_odd;             
        return 0; 
    }
}
