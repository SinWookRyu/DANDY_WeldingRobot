// CMD_CWEAV.C is the program for CWEAV cmds. 
// CWEAV consist of 3 weaving motion. 
// 2014.02.05 mrch0
#include "taskexec_def.h"
#include "coord.h"
#include "arg.h"
#include "weave.h"
#include "cmd.h"

#if 0
// i_mot_prev : motion index of prev. weaving
// y_dir : 1./-1. Dir of Horz Weave of Cweave
// ang_wv : Weaving Plane Angle which is from WVF. 
static void CweaveDirGet(POS* yw, DBL* ang_yw, POS xw, DBL y_dir, 
                         int i_mot_prev, DBL ang_wv); 
#else
// Outputs 'yw' with prev. weaving. If 1st, this is not used.
// - p_final : final weaving pos
// - p_start : start weaving pos
// - xw_prev : xw of prev. weaving
// - yw_prev : yw of prev. weaving 
// - i_mot_prev : motion index of prev. weaving
static void CweaveDirGet(POS* yw, const POS* p_final, const POS* p_start, 
                        const POS* xw_prev, const POS* yw_prev, int i_mot_prev); 
#endif

static int s_stop_cweave; 
static DBL s_time_cweave; 

#if 0
static TRANS s_Tf, s_Tv1, s_Tv2;    // Cweave Used Points. Modified from Tf_in & Tv_in.    
static UNT   s_conf;                // robot pose configuration
static DBL   s_y_dir;               // Cweave Dir of Y. To what dir the horz weaving will go. 
static DANDY_JOB_WEAV s_wvf; 

#define CWEAVE_MODE_NONE     0      // 
#define CWEAVE_MODE_VERT     1      // Vertical Weaving (start)
#define CWEAVE_MODE_DWELL_V  2      // Dwell After Vert. Weaving. Just waits btw Vert Weav & Round Weav
#define CWEAVE_MODE_ROUND    3      // Rount Weaving
#define CWEAVE_MODE_DWELL_H  4      // Dwell Before Horz. Weaving.Just waits btw Rount Weav & Horz Weav
#define CWEAVE_MODE_HORZ     5      // Horizontal Weaving (final)
#endif 

UNT Cweave_IsWeavingMode(int mode)
{
    return (mode == CWEAVE_MODE_VERT  || 
            mode == CWEAVE_MODE_ROUND || 
            mode == CWEAVE_MODE_HORZ  )? ON : OFF; 
}

void Cweave_Reset_Stop(void)
{
    s_stop_cweave = 0; 
}

void Cweave_Reset_Time(void)
{
    s_time_cweave = 0; 
}

int CmdSet_CWEAV(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_WEAVE* arg;         
    DBL  d_vert;   
    DBL  d_horz;     
    int  ret; 
    int  crd; 
    TRANS Ts, Tf_in, Tv_in;     // Input by Cmd Arg. These are made by Touching.            
    POS  yw; 
    DBL  ang_yw; 
    
    arg = &g_rcmd[index].arg.argWeave; 

    // weaving file
    ret = WvfFromArg(&cwv_wvf, &arg->valWvf, g_n_shift); 
    if(ret)
    {
        return ret; 
    }    

    // Get Ref Coord 
    ret = Arg_Scalar(&crd, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }
    // vertical distance
    ret = DistFromArg(&d_vert, &arg->valDistVert); 
    if(ret)
    {
        return ret; 
    }
    d_vert = fabs(d_vert); 
    d_vert = (d_vert < g_cweave_weldleg_dist)? g_cweave_weldleg_dist : d_vert;  // limitation

    // horizontal distance
    ret = DistFromArg(&d_horz, &arg->valDistHorz);  
    if(ret)
    {
        return ret; 
    }
    d_horz = fabs(d_horz); 
    d_horz = (d_horz < g_cweave_weldleg_dist)? g_cweave_weldleg_dist : d_horz;  // limitation

#if 0
    // depth distance
    ret = DistFromArg(&d_depth, &arg->valDepth);  
    if(ret)
    {
        return ret; 
    }
#endif
    // Target wrt BASE->TCP 
    ret = TransFromArg(&Tf_in, NULL, &arg->valTargetPos, crd); 
    if(ret)
    {
        return ret; 
    }
    // Via wrt BASE->TCP 
    // Almost Motion Params including 'conf' for Cweave are from VIA. 
    ret = TransFromArg(&Tv_in, &cwv_conf, &arg->valViawayPos, crd); 
    if(ret)
    {
        return ret; 
    }
 
    // Ts
    g_traj.Forward(&Ts, jnt_start, g_traj.dh); 
    Ts = TRANS_Multi_TRANS(Ts, g_traj.eTt); 
   
    // CWEAVE Y-Dir 
    cwv_y_dir = (Ts.p.y <= Tf_in.p.y)? 1.:-1.; 

    // Modifing Tf, Tv1, Tv2 from Tf_in(Final), Tv_in(Via) (wrt BASE)
    // Collar Weaving Seq. : Ts -> Tv1 -> Tv2 -> Tf (These are on the WV Plane)
    // Tf_in & Tv_in are made by Touching. 
    // Just Tf_in.y is used for Tf. All except Tf.y are from Tv_in. 

    // Tv1 [Start.x,                     Start.y,             Via.z-VERT      ]
    // Tv2 [Via.x + TOUCH_UP - WELD_LEG, Tv1.y +- (3 + HORZ), Tv1.z + 3 + VERT]
    // Tf  [Tv2.x,                       Final.y +- 12,       Tv2.z           ]
    // 3:Weld-leg Dist, 12:Horizontal Margin

    cwv_Tv1.R   = Ts.R; 
    cwv_Tv1.p.x = Ts.p.x; 
    cwv_Tv1.p.y = Ts.p.y;
    cwv_Tv1.p.z = Tv_in.p.z - d_vert;
    // conf_v1 = conf_s; 

    cwv_Tv2.R   = Tv_in.R; 
    cwv_Tv2.p.x = Tv_in.p.x + g_cweave_touch_dist - g_cweave_weldleg_dist; 
    cwv_Tv2.p.y = cwv_Tv1.p.y + cwv_y_dir * (g_cweave_weldleg_dist + d_horz); 
    cwv_Tv2.p.z = cwv_Tv1.p.z + d_vert + g_cweave_weldleg_dist;
    // conf_v2 = conf_v_in; 

    cwv_Tf.R   = cwv_Tv2.R; 
    cwv_Tf.p.x = cwv_Tv2.p.x; 
    cwv_Tf.p.y = Tf_in.p.y - cwv_y_dir * g_cweave_horz_margin; 
    cwv_Tf.p.z = cwv_Tv2.p.z; 

    // Checks Existing of Final Weaving Distance     
    // By matching Cweaving Dir & y-dir of Final Pos & Via2 Pos. 
    if( (cwv_y_dir ==  1 &&  cwv_Tf.p.y <= cwv_Tv2.p.y) || 
        (cwv_y_dir == -1 &&  cwv_Tf.p.y >= cwv_Tv2.p.y) )
    {
        return ERR_CWEAV_SHORT_HORZ;
    }    

    // Dwell Btw Weaving Motion    
    cwv_t_rest = 0.5 * (cwv_wvf.dbDwell1 + cwv_wvf.dbDwell2); 

    // initial weaving dir    
    yw.x = 0.; yw.y = cwv_y_dir; yw.z = 0.; 
    ang_yw = cwv_y_dir * cwv_wvf.dbAngle; 

    // Start to Tv1
    ret = Wv_Set(&g_weave,          &g_traj,            // 0,  // cweave no arc-sensor
                 jnt_start,         &cwv_Tv1, cwv_conf, &yw, ang_yw, 
                 cwv_wvf.dbPitch,   cwv_wvf.dbWidth,    cwv_wvf.dbSpeed, 
                 0.,                0.,                 0., 
                 cwv_wvf.dbDwell1,    cwv_wvf.dbDwell2, cwv_wvf.dbDwell3, 
                 0.,                0.,                 0.); // no start, angle, end distances 

    if(ret)
    {
        return ERRCODE_WEAVE(ret); 
    }

    R_VERB_VRB("CWEAV Starts Vertical Weaving.\n");
    cwv_mode = CWEAVE_MODE_VERT; 
    s_stop_cweave = 0; 
    return 0; 
}
int CmdUpd_CWEAV(void)
{   
    // None Running Case 
    if(cwv_mode == CWEAVE_MODE_NONE)
    {
        return -1; 
    }

    if(cwv_mode == CWEAVE_MODE_VERT)
    {   
        // Weaving Module Update    
        if(!Wv_Update(&g_weave, g_pos_trg, &g_bTt))
        {
            return 0; 
        }

        // Weaving End /////////////////////////////////////////////////////////

        // Condition of End of Cweave
        if(g_weave.error || s_stop_cweave)
        {
            cwv_mode = CWEAVE_MODE_NONE; 
            return -1; 
        }

        // New Mode Setting //
        R_VERB_VRB("CWEAV Starts waiting after Vert. Weaving.\n");
        s_time_cweave = 0.; 
        cwv_mode = CWEAVE_MODE_DWELL_V; 
        return 0;     
    }

    if(cwv_mode == CWEAVE_MODE_DWELL_V)
    {
        // Cweave Tick Update
        s_time_cweave += g_weave.traj->t_samp;         

        // Condition of End of Cweave
        if(s_stop_cweave)
        {
            cwv_mode = CWEAVE_MODE_NONE; 
            return -1; 
        }

        // New Mode Setting //
        if(cwv_t_rest < s_time_cweave)
        {
            POS yw; 
            int ret; 

            CweaveDirGet(&yw, &cwv_Tv2.p, &cwv_Tv1.p, 
                         &g_weave.xw, &g_weave.yw, g_weave.i_mot); 

            ret = Wv_Set(&g_weave,          &g_traj,        // 0,  // cweave no arc-sensor
                         g_pos_trg,         &cwv_Tv2, cwv_conf, &yw, 0., 
                         cwv_wvf.dbPitch,   cwv_wvf.dbWidth,    cwv_wvf.dbSpeed, 
                         0.,                0.,                 0., 
                         cwv_wvf.dbDwell1,  cwv_wvf.dbDwell2,   cwv_wvf.dbDwell3, 
                         0., 0., 0.); // no start & end distances But angle distance        
            if(ret) 
            {   
                ERROR_SET(SECT_CMD_CWEAV, ERRCODE_WEAVE(ret)); 
                cwv_mode = CWEAVE_MODE_NONE; 
                return -1; 
            }   

            R_VERB_VRB("CWEAV Starts Round Weaving.\n");
            cwv_mode = CWEAVE_MODE_ROUND; 
        }
        return 0;         
    }
    
    if(cwv_mode == CWEAVE_MODE_ROUND)
    {   
        // Weaving Module Update    
        if(!Wv_Update(&g_weave, g_pos_trg, &g_bTt))
        {
            return 0; 
        }

        // Weaving End /////////////////////////////////////////////////////////

        // Condition of End of Cweave
        if(g_weave.error || s_stop_cweave)
        {
            cwv_mode = CWEAVE_MODE_NONE; 
            return -1; 
        }

        // New Mode Setting //
        R_VERB_VRB("CWEAV Starts waiting after Round Weaving.\n");
        s_time_cweave = 0.; 
        cwv_mode = CWEAVE_MODE_DWELL_H; 
        return 0;     
    }

    if(cwv_mode == CWEAVE_MODE_DWELL_H)
    {
        // Cweave Tick Update
        s_time_cweave += g_weave.traj->t_samp;         

        // Condition of End of Cweave
        if(s_stop_cweave)
        {
            cwv_mode = CWEAVE_MODE_NONE; 
            return -1; 
        }

        // New Mode Setting //
        if(cwv_t_rest < s_time_cweave)
        {
            POS yw;
            int ret; 
   
            CweaveDirGet(&yw, &cwv_Tf.p, &cwv_Tv2.p, 
                         &g_weave.xw, &g_weave.yw, g_weave.i_mot); 

            ret = Wv_Set(&g_weave,          &g_traj,            // 0,  // cweave no arc-sensor
                         g_pos_trg,         &cwv_Tf, cwv_conf,  &yw, 0., 
                         cwv_wvf.dbPitch,   cwv_wvf.dbWidth,    cwv_wvf.dbSpeed, 
                         0.,                0.,                 0., 
                         cwv_wvf.dbDwell1,  cwv_wvf.dbDwell2,   cwv_wvf.dbDwell3, 
                         0.,                0.,                 0.); // no start & end distances But angle distance
            if(ret)
            {      
                ERROR_SET(SECT_CMD_CWEAV, ERRCODE_WEAVE(ret)); 
                cwv_mode = CWEAVE_MODE_NONE; 
                return -1; 
            }
            R_VERB_VRB("CWEAV Starts Horz Weaving.\n");
            cwv_mode = CWEAVE_MODE_HORZ;             
        }
        return 0;         
    }

    if(cwv_mode == CWEAVE_MODE_HORZ)
    {   
        // Weaving Module Update    
        if(!Wv_Update(&g_weave, g_pos_trg, &g_bTt))
        {
            return 0; 
        }

        // Weaving End /////////////////////////////////////////////////////////
        R_VERB_VRB("CWEAV ended.\n");
        cwv_mode = CWEAVE_MODE_NONE; 
        return -1; 
    }

    // undefined mode
    cwv_mode = CWEAVE_MODE_NONE; 
    return -1; 
}

VOD CmdStp_CWEAV(int f_fast)
{
    Wv_Stop(&g_weave, f_fast); 
    s_stop_cweave = 1; 
}

#if 0
// i_mot_prev : motion index of prev. weaving
// y_dir : 1./-1. Dir of Horz Weave of Cweave
// ang_wv : Weaving Plane Angle which is from WVF. 
static void CweaveDirGet(POS* yw, DBL* ang_yw, POS xw, DBL y_dir, 
                         int i_mot_prev, DBL ang_wv)  
{
    POS s; // rotation axis    
    POS xw_i, yw_i; // start weaving direction
    DBL ang_xw; 

    // initial weaving dir
    xw_i.x = 0.; xw_i.y = 0.;    xw_i.z = 1.; 
    yw_i.x = 0.; yw_i.y = y_dir; yw_i.z = 0.; 

    if(yw)
    {
        // ang_xw is angle btw xw_old to xw
        // xw is rotated ang_xw about 's' axis wrt BASE by xw_i. 
        // yw is rotated ang_xw about 's' axis wrt BASE by yw_i also. 
        ang_xw = POS_GetAngle(&s, xw_i, xw);// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
        *yw = ROT_Multi_POS(ROT_Screw(s, ang_xw), yw_i);     

        // Dir of Start Motion of Next Weaving
        // It remains the same dir as the prev. direction of motion.    
        if(i_mot_prev % 2)
        {
            // odd direction
            *yw = POS_Multi_S(*yw, -1.);
        }
    }
    if(ang_yw)
    {
        // angle of yw is rotation angle of yw about xw. 
        *ang_yw = y_dir * ang_wv;     
    }        
}
#endif

#if 0
// Outputs 'yw' with prev. weaving. If 1st, this is not used.
// - p_final : final weaving pos
// - p_start : start weaving pos
// - xw_prev : xw of prev. weaving
// - yw_prev : yw of prev. weaving 
// - i_mot_prev : motion index of prev. weaving
static void CweaveDirGet(POS* yw, const POS* p_final, const POS* p_start, 
                         const POS* xw_prev, const POS* yw_prev, int i_mot_prev)
{   
    POS s; // rotation axis    
    DBL ang_xw; 

    if(yw)
    {
        // ang_xw is angle btw xw_old to xw
        // xw is rotated ang_xw about 's' axis wrt BASE by xw_i. 
        // yw is rotated ang_xw about 's' axis wrt BASE by yw_i also.         
        ang_xw = POS_GetAngle(&s, *xw_prev, POS_Minus_POS(*p_final, *p_start));// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
        *yw = ROT_Multi_POS(ROT_Screw(s, ang_xw), *yw_prev);           
        
        if(i_mot_prev % 2)
        {
            // odd direction
            *yw = POS_Multi_S(*yw, -1.);
        }
    }           
}
#else
// Outputs 'yw' with prev. weaving. If 1st, this is not used.
// - p_final : final weaving pos
// - p_start : start weaving pos
// - xw_prev : xw of prev. weaving
// - yw_prev : yw of prev. weaving 
// - i_mot_prev : motion index of prev. weaving
static void CweaveDirGet(POS* yw, const POS* p_final, const POS* p_start, 
                         const POS* xw_prev, const POS* yw_prev, int i_mot_prev)
{   
    POS s; // rotation axis    
    DBL ang_xw; 
    POS xw_old, xw_new; 
    
    // Cweave yw doesn't regards x dir. All yw is oriented about x-axis(yz plane)
    xw_old = *xw_prev; 
    xw_old.x = 0; 
    xw_new = POS_Minus_POS(*p_final, *p_start); 
    xw_new.x = 0; 

    if(yw)
    {
        // ang_xw is angle btw xw_old to xw
        // xw is rotated ang_xw about 's' axis wrt BASE by xw_i. 
        // yw is rotated ang_xw about 's' axis wrt BASE by yw_i also.         
        ang_xw = POS_GetAngle(&s, xw_old, xw_new);// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
        *yw = ROT_Multi_POS(ROT_Screw(s, ang_xw), *yw_prev);           
        
        if(i_mot_prev % 2)
        {
            // odd direction
            *yw = POS_Multi_S(*yw, -1.);
        }
    }           
}
#endif 