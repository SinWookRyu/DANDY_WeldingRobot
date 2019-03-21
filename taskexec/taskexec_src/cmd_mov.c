// CMD_MOV.C is the program for movable job cmds. 
// 2013.10.25 mrch0
#include "taskexec_def.h"
#include "coord.h"
#include "arg.h"
#include "weave.h"

void calc_weav_rot(const double rgdbPosStart[3],    // input for start point
                const double rgdbPosTarget[3],   // input for target point
                double dbAngle,                  // input for weaving angle
                double rgdbPlane[3], // output for yw plane vector
                double* pdbRot);      // output for yw plane rotation

static double WeldSpdCheck(double v_cmd); 

////////////////////////////////////////////////////////////////////////////////

int CmdSet_MOVI(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_IMOVE* arg; 
    TRANS bTc;   // Start & Final TCP wrt BASE
    POS s_p, s_r; 
    DBL pos; 
    DBL v_p; 
    int coord; 
    int ret; 

#if 0 // PRINT
    printf("Imov Start : %f %f %f %f %f %f \n", 
        jnt_start[0], jnt_start[1], jnt_start[2], 
        jnt_start[3], jnt_start[4], jnt_start[5]);
#endif 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argIMove; 
    
    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    // Pos Dir    
    s_p.x = 0; 
    s_p.y = 0; 
    s_p.z = 0;    

    ret |= Arg_Scalar(NULL, &s_p.x, &arg->valIncX); 
    ret |= Arg_Scalar(NULL, &s_p.y, &arg->valIncY); 
    ret |= Arg_Scalar(NULL, &s_p.z, &arg->valIncZ); 

    pos = POS_Norm(s_p);
    s_p = POS_SafeUnit(&s_p);

    // POS wrt BASE
    bTc = TargetCoordWrtBase(coord);  // base_T_coord
    s_p = ROT_Multi_POS(bTc.R, s_p); 

    // vel_pos
    ret = PosVelFromArg(&v_p, &arg->valSpeed); 
    if(ret)
    {
        return ret; 
    }

    // Plans Motion with speed of Welding Mode. 
    v_p = WeldSpdCheck(v_p); 

    // Temporary Rotation 
    s_r.x = 0; 
    s_r.y = 0; 
    s_r.z = 1.; 
    
#if 0 // PRINT
    printf("Imov Start Just Before Traj Set : %f %f %f %f %f %f \n", 
        jnt_start[0], jnt_start[1], jnt_start[2], 
        jnt_start[3], jnt_start[4], jnt_start[5]);
    printf("Imov Start Just Before Traj Set : %f %f %f %f %f %f \n", 
        jnt_start[0], jnt_start[1], jnt_start[2], 
        jnt_start[3], jnt_start[4], jnt_start[5]);
#endif 

#if 0
    ret = Traj_Set_Lmot(&g_traj, jnt_start, 
                         &s_p, pos, v_p, 
                         &s_r, 0, 0, 
                         g_traj.Config(g_pos_act, g_traj.dh)); 
#else
    ret = Traj_Set_Lmot(&g_traj, jnt_start, 
                         &s_p, pos, v_p, 
                         &s_r, 0, 0, 
                         g_traj.Config(jnt_start, g_traj.dh)); 
#endif

    return ERRCODE_TRAJ(ret); 
}

int CmdSet_MOVC(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_MOVE* arg; 
    TRANS Tv, Tf;   
    UNT conf_v, conf_f; 
    DBL v_p; 
    int coord; 
    int ret; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argMove; 

    // bTt_s
    // Ts = TRANS_Multi_TRANS(g_bTe, g_eTt); 

    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    // Via wrt BASE->TCP 
    ret = TransFromArg(&Tv, &conf_v, &arg->valViawayPos, coord); 
    if(ret)
    {
        return ret; 
    }

    // Target wrt BASE->TCP 
    ret = TransFromArg(&Tf, &conf_f, &arg->valTargetPos, coord); 
    if(ret)
    {
        return ret; 
    }

    // velocity with welding condition
    PosVelFromArg(&v_p, &arg->valSpeed); 
    v_p = WeldSpdCheck(v_p); 
        
    // Set Traj
    ret = Traj_Set_Cmot_3Pt(&g_traj, jnt_start, &Tv.p, &Tf, conf_v, conf_f, v_p, 0);     
    return ERRCODE_TRAJ(ret); 
}

int CmdSet_MOVO(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_MOVE* arg; 

    TRANS bTt; 
    TRANS Tf;   
    DBL v_r; 
    UNT conf_f; 
    int coord; 
    int ret; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argMove; 
        
    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    // Target wrt BASE->TCP 
    ret = TransFromArg(&Tf, &conf_f, &arg->valTargetPos, coord); 
    if(ret)
    {
        return ret; 
    }
    
#if 0
    // reset pos to no-move pos. 
    Tf.p = g_bTt.p; 
#else
    // Current TCP 
    bTt = TargetCoordWrtBase(COORDINDEX_TCP); 

    // reset pos to no-move pos. 
    Tf.p = bTt.p; 
#endif
    
    // velocity
    RotVelFromArg(&v_r, &arg->valSpeed);     

#if 0 // PRINT 
    printf("[BB] %f %f %f\n", jnt_start[0], jnt_start[1], jnt_start[2]); 
#endif 

    // Set Traj    
    ret = Traj_Lmot_Set2(&g_traj, jnt_start, &Tf, conf_f, 0, v_r);    
    return ERRCODE_TRAJ(ret); 
}

int CmdSet_MOVL(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_MOVE* arg; 
    DANDY_JOB_POS rpos; 

    TRANS Tf;   // Start & Final TCP wrt BASE        
    DBL v_p; 
    UNT conf_f; 
    int coord; 
    int ret; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argMove; 
    
    // Debug
    Arg_RobPos(&rpos, &arg->valTargetPos); 
    R_VERB_VRB("MOVL Trg : %f %f %f %f %f %f, Conf:%x \n", 
        rpos.pos[0], rpos.pos[1], rpos.pos[2], 
        rpos.pos[3], rpos.pos[4], rpos.pos[5], rpos.nConfig); 
    
    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    // Target wrt BASE->TCP 
    ret = TransFromArg(&Tf, &conf_f, &arg->valTargetPos, coord); 
    if(ret)
    {
        return ret; 
    }

    // vel_pos
    ret = PosVelFromArg(&v_p, &arg->valSpeed); 
    if(ret)
    {
        return ret; 
    }    
    v_p = WeldSpdCheck(v_p); 
    
#if 1 // PRINT 
    R_VERB_VRB("MOVL Trg2 : %f %f %f, pose_conf:%d, coord:%s \n",
        Tf.p.x, Tf.p.y, Tf.p.z, conf_f, Str_Coord(coord)); 
    
#endif 
    ret = Traj_Lmot_Set2(&g_traj, jnt_start, &Tf, conf_f, v_p, 0);

    return ERRCODE_TRAJ(ret); 
}

int CmdSet_MOVJ(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_MOVE* arg; 
        
    DBL trg[6]; 
    DBL vel; 
    int coord; 
    int ret; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argMove; 

    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    ret = JointFromArg(trg, &arg->valTargetPos, coord); 
    if(ret)
    {
        return ret; 
    }

    // vel    
    ret = JointVelFromArg(&vel, &arg->valSpeed); 
    if(ret)
    {
        return ret; 
    }   

    ret = Traj_Set_Jmot2(&g_traj, jnt_start, trg, vel); 
    return ERRCODE_TRAJ(ret); 

}

int CmdSet_HOME(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_HOME* arg;     
    DBL  vel;     
    int  ret; 
    int  i_home; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argHome; 
        
    // Get Ref Coord 
    ret = Arg_Scalar(&i_home, NULL, &arg->valHomeFile); 
    if(ret)
    {
        return ret; 
    }

    // vel    
    ret = JointVelFromArg(&vel, &arg->valSpeed); 
    if(ret)
    {
        return ret; 
    }   

    ret = Traj_Set_Jmot2(&g_traj, jnt_start, g_home[i_home].joint, vel); 
    return ERRCODE_TRAJ(ret); 
}

int CmdSet_WEAV(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_WEAVE* arg;     
    DANDY_JOB_WEAV wvf; 
    DBL  d_ang;   
    DBL  d_start; 
    DBL  d_final; 
    int  ret; 
    int  crd; 
    UNT  conf_f; 
    TRANS Ts, Tf; 
    POS  yw;     
    DBL  ang_yw; 

    arg = &g_rcmd[index].arg.argWeave; 

    // Get Ref Coord 
    ret = Arg_Scalar(&crd, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }
    // angle distance
    ret = DistFromArg(&d_ang, &arg->valDistAngle); 
    if(ret)
    {
        return ret; 
    }
    // start distance
    ret = DistFromArg(&d_start, &arg->valDistWeave);  
    if(ret)
    {
        return ret; 
    }
    // final distance
    ret = DistFromArg(&d_final, &arg->valDistEnd);  
    if(ret)
    {
        return ret; 
    }

    // Target wrt BASE->TCP 
    ret = TransFromArg(&Tf, &conf_f, &arg->valTargetPos, crd); 
    if(ret)
    {
        return ret; 
    }

    ret = WvfFromArg(&wvf, &arg->valWvf, g_n_shift); 
    if(ret)
    {
        return ret; 
    }
     
#if 0
    // additional parameters
    // yw wrt BASE 
    yw.x = 0; 
    yw.y = 1.0; 
    yw.z = 0;    

    ret = Wv_Set(&g_weave, &g_traj, jnt_start, &Tf, conf_f, &yw, 
                 wvf.dbAngle, wvf.dbPitch, wvf.dbWidth, wvf.dbSpeed, 
                 wvf.dbDwell1, wvf.dbDwell2, wvf.dbDwell3, d_ang, d_start, d_final); 
#endif

    // Ts
    g_traj.Forward(&Ts, jnt_start, g_traj.dh); 
    Ts = TRANS_Multi_TRANS(Ts, g_traj.eTt); 
    
    // yw & angle of yw
    calc_weav_rot(&Ts.p.x, &Tf.p.x, wvf.dbAngle, &yw.x, &ang_yw); 
        
    ret = Wv_Set(&g_weave,         &g_traj,          // weld_idx.arc_sensor, 
                 jnt_start,        &Tf, conf_f,      &yw, ang_yw, 
                 wvf.dbPitch,      wvf.dbWidth,      wvf.dbSpeed, 
                 wvf.dbStartPitch, wvf.dbStartWidth, wvf.dbStartSpeed, 
                 wvf.dbDwell1,     wvf.dbDwell2,     wvf.dbDwell3, 
                 d_ang, d_start,   d_final); 

    return ERRCODE_WEAVE(ret); 
}

int CmdUpd_Mov(void)
{
    // return Traj_Update(&g_traj); 
    int ret; 
    ret = Traj_Update(&g_traj); 
    ERROR_SET(SECT_CMD_MOV, ERRCODE_TRAJ(g_traj.error)); 
    return ret; 
}

int CmdUpd_WEAV(void)
{
    return Wv_Update(&g_weave, g_pos_trg, &g_bTt); 
}

void CmdStp_Mov(int f_estop)
{    
    (f_estop)? Traj_Stop(&g_traj, 1) : Traj_Stop(&g_traj, 0); 
}

VOD CmdStp_WEAV(int f_fast)
{
    Wv_Stop(&g_weave, f_fast); 
}

////////////////////////////////////////////////////////////////////////////////

// Returns Welding Speed if it is welding currently or not 'v_cmd'. 
// - v_cmd : speed from CMD Argument. 
double WeldSpdCheck(double v_cmd)
{
    if(g_f_weld)
    {
        R_VERB_VRB("With Weld Speed %.1f[mm/s]\n", g_weld_spd*1000.); 
        return g_weld_spd; 
    }
    else
    {
        R_VERB_VRB("With Cmd Speed %.1f[mm/s]\n", v_cmd*1000.); 
        return v_cmd; 
    }
}

    
