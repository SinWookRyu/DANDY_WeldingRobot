// ARG_UNIT.C is the unit modified reading the value of ARG_VALUE. 
// Units of ARG_VALUE are [deg, mm, s,..], After ARG_UNIT.C [rad, mm, ms,..].  
// 
#include "taskexec_def.h"
#include "arg.h"
#include "coord.h"

// Deg -> Rad
void Deg2Rad(double* value, int n)
{
    int i; 
    for(i=0 ; i<n ; i++)
    {
        value[i] = value[i] * PI / 180.;         
    }
}

void Rad2Deg(double* value, int n)
{
    int i; 
    for(i=0 ; i<n ; i++)
    {
        value[i] = value[i] * 180./PI;         
    }
}

////////////////////////////////////////////////////////////////////////////////
// Joint Velocity from ARG_VALUE
// vel = Value of ARG_VALUE * 0.01 (for Joint Vel has type of %)
int JointVelFromArg(double* vel_r, const ARG_VALUE* arg_spd)
{
    int ret; 
    DBL vel; 

    ret = Arg_Scalar(NULL, &vel, arg_spd); 
    if(ret)
    {
        return ret; 
    }     

    if(vel_r)
    {
        *vel_r = vel * 0.01; 
    }
    return 0; 
}

// WVF with [RAD, mm, ms] unit from WVF of ARG_VALUE [DEG, mm, s]. 
// nShift : Shifed Index of Pointer
int WvfFromArg(WEAVE_PAR* wvf, const ARG_VALUE* arg, const unsigned nShift)
{
    int ret;     

    ret = Arg_WeavePar(wvf, arg, nShift); 
    if(ret)
    {
        return ret; 
    }
    
    if(wvf)
    {
        // wvf->dbWidth;			     //  0, weaving width, [mm]
        // wvf->dbPitch;             //  1, weaving pitch, [mm]
        // wvf->dbDepth;             //  2, weaving depth, [mm], unused
        wvf->dbSpeed *= 0.001;       //  3, weaving speed, [mm/s]
        wvf->dbAngle *= PI/180.;     //  4, weaving angle, [deg]
        // wvf->dbSymm;              //  5, start angle, [deg], unused
        wvf->dbDwell1 *= 1000.;      //  6, dwell time 1, [sec]
        wvf->dbDwell2 *= 1000.;      //  7, dwell time 2, [sec]
        wvf->dbDwell3 *= 1000.; 
        // wvf->dbAutoAngle;         //  8, auto angle, 0 or 1, unused
        // wvf->dbStartWidth;        //  9, start width, [mm]
        // wvf->dbStartPitch;        // 10, start pitch, [mm]
        wvf->dbStartSpeed *= 0.001;  // 11, start speed, [mm/s]
    }
    return 0; 
}

// Distance[mm] from ARG_VALUE[mm]
// vel = Value of ARG_VALUE * 1.0
int DistFromArg(double* dist, const ARG_VALUE* arg_dist)
{
    return Arg_Scalar(NULL, dist, arg_dist); 
}


// Angle[rad] from ARG_VALUE[deg]
// ang = Value of ARG_VALUE * PI / 180
int AngFromArg(double* ang, const ARG_VALUE* arg_ang)
{
    int ret; 
    DBL val; 

    ret = Arg_Scalar(NULL, &val, arg_ang); 
    if(ret)
    {
        return ret; 
    }     

    if(ang)
    {
        *ang = val * PI / 180.; 
    }
    return 0;
}

// Position Velocity[mm/ms] from ARG_VALUE[mm/s]
// vel = Value of ARG_VALUE * 0.001 
int PosVelFromArg(double* vel, const ARG_VALUE* arg_spd)
{
    int ret;     

    ret = Arg_Scalar(NULL, vel, arg_spd); 
    if(ret)
    {
        return ret; 
    }     

    if(vel)
    {
        *vel = *vel * 0.001; 
    }
    return 0; 
}

// Time[ms] from ARG_VALUE[s]
// time = Value of ARG_VALUE * 1000.
int TimeFromArg(double* time, const ARG_VALUE* arg_time)
{
    int ret; 

    ret = Arg_Scalar(NULL, time, arg_time); 
    ASSERT_RETURN(!ret, ret); 

    *time *= 1000.;     
    return 0; 
}

// Rotational Velocity[rad/ms] from ARG_VALUE[deg/s]
// vel = Value of ARG_VALUE * 0.001 * PI/180
int RotVelFromArg(double* vel, const ARG_VALUE* arg_spd)
{
    int ret;     

    ret = Arg_Scalar(NULL, vel, arg_spd); 
    if(ret)
    {
        return ret; 
    }     

    if(vel)
    {
        *vel = *vel * 0.001 * PI/180.; 
    }
    return 0; 
}

// ARG_VALUE[mm, deg] -> Joint [rad]
// Config required for CART->JOINT is used by 'Actual Pos'
int JointFromArg(double jnt[6], const ARG_VALUE *arg_val, int coord)
{
    int ret; 
    int conf; 
    TRANS bTc, cTt, bTe; 
    ROBOT_POS rob_pos;     

    // Target Pos from Arg.  
    ret = Arg_RobPos(&rob_pos, arg_val); 
    if(ret)
    {
        return ret; 
    }
   
    // Target is Cartesian type
    if(rob_pos.nConfig == ROBPOS_TYPE_CART)
    {   
        XYZRPY  pose;                                       
        
        // xyzrpy 
        pose.x     = rob_pos.pos[0];   pose.roll  = rob_pos.pos[3] * PI/180.; 
        pose.y     = rob_pos.pos[1];   pose.pitch = rob_pos.pos[4] * PI/180.;
        pose.z     = rob_pos.pos[2];   pose.yaw   = rob_pos.pos[5] * PI/180.;         
                   
        // bTe -> Joint
        bTc = TargetCoordWrtBase(coord);        // base_T_coord
        cTt = TRANS_Xyzrpy(pose);               // coord_T_tcp
        bTe = TRANS_Multi_TRANS(TRANS_Multi_TRANS(bTc, cTt), TRANS_Inv(g_eTt)); 
        conf= g_Config(g_pos_act, g_dh); 
        ret = g_Inverse(jnt, &bTe, conf, g_dh, g_pos_prev, OFF); 
        return ERRCODE_INVERSE(ret); 
        
    }
    // Target is Joint Type
    else
    {
        jnt[0] = rob_pos.pos[0] * PI/180.; jnt[1] = rob_pos.pos[1] * PI/180.; 
        jnt[2] = rob_pos.pos[2] * PI/180.; jnt[3] = rob_pos.pos[3] * PI/180.; 
        jnt[4] = rob_pos.pos[4] * PI/180.; jnt[5] = rob_pos.pos[5] * PI/180.;
        return 0; 
    }    
}

// ARG_VALUE[mm, deg] -> TCP[mm, rad] w/o Ref. Coord
// If Joint Type, wrt BASE->TCP
// - conf : pose config of T. Actual Conf(for CART), Target Conf(for JOINT)
int TransFromArg(TRANS* T, unsigned* conf, const ARG_VALUE *arg_val, int coord)
{
    int ret; 
    ROBOT_POS rob_pos;     
    TRANS bTc, cTt; 

    // Target Pos from Arg.  
    ret = Arg_RobPos(&rob_pos, arg_val); 
    if(ret)
    {
        return ret; 
    }
       
    // Target is Cartesian type
    if(rob_pos.nConfig == ROBPOS_TYPE_CART)
    {       
        XYZRPY  pose;                                       
        
        // xyzrpy 
        pose.x = rob_pos.pos[0];   pose.roll  = rob_pos.pos[3] * PI/180.; 
        pose.y = rob_pos.pos[1];   pose.pitch = rob_pos.pos[4] * PI/180.;
        pose.z = rob_pos.pos[2];   pose.yaw   = rob_pos.pos[5] * PI/180.;         
            
        cTt = TRANS_Xyzrpy(pose);           // coord_T_tcp
        bTc = TargetCoordWrtBase(coord);    // base_T_coord

        // xyzrpy -> TRANS
        if(T)
        {
            *T = TRANS_Multi_TRANS(bTc, cTt);         
        }
        if(conf)
        {
            *conf = g_Config(g_pos_act, g_dh); 
        }
    }
    // Target is Joint Type
    else
    {
        double target[6] = {rob_pos.pos[0] * PI/180., rob_pos.pos[1] * PI/180., 
                            rob_pos.pos[2] * PI/180., rob_pos.pos[3] * PI/180., 
                            rob_pos.pos[4] * PI/180., rob_pos.pos[5] * PI/180.}; 
        
        if(T)
        {
            g_Forward(T, target, g_dh);    
            *T = TRANS_Multi_TRANS(*T, g_eTt); 
        }
        if(conf)
        {
            *conf = g_Config(target, g_dh);         
        }
    }
    return 0; 
}

// nShift : Shifed Index of Pointer
int SwfFromArg(WELD_COND_START* start, const ARG_VALUE *arg, const unsigned nShift)
{
    int ret; 

    ret = Arg_WeldCond_Start(start, arg, nShift); 
    if(ret)
    {
        return ret; 
    }
    
    start->dbArcTime     *= 1000.;
    start->dbPreflowTime *= 1000.; 
    // start->dbVoltage;
    // start->dbArcDist;
    // start->dbCurrent;
    return 0;
}

int MwfFromArg(WELD_COND_MAIN* major, const ARG_VALUE *arg, const unsigned nShift)
{
    int ret; 

    ret = Arg_WeldCond_Main(major, arg, nShift); 
    if(ret)
    {
        return ret; 
    }

    major->dbSpeed *= 0.001; 
    // major->dbCurrent;
    // major->dbVoltage;
    return 0;
}

int EwfFromArg(WELD_COND_END* end, const ARG_VALUE *arg, const unsigned nShift)
{
    int ret; 

    ret = Arg_WeldCond_End(end, arg, nShift); 
    if(ret)
    {
        return ret; 
    }

    end->dbCraterTime *= 1000.;    
    end->dbPostflowTime *= 1000.; 
    // end->dbVoltage;
    // end->dbCurrent;
    return 0;
}
#if 0
void UnitChange_Start(WELD_COND_START* start)
{
    ASSERT_RETURN(start, );
    start->dbArcTime *= 1000.;
    start->dbPreflowTime *= 1000.; 
    start->dbVoltage; 
    start->dbArcDist; 
    start->dbCurrent;     
}

void UnitChange_End(WELD_COND_END* end)
{
    ASSERT_RETURN(end, ); 
    end->dbCraterTime *= 1000.;    
    end->dbPostflowTime *= 1000.; 
    end->dbVoltage;     
    end->dbCurrent; 
}

void UnitChange_Main(WELD_COND_MAIN* major)
{
    ASSERT_RETURN(major, ); 
    major->dbSpeed *= 1000.; 
    major->dbCurrent;    
    major->dbVoltage; 
    
}
#endif
