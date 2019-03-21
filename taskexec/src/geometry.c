#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "geometry.h"

// p * scalar
POS POS_Multi_S(POS p, double s)   
{
    POS po;

    po.x = p.x * s;
    po.y = p.y * s;
    po.z = p.z * s;

    return po;
}

// Make p unit vector 
// ret(0) suc, (-1) fail
int POS_Unit(POS *pP)  
{
    double d;

    if(pP == NULL)
        return -1;

    d = POS_Norm(*pP);
    if(d <= DBL_EPSILON)
        return -1;
    
    *pP = POS_Multi_S(*pP, 1./d);        
    return 0;  
}

// Make p unit vector safely(if fail to unit, return [0,0,1])
POS POS_SafeUnit(const POS *pP)  
{
    double      d;
    const POS   k = {{0.0, 0.0, 1.0}};

    if(pP == NULL)
        return k;

    d = POS_Norm(*pP);
    if(d < DBL_EPSILON)
    {
        return k;
    }
    else
    {
        return POS_Multi_S(*pP, 1./d);        
    }
}

// return norm of 'p'
double POS_Norm(POS p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

// p1 + p2
POS POS_Plus_POS(POS p1, POS p2)
{
    POS p;
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.z = p1.z + p2.z;
    return p;
}

// p1 - p2
POS POS_Minus_POS(POS p1, POS p2)
{
    POS p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    p.z = p1.z - p2.z;
    return p;
}

// p1 x p2
POS POS_Cross(POS p1, POS p2) 
{
    POS p;
    p.x = p1.y*p2.z - p1.z*p2.y;
    p.y = p1.z*p2.x - p1.x*p2.z;
    p.z = p1.x*p2.y - p1.y*p2.x;
    return p;
}

// p1 . p2
double POS_Inner(POS p1, POS p2) 
{
    return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}

// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
double POS_GetAngle(POS *pDir, 
                    POS Ps, 
                    POS Pf)
{
    POS s;
    double th;
    double temp;

    // get angle 'th'
    if(POS_Unit(&Ps) != 0 || POS_Unit(&Pf) != 0)
    {
        if(pDir != NULL)
        {
            memset(pDir, 0, sizeof(POS));
        }
        return 0.0;
    }
    temp = POS_Inner(Ps, Pf);            
    if (temp > 1.0) temp = 1.0;
    if (temp < -1.0) temp = -1.0;
    th = acos(temp); 

    if(pDir == NULL)
    {
        return th; 
    }

    // get dir vector 's'
    s  = POS_Cross(Ps, Pf);
    if(POS_Norm(s) != 0.0)
    {
        POS_Unit(&s);
    }
    else
    {
        memset(pDir, 0, sizeof(POS));
        return 0.0;
    }
    
    // save & return data
    if(pDir != NULL)
        *pDir = s;    
    return th;
}

// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
// Get Dist of 'p' vector to the 'dir' unit-vector. 
double POS_GetDist(POS p, POS dir)
{
    double th; 
    th = POS_GetAngle(NULL, p, dir); 
    return POS_Norm(p) * cos(th); 
}

// R * p
POS ROT_Multi_POS(ROT R, POS p)
{
    POS po;
        
    po.x = R.r11*p.x + R.r12*p.y + R.r13*p.z;
    po.y = R.r21*p.x + R.r22*p.y + R.r23*p.z;
    po.z = R.r31*p.x + R.r32*p.y + R.r33*p.z;

    return po;
}

// Eygen Rot Matrix
ROT ROT_Eye(void)
{
    ROT R;
    memset(&R, 0, sizeof(ROT)); 
    R.r11 = R.r22 = R.r33 = 1.0;
    return R;
}

// gets inverse of rotation mat R
ROT ROT_Inv(ROT R) 
{
    ROT Ro;    

    Ro.r11 = R.r11;  Ro.r12 = R.r21;  Ro.r13 = R.r31;
    Ro.r21 = R.r12;  Ro.r22 = R.r22;  Ro.r23 = R.r32;
    Ro.r31 = R.r13;  Ro.r32 = R.r23;  Ro.r33 = R.r33;

    return Ro;
}

// R = R1*R2
ROT ROT_Multi_ROT(ROT R1, ROT R2)    
{
    ROT R;

    R.u = ROT_Multi_POS(R1, R2.u);
    R.v = ROT_Multi_POS(R1, R2.v);
    R.w = ROT_Multi_POS(R1, R2.w);

    return R;
}

// Calculates screw matrix about 's' vector with 'theta'
ROT ROT_Screw(POS s, double theta) 
{
    ROT R;
    double sx = s.x;
    double sy = s.y;
    double sz = s.z;
    double sth = sin(theta);
    double cth = cos(theta);

    R.r11 = (sx*sx -1.0)*(1.0-cth) + 1.0;
    R.r12 = sx*sy*(1.0-cth) - sz*sth;
    R.r13 = sx*sz*(1.0-cth) + sy*sth;    

    R.r21 = sy*sx*(1.0-cth) + sz*sth;
    R.r22 = (sy*sy-1.0)*(1.0-cth) + 1.0;
    R.r23 = sy*sz*(1.0-cth) -sx*sth;
    
    R.r31 = sz*sx*(1.0-cth) - sy*sth;
    R.r32 = sz*sy*(1.0-cth) + sx*sth;
    R.r33 = (sz*sz-1.0)*(1.0-cth) + 1.0;

    return R;
}


// Calculates rot center vector 's' from screw matrix 
// if '*pTheta' 0.0, s[0] = s[1] = s[2] = 0.0.
int ROT_GetScrewAxis(POS *pS,               //[out]Screw Vector
                     double *pTheta,        //[out]Rotation angle
                     ROT R)                 //[in] Output coord, after rotate.        
{
    double th;
    double _2sth; // 2*sin(th)
    double dir;

    
#if 0
    printf(
        "Screw Rot \n"
        "   %.4f %.4f %.4f\n"
        "   %.4f %.4f %.4f\n"
        "   %.4f %.4f %.4f\n", 
        R.r11, R.r12, R.r13, 
        R.r21, R.r22, R.r23, 
        R.r31, R.r32, R.r33);  
#endif 
    // dir. |'dir'| must not exceed 1.
    dir = (R.r11 + R.r22 + R.r33 - 1.0) / 2.0;
    if(dir > 1.0) 
        dir = 1.0;
    else if(dir < -1.0) 
        dir = -1.0;
    
    // theta
    th = acos(dir);
    _2sth = 2. * sin(th);    
#if 0
    if(_2sth == 0.0)        // no rot, no screw

        return -1;
#else
    if(_2sth*_2sth <= DBL_EPSILON)        // no rot, no screw
    {
        _2sth = DBL_MAX; 
        th = 0.0; 
    }     
#endif

    if(pS != NULL)
    {
        pS->x = (R.r32 - R.r23)/_2sth;
        pS->y = (R.r13 - R.r31)/_2sth;
        pS->z = (R.r21 - R.r12)/_2sth;
    }
    if(pTheta != NULL)
    {
        *pTheta = th;
    }

    return 0;
}

// Sets ROT with matrix type
ROT ROT_Set(double r11, double r12, double r13,
            double r21, double r22, double r23,
            double r31, double r32, double r33)
{
    ROT R;
    
    R.r11 = r11;
    R.r21 = r21;
    R.r31 = r31;

    R.r12 = r12;
    R.r22 = r22;
    R.r32 = r32;

    R.r13 = r13;
    R.r23 = r23;
    R.r33 = r33;

    return R;
}

TRANS TRANS_Set(double t11, double t12, double t13, double t14, 
                double t21, double t22, double t23, double t24, 
                double t31, double t32, double t33, double t34)
{

    TRANS T;
    
    T.t11 = t11;
    T.t21 = t21;
    T.t31 = t31;

    T.t12 = t12;
    T.t22 = t22;
    T.t32 = t32;

    T.t13 = t13;
    T.t23 = t23;
    T.t33 = t33;

    T.t14 = t14;
    T.t24 = t24;
    T.t34 = t34;

    return T;
}

TRANS TRANS_Eye(void)       
{

    TRANS T;

    memset(&T, 0, sizeof(TRANS)); 
    T.t11 = T.t22 = T.t33 = 1.0;
    return T;
}

// calculates coord consist of xyzrpy
TRANS TRANS_Xyzrpy(XYZRPY xyzrpy)
{
    TRANS T;
    double cr = cos(xyzrpy.roll);
    double sr = sin(xyzrpy.roll);
    double cp = cos(xyzrpy.pitch);
    double sp = sin(xyzrpy.pitch);
    double cy = cos(xyzrpy.yaw);
    double sy = sin(xyzrpy.yaw);

    T.t11 = cy*cp;
    T.t21 = sy*cp;
    T.t31 = -sp;

    T.t12 = cy*sp*sr - sy*cr;
    T.t22 = sy*sp*sr + cy*cr;
    T.t32 = cp*sr;
    
    T.t13 = cy*sp*cr + sy*sr;
    T.t23 = sy*sp*cr - cy*sr;
    T.t33 = cp*cr;
    
    T.t14 = xyzrpy.x;
    T.t24 = xyzrpy.y;
    T.t34 = xyzrpy.z;

    return T;
}

// calculates x-y-z-roll-pitch-yaw from coord
XYZRPY TRANS_GetXyzrpy(TRANS T)
{
    XYZRPY xyzrpy;

    // Get XYZ
    xyzrpy.x = T.t14;
    xyzrpy.y = T.t24;
    xyzrpy.z = T.t34;

    // Get RPY(carefully)
    
    // normalizing & orthogornalizing : reducing TRANS errors
    POS_Unit(&T.u);
    POS_Unit(&T.v);
    T.w = POS_Cross(T.u, T.v);

#if 1
    if(T.u.z == 1.0)                        // pitch == -pi/2 : impossible to calc roll/yaw
    {   
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = -atan2(T.v.x, T.v.y);// roll
        xyzrpy.yaw   = 0.0;                 // yaw
    }
    else if(T.u.z == -1.0)                  // pitch == pi/2 : impossible to calc roll/yaw
    {   
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = atan2(T.v.x, T.v.y); // roll
        xyzrpy.yaw   = 0.0;                 // yaw
    }
    else
    {
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = atan2(T.v.z, T.w.z); // roll
        xyzrpy.yaw   = atan2(T.u.y, T.u.x); // yaw 
    }
#else
    
    if(fabs(T.u.z - 1.0) <= DBL_EPSILON)                        // pitch == -pi/2 : impossible to calc roll/yaw
    {   
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = -atan2(T.v.x, T.v.y);// roll
        xyzrpy.yaw   = 0.0;                 // yaw
    }
    else if(fabs(T.u.z - (-1.0)) <= DBL_EPSILON)                  // pitch == pi/2 : impossible to calc roll/yaw
    {   
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = atan2(T.v.x, T.v.y); // roll
        xyzrpy.yaw   = 0.0;                 // yaw
    }
    else
    {
        xyzrpy.pitch = asin(-T.u.z);        // pitch
        xyzrpy.roll  = atan2(T.v.z, T.w.z); // roll
        xyzrpy.yaw   = atan2(T.u.y, T.u.x); // yaw
    }
#endif
    return xyzrpy;
}


// Calculates screw matrix about 's' vector having origin of 'so'.
TRANS TRANS_Screw(POS s,        //[in]  direction vector of rotation.
                  POS so,       //[in]  origin vector of 's'
                  double theta, //[in]  rotation angle about 's'
                  double t)     //[in]  translation accord to 's' from 'so'
{
    TRANS T;
    double sx = s.x;
    double sy = s.y;
    double sz = s.z;
    double sox = so.x;
    double soy = so.y;
    double soz = so.z;

    T.R = ROT_Screw(s, theta);
    
    T.t14 = t*sx - sox*(T.t11-1.0) - soy*T.t12 - soz*T.t13;
    T.t24 = t*sy - sox*T.t21 - soy*(T.t22-1.0) - soz*T.t23;
    T.t34 = t*sz - sox*T.t31 - soy*T.t32 - soz*(T.t33-1.0);
       
    return T;
}

// gets translational moving coord 
TRANS TRANS_Lin(POS pos)            //[in]  translation vector
{
    TRANS T = TRANS_Eye(); 
    T.p = pos;
    return T;
}

// T*[px,py,pz,nPos]'
// nPos(0) : R*p, nPos(1) : T*p = R*p + q
POS TRANS_Multi_POS(TRANS T, POS p)    
{
    // po = T*p;
    POS po = POS_Plus_POS(ROT_Multi_POS(T.R, p), T.p);
    return po;
}

// T1*T2
TRANS TRANS_Multi_TRANS(TRANS T1, TRANS T2)    
{
    TRANS T;
    T.R = ROT_Multi_ROT(T1.R, T2.R);
    T.p = POS_Plus_POS(ROT_Multi_POS(T1.R, T2.p), T1.p);   
    return T;
}

// gets inverse of C
TRANS TRANS_Inv(TRANS T) 
{
    TRANS T_inv = TRANS_Eye();    
    T_inv.R = ROT_Inv(T.R);
    T_inv.p = ROT_Multi_POS(T_inv.R, POS_Multi_S(T.p, -1.0));
    return T_inv;
}

// print coord with precision of %.20f and title
void TRANS_Print12(TRANS T, char *title) 
{
    printf( "%s\n"
            "   % .12f % .12f % .12f % .12f\n" 
            "   % .12f % .12f % .12f % .12f\n" 
            "   % .12f % .12f % .12f % .12f\n", 
            title, 
            T.t11, T.t12, T.t13, T.t14, 
            T.t21, T.t22, T.t23, T.t24, 
            T.t31, T.t32, T.t33, T.t34);            
}

// print coord with precision of %.4f and title
void TRANS_Print4(TRANS T, char *title)
{
    printf( "%s\n"
            "   % .4f % .4f % .4f % .4f\n" 
            "   % .4f % .4f % .4f % .4f\n" 
            "   % .4f % .4f % .4f % .4f\n", 
            title, 
            T.t11, T.t12, T.t13, T.t14, 
            T.t21, T.t22, T.t23, T.t24, 
            T.t31, T.t32, T.t33, T.t34);
}

// Gets center of circle including pos1, pos2 & pos3.
// return : suc(0), invalid input (-1)
int CIR_GetCenter(POS *pCenter, 
                  POS p1, 
                  POS p2, 
                  POS p3)
{
    double a, b, c, d; // direction of plane
    double dbDet;   // determinant    
    
    // matrix A
    double a11, a12, a13;
    double a21, a22, a23;
    double a31, a32, a33;

    POS s, q1, q2;
    double t1, t2, t3;

    //--- gets perpendicular vector to plane [a, b, c] ---//
    q1 = POS_Minus_POS(p3, p1); // q1 = p3 - p1
    q2 = POS_Minus_POS(p2, p1); // q2 = p2 - p1
    s  = POS_Cross(q1, q2);     // s = q1 x q2
    a = s.x;
    b = s.y;
    c = s.z;
    d = (s.x*p1.x + s.y*p1.y + s.z*p1.z + 
         s.x*p2.x + s.y*p2.y + s.z*p2.z) / 2.0;

    //--- so ---//
    // (sox-px)^2 + (soy-py)^2 + (soz-pz)^2 = r^2
    // a*sox + b*soy + c*soz = d
    a11 = 2.0*(p1.x-p2.x);
    a12 = 2.0*(p1.y-p2.y);
    a13 = 2.0*(p1.z-p2.z);

    a21 = 2.0*(p2.x-p3.x);
    a22 = 2.0*(p2.y-p3.y);
    a23 = 2.0*(p2.z-p3.z);

    a31 = a;
    a32 = b;
    a33 = c;
    
    dbDet = (a11*a22*a33 + a12*a23*a31 + a13*a21*a32) - 
            (a11*a23*a32 + a12*a21*a33 + a13*a22*a31);
    if(dbDet*dbDet < DBL_EPSILON)
        return -1;

    t1 = (p1.x*p1.x-p2.x*p2.x) + (p1.y*p1.y-p2.y*p2.y) + (p1.z*p1.z-p2.z*p2.z);
    t2 = (p2.x*p2.x-p3.x*p3.x) + (p2.y*p2.y-p3.y*p3.y) + (p2.z*p2.z-p3.z*p3.z);
    t3 = d;
   
    if(pCenter != NULL)
    {
        pCenter->x = ( (a22*a33-a23*a32)*t1 + (a13*a32-a12*a33)*t2 + (a12*a23-a13*a22)*t3)/dbDet;
        pCenter->y = ( (a23*a31-a21*a33)*t1 + (a11*a33-a13*a31)*t2 + (a13*a21-a11*a23)*t3)/dbDet;
        pCenter->z = ( (a21*a32-a22*a31)*t1 + (a12*a31-a11*a32)*t2 + (a11*a22-a12*a21)*t3)/dbDet;
    }

    return 0;   
}


// Gets circle info's with pos1, pos2 & pos3.
// return : suc(0), invalid input(not 3 pos)(-1)
int CIR_GetCircle(POS *pCenter,        //[o] circle center
                  POS *pDir,           //[o] circle rot dir unit vec.                   
                  double *pAngle,      //[o] circle rot angle
                  double *pRadius,     //[o] circle radius
                  POS Ps,              //[i] start pos
                  POS Pf,              //[i] final pos
                  POS Pv)              //[i] viaway pos                                      
{    
    POS Pc;
    double dbRad;
    POS sv, sf;
    double th_v, th_f;
    double temp;

    // center
    if(CIR_GetCenter(&Pc, Ps, Pv, Pf) != 0)
        return -1;      
    
    // radius             
    Ps = POS_Minus_POS(Ps, Pc);
    Pf = POS_Minus_POS(Pf, Pc);
    Pv = POS_Minus_POS(Pv, Pc);        
    dbRad = (POS_Norm(Ps)+POS_Norm(Pf)+POS_Norm(Pv))/3.0;       

    // circle dir & angle // 

    // !WARNING! unitize to avoid 'acos' error.
    POS_Unit(&Ps);
    POS_Unit(&Pf);
    POS_Unit(&Pv);            
   
    // angle & rot vector btw Ps & Pf.
    temp = POS_Inner(Ps, Pf);            
    if (temp > 1.0) temp = 1.0;
    if (temp < -1.0) temp = -1.0;
    th_f = acos(temp); 
    sf = POS_Cross(Ps, Pf);  // sf = Ps x Pf     

    // angle & rot vector btw Ps & Pv.                        
    temp = POS_Inner(Ps, Pv);            
    if (temp > 1.0) temp = 1.0;
    if (temp < -1.0) temp = -1.0;            
    th_v = acos(temp);   
    sv = POS_Cross(Ps, Pv);  // sv = Ps x Pv        
        
#if 0
    // making sf & sv to bigger one to minimize error
    if(POS_Norm(sf) > POS_Norm(sv))
    {
        sv = sf;
        if(POS_Inner(sf, sv) < 0.0) 
            th_v = 2.0*PI - th_v;
    }            
    else
    {
        sf = sv;         
        if(POS_Inner(sf, sv) < 0.0)
            th_f = 2.0*PI - th_f;
    }

    POS_Unit(&sf);
    POS_Unit(&sv);
#else

    POS_Unit(&sf);
    POS_Unit(&sv);

    // making same direction btw. sv & sf
    if(POS_Inner(sf, sv) < 0.0) 
    {
        th_v = 2.0*PI - th_v;
        sv = POS_Multi_S(sv, -1.0);
    }

#endif
        
    // reversing if th_v > th_f, now th_v & th_f > 0 
    if(th_v > th_f)
    {        
        sf = POS_Multi_S(sf, -1.0);
        sv = POS_Multi_S(sv, -1.0);

        th_f = 2.0*PI - th_f;                
        th_v = 2.0*PI - th_v;                
    }

    // memory copy
    if(pCenter != NULL)
        *pCenter = Pc;
    if(pDir != NULL)
        *pDir =sf;
    if(pAngle != NULL)
        *pAngle = th_f;
    if(pRadius != NULL)
        *pRadius = dbRad;

    return 0;
}

// Gets center of sphere of pos1, pos2, pos3 & pos4
// return : suc(0), invalid input (-1)
int SPH_GetCenter(POS *pCenter, 
                  POS p1, 
                  POS p2, 
                  POS p3, 
                  POS p4)
{
    /*
    p = 0.5*inv(A)*t
    A = [   p1x-p2x, p1y-p2y, p1z-p2z;
            p2x-p3x, p2y-p3y, p2z-p3z;
            p3x-p4x, p3y-p4y, p3z-p4z; ]
    t = [   p1x^2 - p2x^2 + p1y^2 - p2y^2 + p1z^2 - p2z^2 ; 
            p2x^2 - p3x^2 + p2y^2 - p3y^2 + p2z^2 - p3z^2 ; 
            p3x^2 - p4x^2 + p3y^2 - p4y^2 + p3z^2 - p4z^2 ]

    < inv3x3 > 
    M = [a,b,c ; d,e,f ; g,h,i]
    inv(M) = M_ / den
    den = aei+bfg+cdh-afh-bdi-ceg
    M_  = [ei-fh, ch-bi, bf-ce ; fg-di, ai-cg, cd-af ; dh-eg, bg-ah, ae-bd];
    */

    double a = p1.x - p2.x;
    double b = p1.y - p2.y;
    double c = p1.z - p2.z;
    double d = p2.x - p3.x;
    double e = p2.y - p3.y;
    double f = p2.z - p3.z;
    double g = p3.x - p4.x;
    double h = p3.y - p4.y;
    double i = p3.z - p4.z;

    double den = a*e*i + b*f*g + c*d*h - a*f*h - b*d*i - c*e*g;

    POS t = {{0,},};
    t.x = p1.x*p1.x - p2.x*p2.x + p1.y*p1.y - p2.y*p2.y + p1.z*p1.z - p2.z*p2.z; 
    t.y = p2.x*p2.x - p3.x*p3.x + p2.y*p2.y - p3.y*p3.y + p2.z*p2.z - p3.z*p3.z; 
    t.z = p3.x*p3.x - p4.x*p4.x + p3.y*p3.y - p4.y*p4.y + p3.z*p3.z - p4.z*p4.z; 

    if(den == 0.0)
        return -1;

    pCenter->x = 0.5 * ( (e*i-f*h)*t.x + (c*h-b*i)*t.y + (b*f-c*e)*t.z ) / den;
    pCenter->y = 0.5 * ( (f*g-d*i)*t.x + (a*i-c*g)*t.y + (c*d-a*f)*t.z ) / den;
    pCenter->z = 0.5 * ( (d*h-e*g)*t.x + (b*g-a*h)*t.y + (a*e-b*d)*t.z ) / den;

    return 0;
}
