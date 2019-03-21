#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#ifndef PI
#define PI (3.141592653589793238462643383279)
#endif

typedef union
{
    double dbMember[3];

    struct
    {
        double x;
        double y;
        double z;
    };
}POS, *PPOS, *LPPOS;

typedef union 
{
    double dbMember[9];

    struct
    {
        double r11;
        double r21;
        double r31;
    
        double r12;
        double r22;
        double r32;

        double r13;
        double r23;
        double r33;
    };

    struct
    {
        POS u;
        POS v;
        POS w;
    };

}ROT, *PROT, *LPROT;


typedef union
{   
    double dbMember[12];

    struct
    {
        double t11;
        double t21;
        double t31;

        double t12;
        double t22;
        double t32;

        double t13;
        double t23;
        double t33;

        double t14;
        double t24;
        double t34;
    };

    struct 
    {
        POS u;
        POS v;
        POS w;
        POS p;        
    };

    ROT R;

}TRANS, *PTRANS, *LPTRANS;


typedef union
{
    double dbMember[6];

    struct
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

}XYZRPY, *PXYZRPY, *LPXYZRPY;


typedef union
{
    double dbMember[4];

    struct
    {
        double th;
        double d;
        double al;
        double l;
    };

}DH, *PDH, *LPDH;

#if defined(__cplusplus)
extern "C"
{
#endif

///// POS Modules /////

POS POS_Multi_S(POS p, double s);   // po = pi * d;

int POS_Unit(POS *pP);              // Make p unit vector 

// Make p unit vector safely(if fail to unit, return [0,0,1])
POS POS_SafeUnit(const POS *pP);

double POS_Norm(POS p);             // return norm of 'p'

POS POS_Plus_POS(POS p1, POS p2);   // p1 + p2

POS POS_Minus_POS(POS p1, POS p2);  // p1 - p2

POS POS_Cross(POS p1, POS p2);      // p1 x p2

double POS_Inner(POS p1, POS p2);   // p1 . p2

// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
double POS_GetAngle(POS *pDir, POS Ps, POS Pf);

// Get angle(0.0 ~ PI) & dir. unit vector from Ps to Pf
// Get Dist of 'p' vector to the 'dir' unit-vector. 
double POS_GetDist(POS p, POS dir); 

///// ROT Modules /////

POS ROT_Multi_POS(ROT R, POS p);    // R * p

ROT ROT_Set(double r11, double r12, double r13,
            double r21, double r22, double r23,
            double r31, double r32, double r33);

ROT ROT_Eye(void);                  // Eygen Rot Matrix

ROT ROT_Inv(ROT R);                 // Inverse of rotation mat R

ROT ROT_Multi_ROT(ROT R1, ROT R2);  // R = R1*R2

ROT ROT_Screw(POS s, double theta); // Screw matrix about 's' with 'theta'

int ROT_GetScrewAxis(POS *pS,               
                     double *pTheta,        
                     ROT R);


///// TRANS Modules /////
        
// sets TRANS in Matrix form
TRANS TRANS_Set(double t11, double t12, double t13, double t14, 
                double t21, double t22, double t23, double t24, 
                double t31, double t32, double t33, double t34);   

TRANS TRANS_Eye(void);              // Eigen Trans

TRANS TRANS_Xyzrpy(XYZRPY xyzrpy);  // TRANS of xyzrpy 

XYZRPY TRANS_GetXyzrpy(TRANS T);    // xyzrpy from 'T'

TRANS TRANS_Screw(POS s,            // Screw TRANS about 's' from 'so' with 'theta' & 't'
                  POS so,       
                  double theta, 
                  double t);    

TRANS TRANS_Lin(POS pos);                       // translational moving TRANS

POS TRANS_Multi_POS(TRANS T, POS p);

TRANS TRANS_Multi_TRANS(TRANS T1, TRANS T2);    // T1*T2

TRANS TRANS_Inv(TRANS T);                       // inverse

void TRANS_Print12(TRANS T, char *title);       // print T & 'title'. % .12f Precision

void TRANS_Print4(TRANS T, char *title);        // print T & 'title'. % .4f Precision

// Gets center of circle including pos1, pos2 & pos3.
// return : suc(0), invalid input (-1)
int CIR_GetCenter(POS *pCenter, 
                  POS p1, 
                  POS p2, 
                  POS p3);

// Gets circle info's with pos1, pos2 & pos3.
// return : suc(0), invalid input(not 3 pos)(-1)
int CIR_GetCircle(POS *pCenter,        //[o] circle center
                  POS *pDir,           //[o] circle rot dir unit vec.                   
                  double *pAngle,      //[o] circle rot angle
                  double *pRadius,     //[o] circle radius
                  POS Ps,              //[i] start pos
                  POS Pf,              //[i] final pos
                  POS Pv);             //[i] viaway pos 

// Gets sphere center of pos1, pos2, pos3 & pos4
// return : suc(0), invalid input (-1)
int SPH_GetCenter(POS *pCenter, 
                  POS p1, 
                  POS p2, 
                  POS p3, 
                  POS p4);                                     

#if defined(__cplusplus)
}
#endif

#endif
