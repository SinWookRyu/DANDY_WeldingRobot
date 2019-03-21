// MOTION type : Joint Motion(JMOT), Cartesian Motion(CMOT)
// Cartesian Motion Part : Translation Motion(POS) + Orientation Motion(ROT)
// Cartesian Motion Type : Linear Motion(LinMot), Circular Motion(CirMot)
// Joint Type : Revolute Joint, Prismatic Joint
// 
// In Joint Motion 
// 1) Jmot_Set(..)
// 2) Jmot_JointOnParam(..)
// 
// In Cartesian Motion 
// 1) Cmot_KinSet(..) 
// 2) Cmot_SetLinMot(..)
// 3) Cmot_JointOnParam(..)

#ifndef MOTION_H__
#define MOTION_H__

#include "geometry.h"

#if defined (__cplusplus)
extern "C" {
#endif 

// Joint Motion ///////////////////////////////////////////////////////////////
    
#pragma pack(push, 1)
typedef struct 
{    
    double dbJointS;     // [rad] start joint angle
    double dbJointD;     // [rad] displacement joint angle
    double dbSpeed;      // [rad/ms] 
} JMOT; 
#pragma pack(pop)

// 1 Axis Motion Initialization
//[ret] suc(0)
int Jmot_Set(JMOT* mot,         //[out]                          
             double JointS,     //[in ] Start Joint Values
             double JointF,     //[in ] Final Joint Values                 
             double Speed);     //[in ] Speed(>0) [rad/ms]

// Returns joint corresponding to 'u' param.
double Jmot_JointOnParam(const JMOT* mot, double u); 

// Cartesian Motion ////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct {    
    POS Ps;                 // Start POS of TCP wrt BASE
    POS Sp;                 // Translation Dir
    double dist;            // Translation Distance (arc for circle)    
    double v_p;             // [mm/ms] Linear Vel
    ROT Rs;                 // Start ROT of TCP wrt BASE
    POS Sr;                 // Orientation Dir
    double ang;             // Orientation Angle [rad]
    double v_r;             // [rad/ms] Orientational Vel
} LMOT; 
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {    
    POS Ps;                 // Start POS of TCP wrt BASE
    POS Pc;                 // Circle Center Pos
    POS Sp;                 // Circle Cneter Dir
    double arc;             // [mm] Translation Distance (arc for circle)    
    double v_p;             // [mm/ms] Linear Vel
    double rad;             // [mm] Circle Radius
    ROT Rs;                 // Start ROT of TCP wrt BASE
    POS Sr;                 // Orientation Dir
    double ang;             // [rad] Orientation Angle
    double v_o;             // [rad/ms] Orientational Vel
} CMOT; 
#pragma pack(pop)

int Lmot_Set(LMOT* lmot,              // [out]
             const POS* Ps,           // Start POS of TCP wrt BASE 
             const POS* Sp,           // Translation Direction Vector
             double dist,             // Translation Distance
             double v_p,              // [mm/ms] Translational Vel
             const ROT* Rs,           // Start ROT of TCP wrt BASE
             const POS* Sr,           // Orientation Dir
             double ang,              // Rotation Angle              
             double v_r);             // [rad/ms] Orientational Vel

// calculates position(lin motion) on parameter(u)    
void Lmot_PosOnParam(POS *pPos,             //[out] Translation POS on 'u'
                     const LMOT* pLmot,     //[in ] LMOT info's
                     double u);             //[in ] parameter 'u'.    

// calculates ROT on parameter(u)   
void Lmot_RotOnParam(ROT *pR,               //[out] Orientation ROT on 'u'
                     const LMOT* pLmot,     //[in ] LMOT Info's.
                     double u);             //[in ] parameter 'u'

// Circular Motion Setting
int Cmot_Set(CMOT* cmot,              // [out]             
             const POS* Pc,           // Center POS
             const POS* Ps,           // Start POS of TCP wrt BASE 
             const POS* Sp,           // Translation Direction Vector
             double arc,              // Translation Distance(Arc of Circle)
             double v_p,              // [mm/ms] Translational Vel
             const ROT* Rs,           // Start ROT of TCP wrt BASE
             const POS* Sr,           // Orientation Dir
             double ang,              // Rotation Angle              
             double v_o);             // [rad/ms] Orientational Vel

// calculates position(lin motion) on parameter(u)    
void Cmot_PosOnParam(POS *pPos,             //[out] Translation POS on 'u'
                     const CMOT* cmot,     //[in ] LMOT info's
                     double u);             //[in ] parameter 'u'.    

// calculates ROT on parameter(u)   
void Cmot_RotOnParam(ROT *pR,               //[out] Orientation ROT on 'u'
                     const CMOT* cmot,     //[in ] LMOT Info's.
                     double u);             //[in ] parameter 'u'

////////////////////////////////////////////////////////////////////////////////

#if defined (__cplusplus)
}
#endif

#endif