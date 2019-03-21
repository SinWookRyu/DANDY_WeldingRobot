// ARG_GET.C is the program for getting JOB PROGRAM Arguments
// ARG doesn't convert units from DEG to RAD and so on. 
// User convert after/before Value Get/Set.
// 'Arg_Get' represents for ARG_GET Program. 
// 2013-10-25 mrch0

// ARG_SET.C is the program for setting JOB PROGRAM Arguments
// ARG doesn't convert units from DEG to RAD and so on. 
// User convert after/before Value Get/Set.
// 'Arg_Set_xxx' represents for ARG_SET Program. Type of xxx. 
// 'Arg_xxxElem_Set' represents for Program of Setting Argument's Element.
// 2013-10-25 mrch0

#ifndef ARG_H_2013_10_25_
#define ARG_H_2013_10_25_

#include "taskexec_def.h"

#if defined(__cplusplus)
extern "C" {
#endif

// Deg -> Rad
void Deg2Rad(double* value, int n); 
void Rad2Deg(double* value, int n); 

// Argument Getting Module /////////////////////////////////////////////////////

int Arg_Scalar(int *pIntValue, double *pDBValue, const ARG_VALUE *pArgVal);        

int Arg_RobPos(ROBOT_POS *pRobPos, const ARG_VALUE *pArgVal);   

// nShift : Shifed Index of Pointer
int Arg_WeldCond_Start(WELD_COND_START* pWelCon, const ARG_VALUE* pArgVal, const unsigned nShift); 

// nShift : Shifed Index of Pointer
int Arg_WeldCond_Main(WELD_COND_MAIN*  pWelCon, const ARG_VALUE* pArgVal, const unsigned nShift); 

// nShift : Shifed Index of Pointer
int Arg_WeldCond_End(WELD_COND_END*   pWelCon, const ARG_VALUE* pArgVal, const unsigned nShift); 

// nShift : Shifed Index of Pointer
int Arg_WeavePar(WEAVE_PAR *pWeavePar, const ARG_VALUE *pArgVal, const unsigned nShift); 

int Arg_BitElem_Get(int* out, const ARG_VALUE *arg, int index); 

int Arg_JntElem_Get(double* out, const ARG_VALUE* arg, int index);

int Arg_CartElem_Get(double* out, const ARG_VALUE* arg, int index);

// Argument Setting Module /////////////////////////////////////////////////////

int Arg_Set_Byte(const ARG_VALUE *pArgVal, BYTE val) ;

int Arg_Set_Int(const ARG_VALUE *pArgVal, int val);

int Arg_Set_Double(const ARG_VALUE *pArgVal, double val);

int Arg_Set_RobPos(const ARG_VALUE *pArgVal, const ROBOT_POS *pRobPos);

int Arg_Set_WeavePar(const ARG_VALUE *pArgVal, const WEAVE_PAR *pWeavePar);

int Arg_Set_WeldCondStart(const ARG_VALUE *arg, const WELD_COND_START* start);

int Arg_Set_WeldCondMain(const ARG_VALUE *arg, const WELD_COND_MAIN* pri);

int Arg_Set_WeldCondEnd(const ARG_VALUE *arg, const WELD_COND_END* end) ;

int Arg_BitElem_Set(const ARG_VALUE *arg, int elem, int value);

int Arg_JntElem_Set(const ARG_VALUE *arg, int elem, double value);

int Arg_CartElem_Set(const ARG_VALUE *arg, int elem, double value);

// Unit Change Module   ////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Applied Value Getting from ARG_VALUE

// Joint Velocity from ARG_VALUE
// vel = Value of ARG_VALUE * 0.01 (for Joint Vel has type of %)
int JointVelFromArg(double* vel_r, const ARG_VALUE* arg_spd); 

// Distance[mm] from ARG_VALUE[mm]
// vel = Value of ARG_VALUE * 1.0
int DistFromArg(double* dist, const ARG_VALUE* arg_dist); 

// Angle[rad] from ARG_VALUE[deg]
// ang = Value of ARG_VALUE * PI / 180
int AngFromArg(double* ang, const ARG_VALUE* arg_ang); 

// Position Velocity[mm/ms] from ARG_VALUE[mm/s]
// vel = Value of ARG_VALUE * 0.001 
int PosVelFromArg(double* vel, const ARG_VALUE* arg_spd); 

// Time[ms] from ARG_VALUE[s]
// time = Value of ARG_VALUE * 1000.
int TimeFromArg(double* time, const ARG_VALUE* arg_time); 

// Rotational Velocity[rad/ms] from ARG_VALUE[deg/s]
// vel = Value of ARG_VALUE * 0.001 * PI/180
int RotVelFromArg(double* vel, const ARG_VALUE* arg_spd); 

// ARG_VALUE[mm, deg] -> Joint [rad]
// Config required for CART->JOINT is used by 'Actual Pos'
int JointFromArg(double jnt[6], const ARG_VALUE *arg_val, int coord); 

// ARG_VALUE[mm, deg] -> TCP[mm, rad] w/o Ref. Coord
// If Joint Type, wrt BASE->TCP
// - conf : pose config of T. Actual Conf(for CART), Target Conf(for JOINT)
int TransFromArg(TRANS* T, unsigned* conf, const ARG_VALUE *arg_val, int coord);

// WVF with [RAD, mm, ms] unit from WVF of ARG_VALUE [DEG, mm, s]. 
// nShift : Shifed Index of Pointer
int WvfFromArg(WEAVE_PAR* wvf, const ARG_VALUE* arg, const unsigned nShift); 

// [RAD, mm, ms] unit from ARG_VALUE [DEG, mm, s]. 
// nShift : Shifed Index of Pointer
int SwfFromArg(WELD_COND_START* start, const ARG_VALUE *arg, const unsigned nShift); 

// nShift : Shifed Index of Pointer
int MwfFromArg(WELD_COND_MAIN* major, const ARG_VALUE *arg, const unsigned nShift); 

// nShift : Shifed Index of Pointer
int EwfFromArg(WELD_COND_END* end, const ARG_VALUE *arg, const unsigned nShift); 

#if defined(__cplusplus)
}
#endif

#endif 