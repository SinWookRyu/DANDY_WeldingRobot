// PROFILE is for Motion Profile Generation, Calculation Motion, Stop. 
// Prof, PROF, prof represent PROFILE. 
// 

#ifndef PROFILE7_H_
#define PROFILE7_H_

#define PROF7_SECT_NONE      (0) // Not motion 
#define PROF7_SECT_ACC_J1    (1) // Acc-Jerk1 Section 
#define PROF7_SECT_ACC_U     (2) // Acc-Uniform Section 
#define PROF7_SECT_ACC_J2    (3) // Acc-Jerk2 Section 
#define PROF7_SECT_UNI       (4) // Uniform Vel Section
#define PROF7_SECT_DEC_J1    (5) // Dec-Jerk1 Section
#define PROF7_SECT_DEC_U     (6) // Dec-Uniform Section
#define PROF7_SECT_DEC_J2    (7) // Dec-Jerk2 Section
#define PROF7_SECT_OVER      (8) // Section All Over 

#pragma pack(push, 1)
typedef struct 
{   
    // global starting condition
    double t0;
    double p0;
    double v0;
    double a0; 
    double j0; 
    
    // end time of each section
	double t_1a;    // j1-acc
	double t_ua;    // uni-acc
    double t_2a;    // j2-acc
	double t_u;
    double t_1d;
    double t_ud;
    double t_2d;

    // end position of each section
	double p_1a;    // j1-acc
	double p_ua;    // uni-acc
    double p_2a;    // j2-acc
	double p_u;
    double p_1d;
    double p_ud;
    double p_2d;

    // end vel of each section
	double v_1a;    // j1-acc
	double v_ua;    // uni-acc
    double v_2a;    // j2-acc
	double v_u;
    double v_1d;
    double v_ud;
    double v_2d;

    // end acc of each section
	double a_1a;    // j1-acc
	double a_ua;    // uni-acc
    double a_2a;    // j2-acc
	double a_u;
    double a_1d;
    double a_ud;
    double a_2d;

    // end jerk of each section
	double j_1a;    // j1-acc
	double j_ua;    // uni-acc
    double j_2a;    // j2-acc
	double j_u;
    double j_1d;
    double j_ud;
    double j_2d;
}PROFILE7; 
#pragma pack(pop)

#if defined (__cplusplus)
extern "C" {
#endif

// Sets 7 Segment Motion Profile normalized to 0 ~ 1 even if p_trg < 0. 
// time : |--t1a--|--tua--|--t2a--|---tu---|--t1d--|--tud--|--t2d--|
// All jerk used are same value. 
// Acc profile & Dec profile are same. 
// All series of units must be synchronized.     
// If 'pa+pd >= p_trg', Profile no has the uniform vel period. 
// 
// RET   : suc(0), invalid input arg(-1)
// prof  : [o] 7 Segment Profile
// p_trg : [i] Target Pos
// v_trg : [i] Target Vel (>0)
// a     : [i] Accel (>0)
// j     : [i] Jerk (>0)
//
int Prof7_Set(PROFILE7* prof, double p_trg, double v_trg, double a, double j); 

// Sets Deceleration 7-Segment Motion Profile normalized to 0 ~ 1.
// Jerk is not used. 
// time : |-ta=0-|-tu=0-|--t1d=0--|--tud--|--t2d=0--|
// All series of units must be synchronized.     
// 
// RET   : suc(0), invalid input arg(-1)
// prof  : [o] 7-Segment Profile
// p     : [o] New Calculated Target Pos (<>0)
// v0    : [i] Initial Vel (<>0)
// d     : [i] Decel (>0)
// t     : [i] Event Start Time 
// 
int Prof7_Set_Dec(PROFILE7* prof, double* p_trg, double v0, double d, double t); 


// Sets Deceleration 7-Segment Motion Profile normalized to 0 ~ 1.
// Decels are in same time of td (To use 1 profile for every joints whose v0' are different.)
// Jerk is not used. 
// time : |-ta=0-|-tu=0-|--t1d=0--|--tud--|--t2d=0--|
// All series of units must be synchronized.     
// 
// RET   : suc(0), invalid input arg(-1)
// prof  : [o] 7-Segment Profile
// p     : [o] New Calculated Target Pos (<>0)
// v0    : [i] Initial Vel (<>0)
// td    : [i] Decel Time(>0)
// t     : [i] Event Start Time 
//
int Prof7_Set_Dec_Time(PROFILE7* prof, double* p_trg, double v0, double td, double t); 

// Modifies the 7-segment motion profile for the stop motion. 
// If decel <= prof.d (normalized level_), No modification, 
// this promises faster & shorter stop than original. 
// It's possible to call many times
// Jerk is not used. 
// 
// !! WARNING !! 
// Never use modified profile with the prev. time than stop time. 
// This function changes decel info only. Calling with the prev. time causes
// non-continuous Profile. 
// 
// prof: Motion Profile to be modified. 
// d   : Deceleration of stop motion (>0)
// time: Time of starting stop. 0 is the start of motion (>0)
//
void Prof7_Stop(PROFILE7* prof, double p_trg, double dec, double time); 

// Returns the dynamics of PROFILE7 at time of 't'.
// Units are used as used for Prof7_Set(). 
// 
// prof: [i] Pointer to the Motion Profile Data PROFILE7
// v   : [o] Pointer to Velocity of Dynamics
// a   : [o] Pointer to Accel of Dynamics
// t   : [i] Time to get the dynamics on.
double Prof7_Get(const PROFILE7* prof, double* v, double* a, double time); 

// Gets the section of PROFILE7 at time of 't'.
// 
// RET : Profile Section of 't'
// prof: [i] Pointer to the Motion Profile Data PROFILE7
// t   : [i] Time to get the dynamics on.
//
int Prof7_GetSect(const PROFILE7* prof, double time); 

// Returns sync profile between prof1 & prof2
// prof1 & prof2 must be before called SCURVE_SetStop(..).
// 
PROFILE7 Prof7_GetSync(const PROFILE7* prof1, const PROFILE7* prof2);

#if defined(__cplusplus)
}
#endif

#endif