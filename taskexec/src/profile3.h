// PROFILE is for Motion Profile Generation, Calculation Motion, Stop. 
// Prof, PROF, prof represent PROFILE. 
// 

#ifndef PROFILE_H_
#define PROFILE_H_

#define PROF3_SECT_NONE      (0) // Not motion 
#define PROF3_SECT_ACC       (1) // Acceleration Section 
#define PROF3_SECT_UNI       (2) // Uniform Vel Section
#define PROF3_SECT_DEC       (3) // Deceleration Section
#define PROF3_SECT_OVER      (4) // Section All Over 

#pragma pack(push, 1)
typedef struct 
{   
    // target pos, accel, decel
#if 0
    double p_trg;   // unnormalized original target
#endif
	double acc;			
	double dec;			
	
    // global starting condition
    double t0;
    double p0;
    double v0;
    
    // end time of each section
	double t_acc;
	double t_uni;
	double t_dec;

    // end position of each section
	double p_acc;
	double p_uni;
	double p_dec;

    // end vel of each section
	double v_acc;
	double v_uni;
	double v_dec;

}PROFILE3; 
#pragma pack(pop)

#if defined (__cplusplus)
extern "C" {
#endif


// Sets 3 Segment Motion Profile normalized to 0 ~ 1 even if p_trg < 0. 
// time : |--------ta-------|-tu-|-------td--------|
// All series of units must be synchronized.     
// If 'pa+pd >= p_trg', Profile no has the uniform vel period. 
// 
// RET   : suc(0), invalid input arg(-1)
// prof  : [o] 3 Segment Profile
// p_trg : [i] Target Pos
// v_trg : [i] Target Vel (>0)
// a     : [i] Accel (>0)
// d     : [i] Decel (>0)
//
int Prof3_Set(PROFILE3* prof, double p_trg, double v_trg, double a, double d); 


// Sets Deceleration 3 Segment Motion Profile normalized to 0 ~ 1.
// time : |-ta=0-|-tu=0-|-------td--------|
// All series of units must be synchronized.     
// 
// RET   : suc(0), invalid input arg(-1)
// prof  : [o] 3 Segment Profile
// v0    : [i] Initial Vel (<>0)
// d     : [i] Decel (>0)
// t     : [i] Event Start Time 
// 
int Prof3_Set_Dec(PROFILE3* prof, double v0, double d, double t); 

// Gets the section of PROFILE3 at time of 't'.
// 
// RET : Profile Section of 't'
// prof: [i] Pointer to the Motion Profile Data PROFILE3
// t   : [i] Time to get the dynamics on.
//
int Prof3_GetSect(const PROFILE3* prof, double time); 

// Returns the dynamics of PROFILE3 at time of 't'.
// Units are used as used for Prof3_Set(). 
// 
// prof: [i] Pointer to the Motion Profile Data PROFILE3
// v   : [o] Pointer to Velocity of Dynamics
// a   : [o] Pointer to Accel of Dynamics
// t   : [i] Time to get the dynamics on.
double Prof3_Get(const PROFILE3* prof, double* v, double* a, double time); 

// Modifies the 3 segment motion profile for the stop motion. 
// If decel <= prof.d (normalized level_), No modification, 
// this promises faster & shorter stop than original. 
// It's possible to call many times
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
#if 0
void Prof3_Stop(PROFILE3* prof, double dec, double time); 
#else
void Prof3_Stop(PROFILE3* prof, double p_trg, double dec, double time); 
#endif

// Returns sync profile between prof1 & prof2
// prof1 & prof2 must be before called SCURVE_SetStop(..).
// 
PROFILE3 Prof3_GetSync(PROFILE3 prof1, PROFILE3 prof2);

#if defined(__cplusplus)
}
#endif

#endif