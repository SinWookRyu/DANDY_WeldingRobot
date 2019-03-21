// PROFILE is for Motion Profile Generation, Calculation Motion, Stop. 
// Prof, PROF, prof represent PROFILE. 
// 
#include "stdio.h"
#include "assert.h"
#include "float.h"
#include "math.h"
#include "memory.h"
#include "profile3.h"
#include "utility.h"

static void ZeroProfile(PROFILE3* prof); 

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
int Prof3_Set(PROFILE3* prof, double p_trg, double v_trg, double a, double d)
{   
    double p, v; 
    double ta, tu, td;  // time of each section
    double pa, pu, pd;  // dist of each section

    ///// Arguments Validation Check /////

    if(prof == NULL)
    {
        return -1;
    }
    
    // 0-pos Checking : no need to calculate.
    // 0-pos checking must be prior to the 0-vel check. 
    // All SCURVE members 0, SCUR_GetParamPos returns SCURVE_SECT_OVER & 0.
    if(fabs(p_trg) <= EPSILON)
    {   
        memset(prof, 0, sizeof(PROFILE3));        
        return 0;
    }

    // WARNING!! This vel/acc check module must be behind 0-pos check. 
    // Because pos(0), vel(0), acc(0), dec(0) case is ok, 
    // but pos(not 0), vel(0) is problem case. 
    if(v_trg <= EPSILON || a <= EPSILON || d <= EPSILON)
    {
        return -1;    
    }

    ///// Prepare Basic Arguments for Calculate with average acc /////
    
    // normalize (pos -> 1.0)    
    p = fabs(p_trg); 
    v = v_trg/p;
    a = a/p;
    d = d/p;
    p = 1.0;     

    // ta, td with target v
    ta = v/a; 
    td = v/d; 
    pa = 0.5*v*ta; 
    pd = 0.5*v*td; 

    // Basic Param Calc with checking if accel & decel pos exceeds Target Pos
    // If exceeds sets to no uniform Profile. 
    if (p <= pa+pd)
    {
        // v = a*ta = d*td -> td = a*ta/d
        // p = 0.5*a*ta^2 + 0.5*d*td^2
        // p = 0.5*a*ta^2 + 0.5*a^2*ta^2/d = 0.5*a*ta^2(1 + a/d)
        // ta^2 = p / (0.5*a*(1+a/d))
    
        ta = sqrt( p / (0.5*a*(1+a/d)) ); 
        td = a * ta / d; 
        tu = 0.0; 

        v  = a * ta; 
    
        pa = 0.5*v*ta; 
        pd = 0.5*v*td; 
        pu = 0.0; 
    }
    else
    {    
        // v = a*ta = d*td -> td = a*ta/d
        // p = 0.5*a*ta^2 + 0.5*d*td^2 + v*tu
        // tu = (p - 0.5*a*ta^2 - 0.5*d*td^2) / v

        ta = ta; 
        td = td; 
        tu = (p - pa - pd) / v; 

        v = v; 

        pa = pa; 
        pd = pd; 
        pu = v*tu; 
    }

#if 0
    prof->p_trg = p_trg; 
#endif
    prof->acc   = a; 
    prof->dec   = d; 

    prof->t0 = 0; 
    prof->p0 = 0; 
    prof->v0 = 0;
    
    prof->t_acc = ta;     
    prof->t_uni = ta+tu; 
    prof->t_dec = ta+tu+td;     

    prof->p_acc = pa; 
    prof->p_uni = pa+pu; 
    prof->p_dec = 1.0; 
    
    prof->v_acc = v;     
    prof->v_uni = v; 
    prof->v_dec = 0; 

    return 0; 
}

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
int Prof3_Set_Dec(PROFILE3* prof, double v0, double d, double t)
{
    double td, pd, sig; 
        
    sig = (0 <= v0)? 1.0 : -1.0;     

    // Below 0 velocity check 
    v0 = fabs(v0); 
    if (v0 <= EPSILON)
    {   
        memset(prof, 0, sizeof(PROFILE3));        
        return 0;
    }

    // 0 dec check 
    if (d <= EPSILON)
    {
        memset(prof, 0, sizeof(PROFILE3));        
        return -1;
    }

    td = v0 / d; 
    pd = v0*td - 0.5*d*td*td;   // sig '-' caution
#if 0
    prof->p_trg = sig * pd; 
#endif
    prof->acc   = 0; 
    prof->dec   = d/pd; 

    prof->t0 = 0; 
    prof->p0 = 0; 
    prof->v0 = 0;
#if 0 
    prof->t_acc = 0;     
    prof->t_uni = 0; 
    prof->t_dec = td;     
#else
    prof->t_acc = t;     
    prof->t_uni = t; 
    prof->t_dec = t+td;     
#endif

    prof->p_acc = 0; 
    prof->p_uni = 0; 
    prof->p_dec = 1.0; 
    
    prof->v_acc = 0;     
    prof->v_uni = v0/pd; 
    prof->v_dec = 0; 

    return 0; 
}

// Returns the dynamics of PROFILE3 at time of 't'.
// Units are used as used for Prof3_Set(). 
// 
// prof: [i] Pointer to the Motion Profile Data PROFILE3
// v   : [o] Pointer to Velocity of Dynamics
// a   : [o] Pointer to Accel of Dynamics
// t   : [i] Time to get the dynamics on.
double Prof3_Get(const PROFILE3* prof, double* v, double* a, double time)
{   
    double p0, v0, a0, t, u;     

    // data validation 

    ASSERT_RETURN(prof, 0.0); 

    time = (time < 0)? 0 : time; 

    // Parameters for calc
    if(prof->t_dec < time)        // Motion Over
    {
        p0 = prof->p_dec; 
        v0 = prof->v_dec; 
        a0 = 0; 
        t  = time - prof->t_dec;         
    }
    else if(prof->t_uni < time)   // Decel Section 
    {
        p0 = prof->p_uni; 
        v0 = prof->v_uni; 
        a0  =-prof->dec; 
        t  = time - prof->t_uni;         
    }
    else if(prof->t_acc < time)   // Uni-Vel Section
    {
        p0 = prof->p_acc; 
        v0 = prof->v_acc; 
        a0  = 0; 
        t  = time - prof->t_acc;         
    }
    else                           // Accel Section
    {
        p0 = 0; 
        v0 = 0; 
        a0  = prof->acc; 
        t  = time;         
    }    

    // outputs 
    u= p0 + v0*t + 0.5*a0*t*t; 

    if(v != NULL)
    {
        *v = v0 + a0*t; 
    }
    if(a != NULL)
    {
        *a = a0; 
    }
    
    return u;  
}

// Gets the section of PROFILE3 at time of 't'.
// 
// RET : Profile Section of 't'
// prof: [i] Pointer to the Motion Profile Data PROFILE3
// t   : [i] Time to get the dynamics on.
//
int Prof3_GetSect(const PROFILE3* prof, double time)
{   
    int sec; 

    if(prof == NULL || time < 0)
    {
        return PROF3_SECT_NONE; 
    }

    // Parameters for calc
    if(prof->t_dec < time)        // Motion Over
    {        
        sec= PROF3_SECT_OVER; 
    }
    else if(prof->t_uni < time)    // Decel Section 
    {
        sec= PROF3_SECT_DEC; 
    }
    else if(prof->t_acc < time)    // Uni-Vel Section
    {
        sec= PROF3_SECT_UNI; 
    }
    else                            // Accel Section
    {
        sec= PROF3_SECT_ACC; 
    }        
    return sec; 
}

// Modifies the 3 segment motion profile for the stop motion. 
// If decel <= prof.d (normalized level_), No modification, 
// this promises faster & shorter stop than original. 
// It's possible to call many times
// 
// !! WARNING !! 
// Never use prev. time than last used for 'time'. 
// Never use modified profile with the prev. time than stop time. 
// This function changes decel info only. Calling with the prev. time causes
// non-continuous Profile. 
// 
// RET : SUCCESS(0), FAIL(else). If fail Stop immediately. 
// prof: Motion Profile to be modified. 
// d   : Deceleration of stop motion. Not normalized value(>0)
// time: Stop Start Time. Usually 'time_act - sampling' (>0)
#if 0
void Prof3_Stop(PROFILE3* prof, double dec, double time)
#else
void Prof3_Stop(PROFILE3* prof, double p_trg, double dec, double time)
#endif
{
    double d; 
    double p0, v0; 
    double td, pd; 
    
    // Parameter Validation

    ASSERT_RETURN(prof, ;); 
    
    if(dec < EPSILON)
    {
        dec = EPSILON * 2; 
    }
    
    // Zero Profile Setting Case : 0 Target or Stop with Start. 
    // 0 == prof->t_uni : Decel Profile, No Touch
#if 0
    if(prof->p_trg == 0 || (0 < prof->t_uni && time < EPSILON))
#else
    if(p_trg == 0 || (0 < prof->t_uni && time < EPSILON))
#endif
    {
        ZeroProfile(prof);         
        return;         
    }

    // normalization of dec & comparison 
#if 0
    d = dec / fabs(prof->p_trg); 
#else
    d = dec / fabs(p_trg); 
#endif
    // decel is smaller & decel mode is already, no work
    if(d <= prof->dec && prof->t_uni <= time )
    {       
        return;
    }

    // Get Decel Info's
    p0 = Prof3_Get(prof, &v0, NULL, time); 
    td = v0 / d; 
    pd = 0.5 * d * td * td; 

    // Sets Data Modified  
    prof->dec = d; 

    prof->t_uni = time; 
    prof->p_uni = p0; 
    prof->v_uni = v0; 

    prof->t_dec = time + td; 
    prof->p_dec = p0 + pd; 
    prof->v_dec = 0; 
    
    return;     
}
       
#if 0
// Returns sync profile between prof1 & prof2
// prof1 & prof2 must be before called SCURVE_SetStop(..).
PROFILE3 Prof3_GetSync(PROFILE3 prof1, PROFILE3 prof2)
{
	PROFILE3 s;

	double dbTemp1, dbTemp2; 
	double ta, td, tu;	//interval of each sect 
	double pa, pd, pu;			//position of each sect	
    double v, a, d;     

    memset(&s, 0, sizeof(PROFILE3));
    
    dec normalization needed. 

    // 
    // acc time 
    dbTemp1 = prof1.t_acc; 
    dbTemp2 = prof2.t_acc; 
    ta = (dbTemp1 > dbTemp2)? dbTemp1 : dbTemp2;	

    // uni-vel time 
    dbTemp1 = prof1.t_uni - prof1.t_acc; 
    dbTemp2 = prof2.t_uni - prof2.t_acc; 
    tu = (dbTemp1 > dbTemp2)? dbTemp1 : dbTemp2;	

    // dec time 
    dbTemp1 = prof1.t_dec - prof1.t_uni; 
    dbTemp2 = prof2.t_dec - prof2.t_uni; 
    td = (dbTemp1 > dbTemp2)? dbTemp1 : dbTemp2;	

    tv = ta + tu + td; 

    // 0.5*ta*v + 0.5*td*v + (tv-ta-td)v = 1.0 
    v = (tv>EPSILON)? 1.0 / ( tv - 0.5*(ta+td) ) : 0.0;

    // v = ta * a = td * d	
    a = (ta>EPSILON)? v / ta : 0.0;
    d = (td>EPSILON)? v / td : 0.0;

    // extras values for each section
	pa = 0.5 * v * ta;
	pd = 0.5 * v * td;
    pu = v * tu; 

	///// data save /////
    
    s.acc = a;	
	s.dec = d;	

    s.t0 = 0; 
    s.p0 = 0; 
    s.v0 = 0; 
    
    s.t_acc = ta;     
    s.t_uni = ta+tu; 
    s.t_dec = ta+tu+td;     

    s.p_acc = 0.5 * v * ta; 
    s.p_uni = pa+pu; 
    s.p_dec = 1.0; 
    
    s.v_acc = v;     
    s.v_uni = v; 
    s.v_dec = 0;

	return s;
}
#endif

static void ZeroProfile(PROFILE3* prof)
{
    ASSERT_RETURN(prof, ;); 
    
    prof->t_acc = prof->t0; 
    prof->t_uni = prof->t0; 
    prof->t_dec = prof->t0; 

    prof->p_acc = prof->p0; 
    prof->p_uni = prof->p0;
    prof->p_dec = prof->p0; 
    
    prof->v_acc = prof->v0; 
    prof->v_uni = prof->v0; 
    prof->v_dec = prof->v0; 
}
