// PROFILE is for Motion Profile Generation, Calculation Motion, Stop. 
// Prof, PROF, prof represent PROFILE. 
// 
#include "stdio.h"
#include "assert.h"
#include "float.h"
#include "math.h"
#include "memory.h"
#include "profile7.h"
#include "utility.h"

// This is safe than 'prof' memset(0) to return Frozen-Profile. 
// Because time 0 and position 0 can make impulse move from 'act' to 0. 
static void ZeroProfile(PROFILE7* prof, double p0, double t0); 

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
int Prof7_Set(PROFILE7* prof, double p, double v, double a, double j) 
{   
    double vja, vua; 
    double tja, tua, tu, ta, tp;    // time of each section
    double pja, pua, pa, pd;        // dist of each section
   
    ///// Arguments Validation Check /////

    if(prof == NULL)
    {
        return -1;
    }
    
    // 0-pos Checking : no need to calculate.
    // 0-pos checking must be prior to the 0-vel check. 
    // All SCURVE members 0, SCUR_GetParamPos returns SCURVE_SECT_OVER & 0.
    if(fabs(p) <= EPSILON)
    {   
        memset(prof, 0, sizeof(PROFILE7));      
        return 0;
    }

    // WARNING!! This vel/acc check module must be behind 0-pos check. 
    // Because pos(0), vel(0), acc(0), dec(0) case is ok, 
    // but pos(not 0), vel(0) is problem case. 
    if(v <= EPSILON || a <= EPSILON || j <= EPSILON)
    {
        return -1;    
    }
    
    // normalize (pos -> 1.0)    
    p = fabs(p); 
    v = v/p;
    a = a/p;    
    j = j/p; 
    p = 1.0;     

    // Checks 'a' satisfies 'v' & Modifies 'a' 
    // v by jerk : v = a*tja
    // v = a*(tja+tua)    
    tja = a/j; 
    tua = v/a - tja; 
    if(v < a*tja)
    {
        tja = sqrt(v/j);         
        tua = 0.; 
        a = j*tja; 
    }

    // Critial 'p' of no uniform vel
    pa = 0.5*v*(tja + tua + tja); 
    pd = pa; 
    
    // normal 7-seg
    if(pa + pd <= p)
    {   
        tu = (p - pa - pd) / v; 
        tja = tja;    
        tua = tua; 
        v = v; 
        a = a; 
    }
    // no uniform v
    else
    {
        double v_; 

        // critial 'p' & 'v' of jerk only. 
        v_ = a*tja;     
        pa = v*tja; 
        pd = pa; 

        // jerk only profile
        if(p <= pa+pd)
        {
            // p = v*tja*2
            // v = j*tja*tja
            // p = 2*j*tja^3
            // tja = (0.5*p/j)^(1/3)        
            tja = pow(0.5*p/j, 1./3.);             
            tua = 0.;             
            tu  = 0.; 

            a = j*tja; 
            v = a*tja; 
        }
        // uni-acc profile
        else
        {
            double tmp_a, tmp_b; 

            // v*v/a + v*tja = p
            tmp_a = 1./a; 
            tmp_b = tja; 
            v = (-tmp_b + sqrt(tmp_b*tmp_b + 4.*tmp_a*p)) / (2.*tmp_a); 
            a = a; 

            // a*(tja+tua) = v; 
            tua = v/a - tja; 
            tja = tja; 
            tu = 0.; 
        }
    }

    // 
    vja = 0.5*j*tja*tja; 
    vua = a*tua; 
    pja = j*tja*tja*tja / 6.; 
    pua = v*tua * 0.5; 
    pa = v * (tja + tua + tja) * 0.5; 
    ta = tja + tja + tua; 
    tp = ta + tu + ta; 
    
    // end time of each section
    prof->t0 = 0.;
	prof->t_1a = tja; 
	prof->t_ua = tja + tua; 
    prof->t_2a = ta;  
	prof->t_u  = ta + tu;  
    prof->t_1d = ta + tu + tja;
    prof->t_ud = tp - tja;
    prof->t_2d = tp;

    // end position of each section
    prof->p0 = 0.;
	prof->p_1a = pja;    
	prof->p_ua = pja + pua;
    prof->p_2a = pa;    
	prof->p_u  = p - pa; 
    prof->p_1d = p - pja - pua;
    prof->p_ud = p - pja;
    prof->p_2d = p;

    // end vel of each section
    prof->v0 = 0.;
	prof->v_1a = vja;    
	prof->v_ua = vja + vua;    
    prof->v_2a = v;    
	prof->v_u  = v; 
    prof->v_1d = vja + vua;
    prof->v_ud = vja;
    prof->v_2d = 0.;

    // end acc of each section
    prof->a0 = 0.; 
	prof->a_1a = a;   
	prof->a_ua = a;   
    prof->a_2a = 0.;   
	prof->a_u  = 0.; 
    prof->a_1d = -a;
    prof->a_ud = -a;
    prof->a_2d = 0.;

    // end jerk of each section
    prof->j0 = j; 
	prof->j_1a = 0.;   
	prof->j_ua = -j;   
    prof->j_2a = 0.;   
	prof->j_u  = -j; 
    prof->j_1d = 0.;
    prof->j_ud = j;
    prof->j_2d = 0.;

    return 0; 
}

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
int Prof7_Set_Dec(PROFILE7* prof, double* p_trg, double v0, double d, double t)
{
    double td, pd, sig, v;     
        
    sig = (0 <= v0)? 1.0 : -1.0;     

    // Below 0 velocity check 
    v0 = fabs(v0); 
    if (v0 <= EPSILON)
    {
#if 0
        memset(prof, 0, sizeof(PROFILE7));        
        return 0;
#endif 
        pd = 0.0;
        td = 0.0; 
        v = 0.0; 
        d = 0.0; 
        goto POST_WORK; 
    }

    // 0 dec check 
    if (d <= EPSILON)
    {
#if 0
        memset(prof, 0, sizeof(PROFILE7));        
        return -1;
#endif
        pd = 0.0;
        td = 0.0; 
        v = 0.0; 
        d = 0.0; 
        goto POST_WORK; 
    }

    td = v0 / d; 
    pd = v0*td - 0.5*d*td*td;   // sig '-' caution
    v = v0 / pd; 
    d = d  / pd; 

POST_WORK:

    if(p_trg)
    {
        *p_trg = sig * pd; 
    }
    if(prof)
    {
        // end time of each section
        prof->t0 = 0.;
	    prof->t_1a = t; 
	    prof->t_ua = t; 
        prof->t_2a = t;  
	    prof->t_u  = t;  
        prof->t_1d = t;
        prof->t_ud = t+td;
        prof->t_2d = t+td;

        // end position of each section
        prof->p0 = 0.;
	    prof->p_1a = 0;
	    prof->p_ua = 0;
        prof->p_2a = 0;
	    prof->p_u  = 0;
        prof->p_1d = 0;
        prof->p_ud = 1.;
        prof->p_2d = 1.;

        // end vel of each section
	    prof->v0 = 0.;
        prof->v_1a = 0.;
	    prof->v_ua = 0.; 
        prof->v_2a = 0.;
	    prof->v_u  = v;
        prof->v_1d = v;
        prof->v_ud = 0.;
        prof->v_2d = 0.;

        // end acc of each section
	    prof->a0 = 0.; 
        prof->a_1a = 0.;
	    prof->a_ua = 0.;
        prof->a_2a = 0.; 
	    prof->a_u  = 0.;
        prof->a_1d = -d;
        prof->a_ud = -d;
        prof->a_2d = 0.;

        // end jerk of each section
        prof->j0 = 0.; 
	    prof->j_1a = 0.;   
	    prof->j_ua = 0.;   
        prof->j_2a = 0.;   
	    prof->j_u  = 0.; 
        prof->j_1d = 0.;
        prof->j_ud = 0.;
        prof->j_2d = 0.;
    }
    return 0;
}

#if 0
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
int Prof7_Set_Dec_Time(PROFILE7* prof, double* p_trg, double v0, double td, double t)
{
    double d, pd, sig, v;     
        
    sig = (0 <= v0)? 1.0 : -1.0;     

    // Below 0 velocity check 
    v0 = fabs(v0); 
    if (v0 <= EPSILON)
    {
        pd = 0.0;
        td = 0.0; 
        v = 0.0; 
        d = 0.0; 
        goto POST_WORK; 
    }

    // 0 dec check 
    if (td <= EPSILON)
    {
#if 0
        memset(prof, 0, sizeof(PROFILE7));        
        return -1;
#endif
        pd = 0.0;
        td = 0.0; 
        v = 0.0; 
        d = LARGE_NUM; 
        goto POST_WORK; 
    }

    d = v0 / td; 
    pd = v0*td - 0.5*d*td*td;   // sig '-' caution
    v = v0 / pd; 
    d = d  / pd; 

POST_WORK:

    if(p_trg)
    {
        *p_trg = sig * pd; 
    }
    if(prof)
    {
        // end time of each section
        prof->t0 = 0.;
	    prof->t_1a = t; 
	    prof->t_ua = t; 
        prof->t_2a = t;  
	    prof->t_u  = t;  
        prof->t_1d = t;
        prof->t_ud = t+td;
        prof->t_2d = t+td;

        // end position of each section
        prof->p0 = 0.;
	    prof->p_1a = 0;
	    prof->p_ua = 0;
        prof->p_2a = 0;
	    prof->p_u  = 0;
        prof->p_1d = 0;
        prof->p_ud = 1.;
        prof->p_2d = 1.;

        // end vel of each section
	    prof->v0 = 0.;
        prof->v_1a = 0.;
	    prof->v_ua = 0.; 
        prof->v_2a = 0.;
	    prof->v_u  = v;
        prof->v_1d = v;
        prof->v_ud = 0.;
        prof->v_2d = 0.;

        // end acc of each section
	    prof->a0 = 0.; 
        prof->a_1a = 0.;
	    prof->a_ua = 0.;
        prof->a_2a = 0.; 
	    prof->a_u  = 0.;
        prof->a_1d = -d;
        prof->a_ud = -d;
        prof->a_2d = 0.;

        // end jerk of each section
        prof->j0 = 0.; 
	    prof->j_1a = 0.;   
	    prof->j_ua = 0.;   
        prof->j_2a = 0.;   
	    prof->j_u  = 0.; 
        prof->j_1d = 0.;
        prof->j_ud = 0.;
        prof->j_2d = 0.;
    }
    return 0;
}
#endif 

#include "taskexec_def.h"
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
#if 0
double g_temp_d; 
double g_temp_ud; 
double g_temp_tu; 
double g_temp_t; 
#endif

void Prof7_Stop(PROFILE7* prof, double p_trg, double dec, double t)
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
    // 0 == prof->t_u : Decel Profile, No Touch
    if(p_trg == 0 || (0 < prof->t_u && t < EPSILON))
    {
        // memset(prof, 0, sizeof(PROFILE7));         
        ZeroProfile(prof, 0., t); 
        return;         
    }

    // normalization of dec & comparison 
    d = dec / fabs(p_trg); 

#if 0   // 'd <= fabs(prof->a_ud)' makes re-stopping problem. 
    // decel is smaller & decel mode is already, no work
    if(d <= fabs(prof->a_ud) && prof->t_u <= t)
    {   
        g_temp_d = d; 
        g_temp_ud = prof->a_ud; 
        g_temp_tu = prof->t_u; 
        g_temp_t = t; 
        return;
    }
#else
    // decel mode is already, no work
    if(prof->t_u <= t)
    {   
#if 0
        g_temp_d = d; 
        g_temp_ud = prof->a_ud; 
        g_temp_tu = prof->t_u; 
        g_temp_t = t; 
#endif
        return;
    }
#endif

    // Get Decel Info's
    p0 = Prof7_Get(prof, &v0, NULL, t); 
    td = v0 / d; 
    pd = 0.5 * d * td * td; 
    
    // end time of each section
	prof->t_u  = t;  
    prof->t_1d = t;
    prof->t_ud = t+td;
    prof->t_2d = t+td;

    // end position of each section
	prof->p_u  = p0;
    prof->p_1d = p0;
    prof->p_ud = p0+pd;
    prof->p_2d = p0+pd;

    // end vel of each section
	prof->v_u  = v0;
    prof->v_1d = v0;
    prof->v_ud = 0.;
    prof->v_2d = 0.;

    // end acc of each section
	prof->a_u  = 0.;
    prof->a_1d = -d;
    prof->a_ud = -d;
    prof->a_2d = 0.;
    
    // end jerk of each section
    prof->j_u  = 0.; 
    prof->j_1d = 0.;
    prof->j_ud = 0.;
    prof->j_2d = 0.;
    
    return;     
}



// Returns the dynamics of PROFILE7 at time of 't'.
// Units are used as used for Prof7_Set(). 
// 
// prof: [i] Pointer to the Motion Profile Data PROFILE7
// v   : [o] Pointer to Velocity of Dynamics
// a   : [o] Pointer to Accel of Dynamics
// t   : [i] Time to get the dynamics on.
double Prof7_Get(const PROFILE7* prof, double* v, double* a, double time)
{   
    double p0, v0, a0, j, t, u;     

    // data validation 

    ASSERT_RETURN(prof, 0.0); 

    time = (time < 0)? 0 : time; 

    // Parameters for calc
    if(prof->t_2d < time)        // Motion Over
    {
        p0 = prof->p_2d;
        v0 = 0.; 
        a0 = 0.; 
        j  = 0.; 
        t  = time - prof->t_2d; 
    }
    else if(prof->t_ud < time) // Decel Section 
    {
        p0 = prof->p_ud;
        v0 = prof->v_ud; 
        a0 = prof->a_ud; 
        j  = prof->j_ud;
        t  = time - prof->t_ud; 
    }
    else if(prof->t_1d < time) 
    {
        p0 = prof->p_1d; 
        v0 = prof->v_1d; 
        a0 = prof->a_1d;
        j  = prof->j_1d;
        t  = time - prof->t_1d; 
    }
    else if(prof->t_u < time) 
    {
        p0 = prof->p_u; 
        v0 = prof->v_u; 
        a0 = prof->a_u; 
        j  = prof->j_u; 
        t  = time - prof->t_u; 
    }
    else if(prof->t_2a < time) 
    {
        p0 = prof->p_2a; 
        v0 = prof->v_2a; 
        a0 = prof->a_2a; 
        j  = prof->j_2a; 
        t  = time - prof->t_2a; 
    }
    else if(prof->t_ua < time) 
    {
        p0 = prof->p_ua; 
        v0 = prof->v_ua; 
        a0 = prof->a_ua; 
        j  = prof->j_ua; 
        t  = time - prof->t_ua; 
    }
    else if(prof->t_1a < time) 
    {
        p0 = prof->p_1a; 
        v0 = prof->v_1a; 
        a0 = prof->a_1a; 
        j  = prof->j_1a; 
        t  = time - prof->t_1a; 
    }
    else 
    {
        p0 = prof->p0; 
        v0 = prof->v0; 
        a0 = prof->a0; 
        j  = prof->j0; 
        t  = time - prof->t0;
    }

    // outputs 
    u= p0 + v0*t + 0.5*a0*t*t + j*t*t*t/6.; 

    if(v != NULL)
    {
        *v = v0 + a0*t + 0.5*j*t*t; 
    }
    if(a != NULL)
    {
        *a = a0 + j*t; 
    }
    
    return u;  
}

// Gets the section of PROFILE7 at time of 't'.
// 
// RET : Profile Section of 't'
// prof: [i] Pointer to the Motion Profile Data PROFILE7
// t   : [i] Time to get the dynamics on.
//
int Prof7_GetSect(const PROFILE7* prof, double time)
{   
    int sec; 

    if(prof == NULL || time < 0)
    {
        return PROF7_SECT_NONE; 
    }
    
    time = (time < 0)? 0 : time; 

    // Parameters for calc
    if(prof->t_2d < time)       // Motion Over
    {
        sec= PROF7_SECT_OVER; 
    }
    else if(prof->t_ud < time)  // Decel Section 
    {   
        sec= PROF7_SECT_DEC_J2;
    }
    else if(prof->t_1d < time) 
    {
        sec= PROF7_SECT_DEC_U;
    }
    else if(prof->t_u < time) 
    {
        sec= PROF7_SECT_DEC_J1;
    }
    else if(prof->t_2a < time) 
    {
        sec= PROF7_SECT_UNI;
    }
    else if(prof->t_ua < time) 
    {
        sec= PROF7_SECT_ACC_J2;
    }
    else if(prof->t_1a < time) 
    {
        sec= PROF7_SECT_ACC_U;
    }
    else 
    {
        sec= PROF7_SECT_ACC_J1;
    }
    return sec; 
}
       
// Returns sync profile between prof1 & prof2
// prof1 & prof2 must be before called SCURVE_SetStop(..).
PROFILE7 Prof7_GetSync(const PROFILE7* prof1, const PROFILE7* prof2)
{	
#if 1
    if(!prof1 || !prof2)
    {
        PROFILE7 prof; 
        memset(&prof, 0, sizeof(prof)); 
        return prof; 
    }
    else
    {
        return (prof1->t_2d > prof2->t_2d)? *prof1 : *prof2; 
    }
#else
    PROFILE7 s;

	double dbTemp1, dbTemp2; 
	double ta, td, tu;	//interval of each sect 
	double pa, pd, pu;			//position of each sect	
    double v, a, d;     

    memset(&s, 0, sizeof(PROFILE7));
    
    // dec normalization needed. 

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
#endif
	
}

static void ZeroProfile(PROFILE7* prof, double p0, double t0)
{
    ASSERT_RETURN(prof, ;); 
    
    memset(prof, 0, sizeof(PROFILE7)); 

    prof->t0   = t0; 
    prof->t_1a = t0; 
    prof->t_ua = t0; 
    prof->t_2a = t0; 
    prof->t_u  = t0; 
    prof->t_1d = t0; 
    prof->t_ud = t0; 
    prof->t_2d = t0; 

    prof->p0   = p0; 
    prof->p_1a = p0; 
    prof->p_ua = p0; 
    prof->p_2a = p0; 
    prof->p_u  = p0; 
    prof->p_1d = p0; 
    prof->p_ud = p0; 
    prof->p_2d = p0; 
}
