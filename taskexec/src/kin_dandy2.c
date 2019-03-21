#include "math.h"
#include "float.h"
#include "geometry.h"
#include "kinematics.h"
#include "utility.h"
#include <stdio.h>

// 
void For_Dandy2(TRANS *bTe,             //[out]trans of base->6
                const double *th,      //[in] physical joint.
                const DH *dh)           //[in] dh-param
{ 
    double temp;
    double l1, l2, l3, d1, d4;     
    double c1, c2, c23, c4, c5, c6; 
    double s1, s2, s23, s4, s5, s6; 
    double u36x, u36y, u36z; 
    double v36x, v36y, v36z; 
    double w36x, w36y, w36z; 
    double ux, uy, uz; 
    double vx, vy, vz; 
    double wx, wy, wz; 
    double px, py, pz; 

    d1 = dh[0].d; 
    d4 = dh[3].d; 
    l1 = dh[0].l; 
    l2 = dh[1].l; 
    l3 = dh[2].l; 

    c1 = cos(th[0]); 
    s1 = sin(th[0]); 

    c2 = cos(th[1]); 
    s2 = sin(th[1]); 
   
    c23 = cos(th[1]+th[2]); 
    s23 = sin(th[1]+th[2]); 

    c4 = cos(th[3]); 
    c5 = cos(th[4]); 
    c6 = cos(th[5]); 

    s4 = sin(th[3]); 
    s5 = sin(th[4]); 
    s6 = sin(th[5]); 

    // 3R6
    u36x = c4*c5*c6 - s4*s6; 
    u36y = s4*c5*c6 + c4*s6; 
    u36z = s5*c6; 

    v36x = -c4*c5*s6 - s4*c6; 
    v36y = -s4*c5*s6 + c4*c6;   // Modified 20130911 : -s4*c5*c6 -> -s4*c5*s6
    v36z = -s5*s6; 

    w36x = -c4*s5; 
    w36y = -s4*s5;
    w36z = c5; 

    // 0R6
    temp = s23*u36x + c23*u36z; 
    ux = temp*c1 + u36y*s1; 
    uy = temp*s1 - u36y*c1; 
    uz = u36x*c23 - u36z*s23; 

    temp = s23*v36x + c23*v36z; 
    vx = temp*c1 + v36y*s1; 
    vy = temp*s1 - v36y*c1; 
    vz = v36x*c23 - v36z*s23; 

    temp = s23*w36x + c23*w36z; 
    wx = temp*c1 + w36y*s1; 
    wy = temp*s1 - w36y*c1; 
    wz = w36x*c23 - w36z*s23; 

    // 0p6
    temp = l1 + l2*s2 + l3*s23 + d4*c23; 
    px = temp*c1; 
    py = temp*s1; 
    pz = d1 +l2*c2 + l3*c23 - d4*s23; 

    // output
    if(bTe)
    {
        bTe->u.x = ux; 
        bTe->u.y = uy; 
        bTe->u.z = uz;

        bTe->v.x = vx;
        bTe->v.y = vy; 
        bTe->v.z = vz;

        bTe->w.x = wx; 
        bTe->w.y = wy; 
        bTe->w.z = wz; 

        bTe->p.x = px;
        bTe->p.y = py;
        bTe->p.z = pz; 
    }
}

// Returns Dandy2 Configuration
//   1st bit : ARM    : FRONT(1) | BACK(0)
//   2nd bit : ELBOW  : ABOVE(1) | BELOW(0)
//   3rd bit : FLIP   : FLIP(1)  | NON(0)
// Conf of Normal Below Looking Pose is 0x07. (ARM-Front, Elbow-Above, Flip)
unsigned Conf_Dandy2(const double* th, const DH *dh)
{
    double l1, l2, l3, d4; 
    double s2, s3, s23, s5, s4; 
    double c3, c4, c23; 
    double ARM, ELBOW, FLIP; 
    unsigned conf; 
    
    ASSERT_RETURN(th, 0x07); 
    ASSERT_RETURN(dh, 0x07); 

    l1 = dh[0].l; 
    l2 = dh[1].l;
    l3 = dh[2].l;
    d4 = dh[3].d;

    s2 = sin(th[1]); 
    s3 = sin(th[2]); 
    c3 = cos(th[2]); 
    s5 = sin(th[4]); 
    s4 = sin(th[3]); 
    c4 = cos(th[3]); 
    s23 = sin(th[1]+th[2]); 
    c23 = cos(th[1]+th[2]); 

    // descriminant
    ARM   = l1 + l2*s2 + l3*s23 + d4*c23; 
    ELBOW = l3*s3 + d4*c3; 
    FLIP  = s5; 

    // configuration
    conf = 0; 
    conf |= (ARM   >= 0)? 0x01 : 0x00; 
    conf |= (ELBOW >= 0)? 0x02 : 0x00; 
    conf |= (FLIP  >= 0)? 0x04 : 0x00; 

#if 0 // PRINT
    printf("th1:%f th2:%f th3:%f th4:%f l1:%f l2:%f l3:%f d4:%f\n", 
        th[1], th[2], th[3], th[4], l1, l2, l3, d4); 
    printf("ARM:%f ELB:%f FLP:%f conf:%x\n", ARM, ELBOW, FLIP, conf);  
#endif
    return conf; 
}



// Gets theta for trans. from base to 6th link, no regards to TCP.
// Ret Suc(0), INVALID_PARAM, SINGULAR_NEAR_Z0, SINGULAR_TH5, UNREACHABLE
// Robot Pose Configuration
// - ARM(1st bit)   : FRONT(1), BACK(0)
// - ELBOW(2nd bit) : ABOVE(1), BELOW(0)
// - WRIST(3rd bit) : FLIP(1),  NO-FLIP(0)
// - 0b111 : FRONT-ABOVE-FLIP - Normal Pose
// 
// When use this on motion running, set 'f_running' & on not running, reset Flag
// 0. The flag is used to prevent so large difference a calculation. So it is no 
// need for just calculating Joint value with CART. 
int Inv_Dandy2( double*      pTh,       // [out]physical joint angles.
                const TRANS* bTe,       // [in] trans from BASE to LINK6                                      
                unsigned     conf,      // [in] configuration of robot pose
                const DH*    pDh,       // [in] dh-param                
                const double th_prev[6],// [in] previouss Joint Value
                unsigned     f_running) // [in] running flag

{
    /*
    dh = [  0,      400,    -pi/2,  60 ; ...
            -pi/2,  0,      0,      350; ...
            0,      0,      -pi/2,  90 ; ...
            0,      390,    pi/2,   0  ; ...
            0,      0,      -pi/2,  0  ; ... 
            0,      0,      0,      0 ]; 
    */
    int    i, n[6]; 
    double rem[6]; 
    double ux, uy, uz; 
    double vx, vy, vz; 
    double wx, wy, wz; 
    double px, py, pz; 
    double l1, l2, l3; 
    double d1, d4; 
    double ARM, ELBOW, WRIST; 
    double c1, c3, c4, c5, c6, c23; 
    double s1, s3, s4, s5, s6, s23; 
    double th1, th2, th3, th4, th5, th6, th23; 
    double u, v, p, q, r; 
    double k1, k2; 
    double temp, den; 
    double th4_prev, th6_prev; 

    if(!pTh || !bTe || !pDh || !th_prev)
    {
        return ERR_INVALID_PARAM; 
    }


    // target transform matrix w.r.t base frame

    ux = bTe->u.x; 
    uy = bTe->u.y;
    uz = bTe->u.z;
    vx = bTe->v.x;
    vy = bTe->v.y;
    vz = bTe->v.z;
    wx = bTe->w.x;
    wy = bTe->w.y;
    wz = bTe->w.z;
    px = bTe->p.x;
    py = bTe->p.y;
    pz = bTe->p.z;
    
    l1 = pDh[0].l;
    l2 = pDh[1].l;
    l3 = pDh[2].l;
    d1 = pDh[0].d;
    d4 = pDh[3].d;

#define INV_MODULAR 1
#ifdef INV_MODULAR // multi turn modular
    // normalization to -pi ~ pi
    // th_prev[i] = 2*PI*n[i] + prev[i]
    {
        for(i=0 ; i<6 ; i++)
        {
            n[i] = (int)(th_prev[i] / (2*PI)); 
            rem[i] = th_prev[i] - ((2*PI) * n[i]); 
            if(PI <= rem[i])
            { 
                rem[i] = rem[i] - 2*PI; 
                n[i]++; 
            }
            if(rem[i] < -PI)
            {
                rem[i] = rem[i] + 2*PI; 
                n[i]--; 
            }
        }
    }

    th4_prev = rem[3]; 
    th6_prev = rem[5]; 
#else
    th4_prev = th_prev[3]; 
    th6_prev = th_prev[5]; 
#endif 

    // Robot Configuration
    ARM   = (conf & 0x01)? 1 : -1; 
    ELBOW = (conf & 0x02)? 1 : -1; 
    WRIST = (conf & 0x04)? 1 : -1; 
        
    // Finding th1 /////////////////////////////////////////////////////////////////
    
    // Singular Point th1 : P is near the z0 axis
    if(sqrt(px*px + py*py) < EPS_O)
    {
        return ERR_SINGULAR_NEAR_Z0; 
    }
    
    c1 = ARM*px; 
    s1 = ARM*py; 
    th1 = atan2(s1, c1); 
#if 0 // PRINT 
    if(isnan(th1))
    {
        printf("INV-0 %f %f %f \n", px, py, ARM); 
        return ERR_INVALID_PARAM; 
    }
#endif


    c1 = cos(th1); // normalization  
    s1 = sin(th1); 

    // Finding th3 /////////////////////////////////////////////////////////////////

    u = px*c1 + py*s1 - l1; 
    v = d1 - pz; 
    p = 2*l2*l3; 
    q =-2*l2*d4; 
    r = u*u + v*v - l2*l2 - l3*l3 - d4*d4; 

    // Unreachable Target Pos
    temp = p*p + q*q - r*r; 
    if(temp < 0)
    {
        return ERR_UNREACHABLE; 
    }

    temp= sqrt(temp);
    den = p*p + q*q; 

    c3 = (p*r - ELBOW*q*temp)/den;
    s3 = (q*r + ELBOW*p*temp)/den;    
    th3 = atan2(s3,c3);

    // normalization for another use
    c3= cos(th3); 
    s3= sin(th3);

    // Finding th23 & th2 //////////////////////////////////////////////////////////

    k1 = px*c1 + py*s1 - l1; 
    k2 = pz - d1; 
    p  = d4 - l2*s3; 
    q  = l3 + l2*c3; 
    den= p*p + q*q; 

    c23 = (k1*p + k2*q) / den; 
    s23 = (k1*q - k2*p) / den; 

    th23 = atan2(s23, c23); 
    th2 = th23 - th3; 

    // normalization
    c23 = cos(th23); 
    s23 = sin(th23); 

    // Finding th5 /////////////////////////////////////////////////////////////////

    p = wx*c1*s23 + wy*s1*s23 + wz*c23; 
    q = wx*s1 - wy*c1; 

    s5 = WRIST*sqrt(p*p + q*q); 
    c5 = wx*c1*c23 + wy*s1*c23 - wz*s23; 
    th5 = atan2(s5, c5); 

    // normalization
    c5 = cos(th5); 
    s5 = sin(th5); 

    // Finding th4 /////////////////////////////////////////////////////////////////

    if(sqrt(s5*s5) < EPS_O)  // Inv Err if so small th5. 
    {        
        return ERR_SINGULAR_TH5; 
    }

    c4 = -(wx*c1*s23 + wy*s1*s23 + wz*c23) / s5; 
    s4 = -(wx*s1 - wy*c1) / s5; 
    th4 = atan2(s4, c4);     

    c4 = cos(th4); 
    s4 = sin(th4); 
    
#ifndef INV_MODULAR
    // ///////////////////////////////////////////////
    // This Comparison Makes Efficient Singular Check. 
    // Singular Error is occured on time not in period. 
    if(fabs(th4 - th4_prev) >= PI/2)
    {
        return ERR_SINGULAR_TH5; 
    }
#endif
    // Finding th6 /////////////////////////////////////////////////////////////////

    c6 =  (ux*c1*c23 + uy*s1*c23 - uz*s23) / s5; 
    s6 = -(vx*c1*c23 + vy*s1*c23 - vz*s23) / s5; 
    th6 = atan2(s6, c6);

    c6 = cos(th6);
    s6 = sin(th6); 

#ifndef INV_MODULAR
    // ///////////////////////////////////////////////
    // This Comparison Makes Efficient Singular Check. 
    // Singular Error is occured on time not in period. 
    if(fabs(th6 - th6_prev) >= PI/2)
    {
        return ERR_SINGULAR_TH5; 
    }
#endif

    // Returns
    if(pTh)
    {
        pTh[0] = th1; 
        pTh[1] = th2; 
        pTh[2] = th3; 
        pTh[3] = th4; 
        pTh[4] = th5; 
        pTh[5] = th6; 
    }

#if 0 // PRINT 
    if(isnan(pTh[0]))
    {
        printf("INV-1 %f %f %f \n", th1, th2, th3); 
        return ERR_INVALID_PARAM; 
    }
#endif

#ifdef INV_MODULAR // multi-turn modular
    // unnormalization from -pi~pi
    for(i=0 ; i<6 ; i++)
    {        
        // another comparison is 'SOL +-2PI'
        temp   = (rem[i] <= pTh[i])? pTh[i]-(2*PI) : pTh[i]+(2*PI); 
        // Modular Target Value 
        pTh[i] = (fabs(pTh[i]-rem[i]) < fabs(temp-rem[i]))? pTh[i] : temp; 
        // Original Turn is applied to Modular. 
        pTh[i] = n[i]*2*PI + pTh[i]; 
    }

#if 0 // PRINT 
    if(isnan(pTh[0]))
    {
        printf("INV-2 REM : %f %f %f \n", rem[0], rem[1], rem[2]); 
        return ERR_INVALID_PARAM; 
    }

    // ///////////////////////////////////////////////
    // This Comparison Makes Efficient Singular Check. 
    // Singular Error is occured on time not in period. 
    // Though singular makes th[3]-th_prev[3] PI/2 or PI in ideal, We use PI/4 for real world.
    // 
    if(f_running)
    {
        if(fabs(pTh[3] - th_prev[3]) >= PI/4 || fabs(pTh[5] - th_prev[5]) >= PI/4)
        {
            return ERR_SINGULAR_TH5; 
        }
    
        for(i=0 ; i<6 ; i++)
        {
            if(fabs(pTh[i] - th_prev[i]) >= PI/10.)
            {
                return ERR_LARGE_VEL; 
            }
        }
    }
#endif
#endif 
    return 0; 
}
