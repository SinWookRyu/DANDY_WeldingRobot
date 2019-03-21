#include "float.h"
#include "stdio.h"
#include "memory.h"
#include "motion.h"
#include "utility.h"

// 1 Axis Motion Initialization
//[ret] suc(0)
int Jmot_Set(JMOT* mot,             //[out]                                    
             double JointS,         //[in ] Start Joint Values
             double JointF,         //[in ] Final Joint Values                 
             double Speed)          //[rad/ms] (>0) 
{      
    if(mot == NULL)
    {
        return -1; 
    }

    mot->dbJointS = JointS; 
    mot->dbJointD = JointF - JointS; 
    mot->dbSpeed  = Speed; 
    
    return 0;
}

// Returns joint corresponding to 'u' param.
double Jmot_JointOnParam(const JMOT* mot, double u)
{   
    return (mot)? mot->dbJointS + u*mot->dbJointD : mot->dbJointS; 
}

#if 0
int Cmot_SetKin(CMOT* lmot,                 // [out]
                int (*Inverse)(double* th, const TRANS* bTe, unsigned conf, const DH dh[6], const double* th_prev), 
                const DH dh[6], 
                const TRANS* eTt)
{
    ASSERT_RETURN(Inverse, -1);     
    ASSERT_RETURN(eTt, -1);
    ASSERT_RETURN(dh, -1); 

    lmot->Inverse = Inverse; 
    lmot->eTt = *eTt;
    memcpy(lmot->dh, dh, sizeof(lmot->dh)); 
    
    return 0; 
}
#endif

#if 0
// To Set LIN Mot Fix, Set Norm of 'Sp' 0 or 'dbDispPris' 0
// To Set ROT Mot Fix, Set Norm of 'Sp' 0 or 'dbAngRot' 0
int Cmot_SetLinMot(CMOT* lmot,              // [out]
                   const POS* Ps,           // Start Point
                   const POS* Sp,           // Prismatic Dir. 
                   double dbDispPris,       // Prismatic Move Distance
                   const ROT* Rs,           // Start ROT
                   const POS* Sr,           // Rotation Dir
                   double dbAngRot,         // Rotation Angle 
                   int (*Inverse)(double* th, const TRANS* bTe, unsigned conf, const DH dh[6], const double* th_prev), 
                   const DH dh[6],          // dh parameter
                   const TRANS* eTt,        // END->TCP. If Null, Eye is used.  
                   unsigned conf)           // Robot Configuration
{
    int i; 
    TRANS EYE; 

    EYE = TRANS_Eye(); 

    ASSERT_RETURN(Ps, -1); 
    ASSERT_RETURN(Sp, -1); 
    ASSERT_RETURN(Rs, -1); 
    ASSERT_RETURN(Sr, -1); 
    ASSERT_RETURN(Inverse, -1); 
    ASSERT_RETURN(dh, -1); 

    if(lmot != NULL)
    {
        // memset(pCmot, 0, sizeof(PCMOT));
        
        lmot->Ps = *Ps;
        lmot->Sp = *Sp;            
        lmot->dist = dbDispPris;
        lmot->Rs = *Rs;
        lmot->Sr = *Sr;
        lmot->ang = dbAngRot;  
        lmot->Inverse = Inverse; 
        lmot->eTt = (eTt)? *eTt : EYE; 
        lmot->conf = conf; 
        for(i=0 ; i<6 ; i++)
        {
            lmot->dh[i] = dh[i]; 
        }
    }
    // check 'Sp' vector Unit
    if(POS_Norm(lmot->Sp) <= EPSILON)
    {
        lmot->dist = 0; 

        lmot->Sp.x = 0; 
        lmot->Sp.y = 0; 
        lmot->Sp.z = 1.0; 
    }
    POS_Unit(&lmot->Sp); 

    // check 'Sr' vector Unit
    if(POS_Norm(lmot->Sr) <= EPSILON)
    {
        lmot->ang = 0; 

        lmot->Sr.x = 0; 
        lmot->Sr.y = 0; 
        lmot->Sr.z = 1.0; 
    }
    POS_Unit(&lmot->Sr); 
    return 0;
}
#elif 0
// To Set LIN Mot Fix, Set Norm of 'Sp' 0 or 'dbDispPris' 0
// To Set ROT Mot Fix, Set Norm of 'Sp' 0 or 'dbAngRot' 0
int Cmot_SetLinMot(CMOT* lmot,              // [out]
                   const POS* Ps,           // Start Point
                   const POS* Sp,           // Prismatic Dir. 
                   double dbDispPris,       // Prismatic Move Distance
                   const ROT* Rs,           // Start ROT
                   const POS* Sr,           // Rotation Dir
                   double dbAngRot,         // Rotation Angle                    
                   unsigned conf)           // Robot Configuration
{   
    ASSERT_RETURN(Ps, -1); 
    ASSERT_RETURN(Sp, -1); 
    ASSERT_RETURN(Rs, -1); 
    ASSERT_RETURN(Sr, -1); 
    
    if(lmot != NULL)
    {
        // memset(pCmot, 0, sizeof(PCMOT));
        
        lmot->Ps = *Ps;
        lmot->Sp = *Sp;            
        lmot->dist = dbDispPris;
        lmot->Rs = *Rs;
        lmot->Sr = *Sr;
        lmot->ang = dbAngRot;  
        lmot->conf = conf;         
    }
    // check 'Sp' vector Unit
    if(POS_Norm(lmot->Sp) <= EPSILON)
    {
        lmot->dist = 0; 

        lmot->Sp.x = 0; 
        lmot->Sp.y = 0; 
        lmot->Sp.z = 1.0; 
    }
    POS_Unit(&lmot->Sp); 

    // check 'Sr' vector Unit
    if(POS_Norm(lmot->Sr) <= EPSILON)
    {
        lmot->ang = 0; 

        lmot->Sr.x = 0; 
        lmot->Sr.y = 0; 
        lmot->Sr.z = 1.0; 
    }
    POS_Unit(&lmot->Sr); 
    return 0;
}

// calculates position(lin motion) on parameter(u)    
void Cmot_PosOnParam(POS *pPos,                 //[out] displacement for position on 'u'
                     const CMOT* mot,           //[in ] CMOT info's
                     double u)                  //[in ] parameter 'u'.    
{   
    POS p; 

    ASSERT_RETURN(mot, ;); 

    p.x = mot->Ps.x + mot->Sp.x * mot->dist * u;
    p.y = mot->Ps.y + mot->Sp.y * mot->dist * u;          
    p.z = mot->Ps.z + mot->Sp.z * mot->dist * u;
    
    if(pPos != NULL)
    {
        *pPos = p;
    }
}

// calculates ROT on parameter(u)   
void Cmot_RotOnParam(ROT *pR,                   //[out] 3x3 rotational matrix on 'u'
                     const CMOT* mot,           //[in ] CMOT Info's.
                     double u)                  //[in ] parameter 'u'
{
    ROT Rtarget, Rscrew;
        
    ASSERT_RETURN(mot, ;); 

    // R_target(u) = Rscrew(u) * R_start.
    Rscrew  = ROT_Screw(mot->Sr, u*mot->ang);
    Rtarget = ROT_Multi_ROT(Rscrew, mot->Rs);    
    
    if(pR != NULL)
    {
        *pR = Rtarget;
    }
}
#elif 1
// To Set LIN Mot Fix, Set Norm of 'Sp' 0 or 'dbDispPris' 0
// To Set ROT Mot Fix, Set Norm of 'Sp' 0 or 'dbAngRot' 0
int Lmot_Set(LMOT* lmot,              // [out]
             const POS* Ps,           // Start Point
             const POS* Sp,           // Prismatic Dir. 
             double dbDispPris,       // Prismatic Move Distance
             double v_p,              // [mm/ms] Translational Vel
             const ROT* Rs,           // Start ROT
             const POS* Sr,           // Rotation Dir
             double dbAngRot,         // Rotation Angle                                 
             double v_r)              // [rad/ms] Orientational Vel
{   
    ASSERT_RETURN(Ps, -1); 
    ASSERT_RETURN(Sp, -1); 
    ASSERT_RETURN(Rs, -1); 
    ASSERT_RETURN(Sr, -1); 
    
    if(lmot != NULL)
    {
        // memset(pCmot, 0, sizeof(PCMOT));
        
        lmot->Ps = *Ps;
        lmot->Sp = *Sp;            
        lmot->dist = dbDispPris;
        lmot->Rs = *Rs;
        lmot->Sr = *Sr;
        lmot->ang = dbAngRot;  
    }
    // check 'Sp' vector Unit
    if(POS_Norm(lmot->Sp) <= EPSILON)
    {
        lmot->dist = 0; 

        lmot->Sp.x = 0; 
        lmot->Sp.y = 0; 
        lmot->Sp.z = 1.0; 
    }
    POS_Unit(&lmot->Sp); 

    // check 'Sr' vector Unit
    if(POS_Norm(lmot->Sr) <= EPSILON)
    {
        lmot->ang = 0; 

        lmot->Sr.x = 0; 
        lmot->Sr.y = 0; 
        lmot->Sr.z = 1.0; 
    }
    POS_Unit(&lmot->Sr); 
    return 0;
}

// calculates position(lin motion) on parameter(u)    
void Lmot_PosOnParam(POS *pPos,                 //[out] displacement for position on 'u'
                     const LMOT* mot,           //[in ] CMOT info's
                     double u)                  //[in ] parameter 'u'.    
{   
    POS p; 

    ASSERT_RETURN(mot, ;); 

    p.x = mot->Ps.x + mot->Sp.x * mot->dist * u;
    p.y = mot->Ps.y + mot->Sp.y * mot->dist * u;          
    p.z = mot->Ps.z + mot->Sp.z * mot->dist * u;
    
    if(pPos != NULL)
    {
        *pPos = p;
    }
}

// calculates ROT on parameter(u)   
void Lmot_RotOnParam(ROT *pR,                   //[out] 3x3 rotational matrix on 'u'
                     const LMOT* mot,           //[in ] CMOT Info's.
                     double u)                  //[in ] parameter 'u'
{
    ROT Rtarget, Rscrew;
        
    ASSERT_RETURN(mot, ;); 

    // R_target(u) = Rscrew(u) * R_start.
    Rscrew  = ROT_Screw(mot->Sr, u*mot->ang);
    Rtarget = ROT_Multi_ROT(Rscrew, mot->Rs);    
    
#if 0 // PRINT
    if(isnan(mot->Sr.x))
    {
        printf("SCREW x:%f y:%f z:%f ang:%f u:%f\n", mot->Sr.x,mot->Sr.y, mot->Sr.z, mot->ang, u); 
    }
#endif 

    if(pR != NULL)
    {
        *pR = Rtarget;
    }
}

// Circular Motion Setting
int Cmot_Set(CMOT* cmot,              // [out]             
             const POS* Pc,           // Center POS
             const POS* Ps,           // Start POS of TCP wrt BASE 
             const POS* Sp,           // Translation Direction Vector
             double arc,              // Translation Distance(Arc of Circle)
             double v_p,              // [mm/ms] Linear Vel
             const ROT* Rs,           // Start ROT of TCP wrt BASE
             const POS* Sr,           // Orientation Dir
             double ang,              // Rotation Angle       
             double v_o)              // [rad/ms] Roatational Vel
{     
    ASSERT_RETURN(Ps, -1); 
    ASSERT_RETURN(Pc, -1); 
    ASSERT_RETURN(Sp, -1); 
    ASSERT_RETURN(Rs, -1); 
    ASSERT_RETURN(Sr, -1); 
    
    if(cmot != NULL)
    {
        // memset(pCmot, 0, sizeof(PCMOT));
        
        cmot->Ps = *Ps;
        cmot->Pc = *Pc;   
        cmot->Sp = *Sp;                    
        cmot->arc = arc;
        cmot->Rs = *Rs;
        cmot->Sr = *Sr;
        cmot->ang = ang;  
        cmot->v_p = v_p; 
        cmot->v_o = v_o; 
    }
    // check 'Sp' vector Unit
    if(POS_Norm(cmot->Sp) <= EPSILON)
    {
        cmot->arc = 0; 
        cmot->Sp.x = 0; 
        cmot->Sp.y = 0; 
        cmot->Sp.z = 1.0; 
    }
    POS_Unit(&cmot->Sp); 

    // check 'Sr' vector Unit
    if(POS_Norm(cmot->Sr) <= EPSILON)
    {
        cmot->ang = 0; 
        cmot->Sr.x = 0; 
        cmot->Sr.y = 0; 
        cmot->Sr.z = 1.0; 
    }
    POS_Unit(&cmot->Sr); 

    cmot->rad = POS_Norm(POS_Minus_POS(cmot->Pc, cmot->Ps)); 
    return 0;
}

// calculate translation position on parameter(u)     
// p_target(u) = T_screw(u) * p_start
void Cmot_PosOnParam(POS *pos,                  //[out] displacement for position on 'u'
                     const CMOT* mot,           //[in ] CMOT info's
                     double u)                  //[in ] parameter 'u'.  
{       
    TRANS  T; 


    double ang; 

    ASSERT_RETURN(mot, ;); 
    ASSERT_RETURN(pos, ;);

    ang = mot->arc * u / mot->rad; 
    T   = TRANS_Screw(mot->Sp, mot->Pc, ang, 0.0);
    *pos= TRANS_Multi_POS(T, mot->Ps);        
}    


// calculates ROT on parameter(u)   
void Cmot_RotOnParam(ROT *pR,                   //[out] 3x3 rotational matrix on 'u'
                     const CMOT* mot,           //[in ] CMOT Info's.
                     double u)                  //[in ] parameter 'u'
{
    ROT Rtarget, Rscrew;
        
    ASSERT_RETURN(mot, ;); 

    // R_target(u) = Rscrew(u) * R_start.
    Rscrew  = ROT_Screw(mot->Sr, u*mot->ang);
    Rtarget = ROT_Multi_ROT(Rscrew, mot->Rs);    
    
    if(pR != NULL)
    {
        *pR = Rtarget;
    }
}

#else
// To Set LIN Mot Fix, Set Norm of 'Sp' 0 or 'dbDispPris' 0
int Lmot_Set(LMOT* lmot,              // [out]
             const POS* Ps,           // Start Point
             const POS* Sp,           // Prismatic Dir. 
             double dbDispPris)       // Prismatic Move Distance                   
{   
    ASSERT_RETURN(Ps, -1); 
    ASSERT_RETURN(Sp, -1);     

    if(lmot != NULL)
    {
        // memset(pCmot, 0, sizeof(PCMOT));
        
        lmot->Ps = *Ps;
        lmot->Sp = *Sp;            
        lmot->dist = dbDispPris;        
    }
    // check 'Sp' vector Unit
    if(POS_Norm(lmot->Sp) <= EPSILON)
    {
        lmot->dist = 0; 

        lmot->Sp.x = 0; 
        lmot->Sp.y = 0; 
        lmot->Sp.z = 1.0; 
    }
    POS_Unit(&lmot->Sp); 
    return 0;
}


// calculates position(lin motion) on parameter(u)    
void Lmot_PosOnParam(POS *pPos,                 //[out] displacement for position on 'u'
                     const LMOT* mot,           //[in ] CMOT info's
                     double u)                  //[in ] parameter 'u'.    
{   
    POS p; 

    ASSERT_RETURN(mot, ;); 

    p.x = mot->Ps.x + mot->Sp.x * mot->dist * u;
    p.y = mot->Ps.y + mot->Sp.y * mot->dist * u;          
    p.z = mot->Ps.z + mot->Sp.z * mot->dist * u;
    
    if(pPos != NULL)
    {
        *pPos = p;
    }
}

// calculates ROT on parameter(u)   
void Rmot_RotOnParam(ROT *pR,                   //[out] 3x3 rotational matrix on 'u'
                     const RMOT* mot,           //[in ] CMOT Info's.
                     double u)                  //[in ] parameter 'u'
{
    ROT Rtarget, Rscrew;
        
    ASSERT_RETURN(mot, ;); 

    // R_target(u) = Rscrew(u) * R_start.
    Rscrew  = ROT_Screw(mot->Sr, u*mot->ang);
    Rtarget = ROT_Multi_ROT(Rscrew, mot->Rs);    
    
    if(pR != NULL)
    {
        *pR = Rtarget;
    }
}

// To Set ROT Mot Fix, Set Norm of 'Sp' 0 or 'dbAngRot' 0
int Rmot_Set(RMOT* rmot,              // [out]
             const ROT* Rs,           // Start ROT
             const POS* Sr,           // Rotation Dir
             double dbAngRot)         // Rotation Angle 
{   
    ASSERT_RETURN(Rs, -1); 
    ASSERT_RETURN(Sr, -1); 
    
    if(rmot != NULL)
    {
        rmot->Rs = *Rs;
        rmot->Sr = *Sr;
        rmot->ang = dbAngRot;          
    }

    // check 'Sr' vector Unit
    if(POS_Norm(rmot->Sr) <= EPSILON)
    {
        rmot->ang = 0; 

        rmot->Sr.x = 0; 
        rmot->Sr.y = 0; 
        rmot->Sr.z = 1.0; 
    }
    POS_Unit(&rmot->Sr); 
    return 0;
}
#endif

#if 0
// calculate translation position on parameter(u)     
// p_target(u) = T_screw(u) * p_start
void Cir_PosOnParam(POS*         pos,    //[out] pos on 'u'
                   const CIR*   mot,    //[in ] Circular Motion Info's.
                   double       u)      //[in ] Profile Progress. 
{       
    TRANS  T; 
    double ang; 

    ASSERT_RETURN(mot, -1); 
    ASSERT_RETURN(jnt, -1);

    ang = mot->arc * u / mot->rad; 
    T = TRANS_Screw(mot->s_p, mot->cent, ang, 0.0);
    p = TRANS_Multi_POS(T, mot->Ps);        

    return 0; 
}    
#endif

#if 0
// calculate joint value on parameter(u)     
// Ret Suc(0), INVALID_PARAM, SINGULAR_NEAR_Z0, SINGULAR_TH5, UNREACHABLE
int Cmot_JointOnParam(double *pJoint,         //[out] joints value on parameter u
                      const CMOT* mot,        //[in ] CMOT Info's.
                      double u_pos,           //[in]  linear increment. p(i) = ps + u(i)*dist_lin;    
                      double u_rot,           //[in]  orient increment. r(i) = rs + u(i)*dist_rot;  
                      const double* j_prev)   //[in] Prev. Joint
{
    TRANS bTt, bTe;

    ASSERT_RETURN(mot, -1); 

    Cmot_RotOnParam(&bTt.R, mot, u_rot);
    Cmot_PosOnParam(&bTt.p, mot, u_pos);            
    bTe = TRANS_Multi_TRANS(bTt, TRANS_Inv(mot->eTt)); // bTe = bTt * tTe
    return (mot->Inverse)(pJoint, &bTe, mot->conf, mot->dh, j_prev);
}
#endif