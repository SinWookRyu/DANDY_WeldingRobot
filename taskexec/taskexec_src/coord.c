// COORD.C is Dandy2 Coordinate Management Program. 

#include "coord.h"
#include "arg.h"

#if 1
// Returns COORD wrt BASE. 
// CAUTION!! COORDINDEX_TCP uses bTe with Target Joint. 
TRANS TargetCoordWrtBase(int coord)
{
    const TRANS bTe = g_bTe; 

    switch(coord)
    {
    case COORDINDEX_JOINT:
    case COORDINDEX_BASE:    
        return TRANS_Eye();         

    case COORDINDEX_WORLD:
        return g_bTw; 
        
    case COORDINDEX_END:
        return bTe; 
    
    case COORDINDEX_TCP:
        // bP = bTt*eP = bTe*eTt*eP
        return TRANS_Multi_TRANS(bTe, g_eTt);
        
    case COORDINDEX_SENSOR:
        // bP = bTt*eP = bTe*eTt*tTs*sP        
        return TRANS_Multi_TRANS(TRANS_Multi_TRANS(bTe, g_eTt), g_tTs); 
        
    default:
        // User Coord
        // bP = bTu*uP = bTw*wTu*uP
        if(0 <= coord && coord < MAX_USER_COORD_COUNT)
        {
            XYZRPY user; 
            COORD_EULER src; 
            
            src = g_pshm_sys_conf->robot[0].user[coord];
            user.x = src.x; 
            user.y = src.y; 
            user.z = src.z;
            user.roll  = src.rol;  
            user.pitch = src.pit; 
            user.yaw   = src.yaw; 
                        
            return TRANS_Multi_TRANS(g_bTw, TRANS_Xyzrpy(user));    // bTw * wTu            
        }
        // default, No Transfrom
        else
        {
            return TRANS_Eye(); 
        }
    }
}

// Returns COORD wrt BASE. 
// CAUTION!! COORDINDEX_TCP uses bTe with Actual Joint. 
TRANS ActualCoordWrtBase(int coord)
{
    const TRANS bTe = g_bTe_act;

    switch(coord)
    {
    case COORDINDEX_JOINT:
    case COORDINDEX_BASE:    
        return TRANS_Eye();         

    case COORDINDEX_WORLD:
        return g_bTw; 
        
    case COORDINDEX_END:
        return bTe; 
    
    case COORDINDEX_TCP:
        // bP = bTt*eP = bTe*eTt*eP
        return TRANS_Multi_TRANS(bTe, g_eTt);
        
    case COORDINDEX_SENSOR:
        // bP = bTt*eP = bTe*eTt*tTs*sP        
        return TRANS_Multi_TRANS(TRANS_Multi_TRANS(bTe, g_eTt), g_tTs); 
        
    default:
        // User Coord
        // bP = bTu*uP = bTw*wTu*uP
        if(0 <= coord && coord < MAX_USER_COORD_COUNT)
        {
            XYZRPY user; 
            COORD_EULER src; 
            
            src = g_pshm_sys_conf->robot[0].user[coord];
            user.x = src.x; 
            user.y = src.y; 
            user.z = src.z;
            user.roll  = src.rol;  
            user.pitch = src.pit; 
            user.yaw   = src.yaw; 
                        
            return TRANS_Multi_TRANS(g_bTw, TRANS_Xyzrpy(user));    // bTw * wTu            
        }
        // default, No Transfrom
        else
        {
            return TRANS_Eye(); 
        }
    }
}

#else

// Returns COORD wrt BASE. 
// CAUTION!! COORDINDEX_TCP uses bTe with Actual bTe. 
TRANS CoordWrtBase(int coord, TRANS bTe)
{
    switch(coord)
    {
    case COORDINDEX_JOINT:
    case COORDINDEX_BASE:    
        return TRANS_Eye();         

    case COORDINDEX_WORLD:
        return g_bTw;         
        
    case COORDINDEX_END:
        return bTe; 
    
    case COORDINDEX_TCP:
        // bP = bTt*eP = bTe*eTt*eP
        return TRANS_Multi_TRANS(bTe, g_eTt);
        
    case COORDINDEX_SENSOR:
        // bP = bTt*eP = bTe*eTt*tTs*sP        
        return TRANS_Multi_TRANS(TRANS_Multi_TRANS(bTe, g_eTt), g_tTs); 
        
    default:
        // User Coord is described rwt WORLD
        // bP = bTu*uP = bTw*wTu*uP
        if(0 <= coord && coord < MAX_USER_COORD_COUNT)
        {
            XYZRPY user; 
            COORD_EULER src; 
            
            src = g_pshm_sys_conf->robot[0].user[coord];
            user.x = src.x; 
            user.y = src.y; 
            user.z = src.z;
            user.roll  = src.rol;  
            user.pitch = src.pit; 
            user.yaw   = src.yaw; 
                        
            return TRANS_Multi_TRANS(g_bTw, TRANS_Xyzrpy(user));    // bTw * wTu            
        }
        // default, No Transfrom
        else
        {
            return TRANS_Eye(); 
        }
    }
}
#endif

TRANS BaseWrtTargetCoord(int coord)
{
    // Inv(bTc)
    return TRANS_Inv(TargetCoordWrtBase(coord));     
}

#if 0
void TcpWrtTargetCoord(TRANS* T, int coord)
{
    if(!T)
    {
        return;
    }    
    *T = TRANS_Multi_TRANS(g_bTe, g_eTt);  // bTt
    *T = TRANS_Multi_TRANS(BaseWrtTargetCoord(coord), *T);    // cTt = cTb * bTt    
}
#endif
void TcpWrtTargetCoord(TRANS* T, int coord)
{
    if(!T)
    {
        return;
    }        
    *T = TRANS_Multi_TRANS(BaseWrtTargetCoord(coord), g_bTt);    // cTt = cTb * bTt    
}

TRANS BaseWrtActualCoord(int coord)
{
    // Inv(bTc)
    return TRANS_Inv(ActualCoordWrtBase(coord));     
}

#if 0
void TcpWrtActualCoord(TRANS* T, int coord)
{
    if(!T)
    {
        return;
    }    
    *T = TRANS_Multi_TRANS(g_bTe_act, g_eTt);  // bTt
    *T = TRANS_Multi_TRANS(BaseWrtActualCoord(coord), *T);    // cTt = cTb * bTt    
}
#endif
void TcpWrtActualCoord(TRANS* T, int coord)
{
    if(!T)
    {
        return;
    }        
    *T = TRANS_Multi_TRANS(BaseWrtActualCoord(coord), g_bTt_act);    // cTt = cTb * bTt    
}

// JOINT -> ROBPOS wrt JOINT
// ROBPOS is described by deg
void JOINT_2_ROBPOS(ROBOT_POS* rpos, const double* joint)
{    
    if(rpos && joint)
    {
        rpos->pos[0] = joint[0] * 180./PI;      
        rpos->pos[1] = joint[1] * 180./PI;  
        rpos->pos[2] = joint[2] * 180./PI;  
        rpos->pos[3] = joint[3] * 180./PI;  
        rpos->pos[4] = joint[4] * 180./PI;  
        rpos->pos[5] = joint[5] * 180./PI;  
        rpos->nConfig = DANDY_JOB_POS_JOINT;
    }    
}

#if 0
// bT wrt BASE -> ROBPOS wrt Coord 
// ROBPOS is described by mm and deg
void TRANS_2_ROBPOS(ROBOT_POS* rpos, int coord, const TRANS* bT)
{   
    TRANS cTb; 
    XYZRPY xyzrpy; 
        
    // Change Coord cTt = cTb * bTt
    cTb = BaseWrtCoord(coord); 
    
    // TRANS -> xyzrpy
    xyzrpy = TRANS_GetXyzrpy(TRANS_Multi_TRANS(cTb, *bT)); 

    // xyzrpy 
    rpos->pos[0] = xyzrpy.x; rpos->pos[3] = xyzrpy.roll  * 180./PI; 
    rpos->pos[1] = xyzrpy.y; rpos->pos[4] = xyzrpy.pitch * 180./PI; 
    rpos->pos[2] = xyzrpy.z; rpos->pos[5] = xyzrpy.yaw   * 180./PI; 
    rpos->nConfig = DANDY_JOB_POS_CART;     
}
#endif
// T-> ROBPOS. ROBPOS is described by mm and deg. T by mm and rad. 
void TRANS_2_ROBPOS(ROBOT_POS* rpos, const TRANS* T)
{   
    XYZRPY xyzrpy; 
        
    // TRANS -> xyzrpy
    xyzrpy = TRANS_GetXyzrpy(*T); 

    // xyzrpy 
    rpos->pos[0] = xyzrpy.x; rpos->pos[3] = xyzrpy.roll  * 180./PI; 
    rpos->pos[1] = xyzrpy.y; rpos->pos[4] = xyzrpy.pitch * 180./PI; 
    rpos->pos[2] = xyzrpy.z; rpos->pos[5] = xyzrpy.yaw   * 180./PI; 
    rpos->nConfig = DANDY_JOB_POS_CART;     
}
