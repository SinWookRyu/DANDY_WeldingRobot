// CMD_COORD.C is CMD Program related to the coordinate management. 
#include "taskexec_def.h"
#include "coord.h"
#include "arg.h"

int Cmd_Set_GETP(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_GETPOS* arg; 
    ROBOT_POS rpos; 
    double offset[6]; 
    int coord; 
    int ret; 

    arg = &g_rcmd[index].arg.argGetPos; 

    // Check Ref-Coord & Element Expression are Same. If not Error Return. 
    
    // Get Ref Coord
    ret = Arg_Scalar(&coord, NULL, &arg->valRefCoord); 
    if(ret)
    {
        return ret; 
    }

    // Coord & Element Expression Type must be Same. 
    // coord == COORD_CART(BASE, WLD..) && nOffsetType == DANDY_JOB_OFFSET_CART
    // coord == COORD_JOINT             && nOffsetType == DANDY_JOB_OFFSET_JOINT
    if(coord == COORDINDEX_JOINT && arg->nOffsetType != DANDY_JOB_OFFSET_JOINT)
    {
        return ERR_NO_JNT_ACCESS; 
    }
    if(coord != COORDINDEX_JOINT && arg->nOffsetType == DANDY_JOB_OFFSET_JOINT)
    {
        return ERR_NO_CART_ACCESS; 
    }

    // Joint Case 
    if(coord == COORDINDEX_JOINT)
    {
        Arg_Scalar(NULL, &offset[0], &arg->offset.rgJoint[0]); 
        Arg_Scalar(NULL, &offset[1], &arg->offset.rgJoint[1]); 
        Arg_Scalar(NULL, &offset[2], &arg->offset.rgJoint[2]); 
        Arg_Scalar(NULL, &offset[3], &arg->offset.rgJoint[3]); 
        Arg_Scalar(NULL, &offset[4], &arg->offset.rgJoint[4]); 
        Arg_Scalar(NULL, &offset[5], &arg->offset.rgJoint[5]); 

        Deg2Rad(offset, 6); 

        rpos.pos[0] = g_pos_trg[0] + offset[0]; 
        rpos.pos[1] = g_pos_trg[1] + offset[1]; 
        rpos.pos[2] = g_pos_trg[2] + offset[2]; 
        rpos.pos[3] = g_pos_trg[3] + offset[3]; 
        rpos.pos[4] = g_pos_trg[4] + offset[4]; 
        rpos.pos[5] = g_pos_trg[5] + offset[5]; 

        rpos.nConfig = DANDY_JOB_POS_JOINT; 

        Rad2Deg(rpos.pos, 6); 

        return Arg_Set_RobPos(&arg->valPos, &rpos);         
    }
    // Cart Case 
    else
    {
        TRANS bTt, bTc, cTb; 
        XYZRPY xyzrpy; 
        
        // Change Coord cTt = cTb * bTt
        
        // cTb
        bTc = TargetCoordWrtBase(coord);  // base_T_coord  
        cTb = TRANS_Inv(bTc); 
        
        // bTt
        bTt = TRANS_Multi_TRANS(g_bTe, g_traj.eTt); 

        // TRANS -> xyzrpy
        xyzrpy = TRANS_GetXyzrpy(TRANS_Multi_TRANS(cTb, bTt)); 

        // Offset 
        Arg_Scalar(NULL, &offset[0], &arg->offset.cart.valX); 
        Arg_Scalar(NULL, &offset[1], &arg->offset.cart.valY); 
        Arg_Scalar(NULL, &offset[2], &arg->offset.cart.valZ); 
        Arg_Scalar(NULL, &offset[3], &arg->offset.cart.valRX); 
        Arg_Scalar(NULL, &offset[4], &arg->offset.cart.valRY); 
        Arg_Scalar(NULL, &offset[5], &arg->offset.cart.valRZ); 
        
        // Deg -> Rad
        Deg2Rad(&offset[3], 3); 

        // xyzrpy 
        rpos.pos[0] = xyzrpy.x + offset[0]; rpos.pos[3] = xyzrpy.roll  + offset[3]; 
        rpos.pos[1] = xyzrpy.y + offset[1]; rpos.pos[4] = xyzrpy.pitch + offset[4]; 
        rpos.pos[2] = xyzrpy.z + offset[2]; rpos.pos[5] = xyzrpy.yaw   + offset[5]; 

        rpos.nConfig = DANDY_JOB_POS_CART; 

        // RAD -> DEG
        Rad2Deg(&rpos.pos[3], 3); 
        return Arg_Set_RobPos(&arg->valPos, &rpos);  
    }
}