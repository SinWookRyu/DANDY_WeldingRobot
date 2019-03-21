// RUN_JOG.C is Runtime Service Jog Mode Program. 
// RunServ_Jog
#include "taskexec_def.h"
#include "float.h"

#define JOG_DIST_JOINT(axis_)   (100000 * g_vel_axis_max[(axis_)])  // max speed 100s moving dist
#define JOG_DIST_LIN            (100000 * g_vel_lin_max)    
#define JOG_DIST_ORI            (100000 * g_vel_ori_max)

////////////////////////////////////////////////////////////////////////////////

RUNPARAM_JOG jog_param; 

static int JogPlan_Joint(const RUNPARAM_JOG* jog);
static int JogPlan_Cart(const RUNPARAM_JOG* jog); 

// Jog Control Var's
static int   s_mode = 0;    // Idle Status. Idle(0), Running(Else)
static int   s_stop = 0;    // Stop Request Flag
static int   s_mod_stop = 0;// Stop Mode. 0:Normal, 1:Quick
static int   s_dog = 0; 	// [ms] watchdog time (jog_keep<s_dog, Stops Jog)

////////////////////////////////////////////////////////////////////////////////

// Inits Jog Service
// If Jog is already running, Watch-dog 's_dog' is reset.

int RunServJog_Init(const RUNPARAM_JOG* jog)
{	
    int ret = 0; 
        
    ASSERT_RETURN(jog, -1); 
    
    if(ROB_AXIS_COUNT <= jog->axis)
    {        
        return ERR_INVALID_AXIS; 
    }

    // Jog Planning 
    if(jog->coord == COORDINDEX_JOINT)
    {
        ret = JogPlan_Joint(jog);        
    }
    else
    {
        ret = JogPlan_Cart(jog); 
    }

    if(!ret)
    {
        s_mode = 1; 
        s_stop = 0; 
        s_mod_stop = 0; 
        s_dog  = 0;
        jog_param = *jog;         
    }

    return ret; 
}

// return : 0(running), ELSE(end of process)
int RunServJog_Update(void)
{
    // Jog Idle Check
    if(s_mode == 0)
    {        
        return 1; 
    }

    // End of Jog Check. 
    if(Traj_Update(&g_traj))
    {
        ERROR_SET(SECT_JOG, ERRCODE_TRAJ(g_traj.error)); 
        s_mode = 0; 
        return -1;  // end of Trajectory
    }

    // Update Jog Watch-dog
	s_dog += g_t_samp;   

    // error, quick stop
    if(g_error.code)
    {        
        Traj_Stop(&g_traj, 1);
    }
    // Watchdog Exfired 
    if(jog_param.keep <= s_dog)
    {     
        Traj_Stop(&g_traj, 0);
    }
    if(s_stop)
    {     
        Traj_Stop(&g_traj, s_mod_stop);        
    }

    return 0;
}

// Stop Request
void RunServJog_Stop(int f_quick)
{
    s_stop = 1;
    s_mod_stop = (f_quick)? 1:0; 
}

// Resets Jog Watch-dog
void Jog_WatchDogReset(void)
{
	s_dog = 0;
}

////////////////////////////////////////////////////////////////////////////////

static int JogPlan_Joint(const RUNPARAM_JOG* jog)    
{
    double pos_trg[6], v[6];     
    unsigned axis; 
    unsigned ret; 

    ASSERT_RETURN(jog, -1); 

    // get virtual pos
    axis = jog->axis;
    memcpy(pos_trg, g_pos_act, sizeof(pos_trg)); 
    pos_trg[axis] = g_pos_act[axis] + jog->dir * JOG_DIST_JOINT(axis);
    v[0] = jog->vel; v[1] = jog->vel; v[2] = jog->vel; 
    v[3] = jog->vel; v[4] = jog->vel; v[5] = jog->vel; 
    
    ret = Traj_Set_Jmot(&g_traj, g_pos_act, pos_trg, v);
    return ERRCODE_TRAJ(ret); 
}

static int JogPlan_Cart(const RUNPARAM_JOG* jog)    
{   
    unsigned conf_trg; 
    double pos, rot; 

    ASSERT_RETURN(jog, -1); 
    
    conf_trg = g_Config(g_pos_act, g_dh); 
#if 0
    pos = (POS_Norm(jog->s_lin) < EPS_T)? 0 : JOG_DIST_LIN; 
    rot = (POS_Norm(jog->s_rot) < EPS_T)? 0 : JOG_DIST_ORI; 
#endif 
    pos = (POS_Norm(jog->s_lin) < EPS_P)? 0 : JOG_DIST_LIN; 
    rot = (POS_Norm(jog->s_rot) < EPS_O)? 0 : JOG_DIST_ORI; 

    return Traj_Set_Lmot(&g_traj, g_pos_act, 
                &jog->s_lin, pos, jog->v_lin, 
                &jog->s_rot, rot, jog->v_rot, 
                conf_trg); 
}