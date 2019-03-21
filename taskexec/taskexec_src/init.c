////////////////////////////////////////////////////////////////////////////////
// INIT.C is initialzation program for TASKEXEC
// 2013-05-19 mrch0

#include "taskexec_def.h"
#include "te_serv.h"
#include "stdio.h"
#include "float.h"
#include "call_stack.h"
#include "sensor.h"

extern void RestartDataSet(UNT f_set, UNT mode_prog);  

////////////////////////////////////////////////////////////////////////////////

static void ConsoleCloseHandler(int n)
{
    int time_wait; 

    for(time_wait=0 ; time_wait<=WAITTIME_THR_EXIT ; time_wait+=WAITTIME_SLICE)
	{	
		if(g_f_taskexec_exit)
		{
			VERBOSE_VERBOSE("Checked 'TASKEXEC' thread exit.\n"); 
			break; 
		}
        MSG_SendPulse(g_co_te, TESERV_EXIT, 0); 
		THREAD_Sleep(WAITTIME_SLICE); 			
	}
}

////////////////////////////////////////////////////////////////////////////////

void Init_VerboseInit(void)
{
    VERBOSE_CleanDirtyFile(TE_VERBOSE_NAME); 

    VERBOSE_Create(TE_VERBOSE_NAME, TE_VERBOSE_PREFIX); 

    VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
	VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, TRUE);
    VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, TRUE);

    switch(g_n_print_opt)
    {
    case ARG_NONE:
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   0);
        break;
    case ARG_ERR:
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   1);
        break;
    case ARG_WARN:
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   1);
        break;
    case ARG_MSG:
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, 0);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   1);
        break;
    case ARG_VERB:
    default: 
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, 1);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   1);              
    }
}

void Init_VerboseUninit(void)
{    
	VERBOSE_WaitForComplete();
    VERBOSE_Destroy();
}

int Init_TeResourceInit(void)
{
	unsigned long time_wait;
    int code; 
    
    Init_RobotInfo();               // This call needs before Runtime Running. 

    // TE Service Handler Mapping Check
    code = TeServ_CheckMapping();   // return : failed service code
    if(0 <= code)
    {
		VERBOSE_ERROR("Failed to check TE service Handler(%d) mapping. %s\n", 
            code, TeServ_GetName(code));
        return -1; 
    }
	else
	{
		VERBOSE_VERBOSE("Checked TE service handler mapping.\n"); 
	}

	// Channel Init	
    g_ch_te = MSG_CreateNamedChannel(TE_CHANNEL_NAME);
    if (g_ch_te == INVALID_CHID)
    {
		VERBOSE_ERROR("Failed to create the channel. Name:%s\n", TE_CHANNEL_NAME);
        return -1; 
    }
	else
	{
		VERBOSE_VERBOSE("Created the channel. Name:%s ID:%d \n", TE_CHANNEL_NAME, g_ch_te);
	}

	// Shm of te init
	g_hshm_te_test = SHM_Create(SHMNAME_TE_TEST, sizeof(SHM_TE_STATUS)); 
	if(g_hshm_te_test == -1)
	{
		VERBOSE_ERROR("Failed to create SHM handle.\n"); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created SHM handle.\n"); 
	}

	g_pshm_te_test = (volatile SHM_TE_STATUS*)SHM_Map(g_hshm_te_test, sizeof(SHM_TE_STATUS)); 
	if(g_pshm_te_test == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map SHM.\n"); 
		return -1; 
	}
	else
	{
		g_pshm_te_test->size = sizeof(SHM_TE_STATUS); 
		VERBOSE_VERBOSE("Mapped SHM. Size:%dbytes\n", g_pshm_te_test->size); 
	}

    g_hshm_sensor_rm = SHM_Create(SHMNAME_SENSOR_RM, sizeof(shm_mmi_task_t)); 
	if(g_hshm_sensor_rm == -1)
	{
		VERBOSE_ERROR("Failed to create %s handle.\n", SHMNAME_SENSOR_RM); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created %s handle.\n", SHMNAME_SENSOR_RM); 
	}
	shm_mmi = (volatile shm_mmi_task_t*)SHM_Map(g_hshm_sensor_rm, sizeof(shm_mmi_task_t)); 
	if(shm_mmi == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map %s.\n", SHMNAME_SENSOR_RM); 
		return -1; 
	}
	else
	{
        shm_mmi->size = sizeof(shm_mmi_task_t); 
        VERBOSE_VERBOSE("Mapped %s. Size:%dbytes\n", SHMNAME_SENSOR_RM, shm_mmi->size); 
	}

    // 
    g_hshm_sensor_sc = SHM_Create(SHMNAME_SENSOR_SC, sizeof(shm_task_servo_t)); 
	if(g_hshm_sensor_sc == -1)
	{
		VERBOSE_ERROR("Failed to create %s handle.\n", SHMNAME_SENSOR_SC); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created %s handle.\n", SHMNAME_SENSOR_SC); 
	}
	shm_servo = (volatile shm_task_servo_t*)SHM_Map(g_hshm_sensor_sc, sizeof(shm_task_servo_t)); 
	if(shm_servo == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map %s.\n", SHMNAME_SENSOR_SC); 
		return -1; 
	}
	else
	{
        shm_servo->size = sizeof(shm_task_servo_t); 
        VERBOSE_VERBOSE("Mapped %s. Size:%dbytes\n", SHMNAME_SENSOR_SC, shm_servo->size); 
	}

    // shm for restart    
    g_hshm_te_restart = SHM_Create(SHMNAME_RESTART, sizeof(SHM_RESTART) + sizeof(RESTART_DATA)); 
	if(g_hshm_te_restart == -1)
	{
		VERBOSE_ERROR("Failed to create SHMNAME_RESTART handle.\n"); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created SHMNAME_RESTART handle.\n"); 
	}
	g_pshm_restart = (volatile SHM_RESTART*)SHM_Map(g_hshm_te_restart, sizeof(SHM_RESTART) + sizeof(RESTART_DATA)); 
	if(g_pshm_restart == NULL) 
	{ 
		VERBOSE_ERROR("Cannot map SHMNAME_RESTART.\n"); 
		return -1; 
	}
	else
	{
        g_pshm_restart->size_total = sizeof(SHM_RESTART) + sizeof(RESTART_DATA);         
        VERBOSE_VERBOSE("Mapped SHMNAME_RESTART. Size:%dbytes\n", g_pshm_restart->size_total); 

        // default restart value to use temporarily. These are changed by RM. 
        g_pshm_restart->moving_type = RESTART_MOVE_TYPE_JNT; 
        g_pshm_restart->path_speed = 0.1;  
#if 0
        g_pshm_restart->over_dist  = 30.;         
#endif 
        g_pshm_restart->d_overlap_horz = 30.;         
        g_pshm_restart->d_overlap_vert = 30.;         
	}
    
	// Thread Init
#if 1
	g_hthr_run = THREAD_Create(RuntimeThread, NULL, 0, 152,
								THREAD_DETACHED, THREAD_POLICY_FIFO); 
#else
    g_hthr_run = THREAD_Create(RuntimeThread, NULL, 0, 29,
								THREAD_DETACHED, THREAD_POLICY_RR); 
#endif
	if(g_hthr_run == THREAD_INVALID)
	{
		VERBOSE_ERROR("Cannot create 'RuntimeThread'.\n"); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created 'RuntimeThread'.\n"); 
	}

	// Waits Thread Ready.
    VERBOSE_VERBOSE("Waits for 'Runtime' thread ready.\n"); 

	for(time_wait=0 ; time_wait<=WAITTIME_THR_READY ; time_wait+=WAITTIME_SLICE)
	{	
		if(g_f_runtime_ready == TRUE)
		{
			VERBOSE_VERBOSE("Checked 'Runtime' thread ready.\n"); 
			break; 
		}
		THREAD_Sleep(WAITTIME_SLICE); 
	}
    if(WAITTIME_THR_READY <= time_wait)
	{
		VERBOSE_ERROR("Failed to wait for 'Runtime' thread ready.\n"); 
		return -1; 
	}

    // Thread Init
	g_hthr_vga = THREAD_Create(VgaThread, NULL, 0, THREAD_PRIO_BELOW_NORMAL,
							   THREAD_DETACHED, THREAD_POLICY_RR); 
	if(g_hthr_vga == THREAD_INVALID)
	{
		VERBOSE_ERROR("Cannot create 'VGA Thread'.\n"); 
		return -1; 
	}
	else
	{
		VERBOSE_VERBOSE("Created 'VGA Thread'.\n"); 
	}

    // Connection to TASKEXEC
    g_co_te = MSG_AttachConnection(0, g_ch_te); 
    if (g_co_te == INVALID_COID) 
    { 
        VERBOSE_ERROR("Failed to attatch connetion to TASKEXEC thread\n"); 
        return -1; 
    }
    else
    {
        VERBOSE_VERBOSE("Attached the connection to TASKEXEC Thread. ID:%d \n", g_co_te); 
    }

    // Init Check Time Limit
    if(g_ipc_wait > 0)
    {
		g_htmr_exit = TIME_RegTimerPulse(g_co_te, TESERV_EXIT, 0, g_ipc_wait, 0);
		if(g_htmr_exit == -1)
		{
			VERBOSE_ERROR("Failed to register INIT_WAIT timer.\n");
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Waits for RM INIT MSG for %ld[ms]\n", g_ipc_wait);
		}
	}

    // Exit Forced Handler Register 
    DEBUG_SetConsoleHandler(ConsoleCloseHandler); 

	return 0; 
}

void Init_TeResourceUninit(void)
{
	int ret; 
	unsigned long time_wait; 

    // !! Warning !! Timer Handle Stop First 
    // Unregistered INIT_WAIT Timer
    if(g_htmr_exit != -1)
    {
        if(TIME_UnregTimerPulse(g_htmr_exit) == -1)
        {
            VERBOSE_ERROR("Failed to unregister INIT_WAIT timer. Handle:%d \n", g_htmr_exit); 
        }
        else
        {
            VERBOSE_VERBOSE("Unregistered INIT_WAIT timer. Handle:%d \n", g_htmr_exit); 
        }
    }

    // Uninit Connection Connection to TASKEXEC
	if(INVALID_COID != g_co_te) 
	{
		ret = MSG_DetachConnection(g_co_te);
		g_co_te = INVALID_COID; 

		if(-1 == ret)
		{
			VERBOSE_ERROR("Failed to detach the connection to TASKEXEC.\n");
		}
		else
		{
			VERBOSE_VERBOSE("Detached the connection to TASKEXEC. ID:%d\n", g_co_te);			
		}
	}

    // Thread Uninit 
    // VGA Thread Exit 
    g_f_vga_exit = TRUE; 
    VERBOSE_VERBOSE("Waits for VGA thread to exit.\n"); 

    for(time_wait=0 ; time_wait<=WAITTIME_THR_EXIT ; time_wait+=WAITTIME_SLICE)
    {
        if(!g_f_vga_exit)
        {   
            VERBOSE_VERBOSE("Checked VGA thread exit.\n"); 
            break;
        }
        THREAD_Sleep(WAITTIME_SLICE); 
    }
    if(g_f_vga_exit)
    {
        THREAD_Kill(g_hthr_vga); 
		VERBOSE_WARNING("Forced to kill VGA thread.\n");        
    }

    // Runtime Thread Exit 

	// Thread Exit Wait
	if(g_f_runtime_ready)
	{
        // Req to Exit Runtime Thread 

        VERBOSE_VERBOSE("Waits for 'Runtime' thread to exit.\n"); 
        
		MSG_SendPulse(g_co_run, RUNSERV_EXIT, 0); 
		        
		for(time_wait=0 ; time_wait<=WAITTIME_THR_EXIT ; time_wait+=WAITTIME_SLICE)
		{	
			if(g_f_runtime_exit)
			{
				VERBOSE_VERBOSE("Checked 'Runtime' thread exit.\n"); 
				break; 
			}
			THREAD_Sleep(WAITTIME_SLICE); 			
		}

		if(WAITTIME_THR_EXIT <= time_wait)
		{
			THREAD_Kill(g_hthr_run); 
			VERBOSE_WARNING("Forced to kill 'Runtime' thread.\n"); 
		}
	}

	// TE Restart Shared Memory Uninit
    if(NULL != g_pshm_restart)
	{
		SHM_Unmap((void*)g_pshm_restart, sizeof(SHMNAME_RESTART) + sizeof(RESTART_DATA)); 
		g_pshm_restart = NULL; 
		
		VERBOSE_VERBOSE("Unmapped SHMNAME_RESTART. Size:%dbytes\n", sizeof(SHMNAME_RESTART) + sizeof(RESTART_DATA)); 
	}
	if(-1 != g_hshm_te_restart)
	{
		SHM_Destroy(g_hshm_te_restart, SHMNAME_RESTART); 
		g_hshm_te_restart = -1; 

		VERBOSE_VERBOSE("Destroyed SHM. Name:%s\n", SHMNAME_RESTART); 
	}

	// TE Shared Memory Uninit
	if(NULL != g_pshm_te_test)
	{
		SHM_Unmap((void*)g_pshm_te_test, sizeof(SHM_TE_STATUS)); 
		g_pshm_te_test = NULL; 
		
		VERBOSE_VERBOSE("Unmapped SHM. Size:%dbytes\n", sizeof(SHM_TE_STATUS)); 
	}
	if(-1 != g_hshm_te_test)
	{
		SHM_Destroy(g_hshm_te_test, SHMNAME_TE_TEST); 
		g_hshm_te_test = -1; 

		VERBOSE_VERBOSE("Destroyed SHM. Name:%s\n", SHMNAME_TE_TEST); 
	}

	// Uninit Channel
	if(INVALID_CHID != g_ch_te)
	{
		ret = MSG_DestroyChannel(g_ch_te);
		g_ch_te = INVALID_CHID; 

		if(-1 == ret)
		{
			VERBOSE_ERROR("Failed to destroy the channel. Name:%s\n", TE_CHANNEL_NAME);
		}
		else
		{
			VERBOSE_VERBOSE("Destroyed the channel. Name:%s\n", TE_CHANNEL_NAME);
		}
	}
}

int Init_IpcInit(void)
{   
    // Uninit for Re-init
    Init_IpcUninit();
    // Wait btw Uninit(Unmap SHM_SC) & Init(Map SHM_SC)
    THREAD_Sleep(WAITTIME_SLICE); 

    // RM IPC Init /////////////////////////////////////////////////////////////

	if(g_f_no_rm == FALSE)
	{
		///// Connection to RM

		VERBOSE_VERBOSE("Waits for connecting to RM...\n");

		g_co_rm = MSG_WaitForNamedConnection(SYS_RM_CHANNEL_NAME, g_ipc_wait);

		if(g_co_rm == INVALID_COID)
		{
			VERBOSE_ERROR("Failed the connection to RM\n");
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Connected to RM. ID:%d\n", g_co_rm);
		}

		// Open
		// RM Shared Memory Init
		g_hshm_rm_sys = SHM_Open(SHM_RM_SYSSTATUS_NAME);
		if(g_hshm_rm_sys == -1)
		{
			VERBOSE_ERROR("Failed to open %s handle.\n", SHM_RM_SYSSTATUS_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Opened %s. Handle:%d\n", SHM_RM_SYSSTATUS_NAME, g_hshm_rm_sys);
		}

		g_pshm_rm_sys = (volatile SHM_RM_SYSSTATUS*)SHM_Map(g_hshm_rm_sys, sizeof(SHM_RM_SYSSTATUS));

		// Map
		if(g_pshm_rm_sys == NULL)
		{
			VERBOSE_ERROR("Failed to map %s.\n", SHM_RM_SYSSTATUS_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Mapped %s. Size:%dbytes\n", SHM_RM_SYSSTATUS_NAME, sizeof(SHM_RM_SYSSTATUS));

			// Size Check
			if(g_pshm_rm_sys->nSize != sizeof(SHM_RM_SYSSTATUS))
			{
				VERBOSE_ERROR("Mismatched %s Size. Map:%dbytes Chk:%dbytes \n", SHM_RM_SYSSTATUS_NAME, sizeof(SHM_RM_SYSSTATUS), g_pshm_rm_sys->nSize);
#if 0
				SHM_Unmap((void*)g_pshm_rm_sys, sizeof(SHM_RM_SYSSTATUS));
				g_pshm_rm_sys = NULL;

				VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_RM_SYSSTATUS_NAME, sizeof(SHM_RM_SYSSTATUS));
#endif
				return -1;
			}
			else
			{
				VERBOSE_VERBOSE("Matched %s size. Map:%dbytes Chk:%dbytes \n", SHM_RM_SYSSTATUS_NAME, sizeof(SHM_RM_SYSSTATUS), g_pshm_rm_sys->nSize);
			}
		}

		// SHM_RM_SYSCONFIG of Rm
		g_hshm_sys_conf = SHM_Open(SHM_RM_SYSCONFIG_NAME);
		if(g_hshm_sys_conf == -1)
		{
			VERBOSE_ERROR("Failed to open %s handle.\n", SHM_RM_SYSCONFIG_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Opened %s. Handle:%d\n", SHM_RM_SYSCONFIG_NAME, g_hshm_sys_conf);
		}

		g_pshm_sys_conf = (volatile SHM_RM_SYSCONFIG*)SHM_Map(g_hshm_sys_conf, sizeof(SHM_RM_SYSCONFIG));

		// Map
		if(g_pshm_sys_conf == NULL)
		{
			VERBOSE_ERROR("Failed to map %s.\n", SHM_RM_SYSCONFIG_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Mapped %s. Size:%dbytes\n", SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG));

			// Size Check
			if(g_pshm_sys_conf->dwLength != sizeof(SHM_RM_SYSCONFIG))
			{
                VERBOSE_ERROR("Mismatched %s Size. Map:%dbytes Chk:%lubytes \n", SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG), g_pshm_sys_conf->dwLength);
#if 0
				SHM_Unmap((void*)g_pshm_sys_conf, sizeof(SHM_RM_SYSCONFIG));
				g_pshm_sys_conf = NULL;

				VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG));
#endif
				return -1;
			}
			else
			{
				VERBOSE_VERBOSE("Matched %s. Map:%dbytes Chk:%lubytes \n", SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG), g_pshm_sys_conf->dwLength);
			}
		}

        // JOB SHM
        g_hshm_job = SHM_Open(SHM_DANDY_JOB_NAME);
	    if(g_hshm_job == -1)
	    {
		    VERBOSE_ERROR("Failed to open %s.\n", SHM_DANDY_JOB_NAME);
		    return -1;
	    }
	    else
	    {
		    VERBOSE_VERBOSE("Opened %s. Handle:%d\n", SHM_DANDY_JOB_NAME, g_hshm_job);
	    }

	    g_pshm_job = (volatile SHM_DANDY_JOB*)SHM_Map(g_hshm_job, sizeof(SHM_DANDY_JOB));

	    // Map. Double Mapping for Job SHM for Variable Size.
	    if(g_pshm_job == NULL)
	    {
		    VERBOSE_ERROR("Cannot map %s.\n", SHM_DANDY_JOB_NAME);
		    return -1;
	    }
	    else
	    {
            // Job Shm Size 
            g_size_job = GET_SHM_JOB_SIZE(g_pshm_job); 
            VERBOSE_VERBOSE("Mapped %s to get total size 1st Time.\n", SHM_DANDY_JOB_NAME);
            
            // Unmap 1st Mapping
            SHM_Unmap((void*)g_pshm_job, sizeof(SHM_DANDY_JOB)); 
		    g_pshm_job = NULL; 
            VERBOSE_VERBOSE("Unmapped %s for 2nd mapping.\n", SHM_DANDY_JOB_NAME);
		
            // 2nd Mapping. Double Mapping for Job SHM for Variable Size.
            g_pshm_job = (volatile SHM_DANDY_JOB*)SHM_Map(g_hshm_job, g_size_job);            
	        if(g_pshm_job == NULL)
	        {
		        VERBOSE_ERROR("Failed to map %s 2nd time.\n", SHM_DANDY_JOB_NAME);
		        return -1;
	        }

            // Size Check
            if(g_pshm_job->dwLength != g_size_job)
		    {
                VERBOSE_ERROR("Mismatched %s. Map:%dbytes Chk:%lubytes \n", SHM_DANDY_JOB_NAME, g_size_job, g_pshm_job->dwLength);
#if 0
			    SHM_Unmap((void*)g_pshm_job, sizeof(SHM_DANDY_JOB));
			    g_pshm_job = NULL;

			    VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_DANDY_JOB_NAME, g_size_job);
#endif
			    return -1;
		    }
		    else
		    {
                VERBOSE_VERBOSE("Matched %s. Map:%dbytes Chk:%lubytes \n", SHM_DANDY_JOB_NAME, g_size_job, g_pshm_job->dwLength);
                g_rcmd = GET_SHM_JOB_CMD_BUFFER(g_pshm_job);
                g_varT = GET_SHM_JOB_TVA_BUFFER(g_pshm_job);
                g_varP = GET_SHM_JOB_PVA_BUFFER(g_pshm_job);
                g_varB = GET_SHM_JOB_BVA_BUFFER(g_pshm_job);
                g_varI = GET_SHM_JOB_IVA_BUFFER(g_pshm_job);
                g_varR = GET_SHM_JOB_RVA_BUFFER(g_pshm_job);
                g_wvf  = GET_SHM_JOB_WVF_BUFFER(g_pshm_job);
                g_swf  = GET_SHM_JOB_SWF_BUFFER(g_pshm_job); 
                g_mwf  = GET_SHM_JOB_MWF_BUFFER(g_pshm_job);
                g_ewf  = GET_SHM_JOB_EWF_BUFFER(g_pshm_job);                
		    }
	    } 

        // restart data loading
        memcpy(&g_restart, ((BYTE*)g_pshm_restart)+sizeof(SHM_RESTART), 
                   g_pshm_restart->size_total - sizeof(SHM_RESTART)); 
	}
	
	// SC IPC Init /////////////////////////////////////////////////////////////

	if(g_f_no_sc == FALSE)
	{		
		///// Connection to SC

		VERBOSE_VERBOSE("Waits for connecting to %s \n", SC_TIME_CHANNEL);

        g_co_sc = MSG_WaitForNamedConnection(SC_TIME_CHANNEL, g_ipc_wait);
		if(g_co_sc == INVALID_COID)
		{
			VERBOSE_ERROR("Failed the connection. %s\n", SC_TIME_CHANNEL);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Connected to SC. %s ID:%d\n", SC_TIME_CHANNEL, g_co_sc);
		}

        // SC Shared Memory Init

		// Open
        g_hshm_sc_motor = SHM_Open(SC_SHM_NAME);
		if(g_hshm_sc_motor == -1)
		{
			VERBOSE_ERROR("Failed to open SHM handle. Name:%s\n", SC_SHM_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Opened SHM handle. Name:%s Handle:%d\n", SC_SHM_NAME, g_hshm_sc_motor);
		}
        
        // Map
        g_pshm_sc_motor = (volatile SHM_SC_MOTORCTRL*)SHM_Map(g_hshm_sc_motor, sizeof(SHM_SC_MOTORCTRL));		
		if(g_pshm_sc_motor == NULL)
		{
			VERBOSE_ERROR("Cannot map SHM. Name:%s\n", SC_SHM_NAME);
			return -1;
		}
		else
		{
			VERBOSE_VERBOSE("Mapped SHM. Name:%s Size:%dbytes\n", SC_SHM_NAME, sizeof(SHM_SC_MOTORCTRL));
	
			// Size Check
			if(g_pshm_sc_motor->nsize != sizeof(SHM_SC_MOTORCTRL))
			{
				VERBOSE_ERROR("Mismatched SHM size. Name:%s Size:%dbytes Chk:%dbytes \n", SC_SHM_NAME, sizeof(SHM_SC_MOTORCTRL), g_pshm_sc_motor->nsize);
#if 0
				SHM_Unmap((void*)g_pshm_sc_motor, sizeof(SHM_SC_MOTORCTRL));
				g_pshm_sc_motor = NULL;

				VERBOSE_VERBOSE("Unmapped SHM. Name:%s Size:%dbytes\n", SC_SHM_NAME, sizeof(SHM_SC_MOTORCTRL));
#endif
				return -1;
			}
			else
			{
				VERBOSE_VERBOSE("Matched SHM size. Name:%s Size:%dbytes Chk:%dbytes \n", SC_SHM_NAME, sizeof(SHM_SC_MOTORCTRL), g_pshm_sc_motor->nsize);
			}
		}
	}

	// Robot Info's Setting
	Init_RobotInfo();

    // Timer Register which is to SC
    if(Init_TimerInit())
    {
        return -1; 
    }

    // Sets init completion flag in shm & Global
	if(g_pshm_rm_sys != NULL)
	{
		g_pshm_rm_sys->fInitProcTE = TRUE;
	}
    g_f_ipc_init = TRUE; 

    VERBOSE_MESSAGE("Completed IPC resources\n");     
    VERBOSE_MESSAGE("Completed all initilization\n");  
    VERBOSE_MESSAGE("Type 'tepulse 1' to show the version & the service helps.\n"); 

	return 0; 
}

void Init_IpcUninit(void)
{
    // Sets exit completion flag in shm
    if(g_pshm_rm_sys != NULL)
    {        
        g_pshm_rm_sys->fInitProcTE = FALSE;         
    }
    // Reset Flag for IPC Inited
    g_f_ipc_init = FALSE; 
    
    // Timer Register which is to SC
    Init_TimerUninit(); 

    // SC Shared Memory Uninit
	if(NULL != g_pshm_sc_motor)
	{
		SHM_Unmap((void*)g_pshm_sc_motor, sizeof(SHM_SC_MOTORCTRL)); 
		g_pshm_sc_motor = NULL; 
        VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SC_SHM_NAME, sizeof(SHM_SC_MOTORCTRL)); 
	}

	if(-1 != g_hshm_sc_motor)
	{
		SHM_Close(g_hshm_sc_motor);
		g_hshm_sc_motor = -1; 
		VERBOSE_VERBOSE("Closed %s.\n", SC_SHM_NAME);
	}

    ///// Detach SC Connection
    if(INVALID_COID != g_co_sc) 
	{
		MSG_DetachConnection(g_co_sc);
		VERBOSE_VERBOSE("Detached the connection to SC. %s ID:%d\n", SC_TIME_CHANNEL, g_co_sc);
		g_co_sc = INVALID_COID; 
	}
           
    // RM Job SHM Uninit
    if(NULL != g_pshm_job)
	{
		SHM_Unmap((void*)g_pshm_job, g_size_job); 
		g_pshm_job = NULL; 
		
		VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_DANDY_JOB_NAME, g_size_job); 
	}
	if(-1 != g_hshm_job)
	{
		SHM_Close(g_hshm_job);
		g_hshm_job = -1; 

		VERBOSE_VERBOSE("Closed %s.\n", SHM_DANDY_JOB_NAME);
	}

    // RM Sys-conf Mem Uninit
    if(NULL != g_pshm_sys_conf)
	{
		SHM_Unmap((void*)g_pshm_sys_conf, sizeof(SHM_RM_SYSCONFIG)); 
		g_pshm_sys_conf = NULL; 

		VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_RM_SYSCONFIG_NAME, sizeof(SHM_RM_SYSCONFIG)); 
	}
	if(-1 != g_hshm_sys_conf)
	{
		SHM_Close(g_hshm_sys_conf);
		g_hshm_sys_conf = -1; 

		VERBOSE_VERBOSE("Closed the RM SHM. Name:%s\n", SHM_RM_SYSCONFIG_NAME);
	}

	// RM Shared Memory Uinit
	
	if(NULL != g_pshm_rm_sys)
	{
		SHM_Unmap((void*)g_pshm_rm_sys, sizeof(SHM_RM_SYSSTATUS)); 
		g_pshm_rm_sys = NULL; 
		
		VERBOSE_VERBOSE("Unmapped %s. Size:%dbytes\n", SHM_RM_SYSSTATUS_NAME, sizeof(SHM_RM_SYSSTATUS)); 
	}
	if(-1 != g_hshm_rm_sys)
	{
		SHM_Close(g_hshm_rm_sys);
		g_hshm_rm_sys = -1; 

		VERBOSE_VERBOSE("Closed SHM. Name:%s\n", SHM_RM_SYSSTATUS_NAME);
	}

    ///// Detach RM Connection
    if(INVALID_COID != g_co_rm) 
	{
		MSG_DetachConnection(g_co_rm);
		VERBOSE_VERBOSE("Detached the connection to RM. ID:%d\n", g_co_rm);
		g_co_rm = INVALID_COID; 
	}
}

void Init_SetExitFlag(void)
{
    if(g_pshm_rm_sys != NULL)
    {
        g_pshm_rm_sys->fExitProcTE = TRUE;
    }
}

#if 0
// RM Request Version
int RmReqVers(void)
{
	int ret; 
	RMGR_PACKET	 msg; 
	RMGR_REPLY_PACKET rep; 

    msg.nCode = RMGR_SERV_SYSVERSION; 
    msg.nValue = 0; 
    msg.nDataSize = 0; 
	
    VERBOSE_VERBOSE("Send Blocking for version request to RM... Code:%d Val:%d \n", msg.nCode, msg.nValue); 

	ret = MSG_Send(g_co_rm, &msg, sizeof(msg), &rep, sizeof(RMGR_REPLY_PACKET));
	if(ret != 0)
	{
        VERBOSE_ERROR("Failed to send the Vers Req to RM. Code:%d Val:%d \n", msg.nCode, msg.nValue);		
		return -1; 
	}
	else
	{	
        VERBOSE_VERBOSE("Was replyed for Vers Req from RM. Ver:%s Build:%s\n", 
            rep.Data.reply_ver.rgchRM_vers, rep.Data.reply_ver.rgchRM_build); 		
	}
    return 0; 
}
#endif

////////////////////////////////////////////////////////////////////////////////

int Init_RuntimeInit(void)
{    	
    // Channel Init	for External Process
    g_ch_run = MSG_CreateNamedChannel(RUN_CHANNEL_NAME);
    if (g_ch_run == INVALID_CHID)
    {
		R_VERB_ERR("Failed to create the channel for RUNTIME thread. Name:%s\n", RUN_CHANNEL_NAME);
        return -1; 
    }
	else
	{
		R_VERB_VRB("Created the channel for RUNTIME thread. Name:%s ID:%d \n", RUN_CHANNEL_NAME, g_ch_run);
	}

    // Connection to Runtime
    g_co_run = MSG_AttachConnection(0, g_ch_run); 
    if (g_co_run == INVALID_COID) 
    { 
        R_VERB_ERR("Failed to attatch connetion to Runtime Thread\n"); 
        return -1; 
    }
    else
    {
        R_VERB_VRB("Attached the connection to Runtime Thread. ID:%d \n", g_co_run); 
    }

    // VGA
    if(VGA_MapVGAMemory(0) == -1)
    {    
        R_VERB_ERR("Failed to map VGA memory\n"); 
        return -1; 
    }
    else
    {
        R_VERB_VRB("Mapped VGA memory.\n"); 
    }
    // fgcolor : yellow, bkcolor : black 
    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_BLACK)); 
    
    CallStack_Init(); 

	return 0; 
}

void Init_RuntimeUninit(void)
{
	int ret; 	
	
    // VGA
    VGA_UnmapVGAMemory();
    R_VERB_VRB("Unmapped VGA memory.\n"); 

    // Unregistered Runtime Timer
    if(g_htmr_run != -1)
    {
        if(TIME_UnregTimerPulse(g_htmr_run) == -1)
        {
            R_VERB_ERR("Failed to unregister RUNTIME timer. Handle:%d \n", g_htmr_run); 
        }
        else
        {
            R_VERB_VRB("Unregistered RUNTIME timer. Handle:%d \n", g_htmr_run); 
        }
    }

    // Uninit Channel
	if(INVALID_CHID != g_ch_run)
	{
		ret = MSG_DestroyChannel(g_ch_run);
		g_ch_run = INVALID_CHID; 

		if(-1 == ret)
		{
			VERBOSE_ERROR("Failed to destroy the channel.\n");
		}
		else
		{
			VERBOSE_MESSAGE("Destroyed the channel.\n");
		}
	}
}

extern int TIME_RegTimerPulseNanosec(int coid, int nCode, int nValue, int nNanosec, int nOption); 

int Init_TimerInit(void)   // [ms]
{
#if defined(__QNX__)
    struct itimerspec itime;
    struct sigevent se;
    timer_t timerid;
#endif

    Init_TimerUninit();         
    
    /////// If SC runs, SC creates Timer.   /////////
    if(g_co_sc != INVALID_COID)
    {
        return 0;   
    }
    /////// If SC runs, SC creates Timer.   /////////

#if defined(__QNX__)
    itime.it_value.tv_sec  = 0;
	itime.it_value.tv_nsec = BASIC_SAMPLE * g_t_samp; // 999847;    
	itime.it_interval.tv_sec = 0;
	itime.it_interval.tv_nsec = BASIC_SAMPLE * g_t_samp; // 999847;   

    if(g_co_sc != INVALID_COID)
    {    
        SIGEV_PULSE_INIT(&se, g_co_sc, SIGEV_PULSE_PRIO_INHERIT, RUNSERV_TIMER, 0);                 
    }
    else
    {
        SIGEV_PULSE_INIT(&se, g_co_run, SIGEV_PULSE_PRIO_INHERIT, RUNSERV_TIMER, 0);        
    }
    if(g_htmr_run == -1)
    {
        if(timer_create(CLOCK_MONOTONIC, &se, &timerid))
        {
            g_htmr_run = -1;         
        }
        else
        {
            timer_settime(timerid, 0, &itime, NULL);    
            g_htmr_run = (int)timerid;      
        }
    }
#else
    if(g_co_sc != INVALID_COID)
    {
        g_htmr_run = TIME_RegTimerPulse(g_co_sc, RUNSERV_TIMER, 0, g_t_samp, 0);        
    }
    else
    {
        g_htmr_run = TIME_RegTimerPulse(g_co_run, RUNSERV_TIMER, 0, g_t_samp, 0);
    }
#endif

    if(g_htmr_run == -1)
    {              
        VERBOSE_ERROR("Failed to register RUNTIME timer. Req. Sample:%u[ms].\n", g_t_samp); 
        return -1; 
    }
    else
    {
        VERBOSE_VERBOSE("Registered RUNTIME timer. Handle:%d Sample:%u[ms]\n", g_htmr_run, g_t_samp); 
        return 0; 
    }
}

void Init_TimerUninit(void)
{    
    if(g_htmr_run != -1)
    {
#if defined(__QNXNTO__)
        if(timer_delete(g_htmr_run))        
        {
            VERBOSE_ERROR("Failed to Unregister RUNTIME timer. Handle:%d Sys Err:%d\n", g_htmr_run, errno); 
        }
#else
        if(TIME_UnregTimerPulse(g_htmr_run))
        {
            VERBOSE_ERROR("Failed to Unregister RUNTIME timer. Handle:%d Sample:%u[ms]\n", g_htmr_run, g_t_samp); 
        }
#endif  
        else
        {
            VERBOSE_VERBOSE("Unregistered RUNTIME timer. Handle:%d Sample:%u[ms]\n", g_htmr_run, g_t_samp); 
        }
    }
    g_htmr_run = -1; 
}

void Init_RobotInfo(void)
{
	int i;
    double t; 
    XYZRPY tcp; 

	// Setting Var's
	if(NULL == g_pshm_sys_conf)
	{
		// Default Setting
		for(i=0 ; i<MAX_AXIS_COUNT ; i++)
		{
			g_vel_axis_max[i] = DEF_VEL_MAX_AXIS;
			g_acc_axis[i] = DEF_VEL_MAX_AXIS / DEF_TIME_ACCEL;		// 0.4s acc
			g_dec_axis_stop[i] = g_vel_axis_max[i] / DEF_TIME_DECEL_STOP;
			g_dec_axis_estop[i]= g_vel_axis_max[i] / DEF_TIME_DECEL_ESTOP;            

            t = (DEF_TIME_JERK < EPS_T)? EPS_T : DEF_TIME_JERK;
			g_jerk_axis[i] = g_acc_axis[i] / t; 
		}        

        // Vel Limit 
        g_vel_ori_max = DEF_VEL_MAX_ORI;                // [rad/ms]
        g_vel_lin_max = DEF_VEL_MAX_LIN;                // [mm/ms]

        g_acc_lin = DEF_VEL_MAX_LIN / DEF_TIME_ACCEL; 
        g_acc_ori = DEF_VEL_MAX_ORI / DEF_TIME_ACCEL;             
        g_dec_lin_stop = DEF_VEL_MAX_LIN / DEF_TIME_DECEL_STOP; 
        g_dec_lin_estop= DEF_VEL_MAX_LIN / DEF_TIME_DECEL_STOP; 
        g_dec_ori_stop = DEF_VEL_MAX_ORI / DEF_TIME_DECEL_ESTOP; 
        g_dec_ori_estop= DEF_VEL_MAX_ORI / DEF_TIME_DECEL_ESTOP; 
        g_dec_time_touch = DEF_TIME_DECEL_ESTOP; 

        t = (DEF_TIME_JERK < EPS_T)? EPS_T : DEF_TIME_JERK;
        g_jerk_lin = g_acc_lin / t; 
        g_jerk_ori = g_acc_ori / t; 

        // if no limit -180 ~ 180
        g_lim_min[0] = -153 * PI / 180.; 
        g_lim_min[1] = -91  * PI / 180.; 
        g_lim_min[2] = -73  * PI / 180.; 
        g_lim_min[3] = -230 * PI / 180.; 
        g_lim_min[4] = -115 * PI / 180.; 
        g_lim_min[5] = -360 * PI / 180.; 

        g_lim_max[0] =  153 * PI / 180.; 
        g_lim_max[1] =  113 * PI / 180.; 
        g_lim_max[2] =  68  * PI / 180.; 
        g_lim_max[3] =  230 * PI / 180.; 
        g_lim_max[4] =  115 * PI / 180.; 
        g_lim_max[5] =  360 * PI / 180.; 

        // home
        for(i=0 ; i<MAX_HOME_COUNT ; i++)
        {
            memset(&g_home[i].joint, 0, sizeof(g_home[i])); 
        }

        // tcp (is prior to the 'g_eTt' setting)
        tcp.x = 0;          tcp.roll  = 0; 
        tcp.y = 0;          tcp.pitch = 0; 
        tcp.z = DEF_TCP_Z;  tcp.yaw   = 0; 

        // Kinematics
        g_Forward = For_Dandy2; 
        g_Inverse = Inv_Dandy2; 
        g_Config  = Conf_Dandy2;
        g_AxisLogical = NULL; 

        g_dh[0].th = 0; 
        g_dh[1].th = -0.5 * PI; 
        g_dh[2].th = 0; 
        g_dh[3].th = 0; 
        g_dh[4].th = 0; 
        g_dh[5].th = 0; 

        g_dh[0].d = 400.; 
        g_dh[1].d = 0; 
        g_dh[2].d = 0; 
        g_dh[3].d = 390; 
        g_dh[4].d = 0; 
        g_dh[5].d = 0; 

        g_dh[0].al = -0.5*PI; 
        g_dh[1].al =  0; 
        g_dh[2].al = -0.5*PI; 
        g_dh[3].al =  0.5*PI; 
        g_dh[4].al = -0.5*PI; 
        g_dh[5].al =  0; 

        g_dh[0].l = 60; 
        g_dh[1].l = 350; 
        g_dh[2].l = 90; 
        g_dh[3].l = 0; 
        g_dh[4].l = 0; 
        g_dh[5].l = 0; 
                
        // do index
        g_idx_do_arc      = DEF_DO_IDX_ARC; 
        g_idx_do_gas      = DEF_DO_IDX_GAS; 
        g_idx_do_tch_mc   = DEF_DO_IDX_MC; 
        g_idx_do_tch_proc = DEF_DO_IDX_TCH_PROC; 
        g_idx_do_wirefeed = DEF_DO_IDX_WIREFEED; 
        g_idx_do_wireback = DEF_DO_IDX_WIREBACK; 

        // do ON level 
        g_onlev_do_arc      = ON; 
        g_onlev_do_gas      = ON; 
        g_onlev_do_tch_mc   = OFF; 
        g_onlev_do_tch_proc = ON; 
        g_onlev_do_wirefeed = ON; 
        g_onlev_do_wireback = ON; 

        // di index
        g_idx_di_arc      = DEF_DI_IDX_ARC;     
        g_idx_di_tch_proc = DEF_DI_IDX_TCH_PROC; 
        g_idx_di_tch_sig  = DEF_DI_IDX_TCH_SIG; 
        g_idx_di_pwr_fail = DEF_DI_IDX_POWER_FAIL; 
        g_idx_di_no_gas   = DEF_DI_IDX_NO_GAS; 
        
        // di ON level
        g_onlev_di_arc      = ON;     
        g_onlev_di_tch_proc = ON; 
        g_onlev_di_tch_sig  = OFF; 
        g_onlev_di_pwr_fail = ON; 
        g_onlev_di_no_gas   = ON; 
        
        // Cweave 
        g_cweave_touch_dist   = DEF_CWEAVE_TOUCH_DIST; 
        g_cweave_horz_margin  = DEF_CWEAVE_MARGIN_DIST; 
        g_cweave_weldleg_dist = DEF_CWEAVE_WELDLEG_DIST; 

        // Bvar for Special Program Control 
        g_i_bvar_left  = 0; 
        g_i_bvar_right = 0;  
        g_i_bvar_gap_sel = 0; 
	}
	else
	{
        CONFIG_ROBOT* rob; 
        CONFIG_WELDER* welder; 
        rob = (CONFIG_ROBOT*)&g_pshm_sys_conf->robot[0]; 
        welder = (CONFIG_WELDER*)&g_pshm_sys_conf->welder[rob->nWelderIndex[0]]; 

		// Configuration from SHM
		for(i=0 ; i<MAX_AXIS_COUNT ; i++)
		{            
			g_vel_axis_max[i]  = rob->dbMaxJointSpeed[i];
			g_acc_axis[i] 	   = g_vel_axis_max[i] / (rob->dbAccel);            
            g_dec_axis_stop[i] = g_vel_axis_max[i] / (rob->dbDecel);
			g_dec_axis_estop[i]= g_vel_axis_max[i] / (rob->dbDecel_Estop);
            
            t = (rob->dbJerk < EPS_T)? EPS_T : rob->dbJerk; 
            g_jerk_axis[i] 	   = g_acc_axis[i] / t; 
			
            g_lim_min[i] = (rob->axis[i].fSwLim_min)?
								 rob->axis[i].pos_swlim_min : DEF_AXIS_LIMIT_MIN; // -(DBL_MAX-DBL_MIN);
			g_lim_max[i] = (rob->axis[i].fSwLim_max)?
								 rob->axis[i].pos_swlim_max : DEF_AXIS_LIMIT_MAX; // (DBL_MAX-DBL_MIN);
		}

        // Vel limit
        g_vel_lin_max = rob->dbMaxLinearSpeed; 
        g_vel_ori_max = rob->dbMaxOrientSpeed; 
            
        t = rob->dbAccel; 
        t = (EPS_T < t)? t:EPS_T; 
        g_acc_lin = g_vel_lin_max / t ;             
        g_acc_ori = g_vel_ori_max / t ; 
            
        t = rob->dbDecel; 
        t = (EPS_T < t)? t:EPS_T; 
        g_dec_lin_stop = g_vel_lin_max / t ;             
        g_dec_ori_stop = g_vel_ori_max / t ; 

        t = rob->dbDecel_Estop; 
        t = (EPS_T < t)? t:EPS_T; 
        g_dec_lin_estop = g_vel_lin_max / t ;             
        g_dec_ori_estop = g_vel_ori_max / t ; 

        g_dec_time_touch = rob->dbDecel_Touch; 

        t = rob->dbJerk; 
        t = (EPS_T < t)? t:EPS_T; 
        g_jerk_lin = g_acc_lin / t ;             
        g_jerk_ori = g_acc_ori / t ; 

        // home
        for(i=0 ; i<MAX_HOME_COUNT ; i++)
        {            
            memcpy(&g_home[i].joint, (double*)rob->rgdbHomePosVal[i], sizeof(g_home[0].joint));             
        }

        // tcp
        tcp.x     = rob->coordTool.x; 
        tcp.y     = rob->coordTool.y; 
        tcp.z     = rob->coordTool.z; 
        tcp.roll  = rob->coordTool.rol; 
        tcp.pitch = rob->coordTool.pit; 
        tcp.yaw   = rob->coordTool.yaw;     

        // Kinematics
        g_Forward = For_Dandy2; 
        g_Inverse = Inv_Dandy2; 
        g_Config  = Conf_Dandy2;
        g_AxisLogical = NULL; 

        for(i=0 ; i<6 ; i++)
        {
            g_dh[i].th = rob->dh[i].th; 
            g_dh[i].d  = rob->dh[i].d; 
            g_dh[i].al = rob->dh[i].al; 
            g_dh[i].l  = rob->dh[i].l; 
        }        

        // do index
        g_idx_do_arc = welder->dout_portno.nArcOn; 
        g_idx_do_gas = welder->dout_portno.nGasOn; 
        g_idx_do_tch_mc   = welder->dout_portno.nTouchReady; 
        g_idx_do_tch_proc = welder->dout_portno.nTouchStart; 
        g_idx_do_wirefeed = welder->dout_portno.nInchPos; 
        g_idx_do_wireback = welder->dout_portno.nInchNeg; 

        // do on level 
        g_onlev_do_arc = welder->dout_actlev.nArcOn; 
        g_onlev_do_gas = welder->dout_actlev.nGasOn; 
        g_onlev_do_tch_mc   = welder->dout_actlev.nTouchReady; 
        g_onlev_do_tch_proc = welder->dout_actlev.nTouchStart; 
        g_onlev_do_wirefeed = welder->dout_actlev.nInchPos; 
        g_onlev_do_wireback = welder->dout_actlev.nInchNeg; 

        // di index
        g_idx_di_arc = welder->din_portno.nArcOn;     
        g_idx_di_tch_proc = welder->din_portno.nTouchProcess; 
        g_idx_di_tch_sig  = welder->din_portno.nTouchSignal; 
        g_idx_di_pwr_fail = welder->din_portno.nWeldPowerFail; 
        g_idx_di_no_gas   = welder->din_portno.nNoGas; 

        // di level
        g_onlev_di_arc = welder->din_actlev.nArcOn;     
        g_onlev_di_tch_proc = welder->din_actlev.nTouchProcess; 
        g_onlev_di_tch_sig  = welder->din_actlev.nTouchSignal; 
        g_onlev_di_pwr_fail = welder->din_actlev.nWeldPowerFail; 
        g_onlev_di_no_gas   = welder->din_actlev.nNoGas; 
    
        // ao index
        g_idx_ao_volt = welder->nVoltageOutPortNo; 
        g_idx_ao_curr = welder->nCurrentOutPortNo; 
        g_idx_ao_wire = welder->nCurrentOutPortNo; 

        g_cweave_touch_dist   = rob->weld_func.dbCWeavTouchUpDis; 
        g_cweave_horz_margin  = rob->weld_func.dbCWeavHorMargin; 
        g_cweave_weldleg_dist = rob->weld_func.dbCWeavWeldLegDis; 

        // Bvar Indexes for Special Program Control 
        g_i_bvar_left    = rob->weld_func.nLeftWeldBvar; 
        g_i_bvar_right   = rob->weld_func.nRightWeldBvar; 
        g_i_bvar_gap_sel = rob->weld_func.nGapRefBvar; 
	}

    g_bTw = TRANS_Eye(); 
    g_eTt = TRANS_Xyzrpy(tcp); 
    g_tTs = TRANS_Eye(); 

    // Trajectory Setting 
    Traj_Set_Dynamics(&g_traj, g_t_samp, 
                      g_jerk_axis, g_acc_axis, g_dec_axis_stop, g_dec_axis_estop, 
                      g_jerk_lin,  g_acc_lin, g_dec_lin_stop, g_dec_lin_estop,
                      g_jerk_ori,  g_acc_ori, g_dec_ori_stop, g_dec_ori_estop,  
                      g_Inverse, g_Forward, g_Config, g_dh, &g_eTt);     

    Traj_Set_Limit(&g_traj, g_lim_max, g_lim_min, 
                   g_vel_axis_max, g_vel_lin_max, g_vel_ori_max, NEAR_POS); 

    printf("Traj Info's. Unit:[rad], [rad/s], [rad/s^2]\n%s", Traj_Print(&g_traj)); 

    printf("Welder IO Mapping \n%s", Str_WeldIoMap()); 
}

////////////////////////////////////////////////////////////////////////////////
