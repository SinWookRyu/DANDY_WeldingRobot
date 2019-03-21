#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "taskexec_def.h"
#include "te_serv.h"

////////////////////////////////////////////////////////////////////////////////

// Initialization TE
// 0) Argument Proc
// 1) Verbose Init
// 2) TE Resource Init
// 3) Receive Blocking : Init Msg from RM
// 4) IPC Resource Init & Setting Init Flag 
// 5) Receive Blocking : TE Original Serivce 

// Uninitialization TE (As Order of Init)
// 1) TE Resource Uninit
// 2) Ipc Uninit & Setting Exit Flag
// 3) Verbose Uninit
// 4) Exits

int main(int argc, char* argv[])
{	
	MSG_INFO	msg_info; 
	TE_MSG		msg;
    TE_REPLY    reply; 
    
    int n_status; 	
    int	n_reply; 	
	int rcvid; 
	int f_exit = 0;	
	int ret; 

	THREAD_SetPriorityClass(THREAD_CLASS_PRIO_REALTIME);
	THREAD_SetSchedule(0, THREAD_POLICY_FIFO, THREAD_PRIO_ABOVE_NORMAL);    
	printf("TASKEXEC main starts with the priority %d\n", THREAD_GetPriority(0));

	if(Arg_Init(argc, argv))
    {
        return 0; 
    }

#if defined(__QNX__)
    // for qnx time consuming calculation
    CPS = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
#endif

    Init_VerboseInit(); 

	ret = Init_TeResourceInit(); 
	if(ret)
	{
		goto EXIT_PROCESS; 
	}	
    VERBOSE_MESSAGE("Completed to initialized the local resources.\n"); 

	///// Main Service Loop	////////////////////////////////////////////////////

	while(!f_exit) 
	{
        // Receive Blocking
		rcvid = MSG_Receive(g_ch_te, &msg, sizeof(msg), &msg_info);		
        if(rcvid < 0)
		{
			VERBOSE_WARNING("Failed MSG Receiption.\n");
            continue; 
		}

        // Before Init Service Filtering 
        if (g_f_ipc_init == 0 && 
            msg.code != TESERV_EXIT     && msg.code != TESERV_INIT && 
            msg.code != TESERV_LIFE_CHK && msg.code != TESERV_VERSION &&
            msg.code != TESERV_STOP)        
        {
            VERBOSE_WARNING("Unsupport service before init. Code:%d, Val:%d\n", msg.code, msg.value);			
			n_reply = 0; 	
            n_status = -1; 
        }        
        // Main Service Loop. 
        else
        {
            n_status = TeServ_Handler(msg.code, &reply.data, &n_reply, &msg);
            
            if(n_status)
            {
                VERBOSE_WARNING("Unsupport service. Code:%d, Val:%d\n", msg.code, msg.value);						    
            }
            if(msg.code == TESERV_EXIT)
            {
                f_exit = TRUE;
            }                           
        }        

		// Reply 
		if(rcvid == 0)
		{
			;	// pulse, no reply 
		}
		else if(rcvid > 0)
		{   
            // Reply Header Setting
            reply.code  = msg.code; 
            reply.value = msg.value;    
            reply.size  = n_reply; 

			MSG_Reply(rcvid, n_status, &reply, TE_REPLY_HEAD_SIZE + n_reply);
            VERBOSE_VERBOSE("Replied for the message. Code:%d Val:%d Size:%d Ret:%d\n", 
                reply.code, reply.value, TE_REPLY_HEAD_SIZE + n_reply, n_status); 
		}
		else
		{
			VERBOSE_WARNING("Failed MSG Receiption.\n");
		}
	}	

EXIT_PROCESS : 

    VERBOSE_MESSAGE("Enters to the exit process.\n"); 

    Init_TeResourceUninit(); 

    Init_SetExitFlag(); 

    Init_IpcUninit(); 
    
    Init_VerboseUninit();    
    
    printf("TE Exits process. Good bye. Have a nice day!\n"); 
                 
    g_f_taskexec_exit = TRUE; 

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
