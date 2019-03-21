#include <stdio.h>
#include <stdlib.h>

#include "servocon_main.h"
#include "dandy_echo.h"
#include "dandy_msgpass.h"
#include "dandy_thread.h"
#include "CRT.h"


//////////////////////////////////////////////////////////////////////////
//
// MSG_SC_ConnectChannel()
//
// -pName: channel name to connect
// -return: connection ID
int MSG_SC_ConnectChannel(const char* pName, int nOpt)
{
    int coid = -1;
        
    coid  = MSG_AttachNamedConnection(pName);

    if(nOpt != OPT_QUITE)
    {
        if (coid == -1)
        {
            VERBOSE_ERROR("Cannot connect channel : <%s>\n", pName);
        }
        else
        {
            VERBOSE_VERBOSE("Connected channel : <%s>, <COID: %d>\n",
                            pName, coid);
        }
    }

    return coid;
}


////////////////////////////////////////////////////////////////////////////////
//
// MSG_SC_CreateChannel()
//
// -pName : channel name
// -return: channel identifier(CHID)
//
int MSG_SC_CreateChannel(const char* pName)
{
    int chid;

    chid = MSG_CreateNamedChannel(pName);

    if (chid == -1)
    {
        VERBOSE_ERROR("Failed to create channel : <%s>\n", pName);
    }
    else
    {
        VERBOSE_VERBOSE("Created channel : <%s>, <CHID : %d>\n", 
                        pName, 
                        chid);
    }

    return chid;
}


////////////////////////////////////////////////////////////////////////////////
//
// MSG_NamedDestroyChannel()
//
// -pName: channel name to destroy
// -chid : channel identifier
//
int MSG_NamedDestroyChannel(const char* pName, int chid)
{
    int nRet;

    nRet = MSG_DestroyChannel(chid);

    if (nRet == -1)
    {
        VERBOSE_ERROR("Failed to destroy channel : <%s>, <chid : %d>\n",
                      pName, 
                      chid);
    }
    else
    {
        VERBOSE_VERBOSE("Destroyed channel : <%s>, <chid : %d>\n",
                        pName, 
                        chid);
    }

    return nRet;
}


////////////////////////////////////////////////////////////////////////////////
//
// MSG_NamedDetachConnection()
//
// -pName: channel name to detach
// -coid : connection identifier
//
int MSG_NamedDetachConnection(const char* pName, int coid)
{
    int nRet;

    nRet = MSG_DetachConnection(coid);

    if (nRet == -1)
    {
        VERBOSE_ERROR("Failed to detach connection : <%s>, <COID : %d>\n", 
                      pName, 
                      coid);
    }
    else
    {
        VERBOSE_VERBOSE("Detached connection : <%s>, <COID : %d>\n", 
                        pName, 
                        coid);
    }

    return nRet;
}
