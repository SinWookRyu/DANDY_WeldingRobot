/////////////////////////////////////////////////////////////////////////////
//
//
//
//

#ifndef __DANDY_NETWORK_PROTO_H__
#define __DANDY_NETWORK_PROTO_H__

#include "dandy_platform.h"

#include "ipc_robotmgr.h"
#include "ipc_taskexec.h"
#include "ipc_servocon.h"

#include "sys_conf.h"
#include "error_def.h"

///////////////////////////////////////
//
//  __DANDY_PACKET_INLINE__
//

#ifndef __DANDY_NETWORK_INLINE__
#if defined(_MSC_VER)
#define __DANDY_NETWORK_INLINE__    __forceinline
#elif defined(__GNUC__)
#define __DANDY_NETWORK_INLINE__    static __inline__
#else
#define __DANDY_NETWORK_INLINE__    static
#endif
#endif

///////////////////////////////////////
//
//  dandy network supported flags
//

#define DANDY_NETWORK_INET      1   // inet4
#define DANDY_NETWORK_INET6     1   // inet6
#define DANDY_NETWORK_NAME      "DD2015_NETWORK"

///////////////////////////////////////
//
//  packet related definitions
//

// service owner
#define DANDY_SVC_OWNER_BROKER  0   // broker itself (refer to DNB_CODE_xxx)
#define DANDY_SVC_OWNER_RM      1   // robotmgr
#define DANDY_SVC_OWNER_TE      2   // taskexec
#define DANDY_SVC_OWNER_SC      3   // servocon
#define DANDY_SVC_OWNER_INVALID -1  // do not use, internal use only

// service request type
#define DANDY_SVC_TYPE_NONE     0   // just for reply (do not use)
#define DANDY_SVC_TYPE_PULSE    1   // request the service with pulse
#define DANDY_SVC_TYPE_MESSAGE  2   // request the service with message
    // 'pulse' type is fast, but works as asynchronous
    // 'message' type can be checked definite result

// maximum packet data size
#define DANDY_PACKET_HEADER_SIZE        (4 * 8) // checksum/result/param/owner/type/code/value/size
#define DANDY_PACKET_DATA_MAX_SIZE      1024
#define DANDY_PACKET_MAX_SIZE           (DANDY_PACKET_HEADER_SIZE + DANDY_PACKET_DATA_MAX_SIZE)

#define DANDY_PACKET_CHECKSUM_HEADER_SIZE   (4 * 7) // header size except 'checksum'

// network communication error (error code from broker)
// error codes from the broker are less than zero
#define DANDY_PACKET_RESULT_OK          0       // service dispatched successfully
#define DANDY_PACKET_RESULT_FAILURE     -1      // service dispatch failure
#define DANDY_PACKET_RESULT_CHECKSUM    -2      // checksum error
#define DANDY_PACKET_RESULT_ARGUMENT    -3      // packet argument error
#define DANDY_PACKET_RESULT_SIZE        -4      // packet size mismatch or broken packet
#define DANDY_PACKET_RESULT_DNB_FAILURE -5      // DNB service failure
#define DANDY_PACKET_RESULT_DNB_PARTIAL -6

// packet sender should be init the 'result' with this
//      to avoid conflict with packet reflection
#define DANDY_PACKET_RESULT_SENDER      -99

///////////////////////////////////////
//
// DANDY_PACKET
//

#pragma pack(push, 1)
typedef struct _DANDY_PACKET
{
    ///////////////////////////////////
    //
    //  network header
    //

    // checksum for the packet (below of the 'checksum', except this)
    unsigned    checksum;

    // resultant of the service
    //  (must be DANDY_PACKET_RESULT_SENDER on request)
    int     result; // >0 : result from siblings, <0 : result from broker 
    //      broker result DANDY_PACKET_RESULT_OK/FAILURE/...

    // client side parameter (user specific parameter)
    int     param;  // 'param' is reflected same value at recv on sending 'param'

    int     owner;  // service owner (DANDY_SVC_OWNER_RM/TE/SC)
    int     type;   // service request type (DANDY_SVC_TYPE_PULSE/MESSAGE)
                    // 'type' is ignored if owner=DANDY_SVC_OWNER_BROKER

    ///////////////////////////////////
    //
    //  service specifics
    //

    int     code;   // service code
    int     value;  // service value (service code protocol dependant)
    int     size;   // size of the 'data' (zero means no data)
    
    // message body (refer to 'ipc_robotmgr.h', 'ipc_taskexec.h', 'ipc_servocon.h')
    unsigned char   data[DANDY_PACKET_DATA_MAX_SIZE];
} DANDY_PACKET; 
#pragma pack(pop)

///////////////////////////////////////
//
// DANDY_PACKET_SIMPLE
//

#pragma pack(push, 1)
typedef struct _DANDY_PACKET_SIMPLE
{
    // checksum for the packet (below of the 'checksum', except this)
    unsigned    checksum;

    // resultant of the service
    //  (must be DANDY_PACKET_RESULT_SENDER on request)
    int     result;
    int     param;  // user specific param
    // 'param' is reflected same value at recv on send value

    int     owner;  // service owner (DANDY_SVCOWNER_RM/TE/SC)
    int     type;   // service request type (DANDY_SVC_TYPE_PULSE/MESSAGE)

    // service specifics
	int     code;   // service code
	int     value;  // service value (service code protocol dependant)
    int     size;   // must be always zero
} DANDY_PACKET_SIMPLE; 
#pragma pack(pop)

///////////////////////////////////////
//
//  GET_DANDY_PACKET_SIZE()
//  GET_DANDY_PACKET_CHECKSUM_SIZE()
//

// GET_DANDY_PACKET_SIZE()
#define GET_DANDY_PACKET_SIZE(__p)  (DANDY_PACKET_HEADER_SIZE + (__p)->size)

// GET_DANDY_PACKET_CHECKSUM_SIZE()
#define GET_DANDY_PACKET_CHECKSUM_SIZE(__p)     \
        (DANDY_PACKET_CHECKSUM_HEADER_SIZE + (__p)->size)

///////////////////////////////////////
//
//  checksum
//

// DANDY_NETWORK_CalcChecksum()
__DANDY_NETWORK_INLINE__
unsigned DANDY_NETWORK_CalcChecksum(const void* pData, int nSize)
{
    const unsigned char* p = (const unsigned char*) pData;

    unsigned nChecksum = 0;
    int iByte;

    nChecksum = 0;

    for (iByte = 0; iByte < nSize; iByte++)
    {
        nChecksum += *p++;
    }

    return nChecksum;
}

// DANDY_PACKET_CalcChecksum()
__DANDY_NETWORK_INLINE__
unsigned DANDY_PACKET_CalcChecksum(const DANDY_PACKET* pPacket)
{
    return DANDY_NETWORK_CalcChecksum(&pPacket->result,
                                      GET_DANDY_PACKET_CHECKSUM_SIZE(pPacket));
}

/////////////////////////////////////////////////////////////////////////////
//
//  NETWORK BROKER CHANNEL PACKET (no network, local only)
//

#define DNB_MSG_DATA_MAX_SIZE   DANDY_PACKET_DATA_MAX_SIZE

// Format
#pragma pack(push, 1)
typedef struct _DNB_MSG
{
    int             code; 
    int             value;
    int             size;   // size of 'data'
    unsigned char   data[DNB_MSG_DATA_MAX_SIZE];
} DNB_MSG;
#pragma pack(pop)

/////////////////////////////////////////////////////////////////////////////
//
//  NETWORK BORKER SERVICES
//  (DNB = Dandy Network Broker)
//

#define DNB_CODE_EXIT       0       // terminate DNB (only through the channel)
#define DNB_CODE_REFLECT    0       // reflect the packet except 'checksum' and 'result'
                                    //          only through network (peer)
#define DNB_CODE_SB_TIMER   1       // internal use only (sibling connection check, only channel)
#define DNB_CODE_VERSION    2       // DNB version (null terminated string)
#define DNB_CODE_OSVERSION  3       // Operating systems version (null terminated string)
#define DNB_CODE_RECONNECT  4       // reconnect to cores (RM/TE/SC)

///////////////////////////////////////
//
//  DNB_CODE_VERSION
//
//              call                    |   reply
//  code  :     DNB_CODE_OSVERSION      |   calling code
//  value :     0 (ignored)             |   calling value
//  size  :     0                       |   length (inc. null char)
//  data  :     -                       |   DNB version string
//

///////////////////////////////////////
//
//  DNB_CODE_OSVERSION
//
//              call                    |   reply
//  code  :     DNB_CODE_OSVERSION      |   calling code
//  value :     0 = short version       |   calling value (0 ~ 2)
//              1 = long version        |   
//              2 = full version        |   
//  size  :     0                       |   length (inc. null char)
//  data  :     -                       |   operating systems version string
//
#define DNB_VAL_OSVER_SHORT     0
#define DNB_VAL_OSVER_LONG      1
#define DNB_VAL_OSVER_FULL      2

///////////////////////////////////////
//
//  DNB_CODE_RECONNECT
//
//              call                    |   reply
//  code  :     DNB_CODE_RECONNECT      |   calling code
//  value :     0 (all cores, RM/TE/SC) |   calling value (0 ~ 2)
//              1 = reconnect RM        |   
//              2 = reconnect TE        |   
//              3 = reconnect SC        |
//              (use DANDY_SVC_OWNER_BROKER/RM/TE/SC)
//  size  :     0                       |   length (inc. null char)
//  data  :     -                       |   operating systems version string
//

#endif  // __DANDY_NETWORK_PROTO_H__