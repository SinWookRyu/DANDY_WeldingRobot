/////////////////////////////////////////////////////////////////////////////
//
//  timer pulse for Unix include QNX Neutrino
//
//  QNX Neutrino has been supported at base kernel.
//  so all i have to do is just prepare 'sigevent' with SIGEV_PULSE_INIT()
//      and then create the timer by 'timer_create()' with the 'sigevent'
//
//  But the pulse is not standard at other posix os as well as Linux.
//  timer can be supported only signal and its handlers.
//  so we should sned the pulse at the signal handler. (signal is SIGALRM)
//
//  Linux supports just one thread with 'SIGEV_THREAD_ID' for the all timer signal handler.
//  one permanent thread is more effective than each thread for the timer.
//  because we have only one thread for the various timer signal handler.
//  so it will be used less system resources and CPU consumption.
// 

#if !defined(_WIN32)
/////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <unistd.h>
#include <signal.h>

#include <time.h>
#include <pthread.h>

#if defined(__linux__)
#include <sys/syscall.h>    // for gettid()
#endif

///////////////////////////////////////

#include "dandy_msgpass.h"
#include "dandy_thread.h"

#include "dandy_timecon.h"
// #include "dandy_timecon_priv.h"

/////////////////////////////////////////////////////////////////////////////
//
//  TIME_RegTimerPulse()
//

int TIME_RegTimerPulseNanosec(int coid, int nCode, int nValue, int nNanosec, int nOption)
{
    timer_t timerid;
    struct sigevent se;

    struct itimerspec itime;
    int nSec, nNanoRes;

    ///////////////////////////////////
    //
    //
    //

//  TIME_ASSERT(coid != INVALID_COID);
// 	TIME_ASSERT(nNanosec > 0);

    if (nNanosec <= 0)
    {
        // TIME_ERROR("required interval is too illegal : %d > 0\n",
        // 		nNanosec);

        return INVALID_TIMER_PULSE;
    }

    ///////////////////////////////////
    //
    //
    //

    memset(&se, 0, sizeof(se));

    // QNX Neutrino
#if defined(__QNXNTO__)
    SIGEV_PULSE_INIT(&se, coid, SIGEV_PULSE_PRIO_INHERIT, nCode, nValue);

    //  Linux specific
#elif defined(SIGEV_THREAD_ID)
    ///////////////////////////////////
    //
    //  create the timer with 'SIGEV_THREAD_ID' event
    //
    //  sigev_notify_thread_id = thread id by gettid()
    //

    TIME_ASSERT(s_TimerPulseThreadId != 0);

    se.sigev_notify = SIGEV_THREAD_ID;
    se.sigev_signo = SIGALRM;
    //se.sigev_notify_thread_id = s_TimerPulseThreadId;
    se.sigev_notify_function = (void*) s_TimerPulseThreadId;
    se.sigev_value.sival_ptr = (void*) pInfo;

    // generic POSIX
#else
    ///////////////////////////////////
    //
    //  create the timer with 'SIGEV_THREAD' event
    //
    //  sigev_value = thread parameter
    //  sigev_notify_attributes = pthread_attr_t
    //

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_signo = SIGALRM;
    se.sigev_notify_function = (void*) _TIME_NQ_TimerPulseHandler;
    se.sigev_value.sival_ptr = (void*) pInfo;
#endif

    ///////////////////////////////////
    //
    //
    //

    if (timer_create(CLOCK_MONOTONIC,
                     &se,
                     &timerid) == -1)
    {
        // TIME_ERROR("timer_create() error : errno=%d\n", errno);

        return INVALID_TIMER_PULSE;
    }

    // TIME_VERBOSE("timer created : timer=%d\n", (int) timerid);

#if !defined(__QNXNTO__)
    pInfo->timerid = timerid;
#endif

    ///////////////////////////////////
    //
    //
    //

    nSec = nNanosec / (1000 * 1000 * 1000);
    nNanoRes = nNanosec % (1000 * 1000 * 1000);

    itime.it_value.tv_sec  = nSec;
    itime.it_value.tv_nsec = nNanoRes; 

    itime.it_interval.tv_sec = nSec;
    itime.it_interval.tv_nsec = nNanoRes; 

    ///////////////////////////////////
    //
    //
    //

    if (timer_settime(timerid, 0, &itime, NULL) == -1)
    {
        timer_delete(timerid);
        
        // TIME_ERROR("timer_settime() error : errno=%d\n", errno);

        return INVALID_TIMER_PULSE;
    }

    return (int) timerid;
}

#endif  // !defined(_WIN32)
/////////////////////////////////////////////////////////////////////////////
