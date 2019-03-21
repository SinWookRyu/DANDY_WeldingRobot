/*
 * argument.h
 *  This is the header file for argument managing program for TASKEXEC.
 *  2013. 7. 15. mrch0
 */

#ifndef ARGUMENT_H_20130715_
#define ARGUMENT_H_20130715_

#if defined(__cplusplus)
extern "C" {
#endif

// Sub-Argument for Print Option
#define ARG_VERB    ('v')
#define ARG_MSG     ('m')
#define ARG_WARN    ('w')
#define ARG_ERR     ('e')
#define ARG_NONE    ('n')

// No help argument parsing : return 0
int Arg_Init(int argc, char* argv[]);

#if defined(__cplusplus)
}
#endif

#endif
