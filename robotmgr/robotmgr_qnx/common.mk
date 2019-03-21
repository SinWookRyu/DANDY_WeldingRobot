# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#===== CCFLAGS - add the flags to the C compiler command line. 
CCFLAGS+=-w8 -l

#===== EXTRA_INCVPATH - a space-separated list of directories to search for include files.
EXTRA_INCVPATH+=F:/DANDY2015/Lib/dandy_2015_140630/lib/include

#===== EXTRA_LIBVPATH - a space-separated list of directories to search for library files.
EXTRA_LIBVPATH+=F:/DANDY2015/Lib/dandy_2015_140630/lib/archive

#===== EXTRA_SRCVPATH - a space-separated list of directories to search for source files.
EXTRA_SRCVPATH+=F:/DANDY2015/Project/robotmgr/robotmgr_src

#===== LIBS - a space-separated list of library items to be included in the link.
LIBS+=-Bstatic ^dandy_all_qnx -Bdynamic ^malloc -Bstatic m  \
	-Bdynamic

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))

