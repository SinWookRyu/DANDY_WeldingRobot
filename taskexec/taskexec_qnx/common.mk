# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#===== EXTRA_LIBVPATH - a space-separated list of directories to search for library files.
EXTRA_LIBVPATH+=D:/program/dandy2015/lib/archive  \
	D:/program/MIP_QNX_v1.5.180.0/bin/x86/so  \
	F:/DANDY2015/Lib/dandy_2015_140630/lib/archive  \
	F:/DANDY2015/Lib/MRT_QNX_v1.5.180.0/bin/x86/so

#===== LIBS - a space-separated list of library items to be included in the link.
LIBS+=-Bstatic dandy_all_qnx m -Bdynamic mkpaiodev

#===== EXTRA_SRCVPATH - a space-separated list of directories to search for source files.
EXTRA_SRCVPATH+=F:/DANDY2015/Integrated_Project/taskexec/src  \
	F:/DANDY2015/Integrated_Project/taskexec/taskexec_src  \
	F:/DANDY2015/Lib/dandy_2015_140630/lib/include

#===== EXTRA_INCVPATH - a space-separated list of directories to search for include files.
EXTRA_INCVPATH+=F:/DANDY2015/Lib/dandy_2015_140630/lib/include

#===== POST_BUILD - extra steps to do after building the image.
define POST_BUILD
-@$(CP_HOST) $(BUILDNAME) F:/DANDY2015/Integrated_Project/taskexec/taskexec_qnx/bin/$(BUILDNAME)
endef

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

postbuild:
	$(POST_BUILD)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))

