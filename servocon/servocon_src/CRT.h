// C Runtime Library Compatible with Sequre String Method of MS and QNX

#ifndef CRT_H_
#define CRT_H_

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "errno.h"

#if (__STDC_SECURE_LIB__ > 0)

#define CRT_gets(str_, size_str_)               	gets_s(str_, size_str_)
#define CRT_strcpy(trg_, size_trg_, src_)       	strcpy_s(trg_, size_trg_, src_)
#define CRT_strncpy(trg_, size_trg_, src_,cnt_src_)  strncpy_s(trg_, size_trg_, src_, cnt_src_)
#define CRT_itoa(val_, trg_, size_, radix_)      	_itoa_s(val_, trg_, size_, radix_)
#define CRT_sprintf(buffer, buffer_size, stringbuffer, ...)      sprintf_s(buffer, buffer_size, stringbuffer, ##__VA_ARGS__)

#define CRT_fopen(pFile, filename, mode)        	fopen_s(pFile, filename, mode)
#define CRT_strcat(trg_, size_trg_, src_)       	strcat_s(trg_, size_trg_, src_)
#define CRT_access(path_, mode_)					_access(path_, mode_)

#else

#define CRT_gets(str_, size_str_)               	fgets(str_, size_str_, stdin)
#define CRT_strcpy(trg_, size_trg, src_)        	strcpy(trg_, src_)
#define CRT_strncpy(trg_, size_trg_, src_,cnt_src_) strncpy(trg_, src_, cnt_src_)
#define CRT_itoa(val_, trg_, size_, radix_)     	itoa(val_, trg_, radix_)
#define CRT_sprintf(buffer, buffer_size, stringbuffer, ...)      sprintf(buffer, stringbuffer, ##__VA_ARGS__)

#define CRT_fopen(pFile, filename, mode)        	((*(pFile) = fopen(filename, mode))? errno : errno)
#define CRT_strcat(trg_, size_trg_, src_)       	strcat(trg_, src_)
#define CRT_access(path_, mode_)					access(path_, mode_)

#endif

#endif
