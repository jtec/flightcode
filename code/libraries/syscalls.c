/*
 * syscalls.c
 *
 *  Created on: 03.12.2009, modified 2.6.2012
 *      Author: Martin Thomas, 3BSD license
 */

#define SBRK_VERBOSE 1
#define EXIT_VERBOSE 1

/* Set the following non-null to enable a workaround for GNU tools for ARM
 * (launchpad) where libgcc is configured to through exceptions in ldiv on
 * division by 0. This saves around 4kBytes of flash-memory.
 * This may be removed/disabled in future releases of the tool-chain and is also
 * not needed for Codebench lite. */
#define FORCE_DIV_EXCEPTION_OFF 1

#include <sys/types.h>

#include "stm32f4xx.h" /* for __get_PSP() __get_MSP() */

extern int __io_putchar(int);

#undef errno
extern int errno;

#if FORCE_DIV_EXCEPTION_OFF
/* reference: http://devkitpro.org/viewtopic.php?f=2&t=2944 */
void __aeabi_unwind_cpp_pr1(){}
//void __aeabi_unwind_cpp_pr0(){}
#endif

void _init()
{

}

void __dso_handle(){

}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig; /* avoid warnings */
	//errno = EINVAL;
	return -1;
}

void _exit(int status)
{
	while(1) {;}
}

int _getpid(void)
{
	return 1;
}


extern char _end; /* Defined by the linker */
static char *heap_end = 0;

char* get_heap_end(void)
{
	return (char*) heap_end;
}

char* get_stack_top(void)
{
	return (char*) __get_MSP();
	//return (char*) __get_PSP();
}

caddr_t _sbrk(int incr)
{
	char *prev_heap_end;

	// Initialize 'heap_end' the firsts time the function gets called
	if (heap_end == 0) {
		heap_end = &_end;
	}
	prev_heap_end = heap_end;


	char* stackTop = get_stack_top();
	if (heap_end + incr > get_stack_top()) {

		abort();
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _close(int file)
{
	(void)file; /* avoid warning */
	return -1;
}

int _fstat(int file, struct stat *st)
{
	(void)file; /* avoid warning */
	//st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	(void)file; /* avoid warning */
	return 1;
}

int _lseek(int file, int ptr, int dir) {
	(void)file; /* avoid warning */
	(void)ptr; /* avoid warning */
	(void)dir; /* avoid warning */
	return 0;
}

int _read(int file, char *ptr, int len)
{
	(void)file; /* avoid warning */
	(void)ptr; /* avoid warning */
	(void)len; /* avoid warning */
	return 0;
}

int _write(int file, char *ptr, int len)
{
	int todo;

	return 0;
}
