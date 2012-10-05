/*
 * syscalls.h
 *
 *  Created on: 03.12.2009
 *      Author: Martin Thomas, 3BSD license
 */

#ifndef SYSCALLS_H_
#define SYSCALLS_H_

#include <reent.h>
#include <errno.h>
#include <stdlib.h> /* abort */
#include <sys/types.h>
#include <sys/stat.h>

//#include "term_io.h"
//#include "stm32f10x.h" /* for _get_PSP() from core_cm3.h*/

#undef errno
extern int errno;

int _kill(int pid, int sig);
void _exit(int status);


extern char _end; /* Defined by the linker */
static char *heap_end;

char* get_heap_end(void);

char* get_stack_top(void);

caddr_t _sbrk(int incr);

int _close(int file);

int _fstat(int file, struct stat *st);

int _isatty(int file);

int _lseek(int file, int ptr, int dir);

int _read(int file, char *ptr, int len);

int _write(int file, char *ptr, int len);
/*
void _init(){
	return;
}

void _fini(){
	return;
}
*/


#endif /* SYSCALLS_H_ */
