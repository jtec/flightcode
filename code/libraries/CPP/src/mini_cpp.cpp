//////////////////////////////////////////////////////////////////////////////
// Minimal Embedded C++ support, no exception handling, no RTTI
// Date of the Last Update:  Jun 15, 2007
//
//                    Q u a n t u m     L e a P s
//                    ---------------------------
//                    innovating embedded systems
//
// Copyright (C) 2002-2007 Quantum Leaps, LLC. All rights reserved.
//
// Contact information:
// Quantum Leaps Web site:  http://www.quantum-leaps.com
// e-mail:                  info@quantum-leaps.com
//////////////////////////////////////////////////////////////////////////////
// very minor modification to avoid warnings by Martin Thomas 12/2009
// Linking with the object-code from this file saves around 20kB program-memory
// in a demo-application for a Cortex-M3 (thumb2, CS G++ lite Q1/2009).
// Further information can be found in the documents from Quantum Leaps.

#include <stdlib.h>                   // for prototypes of malloc() and free()
#include <reent.h>
#include <errno.h>
#include <stdlib.h> /* abort */
#include <sys/types.h>
#include <sys/stat.h>


extern "C" caddr_t _sbrk(int incr);

//............................................................................
void *operator new(size_t size) throw() {
	void* whatMallocSays = malloc(size);
	// FIXME malloc() passed very high size values to _sbrk(), up to 4096 bytes,
	// and malloc after a few calls started returning 0. As workaround, use directly
	// the next free address on the heap delivered by _sbrk();
	void* whatSBRKSays = _sbrk((int)size);
	return whatSBRKSays;
}
//............................................................................
void operator delete(void *p) throw() {
	free(p);
}
//............................................................................
extern "C" int __aeabi_atexit(void *object,
                              void (*destructor)(void *),
                              void *dso_handle)
{
	// object = object;
	// destructor=destructor;
	// dso_handle=dso_handle;

	return 0;
}
