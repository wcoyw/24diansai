/**
 * @file    retarget.c
 * @brief   Semihosting-free retarget for printf (CCS / TI ARM compiler)
 *
 * For CCS (TI ARM Clang / tiarmclang), the standard C library uses
 * low-level syscalls.  This file provides minimal stubs so printf()
 * works without semihosting.
 *
 * If using GCC (arm-none-eabi) instead of TI compiler, replace with
 * the equivalent newlib stubs or link with --specs=nosys.specs.
 */
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  TI ARM compiler (tiarmclang / armcl) stubs                        */
/* ------------------------------------------------------------------ */
#if defined(__ti__) || defined(__TI_COMPILER_VERSION__)

#include <stdio.h>
#include <stdlib.h>

/*
 * fputc() is already defined in main.c (redirects to UART0).
 * The TI RTS library may need these additional stubs.
 */

int fclose(FILE *f)    { (void)f; return 0; }
int fflush(FILE *f)    { (void)f; return 0; }
int fgetc(FILE *f)     { (void)f; return EOF; }
int fseek(FILE *f, long o, int w) { (void)f; (void)o; (void)w; return EOF; }
long ftell(FILE *f)    { (void)f; return -1L; }

void _exit(int status) { (void)status; while(1) { } }
int _open(const char *path, int flags, ...)  { (void)path; (void)flags; return -1; }
int _close(int fd)     { (void)fd; return -1; }
int _read(int fd, char *buf, unsigned len)   { (void)fd; (void)buf; (void)len; return -1; }
int _write(int fd, const char *buf, unsigned len) { (void)fd; (void)buf; (void)len; return (int)len; }
int _lseek(int fd, long offset, int whence)  { (void)fd; (void)offset; (void)whence; return 0; }
int _isatty(int fd)    { (void)fd; return 1; }
int _fstat(int fd, void *buf) { (void)fd; (void)buf; return 0; }
void * _sbrk(int incr) { (void)incr; return NULL; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
int _getpid(void)      { return 1; }

/* ------------------------------------------------------------------ */
/*  GCC (arm-none-eabi) stubs                                          */
/* ------------------------------------------------------------------ */
#elif defined(__GNUC__) && !defined(__CC_ARM)

/* With --specs=nosys.specs or -lnosys, most stubs are provided. */
/* Add any missing ones here if needed. */

#endif
