// syscalls.c
#include <sys/stat.h>
#include <unistd.h>

int _write(int file, char *ptr, int len) {
    return len;
}
int _read(int file, char *ptr, int len) {
    return 0;
}
int _close(int file) {
    return -1;
}
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}
int _lseek(int file, int ptr, int dir) {
    return 0;
}
int _isatty(int file) {
    return 1;
}
int _getpid(void) {
    return 1;
}
int _kill(int pid, int sig) {
    return -1;
}
void _exit(int status) {
    while (1);
}
void *_sbrk(ptrdiff_t incr) {
    extern char _end;  // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) heap_end = &_end;
    prev_heap_end = heap_end;
    heap_end += incr;
    return (void *)prev_heap_end;
}
