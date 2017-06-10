
#include <unistd.h>

int __errno = 0;
void *_impure_ptr = NULL;

void __sinit(void) {
}

void memset(void *s, int c, size_t n) {
    char *end = (char *)s + n;
    for (char *p = (char *)s; p < end; p++)
        *p = (char)c;
}

size_t strlen(const char *s) {
    const char *start = s;
    while (*s++);
    return s - start - 1;
}
