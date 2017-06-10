
#define SYS_WRITE0 0x04

void write0(const char *c) __attribute__((naked));
void write0(const char *c) {
    __asm__("mov r1, %0" : : "r" (c));
    __asm__("mov r0, %0" : : "I" (SYS_WRITE0));
    __asm__("bkpt 0xab");
    __asm__("bx lr");
}
