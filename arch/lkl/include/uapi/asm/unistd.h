#define __ARCH_WANT_SYSCALL_NO_AT
#define __ARCH_WANT_SYSCALL_DEPRECATED
#define __ARCH_WANT_SYSCALL_NO_FLAGS

//#if __BITS_PER_LONG == 64
#define __ARCH_WANT_SYS_NEWFSTATAT
//#endif

#include <asm-generic/unistd.h>

#define __NR_create_syscall_thread		(__NR_arch_specific_syscall + 0)
#define __NR_virtio_mmio_device_add		(__NR_arch_specific_syscall + 1)
