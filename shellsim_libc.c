/* SPDX-License-Identifier: MIT */
/* Bridge POSIX calls absent from WASI Preview 1 to shellsim's virtual syscalls. */

#include <errno.h>
#include <string.h>
#include <sys/stat.h>

extern int shellsim_path_chmod(const char *, unsigned, unsigned)
    __asm__("shellsim.path_chmod");

int chmod(const char *path, mode_t mode)
{
    int error;
    if (!path) {
        errno = EFAULT;
        return -1;
    }
    error = shellsim_path_chmod(path, strlen(path), (unsigned)mode);
    if (error) {
        errno = error;
        return -1;
    }
    return 0;
}
