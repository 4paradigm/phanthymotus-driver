/* SPDX-License-Identifier: Apache-2.0
 * Build-only workaround for CMake/KWSys Directory::Load retaining errno from
 * successful allocations between readdir calls. See ros2/rcutils#525,
 * issuecomment-4049423490. Reset before each real call, never after it: actual
 * directory errors still propagate. No Driver runtime uses this interposer.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <stddef.h>

static struct dirent *(*next_readdir)(DIR *);
static struct dirent64 *(*next_readdir64)(DIR *);

__attribute__((constructor)) static void resolve_readdir(void)
{
    next_readdir = dlsym(RTLD_NEXT, "readdir");
    next_readdir64 = dlsym(RTLD_NEXT, "readdir64");
}

struct dirent *readdir(DIR *directory)
{
    if (!next_readdir) { errno = ENOSYS; return NULL; }
    errno = 0;
    return next_readdir(directory);
}

struct dirent64 *readdir64(DIR *directory)
{
    if (!next_readdir64) { errno = ENOSYS; return NULL; }
    errno = 0;
    return next_readdir64(directory);
}
