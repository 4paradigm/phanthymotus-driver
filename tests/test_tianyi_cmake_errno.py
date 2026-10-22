"""Real Linux CMake regression: stale allocation errno is not a readdir error.

Run in the isolated ARM64 image on non-Linux development hosts. No robot, ROS
participant, network, system installation or emulation registration is needed.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
GUARD = ROOT / 'x-humanoid/tianyi2.0/deploy/cmake_readdir_errno.c'
pytestmark = pytest.mark.skipif(
    sys.platform != 'linux' or not shutil.which('cc') or not shutil.which('cmake'),
    reason='requires Linux/glibc cc and CMake; run inside the isolated ARM64 build image')


def test_real_cmake_stale_errno_recovers_but_io_error_and_missing_library_fail(tmp_path):
    injection = tmp_path / 'injection.c'
    injection.write_text(r'''
#define _GNU_SOURCE
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
static int selected(DIR *directory) {
    char link[64], value[4096];
    snprintf(link, sizeof(link), "/proc/self/fd/%d", dirfd(directory));
    ssize_t n = readlink(link, value, sizeof(value)-1);
    if (n < 0) return 0;
    value[n] = 0;
    return strcmp(value, getenv("TEST_LIBRARY_DIR")) == 0;
}
#define INTERPOSE(NAME, STRUCT) \
STRUCT *NAME(DIR *directory) { \
    static STRUCT *(*real_call)(DIR *); \
    if (!real_call) real_call = dlsym(RTLD_NEXT, #NAME); \
    int saved = errno; \
    int active = selected(directory); \
    errno = saved; \
    STRUCT *result = real_call(directory); \
    if (active && result) errno = ENOMEM; \
    if (active && !result && getenv("TEST_TRUE_ERROR")) errno = EIO; \
    return result; \
}
INTERPOSE(readdir, struct dirent)
INTERPOSE(readdir64, struct dirent64)
''')
    guard = tmp_path / 'guard.so'
    fault = tmp_path / 'injection.so'
    for source, output in ((GUARD, guard), (injection, fault)):
        subprocess.run(['cc', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                        str(source), '-ldl', '-o', str(output)], check=True, timeout=20)
    library_dir = tmp_path / 'libraries'
    library_dir.mkdir()
    library = library_dir / 'liblookup_probe.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-x', 'c', '-', '-o', str(library)],
                   input='int probe(void) { return 0; }', text=True, check=True, timeout=20)
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(errno_probe NONE)
set(CMAKE_FIND_LIBRARY_PREFIXES lib)
set(CMAKE_FIND_LIBRARY_SUFFIXES .so)
find_library(PROBE NAMES lookup_probe PATHS "''' + str(library_dir) + '''"
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
if(NOT PROBE)
  message(FATAL_ERROR "library lookup failed")
endif()
message(STATUS "library found: ${PROBE}")
''')

    def configure(name, preload='', true_error=False):
        env = {**os.environ, 'TEST_LIBRARY_DIR': str(library_dir)}
        env.pop('LD_PRELOAD', None)
        env.pop('TEST_TRUE_ERROR', None)
        if preload:
            env['LD_PRELOAD'] = preload
        if true_error:
            env['TEST_TRUE_ERROR'] = '1'
        return subprocess.run(['cmake', '-S', str(source), '-B', str(tmp_path / name)],
                              env=env, text=True, capture_output=True, timeout=20)

    assert configure('baseline').returncode == 0
    failed = configure('stale-errno', str(fault))
    assert failed.returncode != 0 and 'library lookup failed' in failed.stderr
    recovered = configure('guarded', str(guard) + ':' + str(fault))
    assert recovered.returncode == 0, recovered.stdout + recovered.stderr
    assert str(library) in recovered.stdout
    real_error = configure('actual-io-error', str(guard) + ':' + str(fault), true_error=True)
    assert real_error.returncode != 0 and 'library lookup failed' in real_error.stderr
    library.unlink()
    missing = configure('missing-library', str(guard))
    assert missing.returncode != 0 and 'library lookup failed' in missing.stderr
