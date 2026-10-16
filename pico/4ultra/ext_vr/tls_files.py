"""Private TLS files opened once and retained through OpenSSL loading (POSIX)."""
from contextlib import contextmanager
import os
from pathlib import Path
import ssl
import stat


@contextmanager
def open_tls_directory(path):
    directory = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        parent = os.fstat(directory)
        if stat.S_IMODE(parent.st_mode) != 0o700 or parent.st_uid != os.geteuid():
            raise ValueError("pico_tls_directory_must_be_owned_private_0700")
        yield directory
    finally:
        os.close(directory)


@contextmanager
def open_tls_file(path):
    path = Path(path)
    with open_tls_directory(path.parent) as directory:
        descriptor = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
        try:
            metadata = os.fstat(descriptor)
            if (not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o600
                    or metadata.st_uid != os.geteuid()):
                raise ValueError("pico_tls_file_must_be_owned_regular_0600")
            yield descriptor
        finally:
            os.close(descriptor)


def read_tls_file(path):
    with open_tls_file(path) as descriptor:
        return _read_pem(descriptor)


def _read_pem(descriptor):
    with os.fdopen(os.dup(descriptor), "rb") as stream:
        data = stream.read(32769)
    if not data or len(data) > 32768:
        raise ValueError("pico_tls_pem_size_invalid")
    os.lseek(descriptor, 0, os.SEEK_SET)
    return data


def load_tls_context(certificate_path, key_path):
    with open_tls_file(certificate_path) as cert, open_tls_file(key_path) as key:
        certificate = _read_pem(cert)
        _read_pem(key)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        # OpenSSL accepts paths rather than descriptors. These POSIX descriptor
        # paths reference our checked, still-open files, never the original names.
        fd_root = "/proc/self/fd" if Path("/proc/self/fd").is_dir() else "/dev/fd"
        context.load_cert_chain(f"{fd_root}/{cert}", f"{fd_root}/{key}")
        return context, certificate
