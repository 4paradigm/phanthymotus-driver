"""FLV record framing and secret-free librtmp publish path."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from live_stream.transport import FLVFramer, RTMPPublisher, RTMPTransportError


def flv_tag(tag_type=9, payload=b"abcd"):
    return (bytes([tag_type]) + len(payload).to_bytes(3, "big")
            + b"\x00\x00\x00\x00" + b"\x00\x00\x00"
            + payload + (11 + len(payload)).to_bytes(4, "big"))


class FakeLibrtmp:
    def __init__(self):
        self.writes = []
        self.closed = False

    def RTMP_Alloc(self):
        return 1234

    def RTMP_Init(self, handle):
        pass

    def RTMP_SetupURL(self, handle, url_buffer):
        return 1

    def RTMP_EnableWrite(self, handle):
        pass

    def RTMP_Connect(self, handle, _):
        return 1

    def RTMP_ConnectStream(self, handle, _):
        return 1

    def RTMP_Write(self, handle, record, length):
        self.writes.append(bytes(record.raw[:length]))
        return length

    def RTMP_Close(self, handle):
        self.closed = True

    def RTMP_Free(self, handle):
        pass


def test_flv_chunks_reassembled_into_complete_tags():
    data = b"FLV\x01\x05\x00\x00\x00\x09\x00\x00\x00\x00" + flv_tag(18) + flv_tag(9)
    f = FLVFramer()
    records = []
    for i in range(0, len(data), 3):
        records.extend(f.feed(data[i:i+3]))
    assert len(records) == 2
    assert records[0].startswith(b"FLV")
    assert records[0].endswith(flv_tag(18))
    assert records[1] == flv_tag(9)


def test_invalid_flv_header_fails_closed():
    with pytest.raises(RTMPTransportError, match="INVALID_FLV_HEADER"):
        FLVFramer().feed(b"BAD_DATA" + b"0" * 32)


def test_librtmp_receives_flv_without_credential_in_argv():
    lib = FakeLibrtmp()
    pub = RTMPPublisher("rtmp://tx.direct.huya.com/huyalive/private-key", library=lib)
    pub.connect()
    pub.send(b"FLV\x01\x05" + b"\x00" * 30)
    pub.close()
    assert len(lib.writes) == 1
    assert lib.closed


def test_librtmp_write_error_is_redacted():
    lib = FakeLibrtmp()
    lib.RTMP_Write = lambda *args: -1
    pub = RTMPPublisher("rtmp://tx.direct.huya.com/huyalive/private-key", library=lib)
    pub.connect()
    with pytest.raises(RTMPTransportError, match="RTMP_WRITE_FAILED") as err:
        pub.send(flv_tag())
    assert "private-key" not in str(err.value)
    pub.close()
