#!/usr/bin/env python3
"""stream_camera.py: a viewer leaving or the network dropping must not break the preview server.

No camera needed (picamera2 is stubbed; the capture thread is not started).

  python test_stream_camera.py
"""
import errno
import socket
import struct
import sys
import threading
import time
import types
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

sys.modules.setdefault("picamera2", types.SimpleNamespace(Picamera2=object))
import stream_camera  # noqa: E402

FAKE_JPG = b"\xff\xd8" + b"x" * 2000 + b"\xff\xd9"


class _RaisingWriter:
    def __init__(self, exc):
        self.exc = exc

    def write(self, _):
        raise self.exc


def _handler_writing_to(writer):
    h = stream_camera.Handler.__new__(stream_camera.Handler)
    h.path = "/stream"
    h.wfile = writer
    h.send_response = lambda *a: None
    h.send_header = lambda *a: None
    h.end_headers = lambda: None
    return h


class TestViewerDisconnect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stream_camera.latest = FAKE_JPG
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), stream_camera.Handler)
        cls.srv.daemon_threads = True
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def test_network_errors_are_swallowed(self):
        """Every way a viewer can disappear ends the stream quietly (old code only caught 2 of these)."""
        for exc in (BrokenPipeError(), ConnectionResetError(), ConnectionAbortedError(), TimeoutError(),
                    OSError(errno.EHOSTUNREACH, "No route to host"), OSError(errno.ENETDOWN, "Network is down")):
            with self.subTest(exc=type(exc).__name__, errno=getattr(exc, "errno", None)):
                _handler_writing_to(_RaisingWriter(exc)).do_GET()      # must return, not raise

    def test_abrupt_disconnect_server_keeps_serving(self):
        for _ in range(3):
            s = socket.create_connection(("127.0.0.1", self.port), timeout=3)
            s.sendall(b"GET /stream HTTP/1.1\r\nHost: x\r\n\r\n")
            got = b""
            while got.count(b"--frame") < 2:
                got += s.recv(65536)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))   # RST, like a dropped Wi-Fi
            s.close()
        time.sleep(0.3)
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/", timeout=3) as r:
            self.assertEqual(r.status, 200)
        s = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        s.sendall(b"GET /stream HTTP/1.1\r\nHost: x\r\n\r\n")
        got = b""
        while FAKE_JPG not in got:
            got += s.recv(65536)
        s.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
