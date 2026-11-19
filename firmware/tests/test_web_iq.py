#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Offline unit and integration tests for host/web/server.py and host/web/dsp.js."""

from __future__ import annotations

import base64
import hashlib
import http.client
import io
import json
import math
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parents[1]
WEB_DIR = REPO_ROOT / "host" / "web"

if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import server  # noqa: E402


def sext12(val: int) -> int:
    val &= 0xFFF
    return (val ^ 0x800) - 0x800


def unpack_u32_12(payload: bytes) -> list[tuple[int, int]]:
    count = len(payload) // 4
    words = struct.unpack(f"<{count}I", payload)
    return [(sext12(w & 0xFFFF), sext12((w >> 16) & 0xFFFF)) for w in words]


class MinimalWSClient:
    """Minimal Python stdlib RFC 6455 WebSocket client for integration testing."""

    def __init__(self, host: str, port: int, path: str = "/ws", timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = bytearray()
        self._handshake(host, port, path)

    def _recv_raw(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise EOFError("Socket closed while reading")
            self._buf.extend(chunk)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def _handshake(self, host: str, port: int, path: str) -> None:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        ).encode("ascii")
        self.sock.sendall(req)

        while b"\r\n\r\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise EOFError("EOF during WS handshake")
            self._buf.extend(chunk)

        idx = self._buf.index(b"\r\n\r\n")
        head = bytes(self._buf[:idx]).decode("latin1")
        del self._buf[: idx + 4]

        lines = head.split("\r\n")
        if "101" not in lines[0]:
            raise AssertionError(f"Expected 101 Switching Protocols, got: {lines[0]!r}")

        headers = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        expected = base64.b64encode(
            hashlib.sha1((key + server.WS_GUID).encode("ascii")).digest()
        ).decode("ascii")
        if headers.get("sec-websocket-accept") != expected:
            raise AssertionError(
                f"Sec-WebSocket-Accept mismatch: {headers.get('sec-websocket-accept')!r} != {expected!r}"
            )

    def send_frame(self, opcode: int, payload: bytes) -> None:
        mask = os.urandom(4)
        n = len(payload)
        b0 = 0x80 | (opcode & 0x0F)
        if n < 126:
            hdr = struct.pack("!BB", b0, 0x80 | n)
        elif n < 65536:
            hdr = struct.pack("!BBH", b0, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", b0, 0x80 | 127, n)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(hdr + mask + masked)

    def send_json(self, obj: dict) -> None:
        self.send_frame(0x1, json.dumps(obj).encode("utf-8"))

    def send_ping(self, payload: bytes = b"ping") -> None:
        self.send_frame(0x9, payload)

    def recv_frame(self) -> tuple[int, bytes]:
        frag_opcode = None
        frag_parts: list[bytes] = []

        while True:
            b0, b1 = self._recv_raw(2)
            fin = bool(b0 & 0x80)
            opcode = b0 & 0x0F
            masked = bool(b1 & 0x80)
            length = b1 & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._recv_raw(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._recv_raw(8))[0]
            mask = self._recv_raw(4) if masked else b""
            payload = self._recv_raw(length) if length else b""
            if masked and payload:
                payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))

            if opcode >= 0x8:
                if opcode == 0x9:
                    self.send_frame(0xA, payload)
                    continue
                return opcode, payload

            if opcode != 0x0:
                if fin:
                    return opcode, payload
                frag_opcode = opcode
                frag_parts = [payload]
            else:
                if frag_opcode is None:
                    raise AssertionError("Unexpected continuation frame")
                frag_parts.append(payload)
                if fin:
                    return frag_opcode, b"".join(frag_parts)

    def close(self) -> None:
        try:
            self.send_frame(0x8, struct.pack("!H", 1000))
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


class ParserAndPackingTest(unittest.TestCase):
    def test_parse_iqstatus_full_with_crlf(self) -> None:
        sample = (
            "fw_state 7\r\n"
            "raw_tick     0x00000aae (2734)\r\n"
            "magic        0x51494d57 (WMIQ)\r\n"
            "version      1\r\n"
            "cmd          0\r\n"
            "status       2 (DONE)\r\n"
            "node         0x00000008 (8)\r\n"
            "length       0x00002000 (8192 words)\r\n"
            "data_addr    0xe0200000\r\n"
            "sample_count 0x00002000 (8192 words)\r\n"
            "tick         0x00000aa0 (2720)\r\n"
            "error        0x00000000\r\n"
            "cr0          0x00000400\r\n"
            "cr2          0x00000000\r\n"
            "cr9          0x0001fffc (131068)\r\n"
            "bw           0\r\n"
            "arg0         0x00000000\r\n"
            "arg1/ret     0x00000012\r\n"
        )
        st = server.parse_iqstatus_output(sample, host="root@192.168.6.2", demo=False)
        self.assertTrue(st["ok"])
        self.assertFalse(st["demo"])
        self.assertEqual(st["host"], "root@192.168.6.2")
        self.assertEqual(st["fw_state"], 7)
        self.assertEqual(st["raw_tick"], 2734)
        self.assertEqual(st["magic"], "0x51494d57")
        self.assertTrue(st["magic_ok"])
        self.assertEqual(st["version"], 1)
        self.assertEqual(st["cmd"], 0)
        self.assertEqual(st["status"], "DONE")
        self.assertEqual(st["status_code"], 2)
        self.assertEqual(st["node"], 8)
        self.assertEqual(st["length"], 8192)
        self.assertEqual(st["data_addr"], "0xe0200000")
        self.assertEqual(st["sample_count"], 8192)
        self.assertEqual(st["tick"], 2720)
        self.assertEqual(st["error"], 0)
        self.assertEqual(st["cr0"], 0x400)
        self.assertEqual(st["cr2"], 0)
        self.assertEqual(st["cr9"], 0x1FFFC)
        self.assertEqual(st["bw"], 0)
        self.assertEqual(st["arg0"], 0)
        self.assertEqual(st["arg1"], 0x12)

        for int_key in (
            "fw_state", "raw_tick", "version", "cmd", "status_code", "node",
            "length", "sample_count", "tick", "error", "cr0", "cr2", "cr9",
            "bw", "arg0", "arg1",
        ):
            self.assertIs(type(st[int_key]), int, f"{int_key} should be int")

    def test_parse_iqstatus_uninit(self) -> None:
        sample = (
            "raw_tick     0x00000010 (16)\n"
            "magic        0x00000000 (UNINIT)\n"
            "version      0\n"
            "cmd          0\n"
            "status       0 (IDLE)\n"
        )
        st = server.parse_iqstatus_output(sample)
        self.assertEqual(st["magic"], "0x00000000")
        self.assertFalse(st["magic_ok"])
        self.assertEqual(st["status"], "IDLE")
        self.assertEqual(st["status_code"], 0)

    def test_parse_iqstatus_with_chinfo_words(self) -> None:
        sample = (
            "fw_state 7\n"
            "raw_tick     0x00000aae (2734)\n"
            "magic        0x51494d57 (WMIQ)\n"
            "status       2 (DONE)\n"
            "node         0x0000000f (15)\n"
            "length       0x00002000 (8192 words)\n"
            "WM[0xe00f8524] = 0x00259130\n"
            "WM[0xe00f8530] = 0x02020101\n"
            "WM[0xe00f8544] = 0x00501bd0\n"
            "WM[0xe00f8550] = 0x03030300\n"
        )
        st = server.parse_iqstatus_output(sample, host="root@192.168.6.2", demo=False)
        self.assertEqual(st["node"], 15)
        self.assertIn("rf", st)
        self.assertEqual(st["rf"]["band0"]["lo_khz"], 2462000)
        self.assertEqual(st["rf"]["band0"]["lo_mhz"], 2462.0)
        self.assertEqual(st["rf"]["band0"]["bw_idx"], 1)
        self.assertEqual(st["rf"]["band0"]["bw_mhz"], 40)
        self.assertEqual(st["rf"]["band1"]["lo_mhz"], 5250.0)
        self.assertEqual(st["rf"]["band1"]["bw_mhz"], 160)

    def test_parse_iqcapture_output(self) -> None:
        stderr = (
            "capture complete -> /tmp/wmsdr_web_iq.bin (32768 bytes from 0xe0200000)\r\n"
            "  mode=direct node=0x8 len=8192 sample_count=8192 cr0=0x00000400 cr9=0x0001fffc err=0x0\r\n"
            "  stats: nonzero=8192/8192 transitions=8191/8191 I_12b=[-2046..1983] Q_12b=[-2001..1963]\r\n"
            "  first 8 words:\r\n"
            "    [ 0] 0x12345678  (I12=  1656, Q12=   564)\r\n"
        )
        meta = server.parse_iqcapture_output(stderr)
        self.assertEqual(meta["mode"], "direct")
        self.assertEqual(meta["node"], 8)
        self.assertEqual(meta["len"], 8192)
        self.assertEqual(meta["sample_count"], 8192)
        self.assertEqual(meta["cr0"], 0x400)
        self.assertEqual(meta["cr9"], 0x1FFFC)
        self.assertEqual(meta["err"], 0)
        self.assertEqual(meta["data_addr"], "0xe0200000")

    def test_pack_wsiq_header(self) -> None:
        hdr = server.pack_wsiq_header(
            seq=42,
            node=8,
            words=1024,
            cr0=0x00000400,
            cr9=0x00001000,
            direct=True,
            demo=True,
            bw=2,
        )
        self.assertEqual(len(hdr), 32)
        fields = struct.unpack("<8I", hdr)
        self.assertEqual(
            fields,
            (0x51495357, 1, 42, 8, 1024, 0x00000400, 0x00001000, 0x0203),
        )

    def test_validate_capture_params(self) -> None:
        self.assertEqual(
            server.validate_capture_params("8", "1024", "2", "1", "multitone"),
            (8, 1024, 2, True, "multitone"),
        )
        for bad in [
            {"node_raw": -1},
            {"node_raw": 64},
            {"len_raw": 0},
            {"len_raw": 131073},
            {"bw_raw": 4},
            {"node_raw": "abc"},
        ]:
            with self.assertRaises(ValueError):
                server.validate_capture_params(**bad)

    def test_cli_defaults_and_ssh_resolution(self) -> None:
        args = server.build_arg_parser().parse_args([])
        self.assertEqual(args.bind, "127.0.0.1")
        self.assertEqual(args.port, 8088)
        self.assertEqual(args.host, "root@192.168.6.2")
        self.assertEqual(args.ssh_cmd, "auto")
        self.assertEqual(args.remote_wmsdr, "/root/wmtest/wmsdr")
        self.assertIsNone(args.samples_dir)
        self.assertFalse(args.demo)
        self.assertFalse(args.no_control_master)

        self.assertEqual(server.resolve_ssh_cmd("native"), ["ssh"])
        self.assertEqual(server.resolve_ssh_cmd("wsl"), ["wsl", "-e", "ssh"])
        with mock.patch("sys.platform", "win32"), mock.patch("shutil.which", return_value="C:\\Windows\\System32\\wsl.exe"):
            self.assertEqual(server.resolve_ssh_cmd("auto"), ["wsl", "-e", "ssh"])
        with mock.patch("sys.platform", "win32"), mock.patch("shutil.which", return_value=None):
            self.assertEqual(server.resolve_ssh_cmd("auto"), ["ssh"])

    def test_mocked_ssh_backend_capture_and_status(self) -> None:
        backend = server.IQBackend(
            host="root@192.168.6.2",
            ssh_cmd="native",
            remote_wmsdr="/root/wmtest/wmsdr",
            demo=False,
            no_control_master=True,
        )
        fake_payload = struct.pack("<4I", 0x00010002, 0x0FFF0800, 0x07FF07FF, 0x00000000)
        fake_stderr = (
            b"capture complete -> /tmp/wmsdr_web_iq.bin (16 bytes from 0xe0200000)\n"
            b"  mode=direct node=0x8 len=4 sample_count=4 cr0=0x00000400 cr9=0x00000010 err=0x0\n"
        )

        with mock.patch("subprocess.run") as m_run:
            m_run.return_value = subprocess.CompletedProcess(
                args=[], returncode=0, stdout=fake_payload, stderr=fake_stderr
            )
            cap = backend.capture(8, 4, 1, True)
            self.assertEqual(cap["payload"], fake_payload)
            self.assertEqual(cap["cr0"], 0x400)
            self.assertEqual(cap["cr9"], 0x10)
            self.assertEqual(cap["sample_count"], 4)
            self.assertEqual(cap["seq"], 1)
            self.assertFalse(cap["demo"])
            cmd_used = m_run.call_args.args[0]
            self.assertEqual(cmd_used[0], "ssh")
            self.assertIn(
                "/root/wmtest/wmsdr iqcapture /tmp/wmsdr_web_iq.bin 8 4 1 direct >&2 && cat /tmp/wmsdr_web_iq.bin",
                cmd_used[-1],
            )

    def test_mocked_ssh_stream_wsiq_and_legacy_fallback(self) -> None:
        backend = server.IQBackend(
            host="root@192.168.6.2",
            ssh_cmd="native",
            remote_wmsdr="/root/wmtest/wmsdr",
            demo=False,
            no_control_master=True,
        )
        p1 = struct.pack("<4I", 1, 2, 3, 4)
        p2 = struct.pack("<4I", 5, 6, 7, 8)
        stream_bytes = (
            server.pack_wsiq_header(1, 8, 4, 0x400, 0x10, bw=0) + p1 +
            server.pack_wsiq_header(2, 8, 4, 0x400, 0x20, bw=0) + p2
        )

        class _FakeProc:
            def __init__(self, out_bytes: bytes, err_bytes: bytes, rc: int) -> None:
                self.stdin = io.BytesIO()
                self.stdout = io.BytesIO(out_bytes)
                self.stderr = io.BytesIO(err_bytes)
                self._rc = rc

            def poll(self):
                return self._rc

            def wait(self, timeout=None):
                return self._rc

            def terminate(self):
                pass

            def kill(self):
                pass

        received = []
        stop_evt = threading.Event()
        with mock.patch("subprocess.Popen", return_value=_FakeProc(stream_bytes, b"", 0)) as m_popen:
            cnt = backend.stream_wsiq(8, 4, 0, False, 0, stop_evt, lambda b: received.append(b) or True)
            self.assertEqual(cnt, 2)
            self.assertEqual(len(received), 2)
            self.assertEqual(received[0][32:], p1)
            self.assertEqual(received[1][32:], p2)
            self.assertIn("exec /root/wmtest/wmsdr iqstream 8 4 0 0 0", m_popen.call_args.args[0][-1])

        with mock.patch("subprocess.Popen", return_value=_FakeProc(b"", b"bad arguments\n", 2)):
            with self.assertRaises(server.LegacyStreamUnsupported):
                backend.stream_wsiq(8, 4, 0, False, 0, threading.Event(), lambda _: True)

        with self.assertRaises(server.BackendError):
            backend.stream_wsiq(15, 8192, 0, True, 0, threading.Event(), lambda _: True)


class WebServerDemoIntegrationTest(unittest.TestCase):
    tmpdir: tempfile.TemporaryDirectory[str]
    samples_dir: Path
    srv: server.IQHttpServer
    thread: threading.Thread
    port: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.samples_dir = Path(cls.tmpdir.name)
        cls.sample_bytes = struct.pack("<4I", 0x00100020, 0x0FFF0800, 0x07FF07FF, 0x01000FE0)
        (cls.samples_dir / "tone_test.bin").write_bytes(cls.sample_bytes)
        (cls.samples_dir / "burst.iq").write_bytes(b"\x01\x02\x03\x04")
        (cls.samples_dir / "ignore.txt").write_text("not a sample", encoding="utf-8")

        cls.srv = server.create_server(
            bind="127.0.0.1",
            port=0,
            demo=True,
            samples_dir=cls.samples_dir,
            quiet=True,
        )
        cls.port = cls.srv.server_port
        cls.thread = threading.Thread(target=cls.srv.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.srv.shutdown()
        cls.srv.server_close()
        cls.thread.join(timeout=3.0)
        cls.tmpdir.cleanup()

    def _request(self, path: str) -> tuple[int, dict[str, str], bytes]:
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5.0)
        try:
            conn.request("GET", path)
            resp = conn.getresponse()
            body = resp.read()
            headers = {k.lower(): v for k, v in resp.getheaders()}
            return resp.status, headers, body
        finally:
            conn.close()

    def test_static_and_config_and_samples(self) -> None:
        for route, marker in (
            ("/", b"CT3003"),
            ("/index.html", b"CT3003"),
            ("/style.css", b"canvas"),
            ("/dsp.js", b"WmsdrDsp"),
            ("/render.js", b"WmsdrRender"),
            ("/app.js", b"/api/config"),
        ):
            status, headers, body = self._request(route)
            self.assertEqual(status, 200, f"GET {route} failed")
            self.assertEqual(headers.get("cache-control"), "no-store")
            self.assertIn(marker, body)

        status, headers, body = self._request("/api/config")
        self.assertEqual(status, 200)
        cfg = json.loads(body.decode("utf-8"))
        self.assertTrue(cfg["ok"])
        self.assertTrue(cfg["demo"])
        self.assertEqual(cfg["default_node"], 15)
        self.assertEqual(cfg["default_len"], 8192)
        self.assertEqual(cfg["default_bw"], 0)
        self.assertEqual(cfg["default_format"], "u32_12")
        self.assertEqual(cfg["default_stream_mode"], "all")
        self.assertEqual(cfg["default_fs_mhz"], 80.0)
        self.assertEqual(cfg["default_lo_mhz"], 2462.0)
        self.assertIn("15", cfg["node_profiles"])
        self.assertEqual(sorted(cfg["samples"]), ["burst.iq", "tone_test.bin"])

        status, _, body = self._request("/api/samples")
        self.assertEqual(status, 200)
        samples = json.loads(body.decode("utf-8"))
        self.assertTrue(samples["ok"])
        names_sizes = {item["name"]: item["size"] for item in samples["files"]}
        self.assertEqual(names_sizes, {"tone_test.bin": 16, "burst.iq": 4})

        status, headers, body = self._request("/samples/tone_test.bin")
        self.assertEqual(status, 200)
        self.assertEqual(headers.get("content-type"), "application/octet-stream")
        self.assertEqual(body, self.sample_bytes)

        for blocked in (
            "/samples/../server.py",
            "/samples/%2e%2e/server.py",
            "/samples/ignore.txt",
            "/samples/missing.bin",
            "/server.py",
        ):
            st, _, _ = self._request(blocked)
            self.assertIn(st, (400, 403, 404), f"Expected rejection for {blocked}, got {st}")

    def test_api_status_and_capture(self) -> None:
        status, _, body = self._request("/api/status")
        self.assertEqual(status, 200)
        st = json.loads(body.decode("utf-8"))
        self.assertTrue(st["ok"])
        self.assertTrue(st["demo"])
        self.assertEqual(st["magic"], "0x51494d57")
        self.assertTrue(st["magic_ok"])
        self.assertEqual(st["fw_state"], 7)

        for preset in ("ofdm_wifi", "multitone", "qpsk_burst", "sweep"):
            status, headers, body = self._request(
                f"/api/capture?node=8&len=1024&bw=0&direct=1&preset={preset}"
            )
            self.assertEqual(status, 200)
            self.assertEqual(headers.get("content-type"), "application/octet-stream")
            self.assertEqual(len(body), 4096)
            self.assertEqual(headers.get("x-iq-node"), "8")
            self.assertEqual(headers.get("x-iq-words"), "1024")
            self.assertEqual(headers.get("x-iq-sample-count"), "1024")
            self.assertTrue(headers.get("x-iq-cr0", "").startswith("0x"))
            self.assertTrue(headers.get("x-iq-cr9", "").startswith("0x"))
            self.assertEqual(headers.get("x-iq-direct"), "1")
            self.assertGreaterEqual(int(headers["x-iq-seq"]), 1)

            pairs = unpack_u32_12(body)
            self.assertEqual(len(pairs), 1024)
            nonzero = sum(1 for iv, qv in pairs if iv != 0 or qv != 0)
            self.assertGreater(nonzero, 900)
            for iv, qv in pairs:
                self.assertGreaterEqual(iv, -2048)
                self.assertLessEqual(iv, 2047)
                self.assertGreaterEqual(qv, -2048)
                self.assertLessEqual(qv, 2047)

        status, _, body = self._request("/api/capture?node=99&len=1024")
        self.assertEqual(status, 400)
        err = json.loads(body.decode("utf-8"))
        self.assertFalse(err["ok"])

    def test_websocket_oneshot_and_stream(self) -> None:
        ws = MinimalWSClient("127.0.0.1", self.port, "/ws")
        try:
            ws.send_json({"type": "status"})
            opcode, data = ws.recv_frame()
            self.assertEqual(opcode, 0x1)
            msg = json.loads(data.decode("utf-8"))
            self.assertEqual(msg["type"], "status")
            self.assertTrue(msg["ok"])
            self.assertFalse(msg["streaming"])
            self.assertTrue(msg["magic_ok"])

            ws.send_json({
                "type": "oneshot",
                "node": 8,
                "len": 512,
                "bw": 0,
                "direct": False,
                "preset": "multitone",
            })
            opcode, frame = ws.recv_frame()
            self.assertEqual(opcode, 0x2)
            self.assertEqual(len(frame), 32 + 512 * 4)
            magic, ver, seq1, node, words, cr0, cr9, flags = struct.unpack("<8I", frame[:32])
            self.assertEqual(magic, 0x51495357)
            self.assertEqual(ver, 1)
            self.assertGreaterEqual(seq1, 1)
            self.assertEqual(node, 8)
            self.assertEqual(words, 512)
            self.assertEqual(cr0, 0x400)
            self.assertEqual(cr9, 512 * 4)
            self.assertEqual(flags & 0x3, 0x2)  # demo=1, direct=0
            pairs = unpack_u32_12(frame[32:])
            self.assertEqual(len(pairs), 512)
            for iv, qv in pairs:
                self.assertTrue(-2048 <= iv <= 2047 and -2048 <= qv <= 2047)

            ws.send_json({
                "type": "start",
                "node": 6,
                "len": 256,
                "bw": 2,
                "direct": True,
                "intervalMs": 30,
                "preset": "qpsk_burst",
            })
            bin_frames = []
            saw_streaming_true = False
            deadline = time.monotonic() + 4.0
            while len(bin_frames) < 2 and time.monotonic() < deadline:
                opcode, data = ws.recv_frame()
                if opcode == 0x1:
                    ev = json.loads(data.decode("utf-8"))
                    if ev.get("type") == "status" and ev.get("streaming") is True:
                        saw_streaming_true = True
                elif opcode == 0x2:
                    bin_frames.append(data)

            self.assertTrue(saw_streaming_true)
            self.assertGreaterEqual(len(bin_frames), 2)
            seqs = []
            for bf in bin_frames:
                self.assertEqual(len(bf), 32 + 256 * 4)
                magic, ver, seq, node, words, _, _, flags = struct.unpack("<8I", bf[:32])
                self.assertEqual(magic, 0x51495357)
                self.assertEqual(ver, 1)
                self.assertEqual(node, 6)
                self.assertEqual(words, 256)
                self.assertEqual(flags, 0x0203)  # direct=1, demo=1, bw=2
                seqs.append(seq)
            self.assertLess(seqs[0], seqs[1])

            ws.send_json({"type": "stop"})
            stopped = False
            deadline = time.monotonic() + 3.0
            while not stopped and time.monotonic() < deadline:
                opcode, data = ws.recv_frame()
                if opcode == 0x1:
                    ev = json.loads(data.decode("utf-8"))
                    if ev.get("type") == "status" and ev.get("streaming") is False:
                        stopped = True
            self.assertTrue(stopped)

            ws.send_ping(b"ct3003")
            opcode, data = ws.recv_frame()
            self.assertEqual(opcode, 0xA)
            self.assertEqual(data, b"ct3003")
        finally:
            ws.close()

    def test_concurrent_captures(self) -> None:
        results: list[tuple[int, int, int]] = []
        lock = threading.Lock()
        barrier = threading.Barrier(4)

        def _worker() -> None:
            barrier.wait(timeout=3.0)
            st, hdrs, body = self._request("/api/capture?node=8&len=256&bw=0&direct=0")
            with lock:
                results.append((st, len(body), int(hdrs.get("x-iq-seq", "0"))))

        threads = [threading.Thread(target=_worker) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        self.assertEqual(len(results), 4)
        for st, size, _ in results:
            self.assertEqual(st, 200)
            self.assertEqual(size, 1024)
        seqs = [seq for _, _, seq in results]
        self.assertEqual(len(set(seqs)), 4)

    def test_websocket_stops_after_repeated_errors(self) -> None:
        err_srv = server.create_server(bind="127.0.0.1", port=0, demo=True, quiet=True)
        err_thread = threading.Thread(target=err_srv.serve_forever, daemon=True)
        err_thread.start()
        try:
            with mock.patch.object(
                err_srv.backend,
                "capture",
                side_effect=server.BackendError("simulated ssh timeout"),
            ):
                ws = MinimalWSClient("127.0.0.1", err_srv.server_port, "/ws")
                try:
                    ws.send_json({
                        "type": "start",
                        "node": 8,
                        "len": 256,
                        "bw": 0,
                        "direct": False,
                        "intervalMs": 20,
                    })
                    errors = []
                    stopped_on_err = False
                    deadline = time.monotonic() + 3.0
                    while not stopped_on_err and time.monotonic() < deadline:
                        opcode, data = ws.recv_frame()
                        if opcode == 0x1:
                            ev = json.loads(data.decode("utf-8"))
                            if ev.get("type") == "error":
                                errors.append(ev.get("message", ""))
                            elif ev.get("type") == "status" and ev.get("streaming") is False and ev.get("ok") is False:
                                stopped_on_err = True
                    self.assertGreaterEqual(len(errors), 3)
                    self.assertTrue(stopped_on_err)
                finally:
                    ws.close()
        finally:
            err_srv.shutdown()
            err_srv.server_close()
            err_thread.join(timeout=3.0)


class DspJsNodeTest(unittest.TestCase):
    @staticmethod
    def _find_node_runner() -> tuple[list[str], str] | None:
        win_node = shutil.which("node")
        if win_node:
            return [win_node], str(WEB_DIR / "dsp.js")
        if sys.platform == "win32" and shutil.which("wsl"):
            probe = subprocess.run(
                ["wsl", "which", "node"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            if probe.returncode == 0 and probe.stdout.strip():
                wsl_path = subprocess.run(
                    ["wsl", "wslpath", "-u", str(WEB_DIR / "dsp.js")],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    check=True,
                ).stdout.strip()
                return ["wsl", "node"], wsl_path
        return None

    def test_dsp_js_via_node(self) -> None:
        runner = self._find_node_runner()
        if runner is None:
            self.skipTest("node executable not available on host or WSL")

        cmd_prefix, dsp_path = runner

        # Generate a WSIQ-framed 1024-sample complex tone at +128/1024 (+0.125 Fs)
        n = 1024
        tone_words = []
        for k in range(n):
            ang = 2.0 * math.pi * 128.0 * k / n
            iv = int(round(1200.0 * math.cos(ang)))
            qv = int(round(1200.0 * math.sin(ang)))
            tone_words.append(((qv & 0xFFF) << 16) | (iv & 0xFFF))
        tone_wsiq = server.pack_wsiq_header(
            seq=7, node=8, words=n, cr0=0x400, cr9=n * 4, direct=True, demo=True, bw=0
        ) + struct.pack(f"<{n}I", *tone_words)

        py_multi = server.pack_wsiq_header(
            seq=8, node=8, words=2048, cr0=0x400, cr9=2048 * 4, direct=False, demo=True, bw=0
        ) + server.SyntheticGenerator().generate("multitone", 2048)

        # Interleaved WF0 (+0.125 Fs on even words) + WF1 (-0.25 Fs on odd words)
        interleaved_words = []
        for k in range(1024):
            a0 = 2.0 * math.pi * 128.0 * k / 1024
            i0 = int(round(900.0 * math.cos(a0)))
            q0 = int(round(900.0 * math.sin(a0)))
            interleaved_words.append(((q0 & 0xFFF) << 16) | (i0 & 0xFFF))
            a1 = 2.0 * math.pi * (-256.0) * k / 1024
            i1 = int(round(900.0 * math.cos(a1)))
            q1 = int(round(900.0 * math.sin(a1)))
            interleaved_words.append(((q1 & 0xFFF) << 16) | (i1 & 0xFFF))
        interleaved_wsiq = server.pack_wsiq_header(
            seq=9, node=15, words=2048, cr0=0x400, cr9=2048 * 4, direct=False, demo=False, bw=0, band=1
        ) + struct.pack("<2048I", *interleaved_words)

        js_script = f"""
const assert = require('assert');
const Dsp = require({json.dumps(dsp_path)});

// 1. Sign extension checks
assert.strictEqual(Dsp.sext(0xFFF, 12), -1);
assert.strictEqual(Dsp.sext(0x800, 12), -2048);
assert.strictEqual(Dsp.sext(0x7FF, 12), 2047);
assert.strictEqual(Dsp.sext(0x000, 12), 0);

// 2. Decode WSIQ tone generated by Python server.py and verify FFT peak bin
const toneB64 = {json.dumps(base64.b64encode(tone_wsiq).decode("ascii"))};
const raw = Uint8Array.from(Buffer.from(toneB64, 'base64')).buffer;
const dec = Dsp.decodeIqBuffer(raw, {{ format: 'u32_12', headerMode: 'auto' }});
assert.strictEqual(dec.count, 1024);
assert.ok(dec.meta && dec.meta.kind === 'wsiq');
assert.strictEqual(dec.meta.seq, 7);
assert.strictEqual(dec.meta.node, 8);
assert.strictEqual(dec.meta.words, 1024);
assert.strictEqual(dec.meta.cr0, 0x400);
assert.strictEqual(dec.iRaw[0], 1200);
assert.strictEqual(dec.qRaw[0], 0);

const spec = Dsp.computeSpectrum(dec.iNorm, dec.qNorm, {{
  fftSize: 1024,
  window: 'hann',
  detector: 'avg',
  dcRemove: false
}});
assert.strictEqual(spec.length, 1024);
let peakBin = 0;
let peakVal = -Infinity;
for (let i = 0; i < spec.length; i++) {{
  if (spec[i] > peakVal) {{
    peakVal = spec[i];
    peakBin = i;
  }}
}}
// fftshift places +128 at 512 + 128 = 640
assert.strictEqual(peakBin, 640, 'Expected FFT peak at bin 640, got ' + peakBin);

// 3. Server-generated multitone WSIQ frame decoded by dsp.js (-0.18 Fs dominant tone -> ~328)
const multiB64 = {json.dumps(base64.b64encode(py_multi).decode("ascii"))};
const multiRaw = Uint8Array.from(Buffer.from(multiB64, 'base64')).buffer;
const decMulti = Dsp.decodeIqBuffer(multiRaw, {{ format: 'u32_12', headerMode: 'wsiq' }});
assert.strictEqual(decMulti.count, 2048);
const specMulti = Dsp.computeSpectrum(decMulti.iNorm, decMulti.qNorm, {{ fftSize: 1024, window: 'hann' }});
let mPeak = 0;
let mVal = -Infinity;
for (let i = 0; i < specMulti.length; i++) {{
  if (specMulti[i] > mVal) {{
    mVal = specMulti[i];
    mPeak = i;
  }}
}}
assert.ok(Math.abs(mPeak - 328) <= 1, 'Expected Python multitone peak near bin 328, got ' + mPeak);

// 4. Built-in JS synthetic generator + stats sanity
const synthBuf = Dsp.generateSyntheticFrame('multitone', 2048);
const decSynth = Dsp.decodeIqBuffer(synthBuf, {{ format: 'u32_12', headerMode: 'none' }});
const st = Dsp.computeStats(decSynth);
assert.strictEqual(st.count, 2048);
assert.ok(st.minI >= -2048 && st.maxI <= 2047);
assert.ok(st.minQ >= -2048 && st.maxQ <= 2047);
assert.ok(st.nonzero > 1800);

// 5. Dual-antenna de-interleave (even=WF0, odd=WF1)
const ilvB64 = {json.dumps(base64.b64encode(interleaved_wsiq).decode("ascii"))};
const ilvRaw = Uint8Array.from(Buffer.from(ilvB64, 'base64')).buffer;
const decEven = Dsp.decodeIqBuffer(ilvRaw, {{ format: 'u32_12', headerMode: 'auto', streamMode: 'even' }});
const decOdd  = Dsp.decodeIqBuffer(ilvRaw, {{ format: 'u32_12', headerMode: 'auto', streamMode: 'odd' }});
assert.strictEqual(decEven.count, 1024);
assert.strictEqual(decOdd.count, 1024);
const specEven = Dsp.computeSpectrum(decEven.iNorm, decEven.qNorm, {{ fftSize: 1024, window: 'hann' }});
const specOdd  = Dsp.computeSpectrum(decOdd.iNorm, decOdd.qNorm, {{ fftSize: 1024, window: 'hann' }});
const argmax = (arr) => arr.reduce((best, v, idx) => (v > arr[best] ? idx : best), 0);
assert.strictEqual(argmax(specEven), 640);
assert.strictEqual(argmax(specOdd), 256);
assert.strictEqual(decEven.meta.band, 1);

console.log('DSP_JS_OK peakBin=' + peakBin + ' multiPeak=' + mPeak + ' peakDb=' + peakVal.toFixed(2));
"""
        proc = subprocess.run(
            cmd_prefix + ["-e", js_script],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        self.assertEqual(
            proc.returncode,
            0,
            f"Node dsp.js test failed:\nstdout={proc.stdout}\nstderr={proc.stderr}",
        )
        self.assertIn("DSP_JS_OK peakBin=640", proc.stdout)


class BandSwitchingDemoTest(unittest.TestCase):
    def test_config_bands_capture_and_ws_band_flag(self) -> None:
        import urllib.request

        srv = server.create_server(bind="127.0.0.1", port=0, demo=True, quiet=True)
        port = srv.server_port
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/config", timeout=3) as r:
                cfg = json.loads(r.read().decode("utf-8"))
            self.assertEqual(cfg.get("default_band"), 0)
            self.assertIn("bands", cfg)
            self.assertEqual(cfg["bands"]["0"]["adc_fs_mhz"], 80.0)
            self.assertEqual(cfg["bands"]["1"]["adc_fs_mhz"], 160.0)
            self.assertEqual(cfg["bands"]["1"]["default_lo_mhz"], 5250.0)

            with urllib.request.urlopen(
                f"http://127.0.0.1:{port}/api/capture?node=15&len=256&bw=0&band=1", timeout=3
            ) as r:
                body = r.read()
                self.assertEqual(len(body), 256 * 4)
                self.assertEqual(r.headers.get("X-IQ-Band"), "1")

            ws = MinimalWSClient("127.0.0.1", port)
            try:
                ws.send_json({"type": "oneshot", "node": 15, "len": 256, "bw": 0, "band": 1, "direct": False})
                op, pl = ws.recv_frame()
                self.assertEqual(op, 0x2)
                magic, _ver, _seq, node, words, _cr0, _cr9, flags = server.WSIQ_HEADER_STRUCT.unpack_from(pl, 0)
                self.assertEqual(magic, server.WSIQ_MAGIC)
                self.assertEqual(node, 15)
                self.assertEqual(words, 256)
                self.assertEqual((flags >> 16) & 1, 1)
            finally:
                ws.close()
        finally:
            srv.shutdown()
            srv.server_close()
            t.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
