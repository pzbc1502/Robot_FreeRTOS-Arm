"""Offline scan wire/client checks. Does not open a serial port."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "UpperComputer"))
import ra6m5_protocol as p
from scan_client import ScanClient


def status(event=2, state=2, flags=0, action=0, result=1, error=0, scan_id=100, seq=22):
    payload = bytes([event, state, flags, action, result, error]) + scan_id.to_bytes(4, "little")
    return p.build_unified_frame(0x82, seq, payload)


class ScanProtocolTests(unittest.TestCase):
    def test_wire_and_legacy(self):
        p.self_test()
        frame = p.build_scan_control(1, 0, 0x12345678, 20)
        self.assertEqual(frame[:12], bytes.fromhex("A5 5A 01 07 14 06 01 00 78 56 34 12"))
        self.assertEqual(len(frame), 14)
        self.assertEqual(p.crc16_modbus(frame[2:-2]), int.from_bytes(frame[-2:], "little"))
        vectors = (
            (p.build_scan_control(1, 0, 100, 20), "A5 5A 01 07 14 06 01 00 64 00 00 00 7F EC"),
            (p.build_scan_control(0, 0, 100, 22), "A5 5A 01 07 16 06 00 00 64 00 00 00 FF E4"),
            (status(5, 5, 2, 1, seq=20), "A5 5A 01 82 14 0A 05 05 02 01 01 00 64 00 00 00 E2 13"),
            (status(8, 11, 6, 1, seq=20), "A5 5A 01 82 14 0A 08 0B 06 01 01 00 64 00 00 00 3E 7A"),
        )
        document = (Path(__file__).resolve().parents[1] / "doc" /
                    "Jetson\u2194RA6M5 \u901a\u4fe1\u534f\u8bae.md").read_text(encoding="utf-8")
        for actual, expected in vectors:
            self.assertEqual(actual, bytes.fromhex(expected))
            self.assertIn(expected, document)

    def test_fragmentation(self):
        data = status() + status(8, 11, 6, 1, seq=20)
        for split in range(len(data) + 1):
            a, remainder = p.parse_ra6_status_stream(data[:split])
            b, remainder = p.parse_ra6_status_stream(remainder + data[split:])
            self.assertEqual(len(a + b), 2)
            self.assertEqual(remainder, b"")

    def test_corruption_and_reserved(self):
        raw = bytearray(status()); raw[-1] ^= 1
        self.assertEqual(p.parse_ra6_status_frames(bytes(raw)), [])
        for raw in (status(state=3), status(state=4), status(state=6), status(state=7), status(event=3), status(event=4), status(flags=1), status(flags=7), status(flags=8), status(result=2)):
            self.assertEqual(p.parse_ra6_status_frames(raw), [])
        raw = b"\xA5\x5A\x01\x82\x14\xFF" + status()
        self.assertEqual(len(p.parse_ra6_status_frames(raw)), 1)

    def test_invalid_commands(self):
        for args in ((1,0,0,1), (2,1,1,1), (2,0,1,1), (2,2,1,1), (0,1,0,1), (1,0,1,0), (1,0,-1,1)):
            with self.assertRaises(ValueError): p.build_scan_control(*args)

    def test_exact_retries(self):
        c = ScanClient(); original = c.start(100,20,0)
        for now in (.31,.62,.93): self.assertIn(original,c.poll(now,lambda:22))
        self.assertNotIn(original,c.poll(1.24,lambda:22))
        self.assertTrue(c.active)

    def test_lost_final_query(self):
        c=ScanClient();c.start(100,20,0)
        c.receive(p.decode_scan_status(status(1,1,action=1,seq=20)))
        c.query(22)
        c.receive(p.decode_scan_status(status(2,11,6,0,seq=22)))
        self.assertTrue(c.confirmed)
        c.receive(p.decode_scan_status(status(1,11,6,1,seq=20)))
        self.assertTrue(c.confirmed)  # ACK never creates or removes the current qualification.
        c.query(23);c.receive(p.decode_scan_status(status(2,9,0,0,error=7,seq=23)))
        self.assertFalse(c.confirmed);self.assertFalse(c.active)

    def test_wrong_id_seq_and_home(self):
        c=ScanClient();c.start(100,20,0)
        for raw in (status(8,11,6,1,scan_id=99,seq=20),
                    status(8,11,6,1,seq=21),status(2,11,6,0,seq=22)):
            self.assertFalse(c.receive(p.decode_scan_status(raw)))
        self.assertFalse(c.confirmed)
        c.receive(p.decode_scan_status(status(6,10,6,1,seq=20)))
        self.assertFalse(c.confirmed)

    def test_continuous_without_middle_handshake(self):
        c=ScanClient();c.start(100,20,0)
        c.receive(p.decode_scan_status(status(1,1,action=1,seq=20)))
        c.query(22);c.receive(p.decode_scan_status(status(2,2,0,0,seq=22)),1)
        self.assertEqual(c.stage_deadline,33)
        c.query(23);c.receive(p.decode_scan_status(status(2,2,0,0,seq=23)),20)
        self.assertEqual(c.stage_deadline,33)
        self.assertFalse(c.confirmed)
        c.receive(p.decode_scan_status(status(5,5,2,1,seq=20)),26)
        self.assertFalse(c.confirmed)
        c.receive(p.decode_scan_status(status(6,10,6,1,seq=20)),46)
        self.assertFalse(c.confirmed)
        c.receive(p.decode_scan_status(status(8,11,6,1,seq=20)),65)
        self.assertTrue(c.confirmed)
        self.assertFalse(hasattr(c, "photo"))
        self.assertTrue(all(frame[6] in (0,1) for frame in c.poll(66,lambda:24)))

    def test_retired_photo_rejection_is_diagnostic_only(self):
        raw=status(1,0,0,2,result=0,error=0x0C,seq=21)
        self.assertIsNotNone(p.decode_scan_status(raw))
        c=ScanClient();c.start(100,20,0)
        self.assertFalse(c.receive(p.decode_scan_status(raw),1))
        self.assertFalse(c.confirmed)

    def test_deadline_not_extended_by_repeated_snapshot(self):
        c=ScanClient();c.start(100,20,0)
        c.query(22)
        c.receive(p.decode_scan_status(status(2,1,0,0,seq=22)),1)
        self.assertEqual(c.stage_deadline,63)
        c.query(23)
        c.receive(p.decode_scan_status(status(2,1,0,0,seq=23)),30)
        self.assertEqual(c.stage_deadline,63)
        self.assertTrue(c.expired(63))

    def test_late_final_after_cancel(self):
        c=ScanClient();c.start(100,20,0);c.abort()
        c.receive(p.decode_scan_status(status(8,11,6,1,seq=20)),1)
        self.assertFalse(c.confirmed)


if __name__ == "__main__":
    unittest.main()
