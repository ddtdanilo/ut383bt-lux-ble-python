"""Unit tests for notification parsing."""

from __future__ import annotations

import unittest
from datetime import UTC

from ut383bt.parser import MeasurementParseError, parse_lux, parse_notification


class ParseLuxTests(unittest.TestCase):
    def test_parse_lux_label(self) -> None:
        self.assertEqual(parse_lux(b"\x00: 123 LUX;\r\n"), 123)

    def test_parse_lx_label_case_insensitively(self) -> None:
        self.assertEqual(parse_lux(b":0042lx;"), 42)

    def test_parse_unlabelled_integer(self) -> None:
        self.assertEqual(parse_lux(b"prefix:987;suffix"), 987)

    def test_parse_bytearray(self) -> None:
        self.assertEqual(parse_lux(bytearray(b":1 LUX;")), 1)

    def test_use_first_complete_frame(self) -> None:
        self.assertEqual(parse_lux(b":12 LUX;:34 LUX;"), 12)

    def test_reject_non_bytes(self) -> None:
        with self.assertRaisesRegex(TypeError, "bytes-like"):
            parse_lux(":12 LUX;")  # type: ignore[arg-type]

    def test_reject_missing_colon(self) -> None:
        with self.assertRaisesRegex(MeasurementParseError, "':'"):
            parse_lux(b"12 LUX;")

    def test_reject_missing_semicolon(self) -> None:
        with self.assertRaisesRegex(MeasurementParseError, "';'"):
            parse_lux(b":12 LUX")

    def test_reject_empty_frame(self) -> None:
        with self.assertRaisesRegex(MeasurementParseError, "empty"):
            parse_lux(b":;")

    def test_reject_non_ascii_frame(self) -> None:
        with self.assertRaisesRegex(MeasurementParseError, "ASCII"):
            parse_lux(b":\xff;")

    def test_reject_frame_without_number(self) -> None:
        with self.assertRaisesRegex(MeasurementParseError, "lux value"):
            parse_lux(b":OVERLOAD;")

    def test_reject_embedded_alphanumeric_number(self) -> None:
        with self.assertRaises(MeasurementParseError):
            parse_lux(b":A12B;")

    def test_build_timestamped_measurement(self) -> None:
        measurement = parse_notification(b":321 LUX;")

        self.assertEqual(measurement.lux, 321)
        self.assertEqual(measurement.raw_payload, b":321 LUX;")
        self.assertEqual(measurement.captured_at.tzinfo, UTC)
        self.assertAlmostEqual(measurement.epoch_seconds, measurement.captured_at.timestamp())


if __name__ == "__main__":
    unittest.main()
