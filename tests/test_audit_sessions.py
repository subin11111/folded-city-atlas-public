import struct
import sys
import unittest
import zlib
import sqlite3
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from audit_sessions import (GPS_FIELDS, connected_components, decode_gps, decode_matrix, find_enu_origin,
                            horizontal_enu, pose_quality, gps_rows, parse_arguments, plot_overview,
                            session_names, write_csv)


class AuditTests(unittest.TestCase):
    def test_rtabmap_compressed_metadata_is_validated(self):
        data = np.array([[1, 3, 7]], dtype="<i4")
        blob = zlib.compress(data.tobytes()) + struct.pack("<iii", 1, 3, 4)
        np.testing.assert_array_equal(decode_matrix(blob), data)
        with self.assertRaises(ValueError):
            decode_matrix(blob[:-12] + struct.pack("<iii", 1, 4, 4))

    def test_zero_transform_is_not_rotation(self):
        pose = np.eye(4)
        pose[:3, :3] = 0
        self.assertFalse(pose_quality(pose)["valid"])
        self.assertTrue(pose_quality(np.eye(4))["valid"])

    def test_exact_enu_origin_and_axes(self):
        np.testing.assert_allclose(horizontal_enu(0., 0., (0., 0.)), [0, 0, 0], atol=1e-9)
        east = horizontal_enu(0., .001, (0., 0.))
        north = horizontal_enu(.001, 0., (0., 0.))
        self.assertAlmostEqual(east[0], 111.3194908, places=5)
        self.assertAlmostEqual(north[1], 110.5742758, places=5)
        self.assertLess(abs(east[1]), 1e-9)
        self.assertLess(abs(north[0]), 1e-9)

    def test_components_include_isolated_nodes(self):
        self.assertEqual(connected_components({1, 2, 3, 4}, [(1, 2), (3, 3)]), [[1, 2], [3], [4]])

    def test_gnss_duplicates_staleness_and_uncertainty(self):
        blob = struct.pack("<6d", 1000., 127., 37., 45., 5., 0.)
        self.assertEqual(decode_gps(blob)[1:3], (127., 37.))
        nodes = [(1, 0, 1000.5, None, None, blob), (2, 0, 1003., None, None, blob)]
        rows, errors = gps_rows(nodes, "outside", (37., 127.))
        self.assertFalse(errors)
        self.assertTrue(rows[0]["coarse_view_eligible"])
        self.assertFalse(rows[1]["coarse_view_eligible"])
        self.assertIn("repeated_measurement_timestamp", rows[1]["quality_exclusion_reasons"])
        self.assertIn("measurement_age", rows[1]["quality_exclusion_reasons"])
        self.assertTrue(all(not row["optimization_prior_applied"] for row in rows))

    def test_cli_legacy_two_db_names_and_three_explicit_names(self):
        legacy = parse_arguments(["outside.db", "lobby.db", "--output", "out", "--skip-plots"])
        self.assertEqual(legacy.sessions, ["outside", "lobby"])
        explicit = parse_arguments(["a.db", "b.db", "c.db", "--output", "out", "--sessions",
                                    "bridge_out1f", "floor2", "bridge_12_sub"])
        self.assertEqual(explicit.dbs, list(map(Path, ["a.db", "b.db", "c.db"])))
        self.assertEqual(explicit.sessions, ["bridge_out1f", "floor2", "bridge_12_sub"])
        self.assertEqual(parse_arguments(["floor2.db", "--output", "out"]).sessions, ["floor2"])

    def test_session_names_reject_path_traversal_duplicates_and_mismatch(self):
        paths = [Path("a.db"), Path("b.db")]
        for names in (["a"], ["a", "a"], ["../outside", "lobby"], ["/tmp/a", "lobby"], [".", "b"]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                session_names(paths, names)
        self.assertEqual(session_names([Path("my capture.db")], None), ["my_capture"])

    def test_origin_skips_missing_or_invalid_fixes_and_records_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = [Path(directory) / name for name in ("missing.db", "later.db")]
            for path in (first, second):
                with sqlite3.connect(path) as connection:
                    connection.execute("CREATE TABLE Node (id INTEGER,gps BLOB)")
                    connection.execute("INSERT INTO Node VALUES (1,NULL)")
            self.assertIsNone(find_enu_origin([first, second]))
            with sqlite3.connect(second) as connection:
                connection.execute("INSERT INTO Node VALUES (2,?)", (b"malformed",))
                connection.execute("INSERT INTO Node VALUES (3,?)", (struct.pack("<6d", 1000., 127., float('nan'), 45., 5., 0.),))
                connection.execute("INSERT INTO Node VALUES (4,?)", (struct.pack("<6d", 1000., 127., 37., 45., 5., 0.),))
            origin = find_enu_origin([first, second])
            self.assertEqual(origin["source_database"], str(second.resolve()))
            self.assertEqual(origin["source_node_id"], 4)
            self.assertEqual((origin["latitude_deg"], origin["longitude_deg"]), (37., 127.))

    def test_missing_gnss_does_not_invent_observations_or_origin(self):
        rows, errors = gps_rows([(1, 0, 1000., None, None, None)], "bridge_out1f", None)
        self.assertEqual((rows, errors), ([], []))
        valid_fix = struct.pack("<6d", 1000., 127., 37., 45., 5., 0.)
        rows, _ = gps_rows([(1, 0, 1000.5, None, None, valid_fix)], "bridge_out1f", None)
        self.assertTrue(np.isnan(rows[0]["east_m"]))
        self.assertFalse(rows[0]["coarse_view_eligible"])
        self.assertIn("no_valid_common_enu_origin", rows[0]["quality_exclusion_reasons"])
        self.assertNotIn("indoor", rows[0]["optimization_prior_policy"])

    def test_three_session_overview_handles_empty_gnss(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            sessions = ["bridge_out1f", "floor2", "bridge_12_sub"]
            for session in sessions:
                (output / session).mkdir()
                write_csv(output / session / "raw_trajectory.csv", [{"id": 1, "x": 0., "y": 0.}])
                write_csv(output / f"{session}_gnss_raw.csv", [], GPS_FIELDS)
            plot_overview(output, sessions)
            self.assertGreater((output / "session_and_gnss_overview.png").stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
