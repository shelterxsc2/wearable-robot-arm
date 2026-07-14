import io
import json
import unittest

from simulation.control_replay import run


class ControlReplayTest(unittest.TestCase):
    def test_first_person_and_exit(self):
        src = io.StringIO(
            '{"type":"mode","mode":"first_person"}\n'
            '{"type":"target","x":-17,"y":35,"z":20}\n'
            '{"type":"head","yaw":5,"pitch":6}\n'
            '{"type":"mode","mode":"face"}\n'
            '{"type":"head","yaw":5,"pitch":6}\n'
        )
        dst = io.StringIO()
        self.assertEqual(run(src, dst), 0)
        rows = [json.loads(line) for line in dst.getvalue().splitlines()]
        self.assertEqual(rows[2]["event"], "uart_target")
        self.assertEqual(rows[2]["j4"], 4.0)
        self.assertEqual(rows[2]["j5"], 185.0)
        self.assertEqual(rows[4]["event"], "no_uart")

    def test_invalid_mode_fails(self):
        dst = io.StringIO()
        self.assertEqual(run(io.StringIO('{"type":"mode","mode":"bad"}\n'), dst), 1)


if __name__ == "__main__":
    unittest.main()
