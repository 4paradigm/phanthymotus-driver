import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stationary_follow import FollowCard

def frame(stamp, persons):
    return {"timestamp": stamp, "count": len(persons), "persons": persons}

class TestMotusFollow(unittest.TestCase):
    def test_left_right_and_center(self):
        c = FollowCard()
        c.start()
        self.assertLess(c.ingest(frame(1,[{"id":1,"position":[.6,0]}]))["vyaw"],0)
        self.assertGreater(c.ingest(frame(2,[{"id":1,"position":[-.6,0]}]))["vyaw"],0)
        self.assertEqual(c.ingest(frame(3,[{"id":1,"position":[0,0]}]))["command"],"stop_move")

    def test_ambiguous_start_and_target_loss(self):
        c = FollowCard()
        c.start()
        self.assertEqual(c.ingest(frame(1,[{"id":1,"position":[0,0]},{"id":2,"position":[0,0]}]))["command"],"none")
        c.ingest(frame(2,[{"id":1,"position":[.3,0]}]))
        self.assertEqual(c.ingest(frame(3,[{"id":2,"position":[.3,0]}]))["reason"],"target_lost")
        self.assertEqual(c.ingest(frame(4,[{"id":1,"position":[.3,0]}]))["command"],"none")

    def test_timeout_and_manual_stop(self):
        c = FollowCard()
        c.start()
        c.ingest(frame(1,[{"id":1,"position":[.5,0]}]))
        self.assertEqual(c.tick(.31)["command"],"stop_move")
        c.start()
        self.assertEqual(c.stop()["command"],"stop_move")

    def test_repeated_frame_stops(self):
        c = FollowCard()
        c.start()
        c.ingest(frame(1,[{"id":1,"position":[.4,0]}]))
        self.assertEqual(c.ingest(frame(1,[{"id":1,"position":[.4,0]}]))["reason"],"stale_pose")

if __name__ == "__main__":
    unittest.main()
