import unittest

from greeting import GreetingController, GreetingObservation


class GreetingControllerTests(unittest.TestCase):
    def test_triggers_after_consecutive_frames(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=3, cooldown_s=10)
        self.assertFalse(controller.update(GreetingObservation(2.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(1.8), now=1))
        self.assertTrue(controller.update(GreetingObservation(1.5), now=2))

    def test_does_not_retrigger_until_person_leaves(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=0)
        self.assertTrue(controller.update(GreetingObservation(1.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(1.0), now=1))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=2))
        self.assertTrue(controller.update(GreetingObservation(1.0), now=3))

    def test_cooldown_blocks_new_trigger(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=10)
        self.assertTrue(controller.update(GreetingObservation(1.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=1))
        self.assertFalse(controller.update(GreetingObservation(1.0), now=2))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=9))
        self.assertTrue(controller.update(GreetingObservation(1.0), now=10))

    def test_invalid_and_missing_distance_are_outside(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=0)
        self.assertFalse(controller.update(None, now=0))
        self.assertFalse(controller.update(GreetingObservation(0), now=1))
        self.assertFalse(controller.update(GreetingObservation(2.1), now=2))


if __name__ == "__main__":
    unittest.main()
