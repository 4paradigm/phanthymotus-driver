"""Offline thread/spawn tests of the publication fence; no ROS or hardware."""

import multiprocessing
import threading
import unittest
from types import SimpleNamespace

from sensor_output import OutputGate


def _blocked_publish(gate, entered, release, result):
    class Publisher:
        def publish(self, message):
            entered.set()
            if not release.wait(8):
                raise RuntimeError("test publisher timed out")

    try:
        result.send(("ok", gate.publish(gate.token(), Publisher(), "frame")))
    except Exception as exc:
        result.send(("error", repr(exc)))
    finally:
        result.close()


class OutputGateConcurrencyTests(unittest.TestCase):
    def test_spawn_blocked_publish_does_not_block_token_but_fences_stop(self):
        ctx = multiprocessing.get_context("spawn")
        gate = OutputGate()
        entered, release = ctx.Event(), ctx.Event()
        parent, child = ctx.Pipe(duplex=False)
        process = ctx.Process(target=_blocked_publish, args=(gate, entered, release, child))
        process.start()
        child.close()
        stopper_done = threading.Event()
        errors = []

        def stop():
            try:
                gate.set_enabled(False)
            except Exception as exc:
                errors.append(exc)
            finally:
                stopper_done.set()

        stopper = None
        try:
            self.assertTrue(entered.wait(5))
            token = gate.token()
            self.assertIsNotNone(token)
            self.assertEqual(gate.status()["published"], 0)
            stopper = threading.Thread(target=stop)
            stopper.start()
            self.assertFalse(stopper_done.wait(0.1))
            self.assertEqual(gate.token(), token)
            release.set()
            self.assertTrue(stopper_done.wait(2))
            self.assertFalse(errors)
            self.assertTrue(parent.poll(5))
            self.assertEqual(parent.recv(), ("ok", True))
            self.assertIsNone(gate.token())
            self.assertEqual(gate.status()["state"], "idle")
            gate.set_enabled(True)
            self.assertNotEqual(gate.token(), token)
            self.assertFalse(gate.status()["ready"])
            sent = []
            publisher = SimpleNamespace(publish=sent.append)
            self.assertFalse(gate.publish(token, publisher, "old"))
            self.assertTrue(gate.publish(gate.token(), publisher, "new"))
            self.assertEqual(sent, ["new"])
        finally:
            release.set()
            if stopper:
                stopper.join(3)
            process.join(5)
            if process.is_alive():
                process.terminate()
                process.join(5)
            parent.close()
        self.assertEqual(process.exitcode, 0)

    def test_stop_timeout_does_not_claim_idle_or_change_epoch(self):
        ctx = multiprocessing.get_context("spawn")
        gate = OutputGate()
        token = gate.token()
        entered, release = ctx.Event(), ctx.Event()
        parent, child = ctx.Pipe(duplex=False)
        process = ctx.Process(target=_blocked_publish, args=(gate, entered, release, child))
        process.start()
        child.close()
        try:
            self.assertTrue(entered.wait(5))
            with self.assertRaisesRegex(TimeoutError, "sensor_output_transition_timeout"):
                gate.set_enabled(False)
            self.assertEqual(gate.token(), token)
            self.assertTrue(gate.status()["enabled"])
            release.set()
            self.assertTrue(parent.poll(5))
            self.assertEqual(parent.recv(), ("ok", True))
            gate.set_enabled(False)
            self.assertIsNone(gate.token())
        finally:
            release.set()
            process.join(5)
            if process.is_alive():
                process.terminate()
                process.join(5)
            parent.close()
        self.assertEqual(process.exitcode, 0)

    def test_nonblocking_state_contention_and_record_false(self):
        gate = OutputGate()
        token = gate.token()
        self.assertTrue(gate._state_lock.acquire(timeout=1))
        try:
            self.assertIsNone(gate.token())
        finally:
            gate._state_lock.release()
        sent = []
        self.assertTrue(gate.publish(token, SimpleNamespace(publish=sent.append), "x", record=False))
        self.assertEqual(sent, ["x"])
        self.assertEqual(gate.status()["published"], 0)
        gate.set_enabled(False)
        gate.set_enabled(True)
        self.assertFalse(gate.publish(token, SimpleNamespace(publish=sent.append), "stale"))
        self.assertEqual(sent, ["x"])

    def test_state_lock_timeout_leaves_transition_unchanged(self):
        gate = OutputGate()
        token = gate.token()
        self.assertTrue(gate._state_lock.acquire(timeout=1))
        try:
            with self.assertRaisesRegex(TimeoutError, "sensor_output_transition_timeout"):
                gate.set_enabled(False)
        finally:
            gate._state_lock.release()
        self.assertEqual(gate.token(), token)
        self.assertTrue(gate.status()["enabled"])
        gate.set_enabled(False)
        self.assertIsNone(gate.token())

    def test_reentrant_stop_inside_publish_cannot_bypass_fence(self):
        gate = OutputGate()
        errors = []
        def publish(_):
            try:
                gate.set_enabled(False)
            except TimeoutError as exc:
                errors.append(str(exc))
        self.assertTrue(gate.publish(gate.token(), SimpleNamespace(publish=publish), "x"))
        self.assertEqual(errors, ["sensor_output_transition_timeout"])
        self.assertTrue(gate.status()["enabled"])
        gate.set_enabled(False)

    def test_publish_exception_releases_fence_without_recording(self):
        gate = OutputGate()
        def broken(_):
            raise ValueError("publish failed")
        with self.assertRaisesRegex(ValueError, "publish failed"):
            gate.publish(gate.token(), SimpleNamespace(publish=broken), "x")
        self.assertEqual(gate.status()["published"], 0)
        gate.set_enabled(False)
        self.assertIsNone(gate.token())


if __name__ == "__main__":
    unittest.main()
