"""S10-only recovery and health for the shared robot ROS executor."""
import threading
import time

PROGRESS_TIMEOUT_S = 3.0


class RobotExecutorGuard:
    def __init__(self, ros2):
        from rclpy.exceptions import InvalidHandle

        self.ros2 = ros2
        self._invalid_handle = InvalidHandle
        self._spin_once = ros2.executor_robot.spin_once
        self._lock = threading.Lock()
        self._entered = threading.Event()
        self._last_progress = time.monotonic()
        self._failure = None
        self._recoveries = 0

    def spin_once(self, *args, **kwargs):
        self._entered.set()
        try:
            result = self._spin_once(*args, **kwargs)
        except self._invalid_handle:
            if not self.ros2.rclpy.ok(context=self.ros2.ctx_robot):
                raise
            with self._lock:
                self._recoveries += 1
                count = self._recoveries
            if count == 1 or count % 100 == 0:
                print(f"[s10 ros2] recovering destroyed entity handle (count={count})", flush=True)
            # Repeated invalid entities must not turn recovery into a busy loop.
            time.sleep(.01)
        except Exception as exc:
            with self._lock:
                self._failure = f"{type(exc).__name__}: {exc}"[:300]
            raise
        else:
            with self._lock:
                self._last_progress = time.monotonic()
            return result

    def status(self):
        with self._lock:
            age = max(0., time.monotonic() - self._last_progress)
            error, recoveries = self._failure, self._recoveries
        threads = self.ros2._threads
        alive = bool(threads and threads[0].is_alive())
        if not error:
            if not self.ros2.rclpy.ok(context=self.ros2.ctx_robot):
                error = "robot ROS context is shut down"
            elif not alive:
                error = "robot ROS executor thread has stopped"
            elif age > PROGRESS_TIMEOUT_S:
                error = f"robot ROS executor has made no progress for {age:.1f} seconds"
        return {"state": "error" if error else "running", "error": error,
                "thread_alive": alive, "last_spin_age_s": round(age, 3), "recoveries": recoveries}


def install_robot_executor_guard(ros2):
    guard = getattr(ros2, "s10_robot_executor", None)
    if guard is None:
        guard = RobotExecutorGuard(ros2)
        ros2.executor_robot.spin_once = guard.spin_once
        ros2.s10_robot_executor = guard
        # run_driver starts spinning before build_plugins. Finish its initial
        # unguarded spin before the S10 factory creates any ROS entities.
        if ros2._threads and not guard._entered.wait(PROGRESS_TIMEOUT_S):
            raise RuntimeError("S10 robot ROS executor did not enter its recovery guard")
    return guard
