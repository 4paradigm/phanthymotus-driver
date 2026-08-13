import pathlib
import unittest


G1_DIR = pathlib.Path(__file__).resolve().parents[1]


class DockerContextTests(unittest.TestCase):
    def test_joint_health_module_is_copied_into_image(self):
        dockerfile = (G1_DIR / "Dockerfile").read_text()
        self.assertIn("COPY joint_health.py /work/joint_health.py", dockerfile)


if __name__ == "__main__":
    unittest.main()
