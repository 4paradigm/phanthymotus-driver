"""Keep image defaults consistent with the mounted loopback DDS profile."""

from pathlib import Path
import unittest


class DDSImageContractTests(unittest.TestCase):
    def test_image_does_not_override_profile_transports(self):
        dockerfile = Path(__file__).resolve().parents[1] / "Dockerfile"
        instructions = "\n".join(
            line for line in dockerfile.read_text().splitlines()
            if not line.lstrip().startswith("#")
        )
        self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", instructions)


if __name__ == "__main__":
    unittest.main()
