"""Voice-only host over real HTTP/DDS; simulated ASR/Pose, no hardware client."""
from smoke_ros import main


if __name__ == "__main__":
    main(host_interaction=True)
