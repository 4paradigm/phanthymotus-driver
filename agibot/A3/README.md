# A3 Ultra sensor notes

The two wrist D405 camera topics are advertised by the robot's
`hal_depth_camera_node`, but on the deployed A3 Ultra image they have a DDS
publisher with no image samples. The driver therefore does not expose wrist
camera cards or subscribe to those topics. This is a robot HAL/firmware issue,
not a frontend topic-name issue; restore the entries in `config.yaml` and
`driver.yaml` only after the vendor node produces samples with
`ros2 topic echo --qos-reliability best_effort`.

The remaining camera preview streams are intentionally resized and rate
limited before JPEG encoding. This bounds CPU and end-to-end latency while
keeping the latest frame available to the dashboard.
