from legacy_device import CameraPointCloudPlugin
from q5_camera_worker import CameraProxy


def make_plugin(plugin_config, namespace, executor, client):
    worker = getattr(client, "camera_worker", None)
    if worker is not None:
        config = dict(plugin_config)
        config["topic"] = f"/{namespace}/camera/pointcloud"
        return CameraProxy(config, worker, "pointcloud")
    return CameraPointCloudPlugin(plugin_config, namespace, executor, client)
