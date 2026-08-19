from legacy_device import CameraRgbPlugin
from q5_camera_worker import CameraProxy

def make_plugin(plugin_config, namespace, executor, client):
    worker = getattr(client, "camera_worker", None)
    if worker is not None:
        config = dict(plugin_config)
        config["topic"] = f"/{namespace}/camera/rgb"
        return CameraProxy(config, worker, "rgb")
    return CameraRgbPlugin(plugin_config, namespace, executor, client)
