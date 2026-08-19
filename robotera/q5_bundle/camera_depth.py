from legacy_device import CameraDepthPlugin
from q5_camera_worker import CameraProxy

def make_plugin(plugin_config, namespace, executor, client):
    worker = getattr(client, "camera_worker", None)
    if worker is not None:
        config = dict(plugin_config)
        config["topic"] = f"/{namespace}/camera/depth_preview"
        return CameraProxy(config, worker, "depth")
    return CameraDepthPlugin(plugin_config, namespace, executor, client)
