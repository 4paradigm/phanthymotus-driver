from legacy_device import CameraDepthPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return CameraDepthPlugin(plugin_config, namespace, executor, client)
