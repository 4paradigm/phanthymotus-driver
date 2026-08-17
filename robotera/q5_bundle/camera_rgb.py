from legacy_device import CameraRgbPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return CameraRgbPlugin(plugin_config, namespace, executor, client)
