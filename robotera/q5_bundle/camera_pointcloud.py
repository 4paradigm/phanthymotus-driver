from legacy_device import CameraPointCloudPlugin


def make_plugin(plugin_config, namespace, executor, client):
    return CameraPointCloudPlugin(plugin_config, namespace, executor, client)
