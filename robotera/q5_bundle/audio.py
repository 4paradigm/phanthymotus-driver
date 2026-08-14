from legacy_device import AudioPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return AudioPlugin(plugin_config, namespace, executor, client)
