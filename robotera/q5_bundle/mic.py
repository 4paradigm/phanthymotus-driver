from legacy_device import MicPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return MicPlugin(plugin_config, namespace, executor, client)
