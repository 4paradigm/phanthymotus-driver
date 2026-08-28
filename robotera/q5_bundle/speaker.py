from legacy_device import SpeakerPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return SpeakerPlugin(plugin_config, namespace, executor, client)
