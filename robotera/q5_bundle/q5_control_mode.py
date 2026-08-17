from legacy_direct_control import Q5ControlModePlugin

def make_plugin(plugin_config, namespace, executor, client):
    return Q5ControlModePlugin(plugin_config, namespace, executor, client)
