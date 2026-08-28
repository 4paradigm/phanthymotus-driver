from legacy_direct_control import ArmControlPlugin

def make_plugin(plugin_config, namespace, executor, client):
    return ArmControlPlugin(plugin_config, namespace, executor, client)
