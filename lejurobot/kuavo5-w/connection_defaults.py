"""Operator-requested defaults for the commissioned Kuavo 5-W/Jaten topology.

Only non-secret topology belongs here. Authentication is supplied at deployment
through private /config secrets, environment variables, or SSH keys.
"""
DEFAULTS = {
    'host': {'address': '127.0.0.1', 'user': 'leju_kuavo', 'ros_ip': '192.168.26.12'},
    'controller': {'address': '192.168.26.1', 'user': 'lab', 'ros_ip': '192.168.26.1'},
    'chassis': {'address': '192.168.26.22', 'user': 'jt', 'ros_ip': '192.168.26.22'},
}
