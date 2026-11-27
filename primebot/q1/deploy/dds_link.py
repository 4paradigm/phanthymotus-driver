"""Mac Docker Desktop DDS relay for a Linux driver sharing the Core network."""
import argparse
import selectors
import socket
import sys
import xml.etree.ElementTree as ET

META, DATA = 17810, 17811
GROUP = "239.255.0.1"


def locator(parent, address, port=None):
    udp = ET.SubElement(ET.SubElement(parent, "locator"), "udpv4")
    ET.SubElement(udp, "address").text = address
    if port:
        ET.SubElement(udp, "port").text = str(port)


def profile(args):
    root = ET.parse("/q1_ws/install/aimdk_msgs/share/aimdk_msgs/config/fastdds_profiles.xml").getroot()
    ns = root.tag.split("}")[0][1:]
    ET.register_namespace("", ns)
    rtps = root.find(".//{*}rtps")
    locator(ET.SubElement(rtps, "defaultUnicastLocatorList"), args.mac, DATA)
    builtin = ET.SubElement(rtps, "builtin")
    locator(ET.SubElement(builtin, "metatrafficUnicastLocatorList"), args.mac, META)
    locator(ET.SubElement(builtin, "initialPeersList"), args.robot)
    ET.ElementTree(root).write(sys.stdout, encoding="unicode")


def discovery(args):
    with selectors.DefaultSelector() as selector:
        announcements = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        announcements.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        announcements.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        announcements.bind(("", 7400))
        announcements.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                  socket.inet_aton(GROUP) + socket.inet_aton(args.mac))
        selector.register(announcements, selectors.EVENT_READ, META)
        for port in (META, DATA):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((args.mac, port))
            selector.register(sock, selectors.EVENT_READ, port)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as forward:
            # Darwin limits an outgoing datagram to the socket send buffer size.
            forward.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65535)
            while True:
                for key, _ in selector.select():
                    packet, source = key.fileobj.recvfrom(65535)
                    if source[0] == args.robot and packet.startswith(b"RTPS"):
                        forward.sendto(packet, ("127.0.0.1", key.data + 2))


def ingress(args):
    core = socket.gethostbyname(args.core)
    with selectors.DefaultSelector() as selector:
        for port in (META, DATA):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("", port))
            selector.register(sock, selectors.EVENT_READ, port)
        while True:
            for key, _ in selector.select():
                packet, _ = key.fileobj.recvfrom(65535)
                key.fileobj.sendto(packet, (core, key.data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("profile", "discovery", "ingress"))
    parser.add_argument("--mac", default="10.1.1.99")
    parser.add_argument("--robot", default="10.1.1.100")
    parser.add_argument("--core", default="core")
    args = parser.parse_args()
    {"profile": profile, "discovery": discovery, "ingress": ingress}[args.mode](args)
