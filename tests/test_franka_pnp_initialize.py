import json
from pathlib import Path
import runpy
import tempfile
import threading
from unittest import TestCase, main
from unittest.mock import patch
from franka.fr3.pnp_remote_teleop import PnpRemoteTeleopPlugin
from franka.fr3 import pnp_initialize

class InitializationTests(TestCase):
    def test_config_does_not_return_password(self):
        p=PnpRemoteTeleopPlugin()
        result=p.dispatch('config', {'pnp_host':'172.16.0.200','ssh_user':'pnp','ssh_password':'test-secret'})
        self.assertNotIn('test-secret', json.dumps(result))
        self.assertEqual(result['state'],'configured')
        self.assertEqual(p.get_tools()[0]['configSchema']['properties']['ssh_password']['format'],'password')

    def test_start_installs_without_follow(self):
        p=PnpRemoteTeleopPlugin()
        p.dispatch('config', {'ssh_password':'test-secret', 'pnp_host':'172.16.0.200'})
        entered, release=threading.Event(),threading.Event()
        def install(host,password,progress,**kwargs):
            entered.set()
            release.wait(2)
            return {'state':'ready'}
        with patch.object(pnp_initialize,'initialize', side_effect=install), patch.object(p,'_ssh') as ssh:
            self.assertEqual(p.dispatch('start',{})['state'],'initializing')
            self.assertTrue(entered.wait(1))
            self.assertEqual(p._password,'')
            p.dispatch('follow',{})
            ssh.assert_not_called()
            release.set()
            p._init_thread.join(3)
            self.assertEqual(p._init['state'],'ready')

    def test_failure_redacts_password(self):
        p=PnpRemoteTeleopPlugin()
        p.dispatch('config',{'ssh_password':'test-secret', 'pnp_host':'172.16.0.200'})
        with patch.object(pnp_initialize,'initialize', side_effect=RuntimeError('test-secret failed')):
            p.dispatch('start',{})
            p._init_thread.join(3)
        self.assertNotIn('test-secret',json.dumps(p._init))
        self.assertEqual(p._init['state'],'error')

    def test_remote_program_compiles(self):
        compile(pnp_initialize.REMOTE,'remote-installer','exec')


class VendorIPTests(TestCase):
    def test_vendor_assignments_and_ambiguous_side(self):
        path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote/read_vendor_ips.py"
        read = runpy.run_path(str(path))["read_vendor_ips"]
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            root = home / "frankateleop"
            marker = root / "right_franka/1_launch_robot.sh"
            marker.parent.mkdir(parents=True)
            marker.touch()
            conf = root / "polymetis/polymetis/python/polymetis/conf"
            for side, address in (("right", "172.16.0.22"), ("left", "172.16.0.23")):
                for kind, name in (("robot_client", f"{side}_franka_hardware"),
                                   ("gripper", f"{side}_franka_hand")):
                    file = conf / kind / (name + ".yaml")
                    file.parent.mkdir(parents=True, exist_ok=True)
                    file.write_text(f'  robot_ip: "{address}"\n')
            self.assertEqual(read(home)["ips"], {"right": "172.16.0.22", "left": "172.16.0.23"})
            (conf / "gripper/left_franka_hand.yaml").write_text('robot_ip: "172.16.0.99"\n')
            result = read(home)
            self.assertEqual(result["ips"], {"right": "172.16.0.22"})
            self.assertIn("left", result["errors"])


class VendorMasterTests(TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote/read_vendor_masters.py"
        self.read = runpy.run_path(str(path))["read_vendor_masters"]
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.home = Path(self.directory.name)
        self.root = self.home / "frankateleop"
        self.usb = self.home / "usb"
        self.usb.mkdir()
        marker = self.root / "teleop/experiments/run_env.py"
        marker.parent.mkdir(parents=True)
        marker.touch()
        self.agent = self.root / "teleop/teleop/agents/teleop_agent.py"
        self.agent.parent.mkdir(parents=True)
        self.ports = {}
        for side, serial in (("right", "NEW_RIGHT_123"), ("left", "NEW_LEFT_456")):
            name = f"usb-FTDI_USB__-__Serial_Converter_{serial}-if00-port0"
            self.ports[side] = "/dev/serial/by-id/" + name
            (self.usb / name).symlink_to("/dev/null")
            script = self.root / f"{side}_franka/4_run_env.sh"
            script.parent.mkdir(parents=True)
            script.write_text("python3 run_env.py " + chr(92) + "\n --teleop_port=" + self.ports[side] + "\n")
        self.agent.write_text("PORT_CONFIG_MAP = " + repr({p: None for p in self.ports.values()}))

    def test_new_serials_keep_vendor_left_right_roles(self):
        result = self.read(self.home, self.usb)
        self.assertEqual(result["arms"]["right"]["serial"], "NEW_RIGHT_123")
        self.assertEqual(result["arms"]["left"]["serial"], "NEW_LEFT_456")
        self.assertTrue(all(arm["ready"] for arm in result["arms"].values()))

    def test_unmapped_serial_is_not_assigned_by_order(self):
        self.agent.write_text("PORT_CONFIG_MAP = " + repr({self.ports["right"]: None}))
        result = self.read(self.home, self.usb)
        self.assertNotIn("left", result["arms"])
        self.assertIn("left", result["errors"])
        self.assertEqual(len(result["unmatched_usb_devices"]), 1)

    def test_duplicate_master_is_rejected(self):
        (self.root / "left_franka/4_run_env.sh").write_text("python3 run_env.py --teleop_port=" + self.ports["right"])
        result = self.read(self.home, self.usb)
        self.assertEqual(result["arms"], {})
        self.assertIn("configuration", result["errors"])


class AlignmentCompletionTests(TestCase):
    def test_options_and_automatic_initialization(self):
        plugin = PnpRemoteTeleopPlugin()
        definition = plugin.get_tools()[0]
        self.assertEqual(definition["inputSchema"]["properties"]["action"]["enum"],
                         ["align", "follow", "stop", "status"])
        self.assertNotIn("auto_initialize", definition["configSchema"]["properties"])
        self.assertEqual(definition["inputSchema"]["x-completion"]["actions"], ["align"])

    def test_completed_alignment_posts_acp(self):
        completed = []
        plugin = PnpRemoteTeleopPlugin(acp_callback=lambda *args: completed.append(args))
        plugin._acp = {"action_id": "align-test", "state": "waiting_for_motion"}
        with patch.object(plugin, "_status", return_value={"state": "aligned", "hosts": {}}):
            plugin._watch_alignment("align-test", threading.Event())
        self.assertEqual(completed[0][:2], ("align-test", "completed"))
        self.assertEqual(plugin._acp["state"], "accepted")

    def test_follow_requires_confirmed_alignment(self):
        plugin = PnpRemoteTeleopPlugin()
        plugin._loaded = True
        with patch.object(plugin, "_status", return_value={"state": "starting_follow"}), patch.object(plugin, "_ssh") as ssh:
            result = plugin.dispatch("follow", {})
        self.assertEqual(result["state"], "error")
        ssh.assert_not_called()

if __name__=='__main__':
    main()
