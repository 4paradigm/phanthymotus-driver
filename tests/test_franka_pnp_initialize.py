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
            p.dispatch('initialize',{})
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

if __name__=='__main__':
    main()
