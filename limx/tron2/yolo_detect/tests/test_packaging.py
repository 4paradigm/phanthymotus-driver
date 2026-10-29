"""Packaging boundary: reuse runtime, no package/OS installation in either stage."""
from pathlib import Path
import re
import unittest


class PackagingTests(unittest.TestCase):
    def setUp(self):
        self.root=Path(__file__).resolve().parents[1]
        self.text=(self.root/'Dockerfile').read_text()

    def test_no_os_package_install_or_unreviewed_base(self):
        self.assertIn('ARG BASE_IMAGE\n',self.text)
        self.assertEqual(re.findall(r'^FROM (.+)$',self.text,re.M),
                         ['${BASE_IMAGE} AS verify','${BASE_IMAGE} AS runtime'])
        self.assertNotRegex(self.text,r'(?m)^ARG BASE_IMAGE=')
        self.assertNotRegex(self.text,r'(?i)\b(apt-get|apt|pip|pip3|curl|wget)\s')

    def test_runtime_has_no_run_or_test_copy(self):
        final=self.text.split('FROM ${BASE_IMAGE} AS runtime',1)[1]
        self.assertNotRegex(final,r'(?m)^RUN ')
        self.assertNotIn('tests/',final)
        copies=re.findall(r'^COPY (.+)$',final,re.M)
        self.assertTrue(copies)
        self.assertTrue(all(c.startswith('--from=verify /opt/tron2-') for c in copies))
        self.assertIn('USER 65532:65532',final)

    def test_no_production_mounts_privilege_or_control_instructions(self):
        self.assertNotRegex(self.text,r'(?m)^(VOLUME|ADD) ')
        self.assertNotIn('/etc/',self.text)
        self.assertNotIn('systemctl',self.text)
        self.assertNotIn('docker.sock',self.text)


if __name__=='__main__':unittest.main(verbosity=2)
