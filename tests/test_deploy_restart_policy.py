"""
test_deploy_restart_policy.py — 驱动在主机重启后必须自己回来。

G1 在一次重启之后所有容器仍然是 exited —— agent-core、驱动、perception 一个都没起来。
要有人 SSH 进去手动 `docker start`，在那之前机器人就是没了；而从外面看，「栈没起来」
和「部署坏了」完全一样，第一反应往往是去查错的地方。

`unless-stopped` 与 `always` 的差别正好落在这里：前者不会启动那些在守护进程停止时
就已经处于停止态的容器，而重启恰好把它们留在那个状态。代价是手动停掉的容器在重启后
也会回来 —— 在这些机器上这是更小的意外。

这条测试盯的是**所有**驱动，不是某一个：新加的驱动往往从既有的 service.yml 拷贝而来，
拷到一份 `unless-stopped` 就会把这个问题带进下一台机器人。

Run: cd phanthymotus-driver && python3 -m pytest tests/test_deploy_restart_policy.py
"""

import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
FRAGMENTS = sorted(ROOT.glob("*/*/deploy/service.yml"))


class RestartPolicyTests(unittest.TestCase):
    def test_fragments_were_found(self):
        """glob 写错会让下面每一条都空转通过。"""
        self.assertGreaterEqual(len(FRAGMENTS), 15,
                                f"只找到 {len(FRAGMENTS)} 个 service.yml，glob 可能不对")

    def test_every_driver_restarts_always(self):
        offenders = []
        for f in FRAGMENTS:
            doc = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
            for name, svc in doc.items():
                if not isinstance(svc, dict):
                    continue
                if svc.get("restart") != "always":
                    offenders.append(f"{f.relative_to(ROOT)}::{name}={svc.get('restart')!r}")
        self.assertEqual(offenders, [],
                         f"这些驱动在主机重启后不会自己回来: {offenders}")

    def test_no_fragment_still_configures_unless_stopped(self):
        """注释里提到它是可以的（那是在解释为什么不用），配置里不行。"""
        for f in FRAGMENTS:
            for line in f.read_text(encoding="utf-8").splitlines():
                if line.strip().startswith("#"):
                    continue
                self.assertNotIn("unless-stopped", line,
                                 f"{f.relative_to(ROOT)}: {line}")

    def test_every_fragment_is_valid_yaml(self):
        """这些文件是被文本替换改的，语法错会在部署时才炸。"""
        for f in FRAGMENTS:
            try:
                yaml.safe_load(f.read_text(encoding="utf-8"))
            except yaml.YAMLError as e:
                self.fail(f"{f.relative_to(ROOT)} 不是有效 YAML: {e}")


if __name__ == "__main__":
    unittest.main()
