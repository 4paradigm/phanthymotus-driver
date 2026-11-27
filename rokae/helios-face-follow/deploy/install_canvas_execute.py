"""Patch the Helios Core canvas overlay to expose the single-card Execute action."""

import fcntl
import os
from pathlib import Path
import shutil
import sys
import time


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError("Unexpected Core canvas source around " + old[:90])
    return source.replace(old, new, 1)


def patch_canvas(source):
    if "data-helios-follow-execute" in source:
        return source
    source = replace_once(source,
        "btn.classList.toggle('locked', !_projectRunning);",
        "btn.classList.toggle('locked', !_projectRunning && !btn.hasAttribute('data-helios-follow-execute'));")
    source = replace_once(source,
        "const fieldsHtml = Object.entries(props).map(([key, def]) => {",
        """const standaloneFollow = toolName === 'helios_face_follow'
      && schema?.['x-helios-canvas-execute'] === 'toggle_person_follow';
    const fieldsHtml = standaloneFollow ? '' : Object.entries(props).map(([key, def]) => {""")
    source = replace_once(source,
        """${hasUsableFields ? `<button class="canvas-exec-btn${_projectRunning ? '' : ' locked'}">▶ 执行</button>` : ''}""",
        """${standaloneFollow ? '<button class="canvas-exec-btn" data-helios-follow-execute title="再次点击停止随动">▶ 执行</button>' : (hasUsableFields ? `<button class="canvas-exec-btn${_projectRunning ? '' : ' locked'}">▶ 执行</button>` : '')}""")
    source = replace_once(source,
        "await _executeCard(el, mcpId, toolName);",
        "await _executeCard(el, mcpId, toolName, standaloneFollow ? id : undefined,\n"
        "          standaloneFollow ? 'toggle_person_follow' : null);")
    source = replace_once(source,
        "async function _executeCard(el, mcpId, toolName, instanceId) {",
        "async function _executeCard(el, mcpId, toolName, instanceId, standaloneAction = null) {")
    source = replace_once(source,
        "  _logActivity('mcp_call', `${toolName} @ ${mcpId}`);",
        """  if (standaloneAction) {
    args.action = standaloneAction;
    if (!args.input_topic) {
      const incoming = _connections.find(c => c.toCardId === instanceId && c.format === 'image/jpeg');
      if (incoming?.fromTopic) args.input_topic = incoming.fromTopic;
    }
  }

  _logActivity('mcp_call', `${toolName} @ ${mcpId}`);""")
    return source


def main():
    path = Path(sys.argv[1])
    # Core bind-mounts this file, so keep the inode and save a rollback copy.
    with path.open("r+", encoding="utf-8") as file:
        fcntl.flock(file, fcntl.LOCK_EX)
        original = file.read()
        updated = patch_canvas(original)
        if updated == original:
            print("Helios Execute already installed")
            return
        backup = path.with_name(path.name + ".before-helios-execute-" + str(time.time_ns()))
        shutil.copy2(path, backup)
        file.seek(0)
        file.write(updated)
        file.truncate()
        file.flush()
        os.fsync(file.fileno())
        print("Installed Helios Execute. Backup:", backup)


if __name__ == "__main__":
    main()
