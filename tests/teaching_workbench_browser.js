// Evaluate in the real browser tab served by tests/serve_tron2_teaching.py.
// The fixture is loopback-only and uses a fake vendor; never run against a robot.
(async () => {
  if (location.hostname !== '127.0.0.1' || document.title !== 'TRON2 teaching local fixture')
    throw new Error('This check requires the local fake-vendor fixture.');
  let d = document.querySelector('iframe').contentDocument;
  const $ = id => d.getElementById(id);
  const assert = (ok, message) => { if (!ok) throw new Error(message); };
  const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
  async function until(test, message, timeout = 10000) {
    window.teachingWorkbenchCheck = {state:'running', step:message};
    const end = Date.now() + timeout;
    while (!test()) { if (Date.now() > end) throw new Error(message + ': ' + $('error').textContent); await wait(100); }
  }
  const click = id => { assert(!$(id).disabled, id + ' is disabled'); $(id).click(); };
  const input = (id, value) => { $(id).value = value; $(id).dispatchEvent(new Event('input', {bubbles:true})); };
  await until(() => $('connection').textContent === '驱动已连接', 'connection');
  assert(!$('prepare').hidden, 'Start with a new fixture root and fresh draft');
  click('check-btn'); await until(() => !$('next-btn').disabled, 'device checks');
  click('next-btn'); await until(() => !$('record').hidden && !$('record-btn').disabled, 'record page');
  assert([...d.querySelectorAll('.steps li')].map(x => x.textContent.replace(/^[0-9✓]/, '')).join('/') === '设备检查/录制/试运行/保存', 'four steps changed');
  click('record-btn'); input('record-task', ''); click('record-start-go');
  assert(!$('record-task-error').hidden, 'empty task accepted');
  input('record-task', '浏览器回归：方块放入托盘'); click('record-start-go');
  await until(() => $('record-start').hidden && !$('finish-btn').hidden, 'start recording');
  await wait(2200); click('finish-btn');
  await until(() => !$('review').hidden && !$('open-training-btn').disabled, 'finish recording');
  assert($('to-save-btn').disabled, 'save allowed without trial/effect');
  $('training-entry').open = true; click('open-training-btn');
  await until(() => !$('training').hidden && !$('annotate-btn').disabled, 'training page');
  assert($('dataset-task').value === '浏览器回归：方块放入托盘', 'task not loaded');
  input('dataset-notes', '修改中，不应被轮询覆盖'); await wait(1200);
  assert($('dataset-notes').value === '修改中，不应被轮询覆盖', 'poll overwrote edited labels');
  click('annotate-btn'); await until(() => $('annotate-state').textContent === '标签已保存' && !$('export-btn').disabled, 'annotate');
  click('export-btn'); await until(() => $('export-status').textContent === '✓ 已导出', 'official training export', 90000);
  assert($('export-help').textContent.includes('已通过官方读取'), 'export not reader verified');
  const exportResult = $('export-help').textContent;
  click('training-back-top'); await until(() => !$('review').hidden && !$('inspect-btn').disabled, 'return');
  click('inspect-btn'); await until(() => !$('trial-btn').disabled, 'trial preflight');
  click('trial-btn'); await until(() => !$('confirm').hidden && !$('confirm-control').disabled, 'confirm dialog');
  click('confirm-control'); click('confirm-go');
  await until(() => !$('confirm-error').hidden, 'empty gripper parameters accepted');
  await until(() => !$('inspect-btn').disabled, 'parameter validation settled');
  input('replay-gripper-speed', '0'); input('replay-gripper-force', '20'); click('confirm-go');
  assert(!$('confirm').hidden, 'invalid speed closed confirmation');
  await until(() => !$('inspect-btn').disabled, 'invalid speed settled');
  input('replay-gripper-speed', '50'); input('replay-gripper-force', '30'); click('confirm-go');
  assert(!$('stop-btn').disabled, 'stop unavailable while trial request is pending');
  await until(() => $('trial-status').textContent === '✓ 试运行已完成', 'fake-vendor trial', 20000);
  click('effect'); click('to-save-btn'); input('skill-name', '浏览器回归技能'); input('skill-conditions', '物体布置固定，手动恢复起点'); click('save-btn');
  await until(() => !$('confirmation-result').hidden, 'skill saved');
  const saved = $('saved-phrase').textContent;
  assert($('new-record-btn') && !$('new-record-btn').hidden, 'saved skill has no next recording entry');
  const frame = document.querySelector('iframe'), previous = d;
  frame.contentWindow.location.reload();
  await until(() => frame.contentDocument !== previous && frame.contentDocument.getElementById('new-record-btn'), 'reopen saved workbench');
  d = frame.contentDocument;
  await until(() => !$('new-record-btn').hidden && !$('new-record-btn').disabled, 'next recording after reopen');
  click('new-record-btn');
  await until(() => !$('prepare').hidden && !$('check-btn').disabled, 'new draft');
  await wait(2200);
  assert(!$('prepare').hidden, 'poll returned new draft to saved skill');
  assert(!$('effect').checked && $('skill-name').value === '' && $('skill-conditions').value === '', 'previous skill state leaked');
  click('check-btn'); await until(() => !$('next-btn').disabled, 'checks before failure');
  click('next-btn'); await until(() => !$('record').hidden && !$('record-btn').disabled, 'second record page');
  click('record-btn'); input('record-task', '断流诊断'); click('record-start-go');
  await until(() => $('record-start').hidden && !$('finish-btn').hidden, 'second recording');
  await wait(1000);
  const before = await (await fetch('/fixture/camera-failure', {method:'POST'})).json();
  await until(() => $('motion-state').textContent.includes('录制失败'), 'camera failure');
  click('finish-btn'); await until(() => !$('training').hidden, 'failed recording diagnosis');
  assert($('export-btn').disabled && $('annotate-btn').disabled, 'incomplete episode allowed export/annotation');
  assert($('quality-rows').textContent.includes('相机数据无效或中断'), 'failure not displayed');
  assert($('export-status').textContent === '尚未导出', 'previous episode export leaked into diagnosis');
  const after = await (await fetch('/fixture/camera-failure', {method:'POST'})).json();
  assert(before.motion_commands === after.motion_commands, 'capture failure sent motion');
  return {passed:true, exportResult, saved, failedEpisodePreserved:true, boundary:'fake vendor, no robot'};
})();
