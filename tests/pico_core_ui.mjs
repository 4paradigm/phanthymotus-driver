// Browser uses ordinary Core modules unchanged; all services are loopback fixtures.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE);
const [base, evidencePath] = process.argv.slice(2);
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME_BINARY });
const page = await browser.newPage();
const errors = [];
page.on('pageerror', e => errors.push(e.message));
page.on('dialog', d => { errors.push(d.message()); d.dismiss(); });
try {
  await page.goto(base);
  const fixture = await (await page.request.get(base + '/fixture')).json();
  const added = await (await page.request.post(base + '/api/mcp', {data: {
    name: 'PICO', url: fixture.mcp_url, category: 'driver', transport: 'http'
  }})).json();
  const mcpId = added.data.id;
  let mcps;
  for (let i = 0; i < 50; i++) {
    mcps = await (await page.request.get(base + '/api/mcp')).json();
    if (JSON.stringify(mcps).includes('teleop_device')) break;
    await new Promise(r => setTimeout(r, 100));
  }
  assert.ok(JSON.stringify(mcps).includes('teleop_device'), 'Core discovered tools/list');
  await page.request.post(base + '/fixture/layout', { data: {mcp_id: mcpId} });
  await page.evaluate(async ({mcpId, tool}) => {
    const sidebar = await import('/js/sidebar.js');
    await sidebar.openInstanceConfigModal(mcpId, tool.name, 'vr-card-1', tool.configSchema, tool.description);
  }, {mcpId, tool: fixture.tool});
  assert.equal(await page.locator('[data-key=driver_installed]').inputValue(), 'false');
  assert.equal(await page.locator('[data-key=pairing_admin_password]').getAttribute('type'), 'password');
  assert.ok((await page.locator('#tool-config-body').textContent()).includes('PICO 设备安装步骤'));
  const url = page.locator('[data-key=installation_url]');
  const urlBehavior = {tag: await url.evaluate(e => e.tagName), readOnly: await url.evaluate(e => e.readOnly), anchors: await page.locator('#tool-config-body a').count()};
  // Record the known host gap; do not call this a clickable/readonly entry.
  assert.deepEqual(urlBehavior, {tag: 'INPUT', readOnly: false, anchors: 0});
  await page.locator('[data-key=driver_installed]').selectOption('true');
  await page.locator('[data-key=pairing_admin_password]').fill('isolated fixture password');
  await page.locator('[data-key=input_filter_ms]').fill('30');
  await page.screenshot({path: evidencePath.replace(/\.json$/, '-gear.png'), fullPage: true});
  await page.locator('#tool-config-save').click();
  await page.waitForFunction(() => document.getElementById('tool-config-overlay').classList.contains('hidden'));
  const saved = await (await page.request.get(base + `/api/canvas/tool-config/${mcpId}/teleop_device/vr-card-1`)).json();
  assert.equal(saved.data.driver_installed, true);
  assert.equal(saved.data.input_filter_ms, 30);
  const packed = await (await page.request.get(base + '/fixture/pack')).json();
  assert.equal(packed.canvas.toolConfigs['d0:teleop_device:vr-card-1'].pairing_admin_password, '');
  assert.ok(!JSON.stringify(packed).includes('isolated fixture password'));
  const started = await (await page.request.post(base + '/api/config/start-project')).json();
  assert.equal(started.ok, true, JSON.stringify(started));
  let driver;
  for (let i = 0; i < 50; i++) {
    driver = await (await page.request.get(base + '/fixture/driver-state')).json();
    if (driver.state === 'collecting') break;
    await new Promise(r => setTimeout(r, 100));
  }
  assert.equal(driver.state, 'collecting', JSON.stringify(driver));
  assert.equal(driver.topic_out[0].format, 'data/teleop-cmd');
  assert.equal(driver.topic_out[0].topic, '/pico/teleop/vr_card_1/command');
  const input = await (await page.request.get(base + '/fixture/input')).json();
  const displayed = await page.evaluate(async input => {
    const {ActivityRenderer} = await import('/js/renderers/activity.js');
    const r = Object.assign({}, ActivityRenderer);
    if (!r.canRender('data/teleop-cmd')) throw new Error('format unsupported');
    r.mount(document.getElementById('monitor'), 'fixture');
    r.onData(new TextEncoder().encode(JSON.stringify(input)), 'data/teleop-cmd');
    return document.getElementById('monitor').textContent;
  }, input);
  assert.ok(displayed.includes(input.text));
  const stopped = await (await page.request.post(base + '/api/config/stop-project')).json();
  assert.equal(stopped.ok, true);
  driver = await (await page.request.get(base + '/fixture/driver-state')).json();
  assert.equal(driver.state, 'idle');
  assert.deepEqual(errors, []);
  await page.screenshot({path: evidencePath.replace(/\.json$/, '.png'), fullPage: true});
  fs.writeFileSync(evidencePath, JSON.stringify({
    core_sha: fixture.core_sha, core_source_sha256: fixture.core_source_sha256,
    source_mode: 'unchanged git archive; real Core JS/API/SQLite and Driver MCP; DDS sink',
    checks: ['registration_discovery', 'gear_default_and_password', 'browser_save_and_typed_config',
      'solution_password_redaction', 'project_start_and_topic', 'activity_monitor_input_text', 'project_stop_routes_to_instance'],
    installation_url_gap: urlBehavior, frontend_errors: errors, result: 'PASS_WITH_KNOWN_URL_GAP'
  }, null, 2) + '\n');
  console.log('CORE INTEGRATION PASS; URL remains non-clickable/editable host input');
} finally { await browser.close(); }
