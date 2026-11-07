"""Self-contained HTML/SVG report; Python standard library only."""
from __future__ import annotations
import html
import math


def _numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def render_report(doc):
    esc = lambda x: html.escape(str(x), quote=True)
    rows = []
    for name, q in doc["quality"].items():
        rows.append("<tr>" + "".join(f"<td>{esc(x)}</td>" for x in (
            name, q["sample_count"], q["trigger_state"],
            q["first_receive_offset_s"], q["last_receive_offset_s"],
            q["max_observed_gap_s"])) + "</tr>")
    series = {"IMU rpy[0] (raw)": [], "IMU rpy[1] (raw)": [],
              "IMU rpy[2] (raw)": [], "Battery SOC (raw)": [],
              "Maximum reported joint temperature (raw)": []}
    events = []
    for sample in doc["samples"]:
        t, p = sample["receive_offset_s"], sample["payload"]
        if sample["source"] == "imu" and isinstance(p.get("rpy"), list):
            for i, value in enumerate(p["rpy"][:3]):
                if _numeric(value):
                    series[f"IMU rpy[{i}] (raw)"].append((t, value))
        if sample["source"] == "battery" and _numeric(p.get("soc")):
            series["Battery SOC (raw)"].append((t, p["soc"]))
        if sample["source"] == "joints" and isinstance(p.get("joints"), list):
            temperatures = [v for joint in p["joints"] if isinstance(joint, dict)
                            for v in (joint.get("temp") if isinstance(joint.get("temp"), list) else [])
                            if _numeric(v)]
            if temperatures:
                series["Maximum reported joint temperature (raw)"].append((t, max(temperatures)))
        if sample["source"] == "motion_events":
            events.append(f"<li>{t:+.3f}s: {esc(str(p)[:1000])}</li>")
    charts = []
    for title, points in series.items():
        if not points:
            charts.append(f"<h3>{esc(title)}</h3><p>No usable numeric samples.</p>")
            continue
        # Split before display sampling so a receive outage is never bridged.
        count = len(points)
        source = 'imu' if title.startswith('IMU') else 'battery' if title.startswith('Battery') else 'joints'
        threshold = doc['sources'][source].get('stale_s', 1)
        segments, segment, previous = [], 0, None
        for t, value in points:
            if previous is not None and t - previous > threshold:
                segment += 1
            segments.append((t, value, segment))
            previous = t
        points = segments
        # Display at most 600 points. JSON retains every captured sample.
        if count > 600:
            points = [points[round(i * (count - 1) / 599)] for i in range(600)]
        lo, hi = min(v for _, v, _ in points), max(v for _, v, _ in points)
        span = hi - lo or 1
        x0, x1 = -doc["requested_pre_s"], doc["requested_post_s"]
        duration = x1 - x0 or 1
        commands, circles, last_segment = [], [], None
        for t, value, segment in points:
            x, y = 40 + (t-x0)/duration*840, 170-(value-lo)/span*130
            commands.append(f"{'M' if segment != last_segment else 'L'}{x:.2f},{y:.2f}")
            circles.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="1.8" fill="#1769aa"/>')
            last_segment = segment
        trigger_x = 40 + (-x0) / duration * 840
        charts.append(f'''<h3>{esc(title)}</h3>
<p>显示 {len(points)} / {count} 个点；抽样展示可能遗漏峰值，原始数据保存在 JSON。超过 {threshold:g}s 的接收间隔处断线。</p>
<svg viewBox="0 0 920 210" role="img" aria-label="{esc(title)}">
<path d="M40 25V175H890" fill="none" stroke="#aaa"/>
<line x1="{trigger_x:.2f}" y1="25" x2="{trigger_x:.2f}" y2="175" stroke="#d24" stroke-dasharray="4"/>
<path d="{' '.join(commands)}" fill="none" stroke="#1769aa" stroke-width="2"/>{''.join(circles)}
<text x="40" y="200">{x0:g}s</text><text x="830" y="200">{x1:g}s</text>
<text x="45" y="20">range {lo:.5g} to {hi:.5g}; trigger = red line</text></svg>''')
    return f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 telemetry — {esc(doc['label'])}</title>
<style>body{{font:16px system-ui;max-width:1100px;margin:32px auto;padding:0 20px;color:#223}}
table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:9px;border:1px solid #ccd;text-align:left}}
.note{{padding:14px;background:#fff5d7}}svg{{width:100%;background:#f8fafc}}li{{overflow-wrap:anywhere}}</style>
<h1>G1 运行数据与事件回溯</h1><p>{esc(doc['label'])}</p>
<p>ID: {esc(doc['clip_id'])} · 触发 UTC: {esc(doc['trigger_utc'])}</p>
<p class="note">触发后时间窗已结束：{esc(doc['post_window_elapsed'])}；中途停止：{esc(doc['interrupted'])}；
达到片段容量上限：{esc(doc['truncated'])}。这些标记不能证明数据完整或机器人健康。</p>
<p>{esc(doc['alignment'])}。使用接收端时间对齐；不推断硬件同步、单位换算或关节名称。</p>
<h2>数据覆盖情况</h2><p>偏移与间隔均使用接收时间。事件流没有固定心跳，无事件不代表系统正常。</p>
<table><tr><th>来源</th><th>样本数</th><th>触发时状态</th><th>最早偏移 (s)</th><th>最晚偏移 (s)</th><th>最大观测间隔 (s)</th></tr>{''.join(rows)}</table>
<h2>事件时间线</h2><ul>{''.join(events) or '<li>未录到事件。</li>'}</ul>
<h2>数据曲线</h2>{''.join(charts)}
<h2>适用边界</h2><ul>{''.join('<li>'+esc(x)+'</li>' for x in doc['limitations'])}</ul>
<p>完整采集记录位于旁边的 clip.json。本报告不加载外部资源。</p>
</html>'''
