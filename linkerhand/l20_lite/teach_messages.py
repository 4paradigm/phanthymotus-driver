"""User-facing Chinese errors for hand_teach; keep native detail in driver logs."""
import errno
import re


LEGACY_REASONS = {
    "stopped by stop_trajectory": "用户已停止录制",
    "glove stream gap": "手套数据断流，超过录制允许的帧间隔",
    "recording length limit reached": "录制达到时长上限",
    "interrupted by stop_motion or driver stop": "录制被停止动作或驱动退出中止",
    "live teleoperation failed": "实时遥操失败",
    "no glove frames received": "没有收到手套数据",
    "driver restarted during recording": "录制期间驱动重启",
    "recording": "轨迹仍在录制",
    "incomplete": "轨迹保存不完整",
}
MODBUS_ERRORS = {
    "Modbus timeout": "等待灵巧手的 Modbus 响应超时，请检查电源、串口和 RS485 接线",
    "unexpected Modbus slave ID": "Modbus 返回了意外的设备地址",
    "Modbus exception code": "灵巧手返回 Modbus 异常",
    "unexpected Modbus function": "Modbus 返回了意外的功能码",
    "Modbus CRC mismatch": "Modbus 响应校验失败",
    "expected 20 register bytes": "灵巧手返回的关节寄存器长度不正确",
    "joint register contains value": "灵巧手返回的关节值超出 0–255",
    "Modbus write acknowledgement": "灵巧手写入确认的寄存器地址或数量不一致",
    "ambiguous hands on": "同一串口检测到多个手侧，无法判断要控制哪只手",
    "duplicate ": "检测到多只同侧灵巧手，无法选择设备",
}
SYSTEM_ERRORS = {
    errno.EACCES: "权限不足",
    errno.EPERM: "权限不足",
    errno.ENOENT: "文件或设备不存在",
    errno.ENOSPC: "存储空间不足",
    errno.EROFS: "存储目录为只读",
    errno.EADDRINUSE: "UDP 端口已被占用",
    errno.EBUSY: "设备或资源正在占用",
    errno.EIO: "设备读写失败",
    errno.ENODEV: "设备已断开",
}


def reason_text(reason):
    if not reason:
        return "原因未记录"
    text = str(reason)
    if text in LEGACY_REASONS:
        return LEGACY_REASONS[text]
    if re.search(r"[一-龥]", text):
        return text
    rule = re.search(r"violates (R[1-4])", text)
    if rule:
        return f"录制数据未通过当时的手型规则（{rule[1]}）"
    return "旧版记录未提供中文原因，请查看驱动日志或原始清单"


def error_text(exc, fallback):
    text = str(exc)
    for prefix, message in MODBUS_ERRORS.items():
        if text.startswith(prefix):
            code = re.search(r"0x[0-9a-fA-F]+", text)
            return f"{message}（{code[0]}）" if code else message
    if isinstance(exc, OSError) and exc.errno in SYSTEM_ERRORS:
        return SYSTEM_ERRORS[exc.errno]
    if isinstance(exc, OSError) and exc.filename:
        return f"无法访问文件或设备「{exc.filename}」"
    if re.search(r"[一-龥]", text):
        return text
    print(f"[hand_teach] {fallback}: {exc!r}", flush=True)
    return fallback
