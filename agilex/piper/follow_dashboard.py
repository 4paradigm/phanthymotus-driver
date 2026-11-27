"""Render a compact tracking dashboard for the Core's image/jpeg monitor."""

from datetime import datetime, timezone


WIDTH, HEIGHT = 720, 430
INK = (41, 37, 31)
MUTED = (115, 105, 92)
GREEN = (95, 128, 36)
AMBER = (37, 109, 179)
RED = (63, 59, 181)
STATES = {
    'following': ('Tracking', GREEN),
    'transient_deviation': ('Short deviation', AMBER),
    'sustained_deviation': ('Sustained deviation', RED),
    'reference_unavailable': ('Waiting for leader', AMBER),
    'feedback_unavailable': ('Waiting for follower', AMBER),
    'unaligned': ('Aligning samples', AMBER),
    'interface_unavailable': ('CAN unavailable', RED),
}


def metric(value, unit, digits=1):
    return '--' if value is None else f'{value:.{digits}f} {unit}'


def render_dashboard(pairs, tolerance_deg, persistence_ms, timestamp):
    import cv2
    import numpy as np

    image = np.full((HEIGHT, WIDTH, 3), (243, 246, 249), dtype=np.uint8)

    def text(value, x, y, scale=0.50, color=INK, weight=1):
        cv2.putText(image, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, weight, cv2.LINE_AA)

    text('ALOHA Leader-Follower Tracking', 18, 34, 0.85, weight=2)
    text(f'Tolerance {tolerance_deg:g} deg   |   Sustained after {persistence_ms:g} ms',
         18, 60, color=MUTED)
    text(datetime.fromtimestamp(timestamp, timezone.utc).strftime('%H:%M:%S UTC'),
         573, 60, color=MUTED)

    panel_width = (WIDTH - 24 - 12 * (len(pairs) - 1)) // len(pairs)
    for index, (name, pair) in enumerate(pairs.items()):
        left = 12 + index * (panel_width + 12)
        right = left + panel_width
        cv2.rectangle(image, (left, 78), (right, 418), (255, 255, 255), -1)
        x = left + 15
        label, color = STATES.get(pair['state'], ('Unavailable', MUTED))
        text(name.upper(), x, 106, 0.62, weight=2)
        text(pair['interface'], right - 85, 106, color=MUTED)
        cv2.circle(image, (x + 5, 128), 4, color, -1, cv2.LINE_AA)
        text(label, x + 16, 135, 0.61, color, 2)

        worst = pair['worst_joint']
        peak = metric(pair['max_error_deg'], 'deg', 2)
        text('Max error', x, 160, color=MUTED)
        text(peak + (f'  /  J{worst}' if worst is not None else ''), x, 186, 0.70, weight=2)
        leader_age = metric(pair['reference_age_ms'], 'ms')
        follower_age = metric(pair['feedback_age_ms'], 'ms')
        text(f'Leader {leader_age}   |   Follower {follower_age}', x, 213, 0.45, MUTED)
        text('Signed joint error  (deg)', x, 242, color=MUTED)

        joints = {joint['joint']: joint for joint in pair['joints']}
        bar_left, bar_right = x + 40, right - 83
        center = (bar_left + bar_right) // 2
        half_width = (bar_right - bar_left) // 2
        for number in range(1, 7):
            y = 255 + (number - 1) * 25
            text(f'J{number}', x, y + 11)
            cv2.rectangle(image, (bar_left, y), (bar_right, y + 12), (241, 237, 231), -1)
            cv2.line(image, (center, y - 2), (center, y + 14), (173, 161, 146), 1)
            joint = joints.get(number)
            if joint is None:
                text('--', bar_right + 10, y + 11, color=MUTED)
                continue
            error = joint['error_deg']
            scale = max(-1.0, min(1.0, error / (2 * tolerance_deg)))
            endpoint = center + round(half_width * scale)
            bar_color = RED if joint['sustained'] else AMBER if abs(error) > tolerance_deg else GREEN
            cv2.rectangle(image, (min(center, endpoint), y + 2), (max(center, endpoint), y + 10), bar_color, -1)
            text(f'{error:+.2f}', bar_right + 10, y + 11, color=bar_color)

    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if not ok:
        raise RuntimeError('Could not encode the tracking dashboard')
    return encoded.tobytes()
