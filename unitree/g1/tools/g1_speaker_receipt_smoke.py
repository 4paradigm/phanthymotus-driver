#!/usr/bin/env python3
"""Manual G1 hardware smoke for correlated Speaker playback receipts.

Run this only in a dedicated test deployment. It replaces the Speaker input
topic, plays the normal startup beep, streams near-silent PCM through the real
body-speaker service, validates the hardware-state receipt, and stops Speaker.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
import uuid

import rclpy
from audio_msgs.msg import AudioChunk
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


AUDIO_EOF_MAGIC = b'\x01\x00\xff\xff\x01\x00\xff\xff'
PCM_BLOCK = b'\x01\x00\xff\xff' * 2400  # 9600 bytes, samples +1/-1
TERMINAL_STATES = {'completed', 'cancelled', 'error'}

INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)
RECEIPT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _speaker_call(url: str, action: str, *, timeout: float, **arguments) -> dict:
    request_body = {
        'jsonrpc': '2.0',
        'id': str(uuid.uuid4()),
        'method': 'tools/call',
        'params': {
            'name': 'speaker',
            'arguments': {'action': action, **arguments},
        },
    }
    request = urllib.request.Request(
        url,
        data=json.dumps(request_body).encode(),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        rpc = json.loads(response.read().decode())
    if 'error' in rpc:
        raise RuntimeError(f'MCP speaker {action} failed: {rpc["error"]}')
    content = rpc.get('result', {}).get('content', [])
    if not content or content[0].get('type') != 'text':
        raise RuntimeError(f'MCP speaker {action} returned invalid content: {rpc}')
    result = json.loads(content[0].get('text', '{}'))
    if not isinstance(result, dict):
        raise RuntimeError(f'MCP speaker {action} returned non-object: {result!r}')
    return result


class _SmokeNode(Node):
    def __init__(self, input_topic: str):
        super().__init__('g1_speaker_receipt_smoke')
        self.input_topic = input_topic.rstrip('/')
        self.receipt_topic = f'{self.input_topic}/speaker_receipts'
        self.receipts: list[dict] = []
        self.publisher = self.create_publisher(
            AudioChunk, self.input_topic, INPUT_QOS,
        )
        self.subscription = self.create_subscription(
            String, self.receipt_topic, self._on_receipt, RECEIPT_QOS,
        )

    def _on_receipt(self, msg: String) -> None:
        try:
            receipt = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(receipt, dict):
            self.receipts.append(receipt)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )

    def wait_for_graph(self, timeout: float) -> dict | None:
        """Wait until both smoke directions are matched before publishing PCM."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            counts = {
                'speaker_input_subscriptions': (
                    self.publisher.get_subscription_count()
                ),
                'receipt_publishers': self.subscription.get_publisher_count(),
            }
            if all(count > 0 for count in counts.values()):
                return counts
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        return None

    def publish(self, utterance_id: str, payload: bytes) -> None:
        msg = AudioChunk()
        msg.header.frame_id = utterance_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'audio/pcm-16k'
        msg.data = list(payload)
        self.publisher.publish(msg)

    def wait_for_terminal(self, session_id: str, utterance_id: str,
                          timeout: float) -> dict | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for receipt in self.receipts:
                if (receipt.get('session_id') == session_id
                        and receipt.get('utterance_id') == utterance_id
                        and receipt.get('state') in TERMINAL_STATES):
                    return receipt
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        return None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--confirm-exclusive-hardware', action='store_true',
        help='confirm this is a dedicated test deployment and Speaker may be stopped',
    )
    parser.add_argument(
        '--mcp-url', default='http://127.0.0.1:15701/mcp',
    )
    parser.add_argument(
        '--input-topic', default='/g1/reviewer/speaker_receipt_smoke',
    )
    parser.add_argument(
        '--timeout', type=float, default=10.0,
        help='seconds to wait for the terminal receipt',
    )
    parser.add_argument(
        '--start-timeout', type=float, default=30.0,
        help='MCP start timeout; includes DDS discovery and the startup sound',
    )
    parser.add_argument(
        '--cleanup-timeout', type=float, default=30.0,
        help='MCP stop timeout used after every attempted start',
    )
    parser.add_argument(
        '--discovery-timeout', type=float, default=10.0,
        help='seconds to wait for ROS input and receipt graph matches',
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.confirm_exclusive_hardware:
        print(
            'Refusing to run without --confirm-exclusive-hardware; '
            'the smoke replaces and then stops the active Speaker session.',
            file=sys.stderr,
        )
        return 2

    utterance_id = f'utt:hardware-smoke-{uuid.uuid4()}'
    node = None
    speaker_start_attempted = False
    evidence = None
    primary_error = None
    cleanup_error = None
    cleanup_result = None
    rclpy.init(args=None)
    try:
        node = _SmokeNode(args.input_topic)
        node.spin_for(0.5)
        # Mark the attempt before the RPC. A timed-out client request can still
        # leave a server-side start handler running, so cleanup is mandatory
        # even when start never returned a response.
        speaker_start_attempted = True
        start = _speaker_call(
            args.mcp_url, 'start', timeout=args.start_timeout,
            input_topic=node.input_topic,
        )
        if start.get('state') != 'ready':
            raise RuntimeError(f'speaker did not become ready: {start}')
        if start.get('completion_mode') != 'hardware_state':
            raise RuntimeError(f'speaker is not in hardware_state mode: {start}')
        play_state = start.get('play_state') or {}
        if not play_state.get('ready') or play_state.get('matched_publishers', 0) < 1:
            raise RuntimeError(f'play-state publisher is not ready: {play_state}')
        session_id = start.get('session_id', '')
        if not session_id:
            raise RuntimeError(f'speaker returned no session_id: {start}')
        graph = node.wait_for_graph(args.discovery_timeout)
        if graph is None:
            raise RuntimeError(
                'ROS graph did not match both Speaker input and receipt paths '
                f'within {args.discovery_timeout}s'
            )
        # Stream 1.5 seconds of effectively inaudible PCM. Non-zero samples
        # avoid firmware silence suppression.
        for _ in range(5):
            node.publish(utterance_id, PCM_BLOCK)
            node.spin_for(0.3)
        node.publish(utterance_id, AUDIO_EOF_MAGIC)

        terminal = node.wait_for_terminal(
            session_id, utterance_id, args.timeout,
        )
        if terminal is None:
            info = _speaker_call(args.mcp_url, 'info', timeout=args.timeout)
            raise RuntimeError(
                f'no terminal receipt within {args.timeout}s; speaker info={info}'
            )
        if terminal.get('state') != 'completed':
            raise RuntimeError(f'non-success terminal receipt: {terminal}')
        if terminal.get('completion_basis') != 'g1_play_state_observed':
            raise RuntimeError(f'non-hardware completion basis: {terminal}')
        if terminal.get('audio_bytes') != 5 * len(PCM_BLOCK):
            raise RuntimeError(f'unexpected audio byte count: {terminal}')

        evidence = {
            'result': 'passed',
            'session_id': session_id,
            'utterance_id': utterance_id,
            'play_state': play_state,
            'ros_graph': graph,
            'terminal_receipt': terminal,
        }
    except Exception as exc:
        primary_error = f'{type(exc).__name__}: {exc}'
    finally:
        if speaker_start_attempted:
            try:
                cleanup_result = _speaker_call(
                    args.mcp_url, 'stop', timeout=args.cleanup_timeout,
                )
                if cleanup_result.get('state') != 'idle':
                    raise RuntimeError(
                        f'speaker cleanup did not become idle: {cleanup_result}'
                    )
            except Exception as exc:
                cleanup_error = f'{type(exc).__name__}: {exc}'
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    if primary_error is not None or cleanup_error is not None:
        failure = {
            'result': 'failed',
            'utterance_id': utterance_id,
        }
        if primary_error is not None:
            failure['error'] = primary_error
        if cleanup_error is not None:
            failure['cleanup_error'] = cleanup_error
        print(json.dumps(failure, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1

    evidence['cleanup'] = cleanup_result
    print(json.dumps(evidence, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
