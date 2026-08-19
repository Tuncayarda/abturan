#!/usr/bin/env python3
"""
ROS 2 control-plane node for the Raspberry Pi camera bridge.

The actual video data plane (rpicam-vid | GStreamer -> H264/MPEG-TS over SRT in
LISTENER mode) runs as a separate process on the *host* (see
scripts/rpi_cam_streamer.py), because the Pi 5 CSI camera stack
(libcamera/rpicam-apps) is tied to the host OS and cannot run inside the jammy
ROS container. No destination IP is configured — a single client connects to
srt://<pi_ip>:<port> and pulls the stream, exactly like the Jetson setup.

This node is the "ros_bridge" side: it owns the camera parameters (resolution,
fps, bitrate, listen port, enabled) and reports live throughput. It talks to the
host streamer through a shared directory of small JSON files (params.json
written here, stats.json read here) — no video passes through ROS.
"""
import json
import os
import time

import rclpy
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

PARAMS_FILE = 'params.json'
STATS_FILE = 'stats.json'
STATS_STALE_SEC = 5.0


def _desc(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


def _desc_int(text: str, lo: int, hi: int) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=text,
        integer_range=[IntegerRange(from_value=lo, to_value=hi, step=0)],
    )


def _desc_float(text: str, lo: float, hi: float) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=text,
        floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)],
    )


class RpiCamNode(Node):

    def __init__(self):
        super().__init__('rpi_cam_node')

        # Where the shared control files live (bind-mounted into the container,
        # and the same path the host streamer watches).
        self.ctrl_dir = self.declare_parameter(
            'ctrl_dir', '/cam_ctrl',
            _desc('Directory shared with the host streamer for params.json/stats.json'),
        ).value

        # -- SRT listener (the client connects here; no IP needed) ---------
        self.port = self.declare_parameter(
            'port', 9003,
            _desc_int('SRT listener port the client connects to (srt://<pi_ip>:port)', 1024, 65535),
        ).value
        # SRT tamponu dogrudan uctan uca gecikmeye yaziliyor: 200 ms olcumde
        # arayuzde 279 ms, 30 ms'de 95 ms. Kablolu tether'da RTT < 1 ms, paket
        # kaybi pratikte sifir; 30 ms fazlasiyla guvenli. SRT iki ucun
        # BUYUGUNU kullanir — istemci tarafi da ayni seviyede olmali.
        self.latency_ms = self.declare_parameter(
            'latency_ms', 30,
            _desc_int('SRT latency buffer (ms)', 20, 8000),
        ).value

        # -- Camera capture parameters (mirrors publish.sh defaults) -------
        self.camera = self.declare_parameter(
            'camera', 0,
            _desc_int('Camera index (rpicam-vid --camera)', 0, 8),
        ).value
        self.width = self.declare_parameter(
            'width', 640,
            _desc_int('Capture width (pixels)', 320, 4096),
        ).value
        self.height = self.declare_parameter(
            'height', 480,
            _desc_int('Capture height (pixels)', 240, 4096),
        ).value
        self.fps = self.declare_parameter(
            'fps', 30,
            _desc_int('Capture frame rate (fps)', 1, 60),
        ).value
        # All-intra kodlamada (intra_period=1) her kare tam kare olduğu için
        # 1 Mbit/s görüntüyü çok bozuyor; 4 Mbit/s makul. Kısa CAT hattında
        # bant genişliği sınırlayıcı değil.
        self.bitrate = self.declare_parameter(
            'bitrate', 4_000_000,
            _desc_int('H264 encoder bitrate (bits/sec)', 100_000, 20_000_000),
        ).value
        # I-frame periyodu. 1 = her kare I: kareler arası bağımlılık kalmaz,
        # yeniden sıralama gecikmesi sıfırlanır. Gecikmeyi umursamayıp bant
        # genişliği kazanmak isteyen 30 gibi bir değere çekebilir.
        self.intra_period = self.declare_parameter(
            'intra_period', 1,
            _desc_int('I-frame period (1 = all-intra, lowest latency)', 1, 300),
        ).value
        self.enabled = self.declare_parameter(
            'enabled', True,
            _desc('Whether the host streamer should be running'),
        ).value

        self._epoch = 0
        os.makedirs(self.ctrl_dir, exist_ok=True)
        self._write_params()

        self.add_on_set_parameters_callback(self._on_params)

        self._throughput_pub = self.create_publisher(Float64, '~/throughput_kbps', 10)
        self._streaming_pub = self.create_publisher(Bool, '~/streaming', 10)
        self._stats_timer = self.create_timer(1.0, self._publish_stats)

        self.get_logger().info(
            f'Started. SRT listener on :{self.port}  '
            f'{self.width}x{self.height}@{self.fps}fps  bitrate={self.bitrate}  '
            f'ctrl_dir={self.ctrl_dir}'
        )

    # ------------------------------------------------------------------
    # Parameter handling -> shared params.json (consumed by host streamer)
    # ------------------------------------------------------------------
    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult

        new_ctrl_dir = next(
            (p.value for p in params if p.name == 'ctrl_dir'),
            self.ctrl_dir,
        )
        try:
            os.makedirs(new_ctrl_dir, exist_ok=True)
        except OSError as e:
            return SetParametersResult(
                successful=False,
                reason=f'Control directory cannot be created: {e}',
            )

        for p in params:
            name = p.name
            if name == 'port':
                self.port = p.value
            elif name == 'latency_ms':
                self.latency_ms = p.value
            elif name == 'camera':
                self.camera = p.value
            elif name == 'width':
                self.width = p.value
            elif name == 'height':
                self.height = p.value
            elif name == 'fps':
                self.fps = p.value
            elif name == 'bitrate':
                self.bitrate = p.value
            elif name == 'intra_period':
                self.intra_period = p.value
            elif name == 'enabled':
                self.enabled = p.value
            elif name == 'ctrl_dir':
                self.ctrl_dir = p.value

        self._write_params()
        return SetParametersResult(successful=True, reason='ok')

    def _write_params(self):
        self._epoch += 1
        payload = {
            'epoch': self._epoch,
            'enabled': bool(self.enabled),
            'port': int(self.port),
            'latency_ms': int(self.latency_ms),
            'camera': int(self.camera),
            'width': int(self.width),
            'height': int(self.height),
            'fps': int(self.fps),
            'bitrate': int(self.bitrate),
            'intra_period': int(self.intra_period),
        }
        path = os.path.join(self.ctrl_dir, PARAMS_FILE)
        tmp_path = path + '.tmp'
        try:
            with open(tmp_path, 'w') as f:
                json.dump(payload, f)
            os.replace(tmp_path, path)  # atomic on the same filesystem
            self.get_logger().info(f'[params] epoch {self._epoch} -> {payload}')
        except OSError as e:
            self.get_logger().error(f'Failed to write {path}: {e}')

    # ------------------------------------------------------------------
    # Throughput / status reporting <- shared stats.json (from host streamer)
    # ------------------------------------------------------------------
    def _publish_stats(self):
        path = os.path.join(self.ctrl_dir, STATS_FILE)
        throughput = 0.0
        streaming = False

        try:
            with open(path) as f:
                stats = json.load(f)
            age = time.time() - float(stats.get('timestamp', 0.0))
            if age <= STATS_STALE_SEC:
                throughput = float(stats.get('throughput_kbps', 0.0))
                streaming = bool(stats.get('streaming', False))
        except (OSError, ValueError, json.JSONDecodeError):
            pass  # no stats yet / streamer not running -> report idle

        self._throughput_pub.publish(Float64(data=throughput))
        self._streaming_pub.publish(Bool(data=streaming))


def main():
    rclpy.init()
    node = RpiCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
