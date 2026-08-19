#!/usr/bin/env python3
"""
Host-side camera data-plane streamer for the Raspberry Pi robot bringup.

Runs OUTSIDE the ROS container (the Pi 5 CSI camera stack — libcamera /
rpicam-apps — is tied to the host OS and cannot run inside the jammy ROS
container). It shares the camera the same way the Jetson did: a GStreamer
pipeline that LISTENS on a port via SRT (no destination IP needed). A single
client connects (as SRT caller) and pulls the H264/MPEG-TS video:

    rpicam-vid ... -o - | gst  fdsrc ! h264parse ! mpegtsmux ! \
        srtsink uri="srt://:PORT?mode=listener"

It is driven by parameters written by the ROS `rpi_cam_node` (running in
the container) through a small shared-file protocol in --ctrl-dir:

    params.json  (written by rpi_cam_node, read here)   -> desired stream config
    stats.json   (written here, read by rpi_cam_node)   -> live throughput/status

No video ever passes through ROS — only these tiny JSON control/status files.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

import gi

gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst  # noqa: E402

PARAMS_FILE = 'params.json'
STATS_FILE = 'stats.json'

PARAMS_POLL_SEC = 0.5
STATS_WRITE_SEC = 1.0

DEFAULT_PARAMS = {
    'epoch': 0,
    'enabled': True,
    'port': 9003,
    'latency_ms': 60,
    'camera': 0,
    'width': 640,
    'height': 480,
    'fps': 30,
    'bitrate': 1_000_000,
}


def _atomic_write_json(path: str, payload: dict) -> None:
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as f:
        json.dump(payload, f)
    os.replace(tmp_path, path)


class CamStreamer:

    def __init__(self, ctrl_dir: str):
        self.ctrl_dir = ctrl_dir
        self.params_path = os.path.join(ctrl_dir, PARAMS_FILE)
        self.stats_path = os.path.join(ctrl_dir, STATS_FILE)

        self._active_epoch = None
        self._active_params = None

        self._rpicam_proc = None
        self._pipeline = None

        self._byte_lock = threading.Lock()
        self._bytes_since_last = 0
        self._last_stats_time = time.monotonic()

        os.makedirs(ctrl_dir, exist_ok=True)
        Gst.init(None)

    # ------------------------------------------------------------------
    # Control-file polling
    # ------------------------------------------------------------------
    def _read_params(self):
        try:
            with open(self.params_path) as f:
                params = json.load(f)
        except (OSError, ValueError, json.JSONDecodeError):
            return None

        merged = dict(DEFAULT_PARAMS)
        merged.update(params)
        return merged

    def poll_params(self):
        params = self._read_params()
        if params is None:
            return True  # nothing written yet, keep polling

        epoch = params.get('epoch')
        config_changed = (
            self._active_params is not None
            and any(params[k] != self._active_params[k]
                    for k in ('enabled', 'port', 'latency_ms', 'camera',
                              'width', 'height', 'fps', 'bitrate'))
        )

        if epoch == self._active_epoch and not config_changed:
            return True  # nothing to do

        print(f'[streamer] params changed (epoch {self._active_epoch} -> {epoch}): {params}',
              flush=True)

        self._stop_pipeline()

        if params['enabled']:
            self._start_pipeline(params)

        self._active_epoch = epoch
        self._active_params = params
        return True

    # ------------------------------------------------------------------
    # Pipeline lifecycle
    # ------------------------------------------------------------------
    def _start_pipeline(self, params):
        cmd = [
            'rpicam-vid', '-t', '0',
            '--nopreview',
            '--camera', str(params['camera']),
            '--width', str(params['width']),
            '--height', str(params['height']),
            '--framerate', str(params['fps']),
            '--codec', 'h264', '--inline',
            '--libav-format', 'h264',
            '--bitrate', str(params['bitrate']),
            '-o', '-',
        ]
        print(f'[streamer] starting: {" ".join(cmd)}', flush=True)

        try:
            self._rpicam_proc = subprocess.Popen(
                # stderr'i host terminaline birak: kamera bulunamamasi,
                # dmaHeap/izin ve libcamera hatalari EOS olarak gizlenmesin.
                cmd, stdout=subprocess.PIPE,
            )
        except OSError as e:
            print(f'[streamer] failed to spawn rpicam-vid: {e}', flush=True)
            self._rpicam_proc = None
            return

        fd = self._rpicam_proc.stdout.fileno()
        # SRT listener: we listen on :PORT, the single client connects as caller
        # (srt://<pi_ip>:PORT). No destination IP is configured here — same model
        # as the Jetson srtsink listener.
        srt_uri = f'srt://:{params["port"]}?mode=listener&latency={params["latency_ms"]}'
        # do-timestamp=true: rpicam-vid sends a raw H264 elementary stream over the
        # pipe with no timestamps; mpegtsmux needs PTS/DTS to emit TS packets, so we
        # let fdsrc stamp buffers with arrival time.
        # The `identity name=meter` element gives us a normal pad to probe for
        # egress byte counting (srtsink's own sink pad does not fire buffer probes).
        gst_desc = (
            f'fdsrc fd={fd} do-timestamp=true ! h264parse ! '
            f'mpegtsmux alignment=7 ! '
            f'identity name=meter silent=false ! '
            f'srtsink name=sink uri="{srt_uri}" wait-for-connection=false sync=false'
        )
        print(f'[streamer] gst pipeline: {gst_desc}', flush=True)

        try:
            self._pipeline = Gst.parse_launch(gst_desc)
        except GLib.Error as e:
            print(f'[streamer] failed to build pipeline: {e}', flush=True)
            self._terminate_rpicam()
            self._pipeline = None
            return

        meter = self._pipeline.get_by_name('meter')
        meter_pad = meter.get_static_pad('src')
        meter_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_buffer)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::error', self._on_bus_error)
        bus.connect('message::eos', self._on_bus_eos)

        with self._byte_lock:
            self._bytes_since_last = 0
            self._last_stats_time = time.monotonic()

        self._pipeline.set_state(Gst.State.PLAYING)

    def _on_buffer(self, pad, info):
        buf = info.get_buffer()
        if buf is not None:
            with self._byte_lock:
                self._bytes_since_last += buf.get_size()
        return Gst.PadProbeReturn.OK

    def _on_bus_error(self, bus, message):
        err, debug = message.parse_error()
        print(f'[streamer] GStreamer error: {err} ({debug})', flush=True)
        self._stop_pipeline()
        self._active_epoch = None  # force restart on next matching poll

    def _on_bus_eos(self, bus, message):
        return_code = None
        if self._rpicam_proc is not None:
            return_code = self._rpicam_proc.poll()
        suffix = '' if return_code is None else f' (rpicam-vid exit={return_code})'
        print(f'[streamer] GStreamer EOS{suffix}', flush=True)
        self._stop_pipeline()
        self._active_epoch = None

    def _terminate_rpicam(self):
        if self._rpicam_proc is None:
            return
        proc = self._rpicam_proc
        self._rpicam_proc = None
        try:
            proc.terminate()
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        except ProcessLookupError:
            pass
        finally:
            if proc.stdout:
                proc.stdout.close()

    def _stop_pipeline(self):
        if self._pipeline is not None:
            print('[streamer] stopping pipeline', flush=True)
            bus = self._pipeline.get_bus()
            bus.remove_signal_watch()
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

        self._terminate_rpicam()

        with self._byte_lock:
            self._bytes_since_last = 0

    # ------------------------------------------------------------------
    # Stats reporting
    # ------------------------------------------------------------------
    def write_stats(self):
        now = time.monotonic()
        with self._byte_lock:
            byte_count = self._bytes_since_last
            self._bytes_since_last = 0
        elapsed = max(now - self._last_stats_time, 1e-6)
        self._last_stats_time = now

        streaming = self._pipeline is not None
        throughput_kbps = (byte_count * 8.0 / 1000.0) / elapsed if streaming else 0.0

        payload = {
            'epoch': self._active_epoch,
            'streaming': streaming,
            'throughput_kbps': round(throughput_kbps, 2),
            'timestamp': time.time(),
        }
        if self._active_params:
            payload.update({
                'port': self._active_params['port'],
                'width': self._active_params['width'],
                'height': self._active_params['height'],
                'fps': self._active_params['fps'],
                'bitrate': self._active_params['bitrate'],
            })

        try:
            _atomic_write_json(self.stats_path, payload)
        except OSError as e:
            print(f'[streamer] failed to write stats: {e}', flush=True)

        return True

    def shutdown(self):
        self._stop_pipeline()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ctrl-dir', default='/home/pi/ros2_ws/cam_ctrl',
                    help='Directory shared with the rpi_cam_node container (bind mount target)')
    args = ap.parse_args()

    streamer = CamStreamer(args.ctrl_dir)
    loop = GLib.MainLoop()

    GLib.timeout_add(int(PARAMS_POLL_SEC * 1000), streamer.poll_params)
    GLib.timeout_add(int(STATS_WRITE_SEC * 1000), streamer.write_stats)

    def _stop(*_):
        print('[streamer] shutting down...', flush=True)
        streamer.shutdown()
        loop.quit()
        return False

    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, _stop)
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, _stop)

    print(f'[streamer] watching {streamer.params_path}', flush=True)
    try:
        loop.run()
    finally:
        streamer.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
