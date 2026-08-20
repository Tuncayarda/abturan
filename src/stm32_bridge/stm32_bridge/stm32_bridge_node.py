#!/usr/bin/env python3
"""
ROS 2 <-> STM32 UART koprusu.

Gorev paylasimi bu robotta soyle:

    Raspberry Pi 5  : kamera (SRT), IMU (I2C), ROS/rosbridge, kinematik
    STM32           : 6 ESC PWM, step motor faz surusu, LED

Pi'nin GPIO'suna hicbir eyleyici bagli DEGIL. Tum eyleyiciler STM32'de,
cunku Linux userspace'te 50 Hz ESC darbesini ve mikro saniyelik step
zamanlamasini garanti edemiyoruz (bkz. step_control.py icindeki jitter
notlari) — STM32 tarafinda ayni is donanim timer'ina birakiliyor.

Hat:  Pi GPIO14 (TXD, pin 8)  -> STM32 PA3 (USART2_RX)
      Pi GPIO15 (RXD, pin 10) <- STM32 PA2 (USART2_TX)
      GND (pin 6) ortak

Abone oldugu konular
--------------------
/control/pwm_cmds          std_msgs/Int32MultiArray  6 kanal, 1000-2000 us (1500 notr)
/control/led               std_msgs/Bool             LED (STM32 PB12)
/control/stepper/velocity  std_msgs/Float32          adim/s, isaretli
/control/stepper/position  std_msgs/Int32            hedef adim (mutlak)
/control/stepper/enable    std_msgs/Bool             false -> bobinleri birak

Yayinladigi konular
-------------------
~/status             std_msgs/String   STM32 STATUS cercevesi, JSON
~/link_ok            std_msgs/Bool     son STATUS tazeyse true
~/stepper_position   std_msgs/Int32    STM32'nin bildirdigi adim sayaci
"""

import json
import threading
import time

import rclpy
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                               ParameterDescriptor, SetParametersResult)
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray, String

from stm32_bridge import protocol as proto

try:
    import serial
except ImportError:  # pragma: no cover - konteynerde python3-serial kurulu
    serial = None


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


class Stm32BridgeNode(Node):

    def __init__(self):
        super().__init__('stm32_bridge_node')

        # -- seri hat -----------------------------------------------------
        self.serial_port = self.declare_parameter(
            'serial_port', '/dev/ttyAMA0',
            _desc('STM32 UART aygiti. Pi 5te GPIO14/15 = /dev/ttyAMA0 '
                  '(config.txt icinde dtoverlay=uart0-pi5 gerekir). '
                  '/dev/serial0 Pi 5te hata ayiklama basligidir, onu KULLANMA.'),
        ).value
        self.baudrate = self.declare_parameter(
            'baudrate', 115200,
            _desc_int('UART hizi (STM32 tarafiyla ayni olmali)', 9600, 921600),
        ).value

        # -- gonderim hizi ve zaman asimlari ------------------------------
        self.send_rate_hz = self.declare_parameter(
            'send_rate_hz', 50.0,
            _desc_float('ESC cercevesi gonderim frekansi', 1.0, 200.0),
        ).value
        self.cmd_timeout = self.declare_parameter(
            'cmd_timeout', 0.5,
            _desc_float('Bu sure boyunca /control/pwm_cmds gelmezse ESCler notre '
                        'cekilir (Pi tarafi failsafe)', 0.05, 10.0),
        ).value
        self.status_timeout = self.declare_parameter(
            'status_timeout', 1.0,
            _desc_float('Bu sure boyunca STATUS gelmezse link_ok=false', 0.1, 10.0),
        ).value

        # -- ESC haritalama ----------------------------------------------
        self.esc_count = self.declare_parameter(
            'esc_count', proto.ESC_COUNT,
            _desc_int('ESC kanal sayisi (STM32 firmware ile ayni)', 1, 6),
        ).value
        self.pwm_neutral = self.declare_parameter(
            'pwm_neutral', 1500,
            _desc_int('Gelen /control/pwm_cmds icinde notr degeri', 0, 2000),
        ).value
        self.pwm_span = self.declare_parameter(
            'pwm_span', 500,
            _desc_int('Notrden tam guce kadar olan aralik (1000-2000 icin 500)',
                      1, 2000),
        ).value
        self.esc_min_us = self.declare_parameter(
            'esc_min_us', proto.ESC_MIN_US,
            _desc_int('Tam geri darbe genisligi (us)', 800, 1500),
        ).value
        self.esc_neutral_us = self.declare_parameter(
            'esc_neutral_us', proto.ESC_NEUTRAL_US,
            _desc_int('Notr darbe genisligi (us)', 1000, 2000),
        ).value
        self.esc_max_us = self.declare_parameter(
            'esc_max_us', proto.ESC_MAX_US,
            _desc_int('Tam ileri darbe genisligi (us)', 1500, 2200),
        ).value
        self.esc_reverse = list(self.declare_parameter(
            'esc_reverse', [False] * proto.ESC_COUNT,
            _desc('Kanal basina yon tersleme (pervane/kablo ters bagliysa)'),
        ).value)

        # -- step motor ---------------------------------------------------
        self.stepper_max_sps = self.declare_parameter(
            'stepper_max_sps', 800,
            _desc_int('Step motor hiz siniri (adim/s). 17HS3401 200 adim/tur, '
                      'yani 800 adim/s ~ 240 rpm', 1, 20000),
        ).value
        self.stepper_default_sps = self.declare_parameter(
            'stepper_default_sps', 400,
            _desc_int('Konum komutlarinda kullanilacak hiz (adim/s)', 1, 20000),
        ).value

        self.add_on_set_parameters_callback(self._on_params)

        # -- durum --------------------------------------------------------
        self._lock = threading.Lock()
        self._serial = None
        self._parser = proto.FrameParser()
        self._running = True

        self._esc_us = [self.esc_neutral_us] * proto.ESC_COUNT
        self._esc_stamp = 0.0
        self._led = False
        self._step_mode = proto.STEP_IDLE
        self._step_speed = 0
        self._step_target = 0
        self._heartbeat_seq = 0
        self._last_status = None
        self._status_stamp = 0.0
        self._tx_errors = 0
        self._pwm_len_warned = self.esc_count

        # -- ROS arayuzu --------------------------------------------------
        self.create_subscription(Int32MultiArray, '/control/pwm_cmds',
                                 self._on_pwm, 10)
        self.create_subscription(Bool, '/control/led', self._on_led, 10)
        self.create_subscription(Float32, '/control/stepper/velocity',
                                 self._on_step_velocity, 10)
        self.create_subscription(Int32, '/control/stepper/position',
                                 self._on_step_position, 10)
        self.create_subscription(Bool, '/control/stepper/enable',
                                 self._on_step_enable, 10)

        self._status_pub = self.create_publisher(String, '~/status', 10)
        self._link_pub = self.create_publisher(Bool, '~/link_ok', 10)
        self._steppos_pub = self.create_publisher(Int32, '~/stepper_position', 10)

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        self._tx_timer = self.create_timer(1.0 / self.send_rate_hz, self._on_tx_tick)
        self._status_timer = self.create_timer(0.2, self._publish_status)

        self.get_logger().info(
            f'STM32 koprusu: {self.serial_port} @ {self.baudrate} baud, '
            f'{self.esc_count} ESC, {self.send_rate_hz:.0f} Hz gonderim'
        )

    # ------------------------------------------------------------------
    # Seri hat: acma / okuma / yazma
    # ------------------------------------------------------------------
    def _open_serial(self):
        """Portu ac. Basarisizsa None don — reader dongusu tekrar dener."""
        if serial is None:
            self.get_logger().error(
                'pyserial yok. Konteynerde: apt-get install -y python3-serial')
            return None
        try:
            return serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.1,        # okuma blokunu kisa tut, kapanis hizli olsun
                write_timeout=0.1,  # STM32 askidaysa yazma dongude kilitlenmesin
            )
        except Exception as error:  # noqa: B902 - SerialException veya OSError
            self.get_logger().warn(f'{self.serial_port} acilamadi: {error}')
            return None

    def _reader_loop(self):
        """STM32'den gelen cerceveleri isle. Kopmada portu yeniden acar."""
        while self._running:
            with self._lock:
                port = self._serial
            if port is None:
                port = self._open_serial()
                if port is None:
                    time.sleep(1.0)
                    continue
                with self._lock:
                    self._serial = port
                self.get_logger().info(f'{self.serial_port} acildi.')

            try:
                # read(1) timeout ile donuyor; in_waiting ile kalani topluca al.
                chunk = port.read(1)
                if port.in_waiting:
                    chunk += port.read(port.in_waiting)
            except Exception as error:  # noqa: B902 - surucu cesitli hata atiyor
                if not self._running:
                    break
                self.get_logger().warn(f'UART okuma hatasi: {error}')
                self._close_serial()
                time.sleep(1.0)
                continue

            if not chunk:
                continue

            for msg_id, payload in self._parser.feed(chunk):
                self._handle_frame(msg_id, payload)

    def _close_serial(self):
        with self._lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except Exception:  # noqa: B902 - kapanista hata onemsiz
                pass

    def _send(self, frame):
        with self._lock:
            port = self._serial
        if port is None:
            return False
        try:
            port.write(frame)
            return True
        except Exception as error:  # noqa: B902
            self._tx_errors += 1
            self.get_logger().warn(f'UART yazma hatasi: {error}')
            self._close_serial()
            return False

    def _handle_frame(self, msg_id, payload):
        if msg_id == proto.MSG_STATUS:
            try:
                self._last_status = proto.decode_status(payload)
            except ValueError as error:
                self.get_logger().warn(f'bozuk STATUS: {error}')
                return
            self._status_stamp = time.monotonic()
            self._steppos_pub.publish(
                Int32(data=int(self._last_status['stepper_position'])))
        elif msg_id == proto.MSG_LOG:
            text = payload.decode('ascii', errors='replace').strip()
            self.get_logger().info(f'[stm32] {text}')
        else:
            self.get_logger().warn(f'bilinmeyen mesaj id 0x{msg_id:02X}')

    # ------------------------------------------------------------------
    # Abonelikler
    # ------------------------------------------------------------------
    def _pwm_to_us(self, value, channel):
        """1000-2000 us (1500 notr) gelir, ayni araliga esler. Farkli bir
        olcek kullanilirsa pwm_neutral/pwm_span ile ayarlanir."""
        norm = (float(value) - self.pwm_neutral) / float(self.pwm_span)
        norm = proto.clamp(norm, -1.0, 1.0)
        if channel < len(self.esc_reverse) and self.esc_reverse[channel]:
            norm = -norm
        if norm >= 0.0:
            return self.esc_neutral_us + norm * (self.esc_max_us - self.esc_neutral_us)
        return self.esc_neutral_us + norm * (self.esc_neutral_us - self.esc_min_us)

    def _on_pwm(self, msg: Int32MultiArray):
        values = list(msg.data)
        if len(values) < self.esc_count:
            # 50 Hz'de her mesajda uyarmak logu bogar; sadece uzunluk
            # degistiginde bir kez yaz.
            if len(values) != self._pwm_len_warned:
                self._pwm_len_warned = len(values)
                self.get_logger().warn(
                    f'/control/pwm_cmds {len(values)} deger tasiyor, '
                    f'{self.esc_count} bekleniyor — eksikler notr sayildi.')
        for i in range(proto.ESC_COUNT):
            if i < self.esc_count and i < len(values):
                self._esc_us[i] = self._pwm_to_us(values[i], i)
            else:
                self._esc_us[i] = self.esc_neutral_us
        self._esc_stamp = time.monotonic()

    def _on_led(self, msg: Bool):
        self._led = bool(msg.data)
        self._send(proto.encode_led(self._led))

    def _on_step_velocity(self, msg: Float32):
        speed = proto.clamp(float(msg.data), -self.stepper_max_sps,
                            self.stepper_max_sps)
        self._step_speed = int(round(speed))
        # Hiz sifirsa modu HOLD'a cekiyoruz: bobinler enerjili kalip
        # pozisyonu tutar. Tamamen birakmak icin stepper/enable=false.
        self._step_mode = proto.STEP_VELOCITY if self._step_speed else proto.STEP_HOLD
        self._send_stepper()

    def _on_step_position(self, msg: Int32):
        self._step_target = int(msg.data)
        self._step_speed = int(self.stepper_default_sps)
        self._step_mode = proto.STEP_POSITION
        self._send_stepper()

    def _on_step_enable(self, msg: Bool):
        if msg.data:
            if self._step_mode == proto.STEP_IDLE:
                self._step_mode = proto.STEP_HOLD
        else:
            self._step_mode = proto.STEP_IDLE
            self._step_speed = 0
        self._send_stepper()

    def _send_stepper(self):
        self._send(proto.encode_stepper(self._step_mode, self._step_speed,
                                        self._step_target))

    # ------------------------------------------------------------------
    # Periyodik gonderim
    # ------------------------------------------------------------------
    def _on_tx_tick(self):
        now = time.monotonic()
        stale = (now - self._esc_stamp) > self.cmd_timeout
        if stale:
            # Joystick/arayuz sustu. STM32'nin kendi failsafe'i de var ama
            # kablo saglamken notru burada uretmek daha hizli.
            pulses = [self.esc_neutral_us] * proto.ESC_COUNT
        else:
            pulses = self._esc_us

        self._send(proto.encode_esc(pulses))

        # Saniyede ~5 heartbeat yeter; STM32 failsafe'i bunu da sayiyor.
        self._heartbeat_seq += 1
        if self._heartbeat_seq % max(1, int(self.send_rate_hz // 5)) == 0:
            self._send(proto.encode_heartbeat(self._heartbeat_seq))

    def _publish_status(self):
        fresh = (self._last_status is not None
                 and (time.monotonic() - self._status_stamp) <= self.status_timeout)
        self._link_pub.publish(Bool(data=fresh))

        payload = {
            'link_ok': fresh,
            'serial_port': self.serial_port,
            'serial_open': self._serial is not None,
            'tx_errors': self._tx_errors,
            'rx_frames': self._parser.ok_count,
            'rx_frame_errors': self._parser.error_count,
            'esc_us': [int(round(v)) for v in self._esc_us[:self.esc_count]],
            'stepper_mode': proto.STEP_MODE_NAMES.get(self._step_mode, '?'),
            'stepper_speed_sps': self._step_speed,
            'stepper_target': self._step_target,
            'led': self._led,
        }
        if fresh:
            payload['stm32'] = self._last_status
        self._status_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    def _on_params(self, params):
        reopen = False
        for p in params:
            if p.name == 'serial_port':
                self.serial_port = p.value
                reopen = True
            elif p.name == 'baudrate':
                self.baudrate = p.value
                reopen = True
            elif hasattr(self, p.name):
                setattr(self, p.name, p.value)
        if reopen:
            self._close_serial()  # reader dongusu yeni ayarla tekrar acar
        return SetParametersResult(successful=True, reason='ok')

    def shutdown(self):
        """Cikista ESCleri notre cek, step motoru birak, LEDi kapat."""
        self._running = False
        self._send(proto.encode_esc([self.esc_neutral_us] * proto.ESC_COUNT))
        self._send(proto.encode_stepper(proto.STEP_IDLE, 0, 0))
        self._send(proto.encode_led(False))
        time.sleep(0.05)  # cerceveler hatta cikana kadar bekle
        self._close_serial()


def main():
    rclpy.init()
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
