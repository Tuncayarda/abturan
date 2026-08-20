#!/usr/bin/env python3
"""
miniROV joystick -> ESC koprusu (rampali surus).

Motor gruplari — bu depoda her yerde ayni isimlendirme kullanilir:

    on   (front)  : 0, 1      indeks 0 = on sag,   1 = on sol
    orta (mid)    : 2, 3      indeks 2 = orta sol, 3 = orta sag
    arka (rear)   : 4, 5      indeks 4 = arka sag, 5 = arka sol

Bu node yatay takimi (on + arka = 0,1,4,5) cubuk + tetiklerden surer:

    sag cubuk yukari/asagi (ry) -> ileri/geri   (surge)
    sol cubuk sag/sol      (lx) -> saga/sola donme (yaw)
    R2 tetigi                   -> saga  git    (1 ve 5)
    L2 tetigi                   -> sola   git    (0 ve 4)
    D-pad sag / sol             -> orta ikiliyi (2,3) TERS yonlerde sur
    X  tusu                     -> her seyi sifirla (hepsi notre)

Not: kullanicinin okudugu tus numaralari tarayici Gamepad API standardina
gore (b4/b5 = LB/RB, b14/b15 = D-pad sol/sag). Arayuzun Joy dizisi SDL
sirasinda oldugu icin buradaki indeksler farkli: D-pad sol = 13, sag = 14.

Yana giderken calisan motorlar YONE gore degisiyor; tek bir cift yonlu
katsayiyla anlatilamadigi icin iki ayri karisim tutuluyor (mix_lat_right /
mix_lat_left). Iki tetige ayni anda basilirsa iki takim da calisir, yani
birbirini goturur — fiziksel olarak dogru davranis bu.

Tetikler ANALOG: ne kadar basarsan o kadar guc. Arayuzun Joy dizisinde
DualShock adlandirmasi kullaniliyor — l2 = eksen 4, r2 = eksen 5 ve ikisi de
0..1 araliginda (ham Linux joy surucusundeki gibi 1..-1 DEGIL).

ILERI/GERI'de dort motor da HER ZAMAN ayni darbeyi alir — esit guc.
DONME ise tanimi geregi farkli yon ister; sol cubugun yatay ekseninden
gelir ve mix_yaw ile dagitilir:

    saga donus (lx > 0) -> 0 ve 4 duz,  1 ve 5 ters
    sola donus (lx < 0) -> tam tersi

Orta ikili (2,3) cubuklardan surulmez; yalnizca LB/RB kaymasi ve D-pad
egmesiyle calisir.

Tusla kademeli guc — ORTA ikili (2,3)
-------------------------------------
Omuz tuslari ORTA motorlarin (2,3) darbesini adim adim kaydirir ve deger
BIRIKIR. Yatay takima (0,1,4,5) dokunmaz — orasi yalnizca cubukla surulur.

    LB (Joy 9)  -> +button_step_us     RB (Joy 10) -> -button_step_us

Bir BASIS = bir tam adim (button_step_us). Tus basili tutulursa
button_hold_delay sonrasinda button_hold_interval'da bir yalnizca
button_hold_step_us kadar (eser miktarda) suruklenir; boylece tutarak ince
ayar, basip birakarak kaba ayar yapilir.

Adim buyuklugu calisirken degistirilebilir: arayuzdeki alan
/ui/minirov/pwm_step (std_msgs/Int32) konusuna yazar, `ros2 param set` de
calisir.

Guvenlik geregi kayma; Joy kesildiginde ve deadman birakildiginda sifirlanir
— aksi halde hat koptuktan sonra arac itmeye devam eder.

Neden ayri bir node
-------------------
joy_to_wrench + thruster_allocator zinciri kuvvet/pinv tabanli calisiyor ve
cikisi yalnizca Joy mesaji geldiginde uretiyor. Burada istenen sey daha
dogrudan: tusa basildigi anda motor tam guce sicramasin, komuta dogru
RAMPALANARAK yaklassin. Bu yuzden bu node sabit frekansta (publish_rate_hz)
kendi dongusunu doner ve her cevrimde mevcut darbeyi hedefe dogru en fazla
ramp_up_us_per_s / ramp_down_us_per_s kadar tasir.

Cikis birimi
------------
/control/pwm_cmds dogrudan ESC darbe genisligi tasir: 1000..2000 us,
1500 = notr. stm32_bridge ayni araligi (pwm_neutral=1500, pwm_span=500)
bekler, thruster_allocator da ayni araliga ayarlidir.

Guvenlik
--------
* joy_timeout boyunca Joy gelmezse hedef notre cekilir ve motorlar
  ramp_down hiziyla notre iner (aniden kesilmez, ama gecikmez de).
* Notre inildikten sonra node yayini birakir; boylece ayni konuya yazan
  thruster_allocator ile carpismaz. Yeni Joy gelince yayin geri baslar.
* deadman_button >= 0 ise, o tusa basili degilken hedef notrdur.
"""

import time
from typing import List

import rclpy
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                                ParameterDescriptor)
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Int32MultiArray

MOTOR_COUNT = 6

# Grup isimlendirmesi tek yerde dursun: log ve parametre varsayilanlari
# buradan turuyor. Yerlesim:
#
#        on sag  = 0      on sol  = 1
#        orta sol= 2      orta sag= 3
#        arka sag= 4      arka sol= 5
#
# Taraf eslesmesi: 0-4 ayni tarafta, 1-5 ayni tarafta. Karisim vektorleri
# yalnizca bu indekslere bakar; sol/sag etiketleri tarif icin, davranisi
# degistirmiyor.
FRONT_MOTORS = (0, 1)
MID_MOTORS = (2, 3)
REAR_MOTORS = (4, 5)


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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MiniRovJoyNode(Node):

    def __init__(self):
        super().__init__('minirov_joy_node')

        # -- konular ------------------------------------------------------
        self.declare_parameter(
            'joy_topic', '/ui/minirov/joy_cmd_vel',
            _desc('Arayuzun miniROV hedefiyle yayinladigi Joy konusu'))
        self.declare_parameter(
            'pwm_topic', '/control/pwm_cmds',
            _desc('ESC darbe cikisi (stm32_bridge bunu dinler)'))

        # -- eksen haritasi ------------------------------------------------
        # Arayuzun Joy mesajinda eksen sirasi: [lx, ly, rx, ry, l2, r2]
        self.declare_parameter(
            'axis_surge', 3,
            _desc_int('Ileri/geri ekseni (sag cubuk dikey = 3)', 0, 15))
        self.declare_parameter(
            'axis_yaw', 0,
            _desc_int('Donme ekseni (sol cubuk yatay = 0)', 0, 15))
        self.declare_parameter(
            'invert_surge', True,
            _desc('SDL ekseninde cubugu yukari itmek negatif deger uretir; '
                  'ileri = pozitif olsun diye varsayilan True'))
        self.declare_parameter(
            'invert_yaw', False,
            _desc('Donme yonu ters geliyorsa True'))
        # Arayuz dizisi [lx, ly, rx, ry, l2, r2] -> l2 = 4, r2 = 5, 0..1.
        self.declare_parameter(
            'axis_right', 5,
            _desc_int('Saga git tetigi (R2 = eksen 5)', 0, 15))
        self.declare_parameter(
            'axis_left', 4,
            _desc_int('Sola git tetigi (L2 = eksen 4)', 0, 15))
        self.declare_parameter(
            'trigger_deadzone', 0.05,
            _desc_float('Tetigin bu kadar altindaki basma 0 sayilir',
                        0.0, 0.9))
        # Orta ikiliyi ters yonlerde surme (egme). Kullanicinin gordugu
        # numaralar b14/b15; arayuzun SDL sirali dizisinde 13/14.
        self.declare_parameter(
            'tilt_button_pos', 14,
            _desc_int('D-pad SAG: mix_tilt yonunde (2 ileri, 3 geri)', -1, 20))
        self.declare_parameter(
            'tilt_button_neg', 13,
            _desc_int('D-pad SOL: ters yonde (2 geri, 3 ileri)', -1, 20))
        self.declare_parameter(
            'tilt_level', 1.0,
            _desc_float('Tusa basiliyken uygulanacak guc (0-1)', 0.0, 1.0))
        self.declare_parameter(
            'deadzone', 0.08,
            _desc_float('Bu esigin altindaki cubuk sapmasi 0 sayilir', 0.0, 0.9))

        # -- karisim (hangi eksen hangi motoru ne yonde surer) -------------
        # Sira: [on sol, on sag, orta sol, orta sag, arka sol, arka sag]
        self.declare_parameter(
            'mix_surge', [1.0, 1.0, 0.0, 0.0, 1.0, 1.0],
            _desc('Motor basina ileri/geri katsayisi (0,1,4,5 = yatay takim)'))
        # Saga donerken 0 ve 4 duz, 1 ve 5 ters; sola donerken isaretler
        # kendiliginden cevriliyor (eksen negatif oluyor).
        self.declare_parameter(
            'mix_yaw', [1.0, -1.0, 0.0, 0.0, 1.0, -1.0],
            _desc('Motor basina donme katsayisi (saga donus icin)'))
        # Tusla kademeli guc: her basista darbeyi bu kadar us kaydir.
        self.declare_parameter(
            'button_step_us', 20,
            _desc_int('Tus basina darbe adimi (us)', 1, 500))
        self.declare_parameter(
            'button_up', 9,
            _desc_int('Kaymayi artiran tus (arayuz dizilimi: 9 = LB)', -1, 20))
        self.declare_parameter(
            'button_down', 10,
            _desc_int('Kaymayi azaltan tus (arayuz dizilimi: 10 = RB)', -1, 20))
        # 5 us / 100 ms = 50 us/s: tam adimin (20 us) dortte biri hizinda,
        # yani tutarak ince ayar hala mumkun ama beklemek gerekmiyor.
        self.declare_parameter(
            'button_hold_step_us', 5,
            _desc_int('Tus BASILI tutulurken her tekrarda eklenecek us',
                      1, 100))
        self.declare_parameter(
            'button_hold_delay', 0.4,
            _desc_float('Basili tutma bu sureyi asinca surukleme baslar (s)',
                        0.05, 5.0))
        # Joy 30 Hz geliyor (33 ms). 0.10 s istemek pratikte 132 ms'e (4
        # mesaj) yuvarlaniyordu; 0.08 ile tekrar 3. mesaja, yani ~100 ms'e
        # oturuyor ve istenen hiz gercekten cikiyor.
        self.declare_parameter(
            'button_hold_interval', 0.08,
            _desc_float('Surukleme sirasinda tekrar araligi (s)', 0.02, 2.0))
        self.declare_parameter(
            'mix_step', [0.0, 0.0, 1.0, 1.0, 0.0, 0.0],
            _desc('Tus kaymasinin motor basina katsayisi; varsayilan ORTA '
                  'ikili (2,3)'))
        self.declare_parameter(
            'step_topic', '/ui/minirov/pwm_step',
            _desc('Adim buyuklugunu calisirken degistiren konu '
                  '(std_msgs/Int32, us)'))

        # Yana gidis: hangi tetik basiliysa o takim calisir. Degerler tetigin
        # basilma miktariyla (0..1) carpilir.
        self.declare_parameter(
            'mix_lat_right', [0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            _desc('R2 basiliyken motor basina katsayi (1 ve 5, ayni yonde)'))
        self.declare_parameter(
            'mix_lat_left', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            _desc('L2 basiliyken motor basina katsayi (0 ve 4, ayni yonde)'))

        # Egme karisimi: iki motor ZIT isaretli olmali, yoksa "ters yonlerde"
        # olmaz. Yalnizca orta ikili (2,3).
        self.declare_parameter(
            'mix_tilt', [0.0, 0.0, 1.0, -1.0, 0.0, 0.0],
            _desc('D-pad egmesinin motor basina katsayisi '
                  '(varsayilan 2 = +, 3 = -)'))

        self.declare_parameter(
            'max_thrust', 1.0,
            _desc_float('Toplam guc siniri (0-1). Tezgah testinde dusurulebilir',
                        0.0, 1.0))

        # -- ESC araligi ---------------------------------------------------
        self.declare_parameter(
            'esc_neutral_us', 1500,
            _desc_int('Notr darbe (us)', 1000, 2000))
        self.declare_parameter(
            'esc_span_us', 500,
            _desc_int('Notrden tam guce mesafe (us). 1500+-500 = 1000..2000',
                      1, 1000))

        # -- rampa ---------------------------------------------------------
        self.declare_parameter(
            'ramp_up_us_per_s', 400.0,
            _desc_float('Notrden uzaklasirken saniyede kac us. 400 -> tam guce '
                        '~1.25 s icinde cikar', 1.0, 5000.0))
        self.declare_parameter(
            'ramp_down_us_per_s', 1000.0,
            _desc_float('Notre donerken saniyede kac us. Yukselmeden hizli '
                        'olmali: birakinca arac hemen yavaslasin', 1.0, 5000.0))
        self.declare_parameter(
            'publish_rate_hz', 50.0,
            _desc_float('Rampa/yayin dongu frekansi (ESC 50 Hz bekler)',
                        1.0, 200.0))

        # -- guvenlik ------------------------------------------------------
        self.declare_parameter(
            'joy_timeout', 0.5,
            _desc_float('Bu sure Joy gelmezse hedef notre cekilir', 0.05, 10.0))
        self.declare_parameter(
            'stop_button', 2,
            _desc_int('Her seyi sifirlayan tus. Xbox X = arayuz dizisinde 2 '
                      '(bu projede PlayStation adiyla "square"). -1 = kapali',
                      -1, 20))
        self.declare_parameter(
            'deadman_button', -1,
            _desc_int('Basili tutulmasi gereken tus indeksi; -1 = kapali '
                      '(arayuz dizilimi: 9 = L1, 10 = R1)', -1, 20))

        self.neutral = int(self.get_parameter('esc_neutral_us').value)

        # Ramp durumu: current gercekten hatta giden darbe, target komutun
        # istedigi darbe. Ikisi arasindaki mesafe her cevrimde kirpilir.
        self.current_us: List[float] = [float(self.neutral)] * MOTOR_COUNT
        self.target_us: List[float] = [float(self.neutral)] * MOTOR_COUNT

        self.last_joy_stamp = 0.0
        self.last_tick = time.monotonic()
        self.idle_frames = 0
        self._axis_warned = False

        # Tuslarla birikmis kayma (us, isaretli) ve kenar yakalama icin
        # tuslarin bir onceki hali.
        self.step_offset = 0.0
        self._prev_up = False
        self._prev_down = False
        # Basili tutma suruklemesi icin: tusa basildigi an ve son tekrar ani.
        self._hold_since = {'up': 0.0, 'down': 0.0}
        self._last_repeat = {'up': 0.0, 'down': 0.0}
        self._stop_active = False

        joy_topic = self.get_parameter('joy_topic').value
        pwm_topic = self.get_parameter('pwm_topic').value

        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.create_subscription(Int32, self.get_parameter('step_topic').value,
                                 self._on_step_size, 10)
        self.publisher = self.create_publisher(Int32MultiArray, pwm_topic, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._on_tick)

        yaw_team = [i for i, v in enumerate(self.get_parameter('mix_yaw').value)
                    if abs(float(v)) > 1e-6]
        step_targets = [i for i, v in enumerate(self.get_parameter('mix_step').value)
                        if abs(float(v)) > 1e-6]
        right_team = [i for i, v in enumerate(self.get_parameter('mix_lat_right').value)
                      if abs(float(v)) > 1e-6]
        left_team = [i for i, v in enumerate(self.get_parameter('mix_lat_left').value)
                     if abs(float(v)) > 1e-6]
        tilt_targets = [i for i, v in enumerate(self.get_parameter('mix_tilt').value)
                        if abs(float(v)) > 1e-6]
        self.get_logger().info(
            f'miniROV joy koprusu: {joy_topic} -> {pwm_topic} | '
            f'on {FRONT_MOTORS} + arka {REAR_MOTORS} sag cubuktan esit guc, '
            f'saga R2 -> {right_team}, sola L2 -> {left_team}, '
            f'egme D-pad {self.get_parameter("tilt_button_neg").value}/'
            f'{self.get_parameter("tilt_button_pos").value} -> {tilt_targets}, '
            f'sifirla tusu {self.get_parameter("stop_button").value}, '
            f'orta {MID_MOTORS} notrde, donme '
            f'{yaw_team or "kapali"} (sol cubuk) | tus adimi '
            f'{self.get_parameter("button_step_us").value} us -> '
            f'{step_targets or "yok"} (basili tutunca '
            f'{self.get_parameter("button_hold_step_us").value} us/'
            f'{self.get_parameter("button_hold_interval").value:.2f} s) | rampa '
            f'{self.get_parameter("ramp_up_us_per_s").value:.0f} us/s yukari, '
            f'{self.get_parameter("ramp_down_us_per_s").value:.0f} us/s asagi'
        )

    # ------------------------------------------------------------------
    # Joy -> hedef darbe
    # ------------------------------------------------------------------
    def _axis(self, axes, idx: int) -> float:
        if idx < 0 or idx >= len(axes):
            if not self._axis_warned:
                self._axis_warned = True
                self.get_logger().warn(
                    f'Joy mesajinda {idx}. eksen yok ({len(axes)} eksen geldi) '
                    '— o eksen 0 sayiliyor.')
            return 0.0
        return float(axes[idx])

    def _deadzone(self, value: float) -> float:
        dead = float(self.get_parameter('deadzone').value)
        if abs(value) < dead:
            return 0.0
        # Esik disinda kalan kismi yeniden 0..1'e yay: esigin hemen ustunde
        # sicrama olmasin.
        span = 1.0 - dead
        scaled = (abs(value) - dead) / span if span > 1e-6 else 1.0
        return _clamp(scaled, 0.0, 1.0) * (1.0 if value > 0.0 else -1.0)

    def _trigger(self, axes, idx: int) -> float:
        """Tetigi 0..1 olarak oku.

        Arayuz zaten 0..1 gonderiyor; negatif bir deger gelirse (ham Linux
        joy surucusu tetikleri 1..-1 verir) bunu basilmamis sayiyoruz ki
        yanlis kaynakta motorlar kendiliginden calismasin.
        """
        value = _clamp(self._axis(axes, idx), 0.0, 1.0)
        dead = float(self.get_parameter('trigger_deadzone').value)
        if value < dead:
            return 0.0
        span = 1.0 - dead
        return _clamp((value - dead) / span, 0.0, 1.0) if span > 1e-6 else 1.0

    @staticmethod
    def _mix(param_value, count: int) -> List[float]:
        values = [float(v) for v in list(param_value)[:count]]
        values += [0.0] * (count - len(values))
        return values

    def _on_step_size(self, msg: Int32) -> None:
        """Arayuzdeki adim alani buraya yazar; parametreyi guncelle."""
        value = int(_clamp(float(msg.data), 1.0, 500.0))
        if value == int(self.get_parameter('button_step_us').value):
            return
        self.set_parameters([Parameter('button_step_us',
                                       Parameter.Type.INTEGER, value)])
        self.get_logger().info(f'tus adimi {value} us olarak guncellendi')

    def _pressed(self, buttons, idx: int) -> bool:
        return 0 <= idx < len(buttons) and bool(buttons[idx])

    def _button_delta(self, name: str, pressed: bool, was_pressed: bool,
                      now: float) -> float:
        """Bir tusun bu cevrimde uretecegi kayma.

        Basma ani tam adim verir; basili tutmak, gecikmeden sonra periyodik
        olarak yalnizca hold_step kadar suruklenir. 30 Hz'de gelen her
        mesajda tam adim saymak araligi bir saniyede bastan sona surerdi.
        """
        if not pressed:
            return 0.0

        if not was_pressed:
            self._hold_since[name] = now
            self._last_repeat[name] = now
            return float(self.get_parameter('button_step_us').value)

        delay = float(self.get_parameter('button_hold_delay').value)
        interval = float(self.get_parameter('button_hold_interval').value)
        if now - self._hold_since[name] < delay:
            return 0.0
        if now - self._last_repeat[name] < interval:
            return 0.0

        self._last_repeat[name] = now
        return float(self.get_parameter('button_hold_step_us').value)

    def _update_step_offset(self, buttons) -> None:
        span = float(self.get_parameter('esc_span_us').value)
        now = time.monotonic()

        up = self._pressed(buttons, int(self.get_parameter('button_up').value))
        down = self._pressed(buttons, int(self.get_parameter('button_down').value))

        self.step_offset += self._button_delta('up', up, self._prev_up, now)
        self.step_offset -= self._button_delta('down', down, self._prev_down, now)

        # Kayma tek basina araligi asamaz: 1000-2000 disina cikacak bir
        # birikim tutmanin anlami yok, tus bosa basilmis olur.
        self.step_offset = _clamp(self.step_offset, -span, span)
        self._prev_up = up
        self._prev_down = down

    def _reset_step_offset(self, reason: str) -> None:
        if abs(self.step_offset) < 0.5:
            return
        self.step_offset = 0.0
        self._prev_up = False
        self._prev_down = False
        self.get_logger().warn(f'tus kaymasi sifirlandi ({reason})')

    def _stop_all(self) -> None:
        """X tusu: hersey notre, birikmis kayma sifir — RAMPASIZ.

        Rampa ani GUC vermeyi engellemek icin var; durmak icin beklemenin
        anlami yok, notr zaten guvenli hal.
        """
        self.step_offset = 0.0
        self._prev_up = False
        self._prev_down = False
        self.target_us = [float(self.neutral)] * MOTOR_COUNT
        self.current_us = [float(self.neutral)] * MOTOR_COUNT

    def _on_joy(self, msg: Joy) -> None:
        self.last_joy_stamp = time.monotonic()

        stop = int(self.get_parameter('stop_button').value)
        if self._pressed(msg.buttons, stop):
            # Basili kaldigi surece hicbir eksen/tus okunmaz: X, kumandadaki
            # her seyin onune gecer.
            if not self._stop_active:
                self._stop_active = True
                self.get_logger().warn('X: tum motorlar sifirlandi')
            self._stop_all()
            return
        self._stop_active = False

        deadman = int(self.get_parameter('deadman_button').value)
        if deadman >= 0:
            held = deadman < len(msg.buttons) and bool(msg.buttons[deadman])
            if not held:
                # Deadman birakildi: tutulan kayma da dusmeli, yoksa tusa
                # tekrar basildiginda arac eski gucle firlar.
                self._reset_step_offset('deadman birakildi')
                self.target_us = [float(self.neutral)] * MOTOR_COUNT
                return

        self._update_step_offset(msg.buttons)

        surge = self._deadzone(self._axis(msg.axes,
                               int(self.get_parameter('axis_surge').value)))
        yaw = self._deadzone(self._axis(msg.axes,
                             int(self.get_parameter('axis_yaw').value)))

        if bool(self.get_parameter('invert_surge').value):
            surge = -surge
        if bool(self.get_parameter('invert_yaw').value):
            yaw = -yaw

        go_right = self._trigger(msg.axes,
                                 int(self.get_parameter('axis_right').value))
        go_left = self._trigger(msg.axes,
                                int(self.get_parameter('axis_left').value))

        mix_surge = self._mix(self.get_parameter('mix_surge').value, MOTOR_COUNT)
        mix_yaw = self._mix(self.get_parameter('mix_yaw').value, MOTOR_COUNT)
        mix_right = self._mix(self.get_parameter('mix_lat_right').value, MOTOR_COUNT)
        mix_left = self._mix(self.get_parameter('mix_lat_left').value, MOTOR_COUNT)

        # Egme: iki tusa birden basilirsa toplam sifir, kendiliginden goturur.
        tilt = 0.0
        if self._pressed(msg.buttons, int(self.get_parameter('tilt_button_pos').value)):
            tilt += 1.0
        if self._pressed(msg.buttons, int(self.get_parameter('tilt_button_neg').value)):
            tilt -= 1.0
        tilt *= float(self.get_parameter('tilt_level').value)
        mix_tilt = self._mix(self.get_parameter('mix_tilt').value, MOTOR_COUNT)

        # Iki tetige birden basilirsa iki takim da calisir: yanal itki
        # birbirini goturur, ozel bir kural gerekmiyor.
        cmd = [mix_surge[i] * surge
               + mix_yaw[i] * yaw
               + mix_right[i] * go_right
               + mix_left[i] * go_left
               + mix_tilt[i] * tilt
               for i in range(MOTOR_COUNT)]

        # Ileri + donme ayni anda geldiginde bir motor 1'i asabilir. Hepsini
        # ayni oranda kucultuyoruz: yon korunur, yalnizca genlik duser.
        peak = max((abs(c) for c in cmd), default=0.0)
        if peak > 1.0:
            cmd = [c / peak for c in cmd]

        scale = float(self.get_parameter('max_thrust').value)
        span = float(self.get_parameter('esc_span_us').value)
        self.neutral = int(self.get_parameter('esc_neutral_us').value)
        mix_step = self._mix(self.get_parameter('mix_step').value, MOTOR_COUNT)

        # Cubuk komutu ile tus kaymasi toplanir: cubuk notrdeyken motorlar
        # birikmis kaymayi tutar, cubuk itilince onun uzerine biner.
        self.target_us = [
            self.neutral + _clamp(_clamp(cmd[i] * scale, -1.0, 1.0) * span
                                  + self.step_offset * mix_step[i],
                                  -span, span)
            for i in range(MOTOR_COUNT)
        ]

    # ------------------------------------------------------------------
    # Rampa dongusu
    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        if dt <= 0.0:
            return

        timeout = float(self.get_parameter('joy_timeout').value)
        stale = (now - self.last_joy_stamp) > timeout
        if stale:
            # Arayuz sustu / kablo gitti: hedef notr. Rampa asagi hizinda
            # inecegi icin kesme yumusak, ama stm32_bridge'in kendi 0.5 s
            # failsafe'i de arkada duruyor.
            self._reset_step_offset('joy kesildi')
            self.target_us = [float(self.neutral)] * MOTOR_COUNT

        up_step = float(self.get_parameter('ramp_up_us_per_s').value) * dt
        down_step = float(self.get_parameter('ramp_down_us_per_s').value) * dt

        moving = False
        for i in range(MOTOR_COUNT):
            cur = self.current_us[i]
            tgt = self.target_us[i]
            delta = tgt - cur
            if abs(delta) < 0.5:
                self.current_us[i] = tgt
                continue

            moving = True
            # Notre dogru mu gidiyoruz yoksa notrden uzaga mi: adim siniri
            # buna gore secilir. Isaret carpimi negatifse hareket notru
            # kucultuyor demektir (ters yone gecis de buraya girer).
            toward_neutral = (cur - self.neutral) * delta < 0.0
            step = down_step if toward_neutral else up_step
            self.current_us[i] = cur + _clamp(delta, -step, step)

        at_neutral = all(abs(v - self.neutral) < 0.5 for v in self.current_us)
        if stale and at_neutral and not moving:
            # Notre indik ve komut yok: yayini birak. Ayni konuya yazan
            # thruster_allocator'i bogmayalim; stm32_bridge zaten kendi
            # failsafe'iyle notru suruyor.
            if self.idle_frames > 3:
                return
            self.idle_frames += 1
        else:
            self.idle_frames = 0

        out = Int32MultiArray()
        out.data = [int(round(v)) for v in self.current_us]
        self.publisher.publish(out)

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        out = Int32MultiArray()
        out.data = [self.neutral] * MOTOR_COUNT
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = MiniRovJoyNode()
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
