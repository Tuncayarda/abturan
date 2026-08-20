import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_config = os.path.join(
        get_package_share_directory('joy_motor_pkg'),
        'config',
        'joy_wrench_allocator.yaml',
    )
    minirov_joy_config = os.path.join(
        get_package_share_directory('joy_motor_pkg'),
        'config',
        'minirov_joy.yaml',
    )
    stm32_config = os.path.join(
        get_package_share_directory('stm32_bridge'),
        'config',
        'stm32_bridge.yaml',
    )
    imu_config = os.path.join(
        get_package_share_directory('imu_bridge'),
        'config',
        'imu.yaml',
    )

    # Initial camera/stream settings. No destination IP — the host streamer
    # listens (SRT) and the single client connects to srt://<pi_ip>:<port>.
    # These can also be changed at runtime via `ros2 param set /front_cam_node ...`
    # — that runtime path is what the rosbridge "ros_bridge" exposes to the UI.
    port_arg = DeclareLaunchArgument('port', default_value='9003')
    width_arg = DeclareLaunchArgument('width', default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')
    bitrate_arg = DeclareLaunchArgument('bitrate', default_value='1000000')

    # STM32 link. On a Pi 5 the GPIO14/15 UART is /dev/ttyAMA0 and needs
    # `dtoverlay=uart0-pi5` in config.txt (setup_hardware.sh does this).
    # /dev/serial0 points at ttyAMA10 — that is the separate debug header,
    # not the GPIO pins the STM32 is wired to.
    serial_arg = DeclareLaunchArgument('stm32_port', default_value='/dev/ttyAMA0')
    baud_arg = DeclareLaunchArgument('stm32_baud', default_value='115200')

    return LaunchDescription([
        port_arg,
        width_arg,
        height_arg,
        fps_arg,
        bitrate_arg,
        serial_arg,
        baud_arg,

        # ── rosbridge ───────────────────────────────────────────────
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'default_call_service_timeout': 5.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True,
            }],
        ),

        # ── Front camera control plane ──────────────────────────────
        # The actual video (H264/MPEG-TS over SRT, listener on :port) is streamed
        # on the HOST by rpi_cam_streamer.py (rpicam-vid + GStreamer); this node
        # only owns the parameters and reports throughput — it does not carry the
        # video. The single client connects to srt://<pi_ip>:<port>.
        Node(
            package='rpi_cam_bridge',
            executable='rpi_cam_node',
            name='front_cam_node',
            output='screen',
            parameters=[{
                'ctrl_dir': '/cam_ctrl',
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'camera': 0,
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                'fps': ParameterValue(LaunchConfiguration('fps'), value_type=int),
                'bitrate': ParameterValue(LaunchConfiguration('bitrate'), value_type=int),
                'enabled': True,
            }],
        ),

        # ── IMU: BNO08x (Pi side, I2C on GPIO2/GPIO3) ───────────────
        # Publishes /imu/data_raw + /imu/data (on-chip fusion quaternion,
        # optionally /imu/mag). Pi 5 needs `dtoverlay=i2c1-pi5,pins_2_3`; if
        # the bus throws EREMOTEIO, the BNO08x is clock-stretching — add
        # `dtparam=i2c_arm_baudrate=50000` too.
        Node(
            package='imu_bridge',
            executable='imu_node',
            name='imu_node',
            parameters=[imu_config],
            output='screen',
        ),

        # ── Joystick → Wrench → Thruster allocator ─────────────────
        # 6 thrusters now: the STM32 board has exactly six ESC PWM outputs.
        Node(
            package='joy_motor_pkg',
            executable='joy_to_wrench',
            name='joy_to_wrench_node',
            parameters=[joy_config],
            output='screen',
        ),
        Node(
            package='joy_motor_pkg',
            executable='thruster_allocator',
            name='thruster_allocator_node',
            parameters=[joy_config],
            output='screen',
        ),

        # ── miniROV: joystick → ESC (rampali, dogrudan surus) ──────
        # Arayuzun joystick hedefi "minirov" iken /ui/minirov/joy_cmd_vel'e
        # yayin yapiyor; bu node sag cubugu on (0,1) + arka (4,5) motorlara
        # rampalayarak dagitir ve /control/pwm_cmds'e 1000-2000 us yazar.
        # Komut yokken yayini biraktigi icin allocator ile carpismaz.
        Node(
            package='joy_motor_pkg',
            executable='minirov_joy',
            name='minirov_joy_node',
            parameters=[minirov_joy_config],
            output='screen',
        ),

        # ── STM32 actuator link (UART) ──────────────────────────────
        # Last hop of the control chain: /control/pwm_cmds (6 ch, 1000-2000 us)
        # plus the
        # LED and stepper topics are framed and sent to the STM32, which
        # owns all the real actuator timing (6 ESC PWM, 4-wire stepper, LED).
        Node(
            package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            parameters=[
                stm32_config,
                {
                    'serial_port': LaunchConfiguration('stm32_port'),
                    'baudrate': ParameterValue(LaunchConfiguration('stm32_baud'),
                                               value_type=int),
                },
            ],
            output='screen',
        ),
    ])
