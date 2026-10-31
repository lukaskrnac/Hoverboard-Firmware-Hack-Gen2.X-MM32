# ROS 2 / ODrive driver support

The firmware speaks ODrive's **CANSimple** protocol, carried over a **UART** instead of CAN.
ODrive's ROS 2 driver, [odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive)
(`odrive_node` and `odrive_ros2_control`), only talks to SocketCAN. It also expects the cyclic messages an
ODrive sends without being asked (heartbeat, encoder estimates, …). The boards can't do that, because
several of them share one UART line and only answer when polled.

`uart_can_bridge.py` closes that gap, so **the unmodified ODrive ROS 2 driver works with the boards**:

```
ros_odrive (odrive_node / odrive_ros2_control)  <->  vcan0  <->  uart_can_bridge.py  <->  UART  <->  boards
```

- It polls each board for the cyclic messages and puts the answers on the CAN interface, as if an ODrive
  had sent them.
- It forwards every frame addressed to a bridged node to the UART: `Set_Axis_State`,
  `Set_Controller_Mode`, `Set_Input_Pos/Vel/Torque`, `Clear_Errors`, `Estop`, `RxSdo`, …
- RTR frames become requests, and their answers go back on CAN (`TxSdo` replies too).
- A board that stops answering stops producing frames, so the driver sees it as gone.

| Cyclic message | Default rate | Option |
|---|---|---|
| Heartbeat (0x01) | 10 Hz | `--heartbeat` |
| Get_Encoder_Estimates (0x09) | 50 Hz | `--encoder` |
| Get_Iq (0x14), Get_Torques (0x1C) | 10 Hz | `--iq`, `--torques` |
| Get_Bus_Voltage_Current (0x17), Get_Error (0x03) | 2 Hz | `--bus`, `--error` |
| Get_Temperature (0x15) | 1 Hz | `--temp` |

One poll costs about 1.7 ms at 115200 baud (with the FTDI adapter's latency timer at 1 ms, which the
bridge sets). That gives roughly 500 polls/s for all boards together, so lower the rates when many boards
share one line. You can also raise the board's `baud` (with a single board: `txpushpull=2`, up to 921600).

## Setup

```bash
# once per boot: a virtual CAN interface (a real can0 also works: the boards then sit next to real ODrives)
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up

# the bridge (needs pyserial; Linux only)
python3 ros/uart_can_bridge.py --port /dev/ttyUSB0 --nodes 0 1 -v
```

Each board's CANSimple node id is its slave id (`axis0.config.can.node_id`, default 0). Check it with
`candump vcan0`: the bridge prints `node N: answering` once a board replies.

## ros_odrive

Build ros_odrive in a colcon workspace (ROS 2 Jazzy or Humble):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/odriverobotics/ros_odrive.git
cd .. && colcon build && source install/setup.bash
```

**odrive_node** (topics and services per axis):

```bash
ros2 run odrive_can odrive_can_node --ros-args -r __node:=odrive_axis0 -p node_id:=0 -p interface:=vcan0
ros2 topic echo /odrive_axis0/controller_status          # pos, vel, torque, Iq, axis state, errors
ros2 topic echo /odrive_axis0/odrive_status              # bus voltage/current, temperature, errors
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
ros2 topic pub -r 20 /odrive_axis0/control_message odrive_can/msg/ControlMessage \
  "{control_mode: 2, input_mode: 1, input_vel: 1.0}"     # 1 turn/s
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
```

**odrive_ros2_control**: `<param name="can">vcan0</param>` in the `<hardware>` block and each board's
node id in its joint (`<param name="node_id">0</param>`), exactly as with real ODrives. A minimal one-wheel
example is in this directory: `hoverboard_wheel.urdf`, `hoverboard_controllers.yaml` and
`hoverboard_wheel.launch.py`. It uses a `JointGroupVelocityController`; add more joints for more boards, or
use `diff_drive_controller` as in ros_odrive's `odrive_botwheel_explorer`.

```bash
ros2 launch ros/hoverboard_wheel.launch.py
ros2 topic pub -r 20 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [6.28]}"  # rad/s
ros2 topic echo /joint_states
```

## Differences from a real ODrive

- **Watchdog always on.** Without a new set point for `axis0.config.watchdog_timeout` seconds (default
  1.0), a speed command falls to 0 and a position is held. Keep publishing set points (ros2_control does
  it every cycle; with `odrive_node`, publish `control_message` at ≥ 2 Hz) or raise the timeout over SDO.
- **Units:** turns, turns/s and Nm as in ODrive. The position comes from the hall sensors (90 steps per turn
  on a typical 15 pole pair hub motor), and the speed is filtered.
- **Not available:** calibration states (the boards are set up with PinFinder), `Get_Temperature`'s motor
  temperature (NaN; the FET temperature is the MCU's internal sensor), `Set_Axis_Node_ID`. Iq and torque
  read NaN or 0 where the board has no phase current sensing.
- `Heartbeat` byte 7 (unused by ODrive) carries the charger and foot-pad sensor bits.
