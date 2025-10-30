# Sub Arm - ROS2 Robotic Arm Control Package

Pakiet ROS2 do sterowania ramieniem roboczym z integracją ODrive poprzez CAN.

## Zawartość

- **`sub_arm/`** - Kod Python (węzły ROS2)
- **`launch/`** - Pliki uruchomieniowe
- **`urdf/`** - Model URDF ramienia
- **`config/`** - Pliki konfiguracyjne

## Główne komponenty

### Węzły ROS2

- **`arm_state_publisher`** - Publikuje stan joints na podstawie inverse kinematics
- **`ik_ui.py`** - Interfejs graficzny do sterowania położeniem ramienia (Matplotlib sliders)
- **`odrive_can.py`** - Komunikacja z silnikami ODrive poprzez CAN
- **`target_publisher.py`** - Publikuje marker celu w RViz
- **`robot_state_publisher`** - Publikuje transformacje TF

### Pliki uruchomieniowe

- **`display.launch.py`** - Pełna symulacja ramienia z UI i RViz
- **`odrive.launch.py`** - Uproszczony launch do testowania ODrive

## Kinematics

### Forward Kinematics (FK)
Oblicza pozycję końcówki na podstawie kątów stawów przy użyciu homogenicznych macierzy transformacji.

```python
from simple_kinematics import forward_kinematics
pos = forward_kinematics([0, 0, 0.05, 0, 0, 0])  # [x, y, z]
```

### Inverse Kinematics (IK)
Oblicza kąty stawów na podstawie żądanego położenia końcówki (używa Jakobianu).

```python
from inverse_kinematics_model import inverse_kinematics
angles = inverse_kinematics([0.1, 0.0, 0.05])  # [q1, q2, q3, q4, q5, q6]
```

## Komunikacja CAN z ODrive

### Setup CAN

```bash
# Konfiguracja interfejsu CAN (CANable2)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Lub przy użyciu slcand dla CANable2
sudo slcand -o -c -s8 /dev/ttyUSB0 can0
sudo ifconfig can0 up

# Weryfikacja
ip link show can0
```

### Topiki ROS2

Publikuj na topiki, aby sterować ODrive:

```bash
# Ustawienie prędkości
ros2 topic pub /odrive/velocity_setpoint std_msgs/Float32 "{data: 50.0}"

# Obrót przez N sekund
ros2 topic pub /odrive/rotate_duration std_msgs/Float32 "{data: 3.0}"

# Limit prędkości
ros2 topic pub /odrive/vel_limit std_msgs/Float32 "{data: 400.0}"

# Limit prądu
ros2 topic pub /odrive/current_limit std_msgs/Float32 "{data: 10.0}"

# Reset błędów
ros2 topic pub /odrive/reset_errors std_msgs/Bool "{data: true}"
```

### CAN Arbitration IDs

Dla ODrive v3.6 z node ID = 0:

- **0x006** - Set Axis Node ID (broadcast)
- **0x0B** - Set Control Mode
- **0x0C** - Set Limits (velocity + current)
- **0x0D** - Set Encoder Mode
- **0x0E** - Set Motor Config
- **0x0C** (command) - Open-loop velocity setpoint
- **0x018** (response) - Error status

## Uruchomienie

### Pełna symulacja z UI

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch sub_arm display.launch.py
```

### Testowanie ODrive

```bash
ros2 launch sub_arm odrive.launch.py
```

## Konfiguracja ODrive

Ważne parametry w ODrive:

```python
odrv0.axis0.motor.config.pole_pairs = 7  # dla typowego BLDC
odrv0.axis0.encoder.config.mode = LOCKIN_SPIN  # tryb sensorless
odrv0.axis0.controller.config.vel_limit = 400  # limit prędkości
odrv0.axis0.motor.config.current_lim = 10  # limit prądu
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
```

## Troubleshooting

### Brak komunikacji CAN

1. Sprawdź interfejs: `ip link show can0`
2. Sprawdź kabel USB CANable2
3. Sprawdź bitrate: powinien być 500kbps
4. Weryfikuj arbitration IDs w logu

### ODrive nie reaguje

1. Sprawdź czy node ID = 0
2. Sprawdź czy ODrive jest w trybie CAN
3. Sprawdź zasilanie motoru
4. Sprawdź czy encoder jest w trybie LOCKIN_SPIN

### Błędy CAN "Invalid argument"

- Sprawdź czy arbitration IDs są poprawne
- Upewnij się że data payload ma prawidłową długość (8 bajtów)

## Dependencies

```bash
pip install rclpy can numpy matplotlib
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select sub_arm
```

## Licencja

MIT
