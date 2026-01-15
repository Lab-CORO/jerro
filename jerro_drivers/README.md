# jerro_drivers

Package ROS2 contenant les drivers pour le robot Jerro.

## Nodes

### 1. encoder_publisher

Lit les encodeurs des deux moteurs et publie les comptages.

**Topics publiés:**
- `/encoder_a` (std_msgs/Int32) - Comptage encodeur moteur A (ticks)
- `/encoder_b` (std_msgs/Int32) - Comptage encodeur moteur B (ticks)

**Fréquence:** 50 Hz

**Lancer:**
```bash
ros2 run jerro_drivers encoder_publisher
```

---

### 2. motor_pid_controller

Contrôle les moteurs avec régulation PID ou commande PWM directe.

**Topics souscrits:**
- `/encoder_a` (std_msgs/Int32) - Comptage encodeur A
- `/encoder_b` (std_msgs/Int32) - Comptage encodeur B
- `/motor/set_speed` (jerro_msgs/MotorSpeed) - Vitesse cible en ticks/sec (mode PID)
- `/motor/motor_RT_cmd` (jerro_msgs/MotorSpeed) - Commande PWM directe -255 à +255

**Actions:**
- `/motor/auto_tune` (jerro_msgs/action/AutoTunePID) - Auto-tuning des gains PID

**Fréquence:** 50 Hz

**Configuration:** `config/motor_pid_config.yaml`

**Lancer:**
```bash
ros2 run jerro_drivers motor_pid_controller
```

**Exemples:**
```bash
# Mode PID - définir vitesse cible en ticks/sec
ros2 topic pub /motor/set_speed jerro_msgs/msg/MotorSpeed "{motor_speed_a: 100.0, motor_speed_b: 100.0}"

# Mode PWM direct - valeurs -255 à +255
ros2 topic pub /motor/motor_RT_cmd jerro_msgs/msg/MotorSpeed "{motor_speed_a: 50.0, motor_speed_b: 50.0}"

# Auto-tuning (moteur A, vitesse cible 200 ticks/sec)
ros2 action send_goal /motor/auto_tune jerro_msgs/action/AutoTunePID "{motor_id: 1, target_velocity: 200.0}"
```

---

### 3. centrale_inertielle

Lit la centrale inertielle MPU6050 (accéléromètre + gyroscope) via I2C avec pigpio.

**Topics publiés:**
- `/imu/data_raw` (sensor_msgs/Imu) - Données IMU brutes
- `/imu/data_reoriented` (sensor_msgs/Imu) - Données IMU réorientées

**Topics souscrits:**
- `/imu/data` (sensor_msgs/Imu) - Données IMU pour réorientation
- `/set_pose` (geometry_msgs/PoseWithCovarianceStamped) - Ajuster le biais de yaw

**TF broadcasts:**
- `base_link` → `imu_frame`

**Fréquence:** 20 Hz

**Lancer:**
```bash
ros2 run jerro_drivers centrale_inertielle
```

---

### 4. servomotor

Contrôle un servomoteur sur GPIO 25.

**Services:**
- `/set_servo_pos` (jerro_msgs/srv/SetServoPos) - Positionner le servo

**Paramètres:**
- Position: 1000-2000 µs (microsecondes)
  - 1000 µs = position minimale
  - 1500 µs = position centrale
  - 2000 µs = position maximale

**Lancer:**
```bash
ros2 run jerro_drivers servomotor
```

**Exemples:**
```bash
# Position centrale
ros2 service call /set_servo_pos jerro_msgs/srv/SetServoPos "{position: 1500}"

# Position minimale
ros2 service call /set_servo_pos jerro_msgs/srv/SetServoPos "{position: 1000}"

# Position maximale
ros2 service call /set_servo_pos jerro_msgs/srv/SetServoPos "{position: 2000}"
```

---

## Hardware

### GPIO utilisés

| Fonction | GPIO | Librairie | Notes |
|----------|------|-----------|-------|
| Moteur 1 DIR | 5 | pigpio | Direction moteur A |
| Moteur 1 PWM | 12 | pigpio | Hardware PWM0 |
| Moteur 2 DIR | 6 | pigpio | Direction moteur B |
| Moteur 2 PWM | 13 | pigpio | Hardware PWM1 |
| Encoder A1 | 23 | pigpio | Moteur A signal A |
| Encoder A2 | 24 | pigpio | Moteur A signal B |
| Encoder B1 | 17 | pigpio | Moteur B signal A |
| Encoder B2 | 27 | pigpio | Moteur B signal B |
| H-Bridge Power | 26 | pigpio | Alimentation H-Bridge |
| H-Bridge Enable M1 | 16 | pigpio | Enable moteur 1 |
| H-Bridge Enable M2 | 22 | pigpio | Enable moteur 2 |
| Servo | 25 | pigpio | Software PWM |
| IMU GND | 4 | pigpio | Ground pour MPU6050 |

### Dépendances

- **pigpio** - Pour tous les périphériques (moteurs, encodeurs, servo, IMU)
  - GPIO: 4,5,6,12,13,16,17,22,23,24,25,26,27
  - I2C: Bus 1 pour MPU6050

---

## Configuration

### Gains PID

Fichier: `config/motor_pid_config.yaml`

```yaml
motor_a:
  Kp: 0.15
  Ki: 0.5
  Kd: 0.005
  integral_max: 150.0
  output_max: 240.0

motor_b:
  Kp: 0.15
  Ki: 0.5
  Kd: 0.005
  integral_max: 150.0
  output_max: 240.0

auto_tune:
  Kp_start: 0.1
  Kp_increment: 0.02
  min_extremum_amplitude: 15.0
  min_period: 0.3
```

---

## Installation

```bash
cd ~/jerro_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jerro_drivers
source install/setup.bash
```

## Lancement complet

```bash
# Terminal 1 - Encodeurs
ros2 run jerro_drivers encoder_publisher

# Terminal 2 - Contrôle moteurs
ros2 run jerro_drivers motor_pid_controller

# Terminal 3 - IMU
ros2 run jerro_drivers centrale_inertielle

# Terminal 4 - Servo
ros2 run jerro_drivers servomotor
```

Ou utiliser un launch file (si disponible):
```bash
ros2 launch jerro_drivers motor_control_launch.py
```
