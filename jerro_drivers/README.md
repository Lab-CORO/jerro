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

# Auto-tuning (moteur A, vitesse cible 200 ticks/sec) - voir la section Auto-tuning
ros2 action send_goal -f /motor/auto_tune jerro_msgs/action/AutoTunePID \
  "{motor_select: 1, target_velocity: 200.0, max_duration: 120.0}"
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

Chaque moteur expose, en plus des gains :

| Paramètre | Rôle |
|---|---|
| `integral_max` | Saturation du terme intégral (anti-windup) |
| `output_max` | Saturation de la sortie PID avant compensation de zone morte |
| `deadband_pwm` | Compensation de friction : PWM ajouté au signe de la sortie |
| `deadband_blend` | Largeur sur laquelle `deadband_pwm` monte progressivement de 0. **Empêche le saut discontinu au passage par zéro**, qui transformait le PID en contrôleur à relais et produisait un cycle limite à basse vitesse. `0.0` rétablit l'ancien comportement |
| `error_deadzone` | Bande d'erreur dans laquelle l'action de commande est annulée |
| `derivative_filter_alpha` | Filtrage passe-bas du terme dérivé (plus bas = plus filtré) |

La dérivée est calculée **sur la mesure** et non sur l'erreur : pas de pic au
changement de consigne, ni au passage de l'erreur dans `error_deadzone`.

### Mesure de vitesse

```yaml
control:
  velocity_filter_alpha: 0.3
  velocity_min_ticks: 4.0      # ticks visés dans la fenêtre de mesure
  velocity_max_window: 0.2     # fenêtre maximale (s)
```

La vitesse est estimée sur une **fenêtre adaptative** : la fenêtre s'élargit
jusqu'à contenir `velocity_min_ticks` ticks, sans dépasser `velocity_max_window`.
Avec la fenêtre fixe de 20 ms, à 200 ticks/s on mesure ~4 ticks par cycle (correct)
mais à 20 ticks/s seulement 0.4 tick : la mesure saute entre 0 et 50 ticks/s, et le
terme dérivé amplifie ce bruit de quantification.

Augmenter `velocity_max_window` améliore la résolution à basse vitesse mais ajoute
du retard de phase et peut déstabiliser la boucle.

---

## Auto-tuning

```bash
ros2 action send_goal -f /motor/auto_tune jerro_msgs/action/AutoTunePID \
  "{motor_select: 1, target_velocity: 200.0, max_duration: 120.0}"
```

`motor_select` : `1` = moteur A, `2` = moteur B, `3` = les deux.
Le `-f` affiche le feedback en continu.

> **Faire le premier essai roues en l'air ou robot calé.** L'identification fait
> volontairement osciller le moteur autour de la consigne.

### Déroulement

1. **Vérification encodeur** — un PWM connu (`encoder_check_pwm`) est appliqué ~1 s.
   Si le compteur ne progresse pas, l'action abandonne immédiatement avec un message
   pointant le câblage, au lieu d'attendre l'expiration du budget de temps.
2. **Identification** de `Ku` et `Tu` (voir méthodes ci-dessous).
3. **Calcul des gains** selon `zn_variant`.
4. **Sauvegarde** dans le fichier désigné par `auto_tune.output_path`.

Pendant toute la durée de l'action, la boucle de contrôle 50 Hz est **suspendue** :
le thread de tuning est seul à piloter les moteurs. Un second goal envoyé pendant
qu'un tuning est en cours est rejeté.

### Méthodes (`auto_tune.method`)

| Méthode | Principe | Durée |
|---|---|---|
| `relay` (défaut) | Retour à relais (Åström–Hägglund) : commutation ±`relay_amplitude` autour du PWM de biais, avec hystérésis dimensionnée sur le bruit mesuré. `Ku = 4d / (π·√(a²−h²))` | ~30 s |
| `sweep` | Balayage de `Kp` jusqu'à apparition spontanée d'une oscillation | ~960 s avec les valeurs par défaut |

Le relais **force** l'oscillation au lieu d'espérer la voir apparaître, d'où la
différence de durée et de fiabilité. Le balayage est conservé comme fallback.

Si `max_duration` est insuffisant pour couvrir tout le balayage, un avertissement
au démarrage indique le `Kp` maximum réellement atteignable.

### Formule des gains (`auto_tune.zn_variant`)

| Variante | Kp | Commentaire |
|---|---|---|
| `classic` | 0.6·Ku | Ziegler-Nichols historique. Vise une décroissance au quart d'amplitude, donc une réponse **volontairement oscillante** (~25 % de dépassement) |
| `no_overshoot` (défaut) | 0.2·Ku | Recommandé pour une boucle de vitesse |
| `pessen` | 0.7·Ku | Plus agressif que `classic` |
| `tyreus_luyben` | 0.45·Ku | Compromis robuste |

Si les gains trouvés donnent une réponse oscillante, la variante est le premier
paramètre à changer — pas la mesure de `Ku`.

### Réglage du relais

| Paramètre | Effet |
|---|---|
| `relay_amplitude` (d) | Trop faible → pas de cycle limite franc. Trop élevé → saturation PWM et `Ku` sous-estimé |
| `relay_hysteresis` (h) | Plancher de l'hystérésis ; la valeur retenue est `max(3·σ_bruit, h)`. Trop bas → le relais commute sur du bruit |
| `relay_min_cycles` | Cycles cohérents requis pour valider la convergence |
| `relay_skip_cycles` | Cycles initiaux ignorés (transitoire) |
| `relay_bias_ramp_rate` | Vitesse de montée (PWM/s) pendant la recherche du biais |

### Persistance des gains

Les gains sont écrits dans `auto_tune.output_path`, qui doit pointer vers le
fichier **source** `config/pid_gains.yaml`. Les gains des **deux** moteurs sont
sauvegardés (l'état vivant des PID), donc tuner un seul moteur n'écrase pas les
gains de l'autre.

`motor_control.launch.py` lit `pid_gains.yaml` depuis `share/` : un `colcon build`
est nécessaire pour que les gains sauvegardés prennent effet au lancement suivant.

```bash
colcon build --packages-select jerro_drivers && source install/setup.bash
```

Si l'écriture échoue, l'action retourne `success: false` avec le chemin fautif.

---

## Installation

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jerro_drivers
source install/setup.bash
```

> ⚠️ `~/.bashrc` source `~/gpa778_ros_ws/install/setup.bash`. Dans un terminal neuf,
> `ros2 run jerro_drivers ...` exécute donc le binaire de **ce** workspace-là, pas
> celui de `~/ros2_ws`. Toujours sourcer `~/ros2_ws/install/setup.bash` avant de
> lancer, et vérifier en cas de doute :
> ```bash
> ros2 pkg prefix jerro_drivers   # doit afficher /home/ubuntu/ros2_ws/install/jerro_drivers
> ```

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
