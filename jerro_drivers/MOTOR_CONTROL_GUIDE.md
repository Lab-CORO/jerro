# Guide de contrôle moteur - jerro_drivers

## Résolution des encodeurs

- **PPR (Pulses Per Revolution)**: 158.2
- **Quadrature**: 158.2 × 4 = **632.8 ticks/tour**
- **Mesure réelle**: ~610-630 ticks/tour (variance normale)

## Paramètres physiques du robot

- **Rayon des roues**: 3.575 cm
- **Distance entre roues**: 16.55 cm
- **RPM max estimé**: ~250 RPM
- **Vitesse max**: ~2500 ticks/sec

## Modes de contrôle

### 1. Mode PID (asservissement en vitesse)

**Topic**: `/motor/set_speed`
**Type**: `jerro_msgs/msg/MotorSpeed`
**Unités**: **ticks/sec** (vitesse angulaire des roues)

**Exemple**:
```bash
# Vitesse normale (1500 ticks/sec)
ros2 topic pub --once /motor/set_speed jerro_msgs/msg/MotorSpeed \
  "{motor_speed_a: 1500.0, motor_speed_b: 1500.0}"

# Rotation sur place (vitesses opposées)
ros2 topic pub --once /motor/set_speed jerro_msgs/msg/MotorSpeed \
  "{motor_speed_a: 800.0, motor_speed_b: -800.0}"

# Arrêt
ros2 topic pub --once /motor/set_speed jerro_msgs/msg/MotorSpeed \
  "{motor_speed_a: 0.0, motor_speed_b: 0.0}"
```

**Plages recommandées**:
- **Vitesse faible**: 500-1000 ticks/sec
- **Vitesse moyenne**: 1000-1500 ticks/sec
- **Vitesse élevée**: 1500-2000 ticks/sec
- **Maximum absolu**: 2500 ticks/sec

### 2. Mode Direct (commandes PWM sans PID)

**Topic**: `/motor/motor_RT_cmd`
**Type**: `jerro_msgs/msg/MotorSpeed`
**Unités**: **Duty cycle PWM** (-200 à +200)

**Exemple**:
```bash
# PWM direct à 50% (duty cycle = 127/255)
ros2 topic pub --once /motor/motor_RT_cmd jerro_msgs/msg/MotorSpeed \
  "{motor_speed_a: 100.0, motor_speed_b: 100.0}"

# PWM maximum
ros2 topic pub --once /motor/motor_RT_cmd jerro_msgs/msg/MotorSpeed \
  "{motor_speed_a: 200.0, motor_speed_b: 200.0}"
```

**Note**: En mode direct, les valeurs sont des PWM bruts, pas des vitesses!

### 3. Auto-tuning (Ziegler-Nichols)

**Action**: `/motor/auto_tune`
**Type**: `jerro_msgs/action/AutoTunePID`

**Exemple**:
```bash
ros2 action send_goal /motor/auto_tune jerro_msgs/action/AutoTunePID \
  "{motor_select: 3, target_velocity: 1000.0, max_duration: 120.0}"
```

**Paramètres**:
- `motor_select`: 1=Motor A, 2=Motor B, 3=Both
- `target_velocity`: Vitesse cible en **ticks/sec** pour l'auto-tuning
- `max_duration`: Durée maximale en secondes

## Conversions utiles

### Ticks/sec → RPM
```
RPM = (ticks/sec) / 632.8 × 60
```

**Exemples**:
- 1000 ticks/sec = 95 RPM
- 1500 ticks/sec = 142 RPM
- 2000 ticks/sec = 190 RPM

### RPM → Ticks/sec
```
ticks/sec = RPM × 632.8 / 60
```

### Vitesse linéaire (m/s)
```
v (m/s) = RPM × 2π × rayon_roue / 60
v (m/s) = (ticks/sec) × 2π × 0.03575 / 632.8
```

**Exemples**:
- 1000 ticks/sec = 0.355 m/s
- 1500 ticks/sec = 0.533 m/s
- 2000 ticks/sec = 0.711 m/s

## Topics disponibles

| Topic | Type | Description |
|-------|------|-------------|
| `/encoder_a` | `std_msgs/Int32` | Compte encodeur roue gauche |
| `/encoder_b` | `std_msgs/Int32` | Compte encodeur roue droite |
| `/motor/set_speed` | `MotorSpeed` | Consigne PID (ticks/sec) |
| `/motor/motor_RT_cmd` | `MotorSpeed` | Commande PWM directe |

## Démarrage

```bash
# 1. Démarrer pigpiod
sudo pigpiod

# 2. Lancer le système
source ~/jerro_ws/install/setup.bash
ros2 launch jerro_drivers motor_control.launch.py
```

## Dépannage

### Les moteurs ne tournent pas
1. Vérifier que `pigpiod` est en cours: `ps aux | grep pigpiod`
2. Vérifier les GPIO enable (26, 16, 22) sont configurés
3. Vérifier les logs pour des erreurs PWM

### Les moteurs tournent dans le mauvais sens
- Encoder A (gauche) décrémente vers l'avant
- Encoder B (droite) incrémente vers l'avant
- Voir `ENCODER_A_INVERT` et `ENCODER_B_INVERT` dans encoder_publisher.cpp

### Le PID oscille ou est instable
1. Lancer l'auto-tuning
2. Vérifier les gains dans `motor_pid_config.yaml`
3. Réduire la consigne de vitesse

## Fichiers de configuration

- **Gains PID**: `/home/ubuntu/jerro_ws/src/jerro_drivers/config/motor_pid_config.yaml`
- **Gains auto-tunés**: `/home/ubuntu/jerro_ws/install/jerro_drivers/share/jerro_drivers/config/pid_gains.yaml`
