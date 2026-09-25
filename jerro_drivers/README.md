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
# Mode PID - définir vitesse cible en ticks/sec (max exploitable ~3465)
ros2 topic pub /motor/set_speed jerro_msgs/msg/MotorSpeed "{motor_speed_a: 1650.0, motor_speed_b: 1650.0}"

# Mode PWM direct - valeurs -255 à +255
ros2 topic pub /motor/motor_RT_cmd jerro_msgs/msg/MotorSpeed "{motor_speed_a: 150.0, motor_speed_b: 150.0}"

# Auto-tuning (les deux moteurs, ~50 % de la vitesse max) - voir la section Auto-tuning
ros2 action send_goal -f /motor/auto_tune jerro_msgs/action/AutoTunePID \
  "{motor_select: 3, target_velocity: 1650.0, max_duration: 180.0}"
```

> La consigne n'est **pas** écrêtée : au-delà de ~3465 ticks/s le nœud émet un
> `High velocity setpoint: …` et le PID sature, laissant une erreur statique.
> C'est à l'émetteur de `/cmd_vel` de plafonner.

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
| `integral_max` | Écrêtage de l'**accumulateur** intégral. Ce n'est pas qu'un anti-windup : la contribution intégrale vaut `Ki × integral`, donc elle ne peut jamais dépasser `Ki × integral_max` en PWM. **À dimensionner sur `output_max / Ki`** — voir ci-dessous |
| `output_max` | Saturation de la sortie PID avant compensation de zone morte |
| `deadband_pwm` | Compensation de friction : PWM ajouté au signe de la sortie |
| `deadband_blend` | Largeur sur laquelle `deadband_pwm` monte progressivement de 0. **Empêche le saut discontinu au passage par zéro**, qui transformait le PID en contrôleur à relais et produisait un cycle limite à basse vitesse. `0.0` rétablit l'ancien comportement |
| `error_deadzone` | Bande d'erreur dans laquelle l'action de commande est annulée |
| `derivative_filter_alpha` | Filtrage passe-bas du terme dérivé (plus bas = plus filtré) |

La dérivée est calculée **sur la mesure** et non sur l'erreur : pas de pic au
changement de consigne, ni au passage de l'erreur dans `error_deadzone`.

#### Dimensionner `integral_max`

`integral_max` borne ce que le PID peut **atteindre** en régime établi, pas
seulement sa marge de windup. Si `Ki × integral_max` est inférieur au PWM
nécessaire pour tenir la consigne, il reste une erreur statique que l'intégrateur
ne peut pas rattraper, quels que soient les gains.

```
integral_max ≈ output_max / Ki
```

Cas réel : avec `integral_max: 150` et `Ki = 0.611`, le plafond valait 92 PWM
alors qu'il faut ~200 PWM pour tenir 3000 ticks/s → **15 % d'erreur statique**.
Passé à 390, la même consigne est tenue à 0.3 %.

`Ki` change à chaque auto-tune, donc **`integral_max` est à recalculer après
chaque identification**. L'action ne l'écrit pas dans `pid_gains.yaml` : c'est
une étape manuelle.

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
  "{motor_select: 3, target_velocity: 1650.0, max_duration: 180.0}"
```

`motor_select` : `1` = moteur A, `2` = moteur B, `3` = les deux.
`target_velocity` est en **ticks/s**. Le `-f` affiche le feedback en continu.

> **Faire le premier essai roues en l'air ou robot calé.** L'identification fait
> volontairement osciller le moteur autour de la consigne.

### Choisir `target_velocity`

**Viser ~50 % de la vitesse max réelle**, soit **1650 ticks/s** (≈ 50 RPM,
≈ 0.17 m/s) avec le matériel actuel. C'est le paramètre le plus important de
l'identification : un cycle limite mesuré au ralenti décrit le frottement sec du
moteur, pas sa dynamique, et les gains qui en sortent ne valent rien à la vitesse
d'usage.

Repères mesurés (balayage PWM direct, roues en l'air) :

| PWM | Moteur A | Moteur B |
|---|---|---|
| 100 | 0 ticks/s (ne décolle pas depuis l'arrêt) | 0 ticks/s |
| 150 | 2693 ticks/s — 81.6 RPM — 0.278 m/s | 2781 ticks/s — 84.3 RPM — 0.287 m/s |
| 200 | 3111 ticks/s — 94.3 RPM — 0.321 m/s | 3128 ticks/s — 94.8 RPM — 0.323 m/s |
| 255 | 3656 ticks/s — 110.8 RPM — 0.377 m/s | 3632 ticks/s — 110.1 RPM — 0.375 m/s |

Avec `output_max: 240`, le maximum exploitable est **~3465 ticks/s ≈ 105 RPM
≈ 0.35 m/s**. Conversions (1980 ticks/tour mesurés, roue de 32.5 mm de rayon) :

```
ticks/s = RPM × 33          RPM   = ticks/s / 33
ticks/s = m/s × 9696        m/s   = ticks/s / 9696
```

> ⚠️ `control.max_rpm` a longtemps valu `30.0`, soit **3.7 fois trop bas**. Cette
> valeur sert de référence au « 50 % du max » : avec elle, la consigne conseillée
> tombait à 495 ticks/s, en réalité 14 % du max. Si vous changez de moteur,
> **remesurez `max_rpm` au balayage PWM avant de relancer une identification**.

### Effet du point de fonctionnement

Deux identifications du même matériel, seule `target_velocity` change :

| `target_velocity` | Tu moteur A | Tu moteur B | Écart |
|---|---|---|---|
| 495 ticks/s (14 % du max) | 0.156 s | 0.120 s | 23 % |
| 1650 ticks/s (50 % du max) | 0.188 s | 0.176 s | 7 % |

Deux moteurs de même référence doivent donner des `Tu` proches. **Un écart
important entre A et B est le signe d'une identification faite trop bas**, pas
forcément d'une différence matérielle.

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

Le relais se déroule en trois phases : **recherche du biais** (trouver le PWM qui
maintient la consigne), **mesure du bruit** (dimensionner l'hystérésis), puis
**cycle limite** (commuter et chronométrer).

| Paramètre | Effet |
|---|---|
| `relay_amplitude` (d) | Amplitude PWM de la commutation autour du biais. Trop faible → pas de cycle limite franc. Trop élevé → saturation PWM et `Ku` sous-estimé. **Réduite silencieusement** à `min(d, biais, 255−biais)` pour garder les deux alternances symétriques : sinon l'une est écrêtée et `Ku` est faux |
| `relay_hysteresis` (h) | Plancher de l'hystérésis ; la valeur retenue est `max(3·σ_bruit, h)`. Trop bas → le relais commute sur du bruit |
| `relay_hysteresis_max_ratio` | Plafond de l'hystérésis, en fraction de la consigne. Sans lui, une mesure bruitée donne `h > consigne` : le relais ne peut plus commuter et l'action tourne dans le vide jusqu'au timeout |
| `relay_noise_duration` | Durée de la phase de mesure du bruit (s) |
| `relay_min_cycles` | Cycles cohérents requis pour valider la convergence |
| `relay_skip_cycles` | Cycles initiaux ignorés (transitoire) |
| `relay_max_switch_gap` | Abandon anticipé si le relais ne commute plus pendant ce délai (s). Évite de consommer tout `max_duration` quand il est manifestement bloqué |

#### Recherche du biais

Le biais **converge** sur l'erreur de vitesse, il ne monte pas en rampe :

```
bias += relay_bias_rate × (erreur / consigne) × dt
```

puis il est figé quand l'erreur relative reste sous `relay_bias_tolerance`
pendant `relay_bias_settle_time`.

| Paramètre | Effet |
|---|---|
| `relay_bias_rate` | Vitesse de convergence (PWM/s à erreur relative pleine échelle) |
| `relay_bias_tolerance` | Erreur relative acceptée pour déclarer la convergence |
| `relay_bias_settle_time` | Durée de stabilité exigée avant de figer le biais |
| `relay_bias_filter_tc` | Constante de temps du filtre appliqué à la vitesse **avant** le test de convergence |

Le test de convergence porte sur une vitesse **filtrée**, pas sur la mesure brute.
L'estimation de vitesse a une fenêtre courte (`velocity_min_ticks: 4.0`, soit
~8 ms à 500 ticks/s) et oscille de ±10 % ; testée telle quelle, elle ne tient
jamais `relay_bias_settle_time` sous `relay_bias_tolerance`, même quand le biais
est correct en moyenne. L'intégrateur du biais, lui, agit toujours sur l'erreur
brute — c'est le seul test de convergence qui est filtré.

Le biais figé est aussi un diagnostic utile : il donne le PWM nécessaire pour
tenir la consigne, donc directement l'`integral_max` requis (`bias / Ki`).

### Persistance des gains

Les gains sont écrits dans `auto_tune.output_path`, qui doit pointer vers le
fichier **source** `config/pid_gains.yaml`. Les gains des **deux** moteurs sont
sauvegardés (l'état vivant des PID), donc tuner un seul moteur n'écrase pas les
gains de l'autre.

`motor_control.launch.py` lit `pid_gains.yaml` depuis `share/` : un `colcon build`
est nécessaire pour que les gains sauvegardés prennent effet au lancement suivant.

Si l'écriture échoue, l'action retourne `success: false` avec le chemin fautif.

### Procédure complète

Les paramètres sont lus **une seule fois, à la construction du nœud**. Aucune
modification de `motor_pid_config.yaml` ni aucun `ros2 param set` ne prend effet
sans redémarrage.

```bash
# 1. Identifier (roues en l'air), à ~50 % de la vitesse max
ros2 action send_goal -f /motor/auto_tune jerro_msgs/action/AutoTunePID \
  "{motor_select: 3, target_velocity: 1650.0, max_duration: 180.0}"

# 2. Recalculer integral_max = output_max / Ki pour CHAQUE moteur,
#    avec les Ki que l'action vient d'écrire, puis éditer
#    config/motor_pid_config.yaml à la main.

# 3. Propager vers share/ et redémarrer
colcon build --packages-select jerro_drivers && source install/setup.bash
ros2 launch jerro_drivers motor_control.launch.py
```

Au démarrage, le nœud journalise les gains qu'il a réellement chargés et la
vitesse max déduite — c'est la vérification la plus rapide :

```
Motor A: Kp=0.057 Ki=0.611 Kd=0.004
Motor B: Kp=0.061 Ki=0.698 Kd=0.004
  - Encoder resolution: 1980.0 ticks/rev (quadrature), vitesse max 105 RPM = 3465 ticks/s
```

### Valider le résultat

Vérifier l'erreur statique en boucle fermée sur **plusieurs** points, dont le haut
de plage : une erreur qui ne se voit qu'à vitesse élevée pointe `integral_max` ou
`output_max`, pas les gains. Résultat attendu, tous points sous 1 % :

| Consigne | Erreur A | Erreur B |
|---|---|---|
| 800 ticks/s (0.083 m/s) | 0.0 % | −0.6 % |
| 1650 ticks/s (0.170 m/s) | +0.4 % | +0.4 % |
| 2400 ticks/s (0.248 m/s) | −0.3 % | +0.3 % |
| 3000 ticks/s (0.309 m/s) | +0.3 % | +0.4 % |

### Diagnostic

| Symptôme | Cause | Correctif |
|---|---|---|
| `timeout pendant la recherche du biais (PWM=…, vitesse X pour une consigne de Y)` avec X proche de Y | Le biais est bon en moyenne, mais la mesure brute sort de `relay_bias_tolerance` à chaque échantillon | Augmenter `relay_bias_filter_tc`, ou relâcher `relay_bias_tolerance` |
| `impossible d'atteindre la consigne (PWM saturé à …)` | `target_velocity` au-delà de ce que le moteur peut fournir | Baisser la consigne ; vérifier `max_rpm` au balayage PWM |
| `amplitude de relais exploitable trop faible (d=…, pour un biais de … PWM)` | Le biais est trop proche de 0 ou de 255 : `d` est réduit sous 5 PWM pour garder le relais symétrique | Choisir une consigne plus éloignée des extrêmes, ou baisser `relay_amplitude` |
| Erreur statique qui n'apparaît qu'en haut de plage | `Ki × integral_max` insuffisant | Redimensionner `integral_max` |
| `Tu` très différents entre A et B | Identification faite trop bas dans la plage | Remonter `target_velocity` à ~50 % du max |
| Encodeur parfaitement figé malgré un PWM direct à 255 | Alimentation du pont en H coupée — voir ci-dessous | `pigs r 26` doit renvoyer `1` ; redémarrer le nœud |

> ⚠️ **Ne pas faire tourner deux `motor_pid_controller` en parallèle.** Le
> destructeur appelle `stopMotors()`, qui met GPIO 26 (alimentation du pont en H)
> et les enables 16/22 à LOW. Quitter une seconde instance par Ctrl-C coupe donc
> les moteurs de l'instance qui tournait encore, et l'auto-tune échoue ou boucle
> dans le vide sans message explicite. Vérifier avec `ros2 node list` : un
> avertissement sur des nœuds de même nom signale le problème.

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
