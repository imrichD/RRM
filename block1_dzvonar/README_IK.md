# Inverzna Kinematika - Implementacia IK Solvera a Motion Managera

## Systemove komponenty

### 1. IK Solver Node (`ik_solver_node`)
- Pocitá inverznú kinematiku pomocou analytických funkcií
- Nájde všetky 4 riešenia (ak sú validné)
- Dynamicky načítava limity kĺbov z `robot_description` URDF modelu
- Automaticky filtruje riešenia, ktoré prekračujú fyzické limity
- Poskytuje dve služby:
  - `ik_all_solutions`: Vychádza všetky platné IK riešenia
  - `ik_best_solution`: Nájde najlepšie riešenie podľa minimálnej zmeny kĺbov

### 2. Motion Manager Node (`motion_manager_node`)
- Orchestrátor pohybu a rozhranie pre používateľa
- Poskytuje službu `move_cartesian` s vstupom (x, y, z) v metroch
- Workflow:
  1.获取aktuálny stav robota z `joint_states`
  2. Požiada IK solver o najlepšie riešenie
  3. Pošle príkaz na pohyb do simulátora (`move_command` z `rrm_sim`)
  4. Čaká na potvrdenie ukončenia pohybu z simulátora

### 3. Cartesian Control Client (`cartesian_control_client`)
- Interaktívne CLI rozhranie na zadávanie kartézskych pozícií
- Voláno `move_cartesian` službu z motion managera
- Menu-driven interface s možnosťou zadávať X, Y, Z súradnice

## Spustenie systému

### Terminal 1: Spustenie hlavného systému
```bash
source /opt/ros/jazzy/setup.bash
source /home/imrich/ros2-ws-rrm/install/setup.bash
cd /home/imrich/ros2-ws-rrm
ros2 launch block1_dzvonar task_1_6.launch.py
```

Tento príkaz spustí:
- Robot state publisher (z `rrm_simple_robot_model`)
- rrm_sim (robot simulator)
- joint_logger (zaznamenáva joint stavy)
- ik_solver (IK solving engine)
- motion_manager (orchestrátor pohybu)
- RViz (vizualizácia) - možno padne na snap knižnicách, to je OK

### Terminal 2: Spustenie interaktívneho klienta
```bash
bash /home/imrich/ros2-ws-rrm/src/RRM_JA/block1_dzvonar/launch/start_cartesian_client.sh
```

KHebo priamo:
```bash
source /opt/ros/jazzy/setup.bash
source /home/imrich/ros2-ws-rrm/install/setup.bash
ros2 run block1_dzvonar cartesian_control_client
```

## Použivanie interaktívneho klienta

```
========================================
   Cartesian Control Interface (RRM)     
========================================

Options:
  [m] Move to Cartesian position
  [q] Quit
> m
```

Po výbere `[m]` zadajte:
- **X (meters):** napr. 0.5
- **Y (meters):** napr. 0.2  
- **Z (meters):** napr. 0.3

Systém automaticky:
1. Vypočíta IK pre danú polohu
2. Vyberie najlepšie riešenie
3. Pohne robotom
4. Vráti výsledok: `[OK]` alebo `[FAIL]`

## Príklady testovacích pozícií

Príklad 1 - Ostrá poloha:
- X: 0.4, Y: 0.0, Z: 0.4 (viac v ústí zmere Z)

Príklad 2 - Bočná poloha:
- X: 0.3, Y: 0.3, Z: 0.3

Príklad 3 - Blízka poloha:
- X: 0.2, Y: 0.1, Z: 0.25

## Dynamika IK Solvera

### Systém 3-DOF robota:
- **q1** (roll): rotácia okolo Z osi, limity [-1.620, 1.620] rad
- **q2** (pitch): predný kĺb, limity [-0.960, 2.182] rad
- **q3** (roll): posledný kĺb, limity [-0.960, 2.182] rad

### Geometória:
- Offset prvého kĺbu (d1): 0.25 m
- Dĺžka druhého článku (a2): 0.4 m
- Dĺžka tretieho článku (a3): 0.3 m

## Diagnostika a problémy

### Ak motion_manager neprijíma požiadavky:
- Skontrolujte, že ik_solver je spustený a načítal limity
- Kontrola: `ros2 service list | grep ik`

### Ak robot nešeká:
- Overte, že robot_sim je spustený
- Kontrola: `ros2 topic list | grep joint_states`

### Ak je zdĺhavý pohyb:
- Zvýšte max_velocity parameter v motion_manager
- Aktuálne nastavenie: 0.5 rad/s (voliteľný parameter)

## Testing bez interaktívneho klienta

Priama služba (z trieda terminálu):
```bash
ros2 service call /move_cartesian block1_dzvonar/srv/MoveCartesian "{x: 0.5, y: 0.2, z: 0.3}"
```

Všetky riešenia IK:
```bash
ros2 service call /ik_all_solutions block1_dzvonar/srv/IkAllSolutions "{x: 0.5, y: 0.2, z: 0.3}"
```

Najlepšie riešenie:
```bash
ros2 service call /ik_best_solution block1_dzvonar/srv/IkBestSolution "{x: 0.5, y: 0.2, z: 0.3, current_j1: 0, current_j2: 0, current_j3: 0}"
```

## Splnenie požiadaviek

✅ **1. [2,5b] Implementácia IK Solvera**
- ✅ Analytický IK solver (4 riešenia)
- ✅ Dynamické nábludanie limitov z URDF
- ✅ Automatická filtrácia riešení mimo limity
- ✅ Dve služby (all_solutions, best_solution)

✅ **2. [2b] Manažér pohybu**
- ✅ Riadiaca Node s orchestráciou
- ✅ Služba move_cartesian s (x, y, z) vstupom
- ✅ Interakcia s IK solverom a simulátorom
- ✅ Potvrdenie ukončenia pohybu

✅ **3. [0,5b] Integrácia a Launch**
- ✅ Launch súbor spúšťa všetky komponenty
- ✅ Systém pripravený na príkazy po spustení
- ✅ Interaktívne rozhranie pre užívateľa

## Poznámky k implementácii

1. **Zdrojový kód**: Všetky komponenty sú implementované v C++ s ROS 2 integration
2. **Real-time**: Motion manager čaká na potvrdenie z simulátora
3. **Robustnosť**: Timeout na všetky ROS 2 služby (15 sekúnd)
4. **Logovanie**: Podrobné INFO a WARN logy pre debugging
