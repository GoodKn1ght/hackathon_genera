# Тестова підсистема falcon_gaze

## Призначення

Тестовий фреймворк для симуляції рою дронів (4× x500_mono_cam) у Gazebo + PX4 SITL: польотна місія, логування позицій та LED-станів, інтерактивний аналіз траєкторій, DTW-вирівнювання та перевірка проходження шляхових точок.

## Структура

```
test/
├── main_drone.py              # Місія лідера (drone 0) — приклад, замініть на власний
├── helper_drones.py           # Місія ведених (drones 1-3) — приклад, замініть на власний
├── logger_node.py             # ROS2-вузол логування (Gazebo pose + LED)
├── run_logged_test.sh         # Скрипт запуску повного тесту
│
├── analysis/
│   ├── analyze_flight.py      # Інтерактивний 3D-плеєр + DTW + метрики
│   ├── convert_mission_to_target.py  # Конвертація mission_XX.json у target waypoints
│   └── config/
│       ├── target_waypoints_times.json      # Еталонні WP (реальний політ)
│
├── logs/
│   ├── flight_flight_20260620_114242.json   # Лог реального польоту
└── results/                     # Збережені метрики (JSON)
```

## Як це працює

1. **`run_logged_test.sh`** запускає `logger_node.py` і польотні скрипти.
2. `main_drone.py` та `helper_drones.py` — це **приклади**. Замініть їх на власні скрипти, які керують дронами через `drone_sdk` або MAVSDK. `run_logged_test.sh` можна налаштувати на запуск будь-яких скриптів замість цих.
3. `logger_node.py` читає позиції з Gazebo через `gz topic -e /world/…/pose/info` та LED-стани через ROS2-топіки; зберігає JSON при завершенні. Логування починається автоматично, коли drone 0 залишає стартову платформу.
4. **`analyze_flight.py`** завантажує лог і будує інтерактивний 3D-графік (+ слайдер часу), виконує DTW-аналіз для ведених дронів, перевіряє проходження шляхових точок і зберігає звіт метрик у `test/results/`.

## Еталонні файли

- `target_waypoints_times.json` — абсолютні ENU-координати 12 WP з реального польоту.
- `convert_mission_to_target.py` — конвертує `mission_XX.json` (Gazebo world coordinates) у формат цільових waypoints для аналізу.

## Використання

```bash
# Повний тест (замініть main_drone.py та helper_drones.py на власні скрипти)
./test/run_logged_test.sh

# Аналіз останнього логу
python3 test/analysis/analyze_flight.py --latest

# Аналіз з перевіркою WP
python3 test/analysis/analyze_flight.py --latest --target test/analysis/config/target_waypoints_times.json

# Конвертація mission JSON у target waypoints
python3 test/analysis/convert_mission_to_target.py resources/scripts/Mission/mission_01.json -o test/analysis/config/my_targets.json
```
