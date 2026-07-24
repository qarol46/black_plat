[***ВЕРНУТЬСЯ***](/body/ros2/bluespace_ai_xsens_ros_mti_driver/README.md)

# Allan Variance ROS
## ROS-пакет, который загружает rosbag с данными IMU и вычисляет параметры дисперсии Аллана

## Оглавление

- [Цель](#цель)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Kalibr](#kalibr)
- [Автор](#автор)

---

## [Цель](#оглавление)
Цель этого инструмента — прочитать длинную последовательность данных IMU и вычислить Angle Random Walk (ARW, случайное блуждание угла), Bias Instability (нестабильность смещения нуля) и Gyro Random Walk (случайное блуждание гироскопа) для гироскопа, а также Velocity Random Walk (VRW, случайное блуждание скорости), Bias Instability и Accel Random Walk (случайное блуждание акселерометра) для акселерометра.

Хотя существует множество open-source инструментов с аналогичной функциональностью, этот пакет обладает следующими особенностями:

- Полностью совместим с ROS. Достаточно записать `rosbag` и передать его как входные данные. Конвертация не требуется.
- Написан на C++ с использованием `rosbag::View`, что позволяет обрабатывать rosbag с максимальной скоростью. Проигрывать (play back) файл bag не нужно.
- Разработан для [Kalibr](https://github.com/ethz-asl/kalibr). Генерирует файл `imu.yaml`.

Этот инструмент рассчитан на запуск внутри контейнеров, т.к. оригинальный пакет написан для ROS 1.

## [Сборка](#оглавление)

Папка allan_variance_ros должна находиться в папке src вашего catkin workspace.
Например: ~/catkin_ws/src/allan_variance_ros

Из директории catkin workspace (например, ~/catkin_ws) выполните:
``catkin build allan_variance_ros``
``source devel/setup.bash``

## [Запуск](#оглавление)

### Запуск в Docker

Собрать образ:
```bash
docker build -t allan_variance ./.devcontainer 
```
Запустить контейнер:
```bash
docker run -it --rm   --env="DISPLAY"   --volume="$HOME/.Xauthority:/root/.Xauthority:rw"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --volume="$(pwd):/home/local/catkin_ws/src/allan_variance"   alanvariance bash
```


1. Убедитесь, что запущен rosmaster — выполните ``roscore`` в терминале.

2. Разместите IMU на демпфированной (виброизолированной) поверхности и запишите данные с IMU в rosbag. Необходимо записать **как минимум** 3 часа данных. Чем длиннее последовательность, тем точнее результат.

3. **Рекомендуется.** Переупорядочить сообщения ROS по временной метке:

  ``rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag``

4. Запустите вычисление вариации Аллана (примеры конфигурационных файлов прилагаются):

  ``rosrun allan_variance_ros allan_variance [путь_к_папке_с_bag] [путь_к_конфигурационному_файлу]``

  Этот шаг вычисляет отклонение Аллана (Allan Deviation) для IMU и генерирует CSV-файл `allan_variance.csv`.

5. Следующий шаг — визуализировать графики и получить параметры IMU. Для этого используется скрипт `analysis.py`, который читает уже готовый CSV-файл из предыдущего шага:

  ``rosrun allan_variance_ros analysis.py --data allan_variance.csv``

  Если у вас есть конфигурационный файл для задания топика и частоты обновления, его тоже можно передать:

  ``rosrun allan_variance_ros analysis.py --data allan_variance.csv --config config/realsense_d425i.yaml``

  Дополнительные параметры скрипта:
  - `--skip N` — использовать только каждую N-ю строку данных (прореживание), по умолчанию 1 (использовать все строки).
  - `--output ИМЯ_ФАЙЛА` — имя выходного yaml-файла, по умолчанию `imu.yaml`.

  Скрипт последовательно открывает два окна с графиками — сначала для акселерометра, затем для гироскопа. Чтобы перейти к следующему графику, закройте текущее окно matplotlib.

  По завершении работы скрипт сохраняет:
  - `imu.yaml` (или имя, указанное в `--output`) — параметры шумов IMU в формате Kalibr, включая параметры ARW/VRW и Bias Instability для каждой оси;
  - `acceleration.png` — график отклонения Аллана для акселерометра;
  - `gyro.png` — график отклонения Аллана для гироскопа.

  В терминал при этом выводятся только две строки:
  ```
  Writing Kalibr imu.yaml file.
  Make sure to update the rostopic and rate in the file if a config file was not provided.
  ```

## [Kalibr](#оглавление)

[Kalibr](https://github.com/ethz-asl/kalibr) — полезный набор инструментов для калибровки камер и IMU. Для калибровки IMU ему нужны параметры шума IMU, сгенерированные в yaml-файле. `allan_variance_ros` автоматически создаёт этот файл как `imu.yaml`:

```
#Accelerometer
accelerometer_noise_density: 0.006308226052016165 
accelerometer_random_walk: 0.00011673723527962174 

#Gyroscope
gyroscope_noise_density: 0.00015198973532354657 
gyroscope_random_walk: 2.664506559330434e-06 

rostopic: '/sensors/imu' #Make sure this is correct
update_rate: 400.0 #Make sure this is correct

```

## [Автор](#оглавление)

[Russell Buchanan](https://www.ripl-lab.com/)

[***ВЕРНУТЬСЯ***](/body/ros2/bluespace_ai_xsens_ros_mti_driver/README.md)