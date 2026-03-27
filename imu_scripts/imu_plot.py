import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.spatial.transform import Rotation as R # Необходим для расчета вращения

# --- Вспомогательная функция расчета траектории ---
def calculate_trajectory(df):
    # Вычисляем дельту времени
    dt_array = np.diff(df['time_data'], prepend=df['time_data'][0])
    
    # Инициализация
    velocity = np.zeros((len(df), 3))
    position = np.zeros((len(df), 3))
    path = 0
    
    # Текущие аккумуляторы
    curr_vel = np.array([0.0, 0.0, 0.0])
    curr_pos = np.array([0.0, 0.0, 0.0])
    r_curr = R.from_quat([0, 0, 0, 1]) # Начальная ориентация (без вращения)

    for i in range(1, len(df)):
        dt = dt_array[i]
        
        # Данные гироскопа (рад/с)
        w = np.array([df['angular_velocity_x'][i], df['angular_velocity_y'][i], df['angular_velocity_z'][i]])
        
        # Обновляем ориентацию (интегрируем гироскоп)
        rot_step = R.from_rotvec(w * dt)
        r_curr = r_curr * rot_step
        
        # Данные акселерометра (м/с^2)
        acc_body = np.array([df['linear_acceleration_x'][i], df['linear_acceleration_y'][i], df['linear_acceleration_z'][i]])
        
        # Переводим ускорение в глобальную систему координат
        acc_global = r_curr.apply(acc_body)
        
        # Вычитаем гравитацию (по оси Z)
        acc_global[2] -= 9.81
        
        # Интегрируем скорость и позицию
        curr_vel += acc_global * dt
        curr_pos += curr_vel * dt
        
        velocity[i] = curr_vel
        position[i] = curr_pos
        if i>1:
            path += np.linalg.norm(curr_pos - position[i-1])
        else:
            path += np.linalg.norm(curr_pos)
        
    return position, path

# --- Основная функция ---
def plot_imu_data():

    parser = argparse.ArgumentParser()
    parser.add_argument('--file_path', type=str, help='TUM data files to plot', default="/home/tms/Projects/black_plat/data/rosbags/imu/imu_data")
    args = parser.parse_args()
    
    file_path = args.file_path

    # Build path relative to the script
    df = pd.read_csv(os.path.join(file_path, 'imu_data.csv'))

    # Создаем сетку графиков 2x2
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams["font.size"] = 25
    fig, ax = plt.subplots(3, figsize=(12, 10))
    fig.suptitle('Зависимость данных IMU от времени для режима HighPerfomaveEDR', fontsize=25)
    
    # 1. Линейное ускорение (Верхний левый)
    ax[0].set_title('Линейное ускорение')
    ax[0].set_xlabel('Время (ч)')
    ax[0].set_ylabel('Ускорение (м/с²)')
    ax[0].plot(df['time_data']/3600, df['linear_acceleration_x'], label='X', color='r')
    ax[0].plot(df['time_data']/3600, df['linear_acceleration_y'], label='Y', color='g')
    ax[0].plot(df['time_data']/3600, df['linear_acceleration_z'], label='Z', color='b')
    ax[0].legend()
    ax[0].grid(True)

    # 2. Угловая скорость (Нижний левый)
    ax[1].set_title('Угловая скорость')
    ax[1].set_xlabel('Время (ч)')
    ax[1].set_ylabel('Скорость (рад/с)')
    ax[1].plot(df['time_data']/3600, df['angular_velocity_x'], label='Roll (X)', color='r')
    ax[1].plot(df['time_data']/3600, df['angular_velocity_y'], label='Pitch (Y)', color='g')
    ax[1].plot(df['time_data']/3600, df['angular_velocity_z'], label='Yaw (Z)', color='b')
    ax[1].legend()
    ax[1].grid(True)

    # 3. Ориентация (Верхний правый)
    # Примечание: Убедитесь, что в CSV есть колонки orientation_x/y/z, иначе код упадет
    # Если их нет, закомментируйте строки plot ниже
    ax[2].set_title('Ориентация')
    ax[2].set_xlabel('Время (ч)')
    ax[2].set_ylabel('Значение')
    ax[2].plot(df['time_data']/3600, df['orientation_x'], label='X', color='r')
    ax[2].plot(df['time_data']/3600, df['orientation_y'], label='Y', color='g')
    ax[2].plot(df['time_data']/3600, df['orientation_z'], label='Z', color='b')
    ax[2].legend()
    ax[2].grid(True)
    
    plt.tight_layout()
    plt.show()

    # ---------------------------------------------------------
    # 4. Положение (Нижний правый) - ИНТЕГРАЦИЯ 3D
    # ---------------------------------------------------------
    
    # Шаг Б: Создаем на его месте (позиция 4 в сетке 2x2) новый 3D график
    ax_3d = plt.subplot(projection='3d')
    
    # Шаг В: Считаем траекторию
    pos, path = calculate_trajectory(df)
    
    # Шаг Г: Рисуем
    ax_3d.set_title('Дрейф позиции (Dead Reckoning)')
    ax_3d.set_xlabel('X (м)')
    ax_3d.set_ylabel('Y (м)')
    ax_3d.set_zlabel('Z (м)')
    
    # Линия пути
    ax_3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], label='Путь', color='purple', linewidth=1)
    
    # Точки старта и конца для наглядности
    ax_3d.scatter(pos[0,0], pos[0,1], pos[0,2], color='green', s=50, label='Старт')
    ax_3d.scatter(pos[-1,0], pos[-1,1], pos[-1,2], color='red', s=50, label='Конец')
    
    ax_3d.legend()
    # ---------------------------------------------------------

    # Автоматическая компоновка
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
    print(path)

if __name__ == '__main__':
    plot_imu_data()