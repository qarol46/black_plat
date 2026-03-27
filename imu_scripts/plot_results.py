# Создайте скрипт plot_results.py
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('slam_metrics.csv')

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

# График 1: CPU и GPU
axes[0,0].plot(df['cpu_total'], label='CPU %')
if 'gpu_util' in df.columns:
    axes[0,0].plot(df['gpu_util'], label='GPU %')
axes[0,0].set_title('Загрузка CPU/GPU')
axes[0,0].legend()

# График 2: Мощность
power_cols = [col for col in df.columns if 'power' in col]
for col in power_cols:
    if col in df.columns and df[col].max() > 0:
        axes[0,1].plot(df[col], label=col)
axes[0,1].set_title('Энергопотребление')
axes[0,1].legend()

# График 3: Память
axes[1,0].plot(df['ram_percent'], label='RAM %')
if 'gpu_mem_used' in df.columns:
    axes[1,0].plot(df['gpu_mem_used']/df['gpu_mem_total'].iloc[0]*100, label='GPU память %')
axes[1,0].set_title('Использование памяти')
axes[1,0].legend()

# График 4: Температура
temp_cols = [col for col in df.columns if 'temp' in col]
for col in temp_cols:
    if col in df.columns:
        axes[1,1].plot(df[col], label=col)
axes[1,1].set_title('Температура')
axes[1,1].legend()

plt.tight_layout()
plt.savefig('slam_metrics.png', dpi=150)
plt.show()