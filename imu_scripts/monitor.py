#!/usr/bin/env python3
import psutil
import time
import csv
import subprocess
import os
import argparse
from datetime import datetime

class SLAMMonitor:
    def __init__(self, output_file="slam_comparison.csv"):
        self.output_file = output_file
        self.header_written = False
        
    def get_process_cpu_memory(self, process_pattern):
        """Получить CPU и память процесса по шаблону имени"""
        total_cpu = 0
        total_memory_mb = 0
        
        for proc in psutil.process_iter(['pid', 'name', 'exe', 'cmdline', 'cpu_percent', 'memory_info']):
            try:
                # Проверяем несколько способов идентификации процесса
                matches = False
                
                # 1. По имени процесса
                if process_pattern.lower() in proc.info['name'].lower():
                    matches = True
                
                # 2. По пути исполняемого файла
                elif proc.info['exe'] and process_pattern in proc.info['exe']:
                    matches = True
                
                # 3. По аргументам командной строки
                elif proc.info['cmdline'] and any(process_pattern in str(arg) for arg in proc.info['cmdline']):
                    matches = True
                
                if matches:
                    # Обновляем CPU (небольшой интервал для точности)
                    p = psutil.Process(proc.info['pid'])
                    p.cpu_percent(interval=0.01)  # Инициализация
                    
                    # Ждем немного для актуальных данных
                    time.sleep(0.05)
                    
                    total_cpu += p.cpu_percent(interval=0.01)
                    total_memory_mb += p.memory_info().rss / 1024 / 1024  # MB
                    
                    # Рекурсивно включаем дочерние процессы
                    for child in p.children(recursive=True):
                        try:
                            child.cpu_percent(interval=0.01)
                            time.sleep(0.05)
                            total_cpu += child.cpu_percent(interval=0.01)
                            total_memory_mb += child.memory_info().rss / 1024 / 1024
                        except:
                            pass
                            
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        return total_cpu, total_memory_mb
    
    def check_ros2_node_active(self, node_name):
        """Проверить активность ROS 2 ноды"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=2
            )
            
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                # Проверяем точное совпадение или частичное
                for n in nodes:
                    if node_name in n:
                        return True
        except:
            pass
        return False
    
    def get_ros2_topic_stats(self, topic_name):
        """Получить статистику по ROS 2 топику"""
        try:
            # Получаем частоту топика
            result = subprocess.run(
                ['ros2', 'topic', 'hz', topic_name, '--window', '3'],
                capture_output=True, text=True, timeout=3
            )
            
            if result.returncode == 0 and 'average rate:' in result.stdout:
                for line in result.stdout.split('\n'):
                    if 'average rate:' in line:
                        rate_str = line.split(':')[1].strip().split()[0]
                        return float(rate_str)
        except:
            pass
        return 0
    
    def get_gpu_metrics(self):
        """Получить метрики GPU"""
        metrics = {}
        try:
            result = subprocess.run([
                'nvidia-smi',
                '--query-gpu=utilization.gpu,memory.used,memory.total,power.draw,temperature.gpu',
                '--format=csv,noheader,nounits'
            ], capture_output=True, text=True, timeout=2)
            
            if result.returncode == 0:
                values = result.stdout.strip().split(', ')
                if len(values) >= 5:
                    metrics = {
                        'gpu_util': float(values[0]),
                        'gpu_mem_used_mb': float(values[1]),
                        'gpu_mem_total_mb': float(values[2]),
                        'gpu_power_w': float(values[3]),
                        'gpu_temp': float(values[4])
                    }
        except:
            pass
        
        # Заполняем нулями если нет данных
        defaults = ['gpu_util', 'gpu_mem_used_mb', 'gpu_mem_total_mb', 'gpu_power_w', 'gpu_temp']
        for key in defaults:
            if key not in metrics:
                metrics[key] = 0
        
        return metrics
    
    def monitor_slam_processes(self, slam_configs, duration=300, interval=2.0):
        """
        Мониторинг различных SLAM процессов
        
        slam_configs: список словарей с конфигурацией:
        [
            {
                'name': 'lio_sam',
                'process_pattern': 'lio_sam',  # для поиска процесса
                'ros_node': '/lio_sam_mapping',  # имя ROS ноды (если есть)
                'key_topics': ['/laser_cloud_surf', '/odometry']  # ключевые топики
            },
            ...
        ]
        """
        start_time = time.time()
        iteration = 0
        
        print("=" * 70)
        print("SLAM PROCESSES MONITOR")
        print("=" * 70)
        print(f"Duration: {duration}s, Interval: {interval}s")
        print(f"Monitoring {len(slam_configs)} SLAM configurations")
        
        for config in slam_configs:
            print(f"  • {config['name']}: process='{config.get('process_pattern', 'N/A')}', "
                  f"node='{config.get('ros_node', 'N/A')}'")
        
        print("=" * 70)
        
        while time.time() - start_time < duration:
            iteration += 1
            current_time = datetime.now().isoformat()
            elapsed = time.time() - start_time
            
            # Базовые системные метрики
            base_metrics = {
                'timestamp': current_time,
                'elapsed_seconds': round(elapsed, 1),
                'iteration': iteration,
                'cpu_total': psutil.cpu_percent(interval=0.1),
                'ram_percent': psutil.virtual_memory().percent,
                'ram_used_gb': psutil.virtual_memory().used / 1024**3,
            }
            
            # GPU метрики
            gpu_metrics = self.get_gpu_metrics()
            
            # Метрики для каждого SLAM процесса
            slam_metrics = {}
            
            for config in slam_configs:
                name = config['name']
                
                # 1. CPU и память процесса
                if 'process_pattern' in config:
                    cpu_usage, mem_mb = self.get_process_cpu_memory(config['process_pattern'])
                    slam_metrics[f'{name}_cpu'] = round(cpu_usage, 1)
                    slam_metrics[f'{name}_mem_mb'] = round(mem_mb, 1)
                
                # 2. Активность ROS ноды
                if 'ros_node' in config:
                    node_active = self.check_ros2_node_active(config['ros_node'])
                    slam_metrics[f'{name}_node_active'] = 1 if node_active else 0
                
                # 3. Частота ключевых топиков
                if 'key_topics' in config:
                    for i, topic in enumerate(config['key_topics'][:3]):  # Ограничимся 3 топиками
                        hz = self.get_ros2_topic_stats(topic)
                        slam_metrics[f'{name}_topic_{i}_hz'] = round(hz, 1)
            
            # Объединяем все метрики
            all_metrics = {**base_metrics, **gpu_metrics, **slam_metrics}
            
            # Сохраняем в CSV
            self.save_to_csv(all_metrics)
            
            # Вывод статуса каждые 10 секунд
            if iteration % max(1, int(10/interval)) == 0:
                self.print_status(all_metrics, elapsed, duration, slam_configs)
            
            # Спим до следующего интервала
            sleep_time = interval - ((time.time() - start_time) % interval)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print(f"\n✅ Мониторинг завершен. Данные в {self.output_file}")
        self.print_summary(slam_configs)
    
    def print_status(self, metrics, elapsed, duration, slam_configs):
        """Вывод текущего статуса"""
        print(f"\n⏱️  [{datetime.now().strftime('%H:%M:%S')}] {int(elapsed)}/{duration} сек")
        print(f"📊 Система: CPU {metrics['cpu_total']:.1f}%, RAM {metrics['ram_percent']:.1f}%")
        
        if metrics['gpu_util'] > 0:
            print(f"🎮 GPU: {metrics['gpu_util']:.1f}% | Память: {metrics['gpu_mem_used_mb']:.0f}MB | "
                  f"Мощность: {metrics['gpu_power_w']:.1f}W")
        
        for config in slam_configs:
            name = config['name']
            cpu_key = f'{name}_cpu'
            mem_key = f'{name}_mem_mb'
            
            if cpu_key in metrics and mem_key in metrics:
                print(f"🔄 {name}: CPU {metrics[cpu_key]:.1f}% | Память {metrics[mem_key]:.0f}MB", end="")
                
                # Добавляем информацию о ноде если есть
                node_key = f'{name}_node_active'
                if node_key in metrics:
                    status = "✅" if metrics[node_key] == 1 else "❌"
                    print(f" | Нода {status}", end="")
                
                # Частота топиков если есть
                for i in range(3):
                    topic_key = f'{name}_topic_{i}_hz'
                    if topic_key in metrics and metrics[topic_key] > 0:
                        if i == 0:
                            print(f" | Топики: ", end="")
                        print(f"{metrics[topic_key]:.1f}Hz ", end="")
                
                print()
    
    def print_summary(self, slam_configs):
        """Вывод итоговой статистики"""
        try:
            import pandas as pd
            df = pd.read_csv(self.output_file)
            
            print("\n" + "=" * 70)
            print("ИТОГОВАЯ СТАТИСТИКА")
            print("=" * 70)
            
            for config in slam_configs:
                name = config['name']
                cpu_key = f'{name}_cpu'
                mem_key = f'{name}_mem_mb'
                
                if cpu_key in df.columns:
                    avg_cpu = df[cpu_key].mean()
                    max_cpu = df[cpu_key].max()
                    avg_mem = df[mem_key].mean() if mem_key in df.columns else 0
                    
                    print(f"\n📈 {name}:")
                    print(f"   CPU: среднее {avg_cpu:.1f}%, максимум {max_cpu:.1f}%")
                    print(f"   Память: среднее {avg_mem:.0f}MB")
            
            print(f"\n📊 Общее:")
            print(f"   Средний CPU системы: {df['cpu_total'].mean():.1f}%")
            print(f"   Средняя память: {df['ram_percent'].mean():.1f}%")
            
            if 'gpu_util' in df.columns:
                print(f"   Средний GPU: {df['gpu_util'].mean():.1f}%")
            
            print(f"\n📁 Полные данные в: {self.output_file}")
            
        except ImportError:
            print("\n⚠️ Для анализа установите pandas: pip install pandas")
        except Exception as e:
            print(f"\n⚠️ Ошибка анализа: {e}")
    
    def save_to_csv(self, data):
        """Сохранить метрики в CSV"""
        file_exists = os.path.exists(self.output_file)
        
        with open(self.output_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=data.keys())
            
            if not file_exists or not self.header_written:
                writer.writeheader()
                self.header_written = True
            
            writer.writerow(data)

def main():
    parser = argparse.ArgumentParser(description='SLAM Processes Monitor')
    parser.add_argument('--duration', type=int, default=300, help='Duration in seconds')
    parser.add_argument('--interval', type=float, default=2.0, help='Monitoring interval')
    parser.add_argument('--output', type=str, default='slam_comparison.csv', help='Output CSV file')
    
    args = parser.parse_args()
    
    # Конфигурация отслеживаемых SLAM процессов
    slam_configs = [
        {
            'name': 'lio_sam',
            'process_pattern': 'lio_sam',
            'ros_node': '/lio_sam_mapping',
            'key_topics': ['/lio_sam/mapping/icp_loop_closure_history_cloud', '/lio_sam/imu/path',
                           '/lio_sam/mapping/odometry_incremental', '/lio_sam/mapping/cloud_registered_raw', 
                           '/lio_sam/mapping/cloud_registered', '/lio_sam/mapping/map_local']
        },
        {
            'name': 'isaac_ros_visual_slam',
            'process_pattern': 'isaac_ros_visual_slam',
            'ros_node': '/visual_slam',
            'key_topics': ['/visual_slam/tracking/odometry', '/visual_slam/vis/observations_cloud']
        },
        {
            'name': 'cartographer',
            'process_pattern': 'cartographer_node',
            'ros_node': '/cartographer_node',
            'key_topics': ['/scan', '/odom', '/map']
        }
    ]
    
    # Настройте под свои нужды:
    # 1. Узнайте точные имена процессов: запустите `ps aux | grep lio_sam`
    # 2. Узнайте точные имена ROS нод: запустите `ros2 node list`
    # 3. Определите ключевые топики для каждого SLAM
    
    monitor = SLAMMonitor(args.output)
    monitor.monitor_slam_processes(
        slam_configs=slam_configs,
        duration=args.duration,
        interval=args.interval
    )

if __name__ == "__main__":
    main()