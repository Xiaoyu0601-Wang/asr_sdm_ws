import sys
import csv
import json
import matplotlib.pyplot as plt
from collections import defaultdict
from tabulate import tabulate

# 修改字体配置（移除中文字体依赖）
plt.rcParams.update({
    'font.sans-serif': ['DejaVu Sans'],  # 使用广泛支持的字体
    'axes.unicode_minus': False,
    'figure.dpi': 150
})

def load_calibration(calib_file):
    """加载校准参数文件"""
    try:
        with open(calib_file) as f:
            return json.load(f)
    except:
        print(f"Warning: No calibration file found at {calib_file}")
        return {
            'accel_bias': [0.0, 0.0, 0.0],
            'accel_scale': [1.0, 1.0, 1.0],
            'gyro_bias': [0.0, 0.0, 0.0]
        }

def process_imu_data(filename, calib_file='calibration.json'):
    calib = load_calibration(calib_file)
    time_stamps = []
    accel = [[] for _ in range(3)]
    gyro = [[] for _ in range(3)]
    temperature = []
    accel_filt = [[] for _ in range(3)]
    gyro_filt = [[] for _ in range(3)]
    temp_filt = []
    temp_windows = defaultdict(list)
    valid_line_count = 0
    
    with open(filename, 'r') as f:
        csv_reader = csv.reader(f)
        next(csv_reader)  # 跳过标题行
        
        for line_num, parts in enumerate(csv_reader, 1):
            if len(parts) != 14:
                print(f"Skipped invalid line {line_num}: Incorrect column count")
                continue
            
            try:
                # 转换前6列为浮点数，最后一列为整数
                accel_raw = list(map(float, parts[0:3]))
                gyro_raw = list(map(float, parts[3:6]))
                temp = int(parts[6])
                accel_filt_raw = list(map(float, parts[7:10]))
                gyro_filt_raw = list(map(float, parts[10:13]))
                temp_filt_raw = int(parts[13])
                
                # 应用校准
                accel_cal = [(accel_raw[i] - calib['accel_bias'][i]) * calib['accel_scale'][i] for i in range(3)]
                gyro_cal = [gyro_raw[i] - calib['gyro_bias'][i] for i in range(3)]
                
                # 时间戳
                t = valid_line_count * 0.005
                time_stamps.append(t)
                valid_line_count += 1
                
                # 存储数据
                for i in range(3):
                    accel[i].append(accel_cal[i])
                    gyro[i].append(gyro_cal[i])
                    accel_filt[i].append(accel_filt_raw[i])
                    gyro_filt[i].append(gyro_filt_raw[i])
                temperature.append(temp)
                temp_filt.append(temp_filt_raw)
                
                # 温度窗口处理
                window_idx = int(t * 10)
                temp_windows[window_idx].append(temp)
                
            except ValueError as e:
                print(f"Skipped invalid line {line_num}: {str(e)}")
                continue

    # 计算统计量
    def calc_stats(data, unit):
        if not all(len(ch) > 0 for ch in data):  # 检查每个数据列是否为空
            return {'bias': [0.0, 0.0, 0.0], 'std': [0.0, 0.0, 0.0], 'unit': unit}
        
        means = [sum(ch)/len(ch) for ch in data]
        stds = [(sum((x-mean)**2 for x in ch)/len(ch))**0.5 for ch, mean in zip(data, means)]
        return {'bias': means, 'std': stds, 'unit': unit}
    
    # 处理温度平均值
    temp_avg = []
    window_centers = []
    for idx in sorted(temp_windows.keys()):
        if temp_windows[idx]:
            window_centers.append(idx*0.1 + 0.05)
            temp_avg.append(sum(temp_windows[idx])/len(temp_windows[idx]))
    return {
        'accel': calc_stats(accel, 'g'),
        'gyro': calc_stats(gyro, 'dps'),
        'temp_raw': (time_stamps, temperature),
        'accel_raw': (time_stamps, accel),
        'gyro_raw': (time_stamps, gyro),
        'accel_filt': (time_stamps, accel_filt),
        'gyro_filt': (time_stamps, gyro_filt),
        'temp_filt': (time_stamps, temp_filt),
        'temp_avg': (window_centers, temp_avg)
    }

def plot_sensor_data(data, filename, sensor_name, filtered=False):
    """绘制传感器数据（英文标签）"""
    plt.figure(figsize=(12, 8))
    
    time_stamps, sensor_data, stats = data
    axes = ['X', 'Y', 'Z']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
    
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(time_stamps, sensor_data[i], 
                color=colors[i], linewidth=0.5, label='Filtered Data' if filtered else 'Raw Data')
        
        # 绘制统计参考线
        plt.axhline(0, color='red', linestyle='--', linewidth=1, label='Zero Reference')
        plt.axhspan(-3*stats['std'][i], 3*stats['std'][i], 
                   color='orange', alpha=0.2, label='±3σ Range')
        plt.ylabel(f'{axes[i]} Axis ({stats["unit"]})')
        plt.grid(True, alpha=0.3)
        plt.legend(loc='upper right', fontsize=8)
        
        if i == 0:
            plt.title(f'{sensor_name} Data\n(Sampling Rate: 200Hz)')
    
    plt.xlabel('Time (seconds)')
    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight')
    plt.close()

def plot_temperature(data, filename):
    """温度数据可视化（英文标签）"""
    plt.figure(figsize=(12, 6))
    
    # 原始数据
    plt.plot(data['temp_raw'][0], data['temp_raw'][1],
            alpha=0.3, linewidth=0.5, label='Raw Data')
    
    # 滑动平均
    plt.scatter(data['temp_avg'][0], data['temp_avg'][1],
               color='red', s=20, zorder=3, label='100ms Average')
    
    # 滤波后的温度数据
    plt.plot(data['temp_filt'][0], data['temp_filt'][1], 
             color='green', alpha=0.7, linewidth=0.8, label='Filtered Data')
    
    plt.title('Temperature Sensor Data')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Temperature (Raw Units)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight')
    plt.close()

def main():
    if len(sys.argv) != 2:
        print("Usage: python process_imu.py <data_file>")
        return

    input_file = sys.argv[1]
    base_name = input_file.rsplit('.', 1)[0]
    results = process_imu_data(input_file)

    # 生成图表文件
    plot_files = {
        'temperature': f"{base_name}_temperature.png",
        'acceleration_raw': f"{base_name}_accel_raw.png",
        'gyroscope_raw': f"{base_name}_gyro_raw.png",
        'acceleration_filtered': f"{base_name}_accel_filtered.png",
        'gyroscope_filtered': f"{base_name}_gyro_filtered.png"
    }
    
    plot_temperature(results, plot_files['temperature'])
    plot_sensor_data(
        (results['accel_raw'][0], results['accel_raw'][1], results['accel']),
        plot_files['acceleration_raw'], 
        "Accelerometer", filtered=False
    )
    plot_sensor_data(
        (results['gyro_raw'][0], results['gyro_raw'][1], results['gyro']),
        plot_files['gyroscope_raw'], 
        "Gyroscope", filtered=False
    )
    plot_sensor_data(
        (results['accel_filt'][0], results['accel_filt'][1], results['accel']),
        plot_files['acceleration_filtered'], 
        "Accelerometer", filtered=True
    )
    plot_sensor_data(
        (results['gyro_filt'][0], results['gyro_filt'][1], results['gyro']),
        plot_files['gyroscope_filtered'], 
        "Gyroscope", filtered=True
    )

    # 保存统计结果
    def save_stats(filename, stats):
        headers = ["Axis", f"Bias ({stats['unit']})", f"Std ({stats['unit']})"]
        rows = [[f"{axis}", f"{bias:.4f}", f"{std:.4f}"] 
               for axis, bias, std in zip("XYZ", stats['bias'], stats['std'])]
        with open(filename, 'w') as f:
            csv.writer(f).writerows([headers] + rows)
    
    save_stats(f"{base_name}_accel_stats.csv", results['accel'])
    save_stats(f"{base_name}_gyro_stats.csv", results['gyro'])

    # 终端输出
    print("\n=== Sensor Statistics ===")
    def print_stats(name, stats):
        print(f"\n{name}:")
        print(tabulate(
            [[f"{a}-Axis", f"{b:.4f} ± {s:.4f} {stats['unit']}"] 
             for a, b, s in zip("XYZ", stats['bias'], stats['std'])],
            headers=["Axis", "Bias (Mean ± Std)"],
            tablefmt="fancy_grid"
        ))
    
    print_stats("Accelerometer", results['accel'])
    print_stats("Gyroscope", results['gyro'])

    print("\nGenerated Files:")
    for f in plot_files.values():
        print(f" - {f}")
    print(f" - {base_name}_accel_stats.csv")
    print(f" - {base_name}_gyro_stats.csv")

if __name__ == "__main__":
    main()
