import serial
from datetime import datetime, timedelta
import csv
import sys

def capture_serial_data(port, baudrate=115200, duration=60):
    """

    从串口捕获指定时长的IMU数据（适配浮点数格式）
    :param port: 串口设备路径 (如 '/dev/ttyUSB0')
    :param baudrate: 波特率 (默认115200)
    :param duration: 采集时长（秒）
    :return: (原始数据, 过滤后数据, 四元数数据, 无效行数)
    """
    raw_data = []
    filtered_data = []
    quat_data = []
    invalid_count = 0
    buffer = bytearray()
    
    try:
        # 初始化串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1  # 读取超时1秒
        )
        print(f"Connected to {ser.name}, starting data acquisition...")

        # 计算结束时间
        end_time = datetime.now() + timedelta(seconds=duration)
        
        while datetime.now() < end_time:
            # 读取串口数据
            data = ser.read_all()
            if data:
                buffer.extend(data)
                
                # 按行分割处理
                while b'\n' in buffer:
                    line_end = buffer.index(b'\n')
                    try:
                        # 解码处理
                        line = buffer[:line_end].decode("utf-8").strip()
                    except UnicodeDecodeError:
                        # 备用解码方案
                        line = buffer[:line_end].decode("ISO-8859-1").strip()
                    finally:
                        del buffer[:line_end+1]  # 删除已处理数据
                    
                    # 处理四元数数据
                    if line.startswith("Quaternion:"):
                        try:
                            parts = line.replace("Quaternion: ", "").split(', ')
                            w = float(parts[0].split('=')[1])
                            x = float(parts[1].split('=')[1])
                            y = float(parts[2].split('=')[1])
                            z = float(parts[3].split('=')[1])
                            quat_data.append([w, x, y, z])
                        except Exception as e:
                            invalid_count += 1
                            print(f"Quaternion parse error: {e}")
                    
                    # 处理原始数据和过滤数据
                    elif line.count(',') == 6:
                        try:
                            # 原始传感器数据（加速度、陀螺仪、温度）
                            parts = line.split(',')
                            accel = [float(parts[0]), float(parts[1]), float(parts[2])]
                            gyro = [float(parts[3]), float(parts[4]), float(parts[5])]
                            temp = int(parts[6])
                            
                            # 保存原始数据
                            raw_data.append(accel + gyro + [temp])
                            
                            # 处理过滤数据（假设此数据格式是相同的）
                            filtered_data.append(accel + gyro + [temp])
                        except (ValueError, IndexError) as e:
                            invalid_count += 1
                            print(f"Data parse error: {e}")
                    else:
                        invalid_count += 1

        print("\nData acquisition completed")
        return raw_data, filtered_data, quat_data, invalid_count

    except serial.SerialException as e:
        print(f"Serial port error: {str(e)}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

def save_imu_data(raw_data, filtered_data, filename):
    """保存原始和过滤后的IMU数据到CSV文件"""
    headers = [
        "Accel_X_raw", "Accel_Y_raw", "Accel_Z_raw",
        "Gyro_X_raw", "Gyro_Y_raw", "Gyro_Z_raw",
        "Temp_raw",
        "Accel_X_filt", "Accel_Y_filt", "Accel_Z_filt",
        "Gyro_X_filt", "Gyro_Y_filt", "Gyro_Z_filt",
        "Temp_filt"
    ]
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        min_len = min(len(raw_data), len(filtered_data))
        for i in range(min_len):
            row = raw_data[i] + filtered_data[i]
            formatted = [
                *raw_data[i][:3],     # 原始加速度
                *raw_data[i][3:6],    # 原始陀螺仪
                raw_data[i][6],       # 原始温度
                *map(lambda x: f"{x:.2f}", filtered_data[i][:3]),  # 过滤加速度
                *map(lambda x: f"{x:.2f}", filtered_data[i][3:6]), # 过滤陀螺仪
                filtered_data[i][6]   # 过滤温度
            ]
            writer.writerow(formatted)

def save_quaternions(quat_data, filename):
    """保存四元数数据到CSV文件"""
    headers = ["w", "x", "y", "z"]
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for q in quat_data:
            writer.writerow([f"{v:.3f}" for v in q])

if __name__ == "__main__":
    # 配置参数
    SERIAL_PORT = "/dev/ttyUSB0"  # 根据实际设备修改
    BAUD_RATE = 115200            # 保持与设备一致
    DURATION = 60                 # 采集时长（秒）

    # 执行数据采集
    raw_data, filtered_data, quat_data, invalid_lines = capture_serial_data(SERIAL_PORT, BAUD_RATE, DURATION)
    
    # 保存数据到CSV文件
    save_imu_data(raw_data, filtered_data, "sensor_data.csv")
    save_quaternions(quat_data, "quaternions.csv")
    
    # 输出统计信息
    print("\n[Acquisition Report]")
    print(f"Raw samples: {len(raw_data)}")
    print(f"Filtered samples: {len(filtered_data)}")
    print(f"Quaternion samples: {len(quat_data)}")
    print(f"Invalid lines: {invalid_lines}")
    print(f"Effective rate: {min(len(raw_data), len(filtered_data), len(quat_data))/DURATION:.1f} Hz")

    print("\nGenerated files:")
    print("- sensor_data.csv : 原始传感器数据 + 过滤后数据")
    print("- quaternions.csv : 四元数数据")
