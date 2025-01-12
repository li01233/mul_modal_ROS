import numpy as np
import matplotlib.pyplot as plt

# 参数设置
c = 3e8  # 光速 (m/s)
f = 10e9  # 雷达频率 (Hz)
lambda_r = c / f  # 波长 (m)
d = 0.5  # 两天线间距 (m)
theta_true = 30  # 目标真实入射角 (度)
theta_true_rad = np.degrees(theta_true)

# 计算相位差
delta_phi = (2 * np.pi * d * np.sin(np.radians(theta_true))) / lambda_r

# 生成模拟回波信号
N = 1024  # 信号长度
t = np.linspace(0, 1e-6, N)  # 时间轴 (秒)
signal1 = np.cos(2 * np.pi * f * t)  # 天线1接收信号
signal2 = np.cos(2 * np.pi * f * t + delta_phi)  # 天线2接收信号

# 快速傅里叶变换 (FFT) 以提取相位信息
fft1 = np.fft.fft(signal1)
fft2 = np.fft.fft(signal2)

# 提取主瓣频率的相位
peak_freq = np.argmax(np.abs(fft1))
phase1 = np.angle(fft1[peak_freq])
phase2 = np.angle(fft2[peak_freq])

# 计算相位差
measured_delta_phi = phase2 - phase1
# 确保相位差在[-pi, pi]范围内
measured_delta_phi = np.mod(measured_delta_phi + np.pi, 2 * np.pi) - np.pi

# 反推出入射角
theta_measured = np.degrees(np.arcsin((measured_delta_phi * lambda_r) / (2 * np.pi * d)))

# 输出结果
print(f"目标真实入射角: {theta_true}°")
print(f"测量得到的入射角: {theta_measured:.2f}°")

# 绘图
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(t, signal1, label='天线1接收信号')
plt.plot(t, signal2, label='天线2接收信号')
plt.title('天线接收的回波信号')
plt.xlabel('时间 (秒)')
plt.ylabel('幅度')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(np.abs(fft1), label='天线1 FFT')
plt.plot(np.abs(fft2), label='天线2 FFT')
plt.title('接收信号的频谱')
plt.xlabel('频率索引')
plt.ylabel('幅度')
plt.legend()

plt.tight_layout()
plt.show()
