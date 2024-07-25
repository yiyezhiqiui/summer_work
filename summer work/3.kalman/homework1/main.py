import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pykalman import KalmanFilter

# 数据文件路径，假设第一列是 x，第二列是 y
data1 = pd.read_csv('./source/homework_data_4.txt', sep=' ', header=None)
data1.columns = ['x', 'y']
x = data1['x'].values
y = data1['y'].values

# 状态矩阵
transition_matrix = np.array([[1, 0], [0, 1]])
# 观测矩阵
observation_matrix = np.array([[1, 0]])
# 状态转移的协方差矩阵
transition_covariance = 0.001 * np.eye(2)
# 观测值的协方差
observation_covariance = 0.1

# 初始状态
initial_state_mean_1 = [y[0], 0]
initial_state_covariance_1 = np.eye(2) * 0.1

# 实例化 KalmanFilter
kf_1 = KalmanFilter(
    transition_matrices=transition_matrix,
    observation_matrices=observation_matrix,
    transition_covariance=transition_covariance,
    observation_covariance=observation_covariance,
    initial_state_mean=initial_state_mean_1,
    initial_state_covariance=initial_state_covariance_1
)

# 使用 KalmanFilter 进行拟合
filtered_state_means1, _ = kf_1.filter(y)

# 绘制拟合结果
plt.figure()
plt.plot(x, y, 'o', label='Actual Prices')
plt.plot(x, filtered_state_means1[:, 0], label='Predicted Prices (Kalman Filter)')
plt.title('Data 1')
plt.legend()
plt.show()
