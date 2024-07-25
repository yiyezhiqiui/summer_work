import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pykalman import KalmanFilter
from mpl_toolkits.mplot3d import Axes3D

# 读取 CSV 文件
data = pd.read_csv('./source/true_states.csv')

# 提取数据
observations = data[['x', 'y', 'z']].values 

# 状态矩阵
transition_matrix = np.array([[1,0,0,1,0,0],
                              [0,1,0,0,1,0],
                              [0,0,1,0,0,1],
                              [0,0,0,1,0,0],
                              [0,0,0,0,1,0],
                              [0,0,0,0,0,1]])
# 观测矩阵
observation_matrix = np.eye(3,6)
# 状态转移的协方差矩阵
transition_covariance = 0.1 * np.eye(6)
# 观测值的协方差
observation_covariance = np.eye(3)*1

# 初始状态
initial_state_mean = [observations[0,0],observations[0,1],observations[0,2],
                      observations[1,0]-observations[0,0],observations[1,1]-observations[0,1],observations[1,2]-observations[1,2]]
initial_state_covariance = np.ones((6, 6))

# 实例化 KalmanFilter
kf = KalmanFilter(
    transition_matrices=transition_matrix,
    observation_matrices=observation_matrix,
    transition_covariance=transition_covariance,
    observation_covariance=observation_covariance,
    initial_state_mean=initial_state_mean,
    initial_state_covariance=initial_state_covariance
)

# 使用 KalmanFilter 进行拟合
filtered_state_means, _ = kf.filter(observations)

#三维画图

# 绘制原始数据和滤波后的估计值
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(observations[:, 0], observations[:, 1], observations[:, 2], c='r', marker='o', label='measures data')
ax.plot(filtered_state_means[:, 0], filtered_state_means[:, 1], filtered_state_means[:, 2],'b-' ,linewidth=4, label='kalman filtered data')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Raw Data and Filtered Data in 3D Space')
ax.legend()

plt.show()

