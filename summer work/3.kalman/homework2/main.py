import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pykalman import KalmanFilter

# 读取 CSV 文件
data = pd.read_csv('./source/stock_prices.csv')

# 提取数据
x = data['Day'].values
y = data['Price'].values

# 状态矩阵
transition_matrix = np.array([[1, 1], [0, 1]])
# 观测矩阵
observation_matrix = np.array([[1, 0]])
# 状态转移的协方差矩阵
transition_covariance = 0.1 * np.eye(2)
# 观测值的协方差
observation_covariance = 1

# 初始状态
initial_state_mean = [y[0], 0]
initial_state_covariance = np.ones((2, 2))

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
filtered_state_means1, _ = kf.filter(y)

# 绘制拟合结果
plt.figure()
plt.plot(x, y, 'o', label='Actual Prices')
plt.plot(x, filtered_state_means1[:, 0], label='Predicted Prices (Kalman Filter)')
plt.title('Data 1')
plt.xlabel('Day')
plt.ylabel('Price')
plt.legend()
plt.show()
