import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import numpy as np
import os
from torch import nn, tensor
import torch
from LSTM_model import LSTMModel
import ast

# 加载CSV文件
def load_csv(file_path):
    df = pd.read_csv(file_path)
    return df

# 将字符串转换为浮点数列表
def convert_string_to_floats(string):
    try:
        return ast.literal_eval(string)
    except:
        return None

# 数据预处理
def preprocess_data(df):
    for column in df.columns:
        if df[column].dtype == object:
            df[column] = df[column].apply(convert_string_to_floats)
            if isinstance(df[column].iloc[0], (list, tuple)):
                col_expansion = pd.DataFrame(df[column].tolist(), index=df.index)
                col_expansion.columns = [f"{column}_{i}" for i in range(len(col_expansion.columns))]
                df = df.drop(column, axis=1).join(col_expansion)
            else:
                df[column] = pd.to_numeric(df[column], errors='coerce').fillna(0)
    return df

# 创建组合数据集
def create_combined_dataset(folder_path, n_time_steps=3):
    all_files = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.csv')]
    combined_data = pd.DataFrame()

    for file in all_files:
        data = load_csv(file)
        data = preprocess_data(data)

        # 丢弃每个文件末尾的n_time_steps行
        if len(data) > n_time_steps:
            data = data[:-n_time_steps]

        combined_data = pd.concat([combined_data, data], ignore_index=True)

    combined_data_scaled = scaler.fit_transform(combined_data)
    X, Y = create_dataset(combined_data_scaled, n_time_steps)
    return X, Y



# 创建数据集
def create_dataset(data, n_time_steps=3):
    X, Y = [], []
    for i in range(len(data) - n_time_steps):
        X.append(data[i:(i + n_time_steps), :-1])
        Y.append(data[i + n_time_steps, -1])
    return np.array(X), np.array(Y)

# 训练模型
def train_model(model, X_train, Y_train, epochs=10, lr=0.001, model_save_path='model.pth'):
    loss_function = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    for i in range(epochs):
        for j in range(len(X_train)):
            optimizer.zero_grad()
            model.hidden_cell = (torch.zeros(1, 1, model.hidden_layer_size),
                                torch.zeros(1, 1, model.hidden_layer_size))

            # 确保输入是Tensor
            x_tensor = torch.tensor(X_train[j], dtype=torch.float32)
            y_tensor = torch.tensor(Y_train[j], dtype=torch.float32).unsqueeze(0)  # 添加维度以匹配尺寸

            y_pred = model(x_tensor)
            single_loss = loss_function(y_pred, y_tensor)
            single_loss.backward()
            optimizer.step()

            if i % 1 == 0:
                print(f'epoch: {i:3} loss: {single_loss.item():10.8f}')
    torch.save(model.state_dict(), model_save_path)


# 主程序
folder_path = '../training_data'
scaler = StandardScaler()

X, Y = create_combined_dataset(folder_path)
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)

input_size = X_train.shape[2]
hidden_layer_size = 100
output_size = 1
model = LSTMModel(input_size, hidden_layer_size, output_size)

train_model(model, X_train, Y_train, model_save_path='final_model.pth')

# 测试模型的代码可以根据需要添加
