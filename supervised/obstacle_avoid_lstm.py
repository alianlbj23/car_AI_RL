import pandas as pd
from sklearn.preprocessing import StandardScaler
import numpy as np
import os
from torch import nn, tensor
import torch
from LSTM_model import LSTMModel

def load_csv(file_path):
    df = pd.read_csv(file_path)
    return df

def create_dataset(data, n_time_steps=3):
    X, Y = [], []
    for i in range(len(data) - n_time_steps):
        X.append(data[i:(i + n_time_steps), :-1])
        Y.append(data[i + n_time_steps, -1])
    return np.array(X), np.array(Y)

def train_model(model, X_train, Y_train, epochs=10, lr=0.001):
    loss_function = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    for i in range(epochs):
        for j in range(len(X_train)):
            optimizer.zero_grad()
            model.hidden_cell = (torch.zeros(1, 1, model.hidden_layer_size),
                                 torch.zeros(1, 1, model.hidden_layer_size))

            y_pred = model(tensor(X_train[j], dtype=torch.float32))
            single_loss = loss_function(y_pred, tensor(Y_train[j], dtype=torch.float32))
            single_loss.backward()
            optimizer.step()

        if i % 1 == 0:
            print(f'epoch: {i:3} loss: {single_loss.item():10.8f}')

folder_path = '../training_data'
all_files = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.csv')]

scaler = StandardScaler()
input_size = hidden_layer_size = output_size = None

for file in all_files:
    data = load_csv(file)
    data_scaled = scaler.fit_transform(data)

    X, Y = create_dataset(data_scaled)
    X, Y = tensor(X, dtype=torch.float32), tensor(Y, dtype=torch.float32)

    if input_size is None:
        input_size = X.shape[2]
        hidden_layer_size = 100
        output_size = 1
        model = LSTMModel(input_size, hidden_layer_size, output_size)

    train_model(model, X, Y)

# 用于测试数据的函数和过程可以根据需要添加
