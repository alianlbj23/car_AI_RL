import torch
import pandas as pd
import numpy as np
from sklearn.preprocessing import StandardScaler
from supervised.LSTM_model import LSTMModel

class LSTMInference:
    def __init__(self, model_path, input_size, hidden_layer_size, output_size, scaler_params):
        self.model = self.load_model(model_path, input_size, hidden_layer_size, output_size)
        self.scaler = StandardScaler()
        self.scaler.mean_, self.scaler.scale_ = scaler_params

    def trans_to_float(self, list_str):
        return [float(i) for i in list_str]

    def round_to_decimal_places(self, value, decimal_places=2):
        return round(value, decimal_places)

    def preprocess_state(self, state_dict):
        df = pd.DataFrame([state_dict])
        df = df.applymap(lambda x: x if isinstance(x, list) else [x])
        df = df.apply(pd.Series.explode)
        return df

    def load_model(self, model_path, input_size, hidden_layer_size, output_size):
        model = LSTMModel(input_size, hidden_layer_size, output_size)
        model.load_state_dict(torch.load(model_path))
        model.eval()
        return model

    def infer_action(self, state_dict):
        df = self.preprocess_state(state_dict)
        data_scaled = self.scaler.transform(df)
        data_tensor = torch.tensor(data_scaled, dtype=torch.float32)

        data_tensor = data_tensor.repeat(3, 1)  # Assuming model needs 3 time steps

        with torch.no_grad():
            action = self.model(data_tensor).numpy().flatten()
        return action
