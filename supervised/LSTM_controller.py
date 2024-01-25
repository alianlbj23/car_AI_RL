# LSTMInference 类请参考之前的回答
from supervised.LSTM_inference import *
import rclpy
from utils.obs_utils import *

class LSTMInferenceController:
    def __init__(self, node, model_path, input_size, hidden_layer_size, output_size, scaler_params, time_steps=3):
        self.node = node
        self.lstm_inference = LSTMInference(
            model_path=model_path,
            input_size=input_size,
            hidden_layer_size=hidden_layer_size,
            output_size=output_size,
            scaler_params=scaler_params,
            time_steps=time_steps
        )

    def run(self):
        while rclpy.ok():
            self.node.reset()
            _, unity_data = wait_for_data(self.node)
            action = self.lstm_inference.infer_action(unity_data)
            self.node.publish_to_unity(action)
