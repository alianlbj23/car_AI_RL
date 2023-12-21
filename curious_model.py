import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

class PredictionModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        """
        state_dim: 状态向量的维度
        action_dim: 动作空间的维度
        hidden_dim: 隐藏层的维度
        """
        super(PredictionModel, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, state_dim)

    def forward(self, state, action):
        state = state.unsqueeze(0) if state.dim() == 1 else state

        if action.dim() == 0 or (action.dim() == 1 and action.size(0) == 1):
            # 如果 action 是标量或者形状为 [1] 的一维张量
            action = action.long().unsqueeze(0)
        else:
            # 如果 action 已经是形状正确的张量
            action = action.long()

        # 确保 action 是一个零维张量（标量）
        if not torch.is_tensor(action):
            action = torch.tensor([action], dtype=torch.int64)
        else:
            action = action.type(torch.int64)

        state = state.unsqueeze(0) if state.dim() == 1 else state
        # one-hot 编码动作
        action_one_hot = F.one_hot(action.to(torch.int64), num_classes=4)
        action_one_hot = action_one_hot.to(torch.float32).squeeze(1)

        print("state : ", state)
        # 合并状态和动作
        x = torch.cat([state, action_one_hot], dim=1)

        # 神经网络的其余部分
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        predicted_state = self.fc3(x)

        return predicted_state


def train_model(model, training_data, optimizer):
    model.train()
    for state, action, next_state in training_data:
        # 确保 state, action, next_state 是 torch Tensors
        state = torch.tensor(state, dtype=torch.float32)
        action = torch.tensor([action], dtype=torch.int64)
        action = F.one_hot(action, num_classes=4).to(torch.float32).squeeze(0)
        next_state = torch.tensor(next_state, dtype=torch.float32)
        
        predicted_next_state = model(state, action)
        loss = F.mse_loss(predicted_next_state, next_state)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

def calculate_curiosity_reward(model, current_state, action, next_state):
    # 确保数据是正确的 Tensor 类型
    if not isinstance(current_state, torch.Tensor):
        current_state = torch.tensor(current_state, dtype=torch.float32)
    if not isinstance(action, torch.Tensor):
        # 对动作进行 one-hot 编码以匹配模型的预期输入
        action = torch.tensor([action], dtype=torch.int64)
        action = F.one_hot(action, num_classes=4).to(torch.float32).squeeze(0)
    if not isinstance(next_state, torch.Tensor):
        next_state = torch.tensor(next_state, dtype=torch.float32)

    predicted_next_state = model(current_state, action)
    next_state = next_state.unsqueeze(0) if next_state.dim() == 1 else next_state
    prediction_error = F.mse_loss(predicted_next_state, next_state)
    return prediction_error.item()  # 返回预测误差作为奖励
