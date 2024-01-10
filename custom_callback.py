from stable_baselines3.common.callbacks import BaseCallback
import datetime

#  用於保存model用
class CustomCallback(BaseCallback):
    def __init__(self, save_path, save_freq, verbose=0):
        super(CustomCallback, self).__init__(verbose)
        self.save_path = save_path
        self.save_freq = save_freq
        self.last_save_step = 0

    def _on_step(self):

        # 定期保存模型
        if self.n_calls - self.last_save_step >= self.save_freq:
            now = datetime.datetime.now()
            self.time_name = datetime.datetime.timestamp(now)
            self.model.save(f"{self.save_path}_{self.n_calls}_{self.time_name}")
            self.last_save_step = self.n_calls

        return True