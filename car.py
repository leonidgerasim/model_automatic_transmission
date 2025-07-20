import pandas as pd
import numpy as np


class Engine:
    def __init__(self):
        self.eng_rpm = 1000 # rpm
        self.inert_eng = 0.08 # kg*m^2
        self.max_rpm = 7000 # rpm
        self.min_rpm = 1000 # rpm
        self.data_torque = pd.read_csv('data_torque_of_engine.csv')

    def engine_update(self, accr_ped, torque_i, dt):
        rpm = self.eng_rpm // 500 * 500
        if self.eng_rpm % 500 > 250:
            rpm += 500
        torque_e = self.data_torque.loc[self.data_torque['ped_pos'] == accr_ped][str(int(rpm))].tolist()[0]
        self.eng_rpm += 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng

    def get(self):
        return self.eng_rpm


class TorqueConvertor:
    def __init__(self):
        pass

