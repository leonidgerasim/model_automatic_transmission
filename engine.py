import pandas as pd
import numpy as np


class Engine:
    def __init__(self):
        self.eng_rpm = 1000 # rpm
        self.inert_eng = 0.08 # kg*m^2
        self.max_rpm = 6000 # rpm
        self.min_rpm = 1000 # rpm
        self.data_torque = pd.read_csv('data/torque_of_engine.csv')

    def engine_update(self, accr_ped, torque_i, dt):
        rpm = self.eng_rpm // 500 * 500
        if self.eng_rpm % 500 > 250:
            rpm += 500

        if 10 <= accr_ped <= 20:
            accr_ped = 20
        elif accr_ped < 10:
            accr_ped = 0
        else:
            accr_ped = accr_ped // 10 * 10

        torque_e = self.data_torque.loc[self.data_torque['ped_pos'] == accr_ped][str(int(rpm))].tolist()[0]

        #if self.eng_rpm + 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng < self.min_rpm:
        if self.eng_rpm + 60 * torque_e * dt / 2 / np.pi / self.inert_eng < self.min_rpm:
            self.eng_rpm = self.min_rpm

        #elif self.eng_rpm + 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng > self.max_rpm:
        elif self.eng_rpm + 60 * torque_e * dt / 2 / np.pi / self.inert_eng > self.max_rpm:
            self.eng_rpm = self.max_rpm
        else:
            #self.eng_rpm += 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng
            self.eng_rpm += 60 * torque_e * dt / 2 / np.pi / self.inert_eng

    def get_rpm(self):
        return self.eng_rpm

def get_d_eng_rpm(rpm):
    rpm = rpm // 500 * 500
    if rpm % 500 > 250:
        rpm += 500

    if rpm < 1000:
        rpm = 1000
    data_torque = pd.read_csv('data/torque_of_engine.csv')
    torque_e = data_torque.loc[data_torque['ped_pos'] == 100][str(int(rpm))].tolist()[0]

    return 60 * torque_e * dt / 2 / np.pi / self.inert_eng
