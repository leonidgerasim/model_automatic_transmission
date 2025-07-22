import pandas as pd
import numpy as np


class Engine:
    def __init__(self):
        self.eng_rpm = 1000 # rpm
        self.inert_eng = 0.08 # kg*m^2
        self.max_rpm = 7000 # rpm
        self.min_rpm = 1000 # rpm
        self.data_torque = pd.read_csv('data/torque_of_engine.csv')

    def engine_update(self, accr_ped, torque_i, dt):
        rpm = self.eng_rpm // 500 * 500
        if self.eng_rpm % 500 > 250:
            rpm += 500
        torque_e = self.data_torque.loc[self.data_torque['ped_pos'] == accr_ped][str(int(rpm))].tolist()[0]
        if self.eng_rpm + 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng < self.min_rpm:
            self.eng_rpm = self.min_rpm
        elif self.eng_rpm + 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng > self.max_rpm:
            self.eng_rpm = self.max_rpm
        else:
            self.eng_rpm += 60 * (torque_e - torque_i) * dt / 2 / np.pi / self.inert_eng

    def get(self):
        return self.eng_rpm


class TorqueConvertor:
    def __init__(self):
        self.density = 900 # kg/m^3
        self.diameter = 0.244 # m
        self.data_coef = pd.read_csv('data/torque_convertor.csv')
        self.fl_area = 0.005 # m^2 effective flow area
        self.coef_impeller = 1  # coef flow rates
        self.angle_str = np.pi / 4
        self.radius_trb_en = 0.1 # m radius entry turbine
        self.radius_imp_en = 0.7 * self.radius_trb_en # m radius entry impeller
        self.radius_str_en = 0.6 * self.radius_trb_en # m radius entry stator
        self.v_imp_en = 0 # m/s velocity entry impeller
        self.v_trb_en = 0 # m/s velocity entry turbine
        self.v_str_en = 0 # m/s velocity entry stator
        self.vol_fl = 0 # m^3/s
        #self.reactor_v = 0 # rad/s angular velocity reactor
        self.torque_i = 0
        self.torque_t = 0
        self.torque_s = 0

    def update(self, eng_rpm, trb_rpm):
        self.cal_fl_v(eng_rpm)
        self.cal_torque_i(eng_rpm)
        self.cal_torque_t(trb_rpm)
        self.cal_torque_r()

    def cal_fl_v(self, eng_rpm):
        eng_v = eng_rpm * 2 * np.pi / 60
        fl_v = self.coef_impeller * eng_v * self.radius_trb_en
        self.vol_fl = self.fl_area * fl_v

    def cal_torque_i(self, eng_rpm):
        eng_v = eng_rpm * 2 * np.pi / 60
        self.v_trb_en = eng_v * self.radius_trb_en
        self.torque_i = self.density * self.vol_fl * (self.radius_trb_en * self.v_trb_en - self.radius_imp_en * self.v_imp_en)

    def cal_torque_t(self, trb_rpm):
        trb_v = trb_rpm * 2 * np.pi / 60
        self.v_str_en = trb_v * self.radius_str_en
        self.torque_t = self.density * self.vol_fl * (self.radius_str_en * self.v_str_en - self.radius_trb_en * self.v_trb_en)

    def cal_torque_r(self):
        self.v_imp_en = self.v_str_en * np.cos(self.angle_str)
        self.torque_s = self.density * self.vol_fl * (self.radius_imp_en * self.v_imp_en - self.radius_str_en * self.v_str_en)

    def get_imp_torque(self):
        return self.torque_i

    def get_trb_torque(self):
        return self.torque_t

    def get_torque_s(self):
        return self.torque_s


if __name__ == "__main__":
    trq_con = TorqueConvertor()
    for i in range(10):
        trq_con.update(3000, 0)
        print(trq_con.get_trb_torque(), trq_con.get_imp_torque(), trq_con.get_torque_s())
