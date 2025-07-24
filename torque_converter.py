import pandas as pd
import numpy as np


class TorqueConverter:
    def __init__(self):
        self.density = 900 # kg/m^3
        self.diameter = 0.244 # m
        #self.data_coef = pd.read_csv('data/torque_convertor.csv')

        self.fl_area = 0.005 # m^2 effective flow area
        self.imp_area = 0.003 # m^2 exit impeller area
        self.trb_area = 0.002 # m^2 exit turbine area
        self.str_area = 0.001 # m^2 exit stator area

        self.coef_impeller = 1  # coef flow rates
        self.angle_imp = - np.pi / 3 # angle exit impeller
        self.angle_trb = np.pi / 2 # angle exit turbine
        self.angle_str = - np.pi / 3 # angle exit stator

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
        self.cal_torque_r(eng_rpm, trb_rpm)

    def cal_fl_v(self, eng_rpm):
        eng_v = eng_rpm * 2 * np.pi / 60
        fl_v = self.coef_impeller * eng_v * self.radius_trb_en
        self.vol_fl = self.fl_area * fl_v

    def cal_torque_i(self, eng_rpm):
        eng_v = eng_rpm * 2 * np.pi / 60
        v_r_i_out = self.vol_fl / 2 / np.pi / self.radius_trb_en / self.imp_area
        self.v_trb_en = eng_v * self.radius_trb_en - v_r_i_out / np.tan(self.angle_imp)
        self.torque_i = self.density * self.vol_fl * (self.radius_trb_en * self.v_trb_en - self.radius_imp_en * self.v_imp_en)

    def cal_torque_t(self, trb_rpm):
        trb_v = trb_rpm * 2 * np.pi / 60
        v_r_t_out = self.vol_fl / 2 / np.pi / self.radius_str_en / self.str_area
        self.v_str_en = trb_v * self.radius_str_en - v_r_t_out / np.tan(self.angle_trb)
        self.torque_t = self.density * self.vol_fl * (self.radius_trb_en * self.v_trb_en - self.radius_str_en * self.v_str_en)

    def cal_torque_r(self, eng_rpm, trb_rpm):
        v_r_s_out = self.vol_fl / 2 / np.pi / self.radius_imp_en / self.fl_area
        if trb_rpm / eng_rpm >= 0.8:
            self.v_imp_en = self.v_str_en * self.radius_str_en / self.radius_imp_en
        else:
            self.v_imp_en = - v_r_s_out / np.tan(self.angle_str)
            #print(self.v_imp_en, v_r_s_out, np.tan(self.angle_str), self.v_trb_en)
        self.torque_s = self.density * self.vol_fl * (self.radius_str_en * self.v_str_en - self.radius_imp_en * self.v_imp_en)

    def get_torque_i(self):
        return self.torque_i

    def get_torque_t(self):
        return self.torque_t

    def get_torque_s(self):
        return self.torque_s


