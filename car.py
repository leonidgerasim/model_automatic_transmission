import pandas as pd
import numpy as np
#from engine import Engine
from torque_converter import TorqueConverter


# class Car:
#     def __init__(self):
#         self.eng = Engine()
#         self.trq_conv = TorqueConverter()
#
#     def update(self, accr_ped, dt):
#         trq_i = self.trq_conv.get_torque_i()
#         self.eng.engine_update(accr_ped, trq_i, dt)
#         eng_rpm = self.eng.get_rpm()
#         self.trq_conv.update(eng_rpm, 0)
#
#     def get_data(self):
#         return self.trq_conv.get_torque_t(), self.eng.get_rpm(), self.trq_conv.get_torque_i()


# class TorqueConverter:
#     def __init__(self, K_max, lockup_ratio, efficiency_curve):
#         self.K_max = K_max
#         self.lockup_ratio = lockup_ratio
#         self.efficiency_curve = efficiency_curve  # Функция f(sr)
#
#     def calculate(self, w_in, T_in, w_out):
#         sr = w_out / w_in  # Отношение скоростей
#
#         if sr >= self.lockup_ratio:
#             # Режим блокировки
#             T_out = T_in * 0.99  # Учет потерь
#             efficiency = 0.99
#         else:
#             # Режим гидротрансформации
#             K = self.K_max * (1 - sr / self.lockup_ratio)  # Упрощенная модель
#             T_out = T_in * K
#             efficiency = self.efficiency_curve(sr)  # Использование кривой
#
#         return T_out, efficiency


# if __name__ == "__main__":
#     trq_con = TorqueConverter()
#     for i in range(10):
#         trq_con.update(1000, 90)
#         print(trq_con.get_torque_t(), trq_con.get_torque_i(), trq_con.get_torque_s())
