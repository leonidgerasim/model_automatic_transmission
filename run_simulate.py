import pandas as pd
import numpy as np
from car import Car
import matplotlib.pyplot as plt
from gearbox import TransmissionModel
from engine import Engine


# class Environment:
#     def __init__(self, road_slop=0, dt=0.1, t=10):
#         self.accr_ped = 0
#         self.brk_ped = 0
#         self.road_slop = road_slop
#         self.dt = dt
#         self.iter_t = 0
#         self.t = t
#         self.car = Car()
#
#     def acceleration(self, press):
#         match press:
#             case 0:
#                 if self.accr_ped - 200 * self.dt >= 0:
#                     self.accr_ped -= 200 * self.dt
#                 else:
#                     self.accr_ped = 0
#             case 1:
#                 if self.accr_ped + 200 * self.dt <= 100:
#                     self.accr_ped += 200 * self.dt
#                 else:
#                     self.accr_ped = 100
#
#     def braking(self, press):
#         match press:
#             case 0:
#                 if self.brk_ped - 200 * self.dt >= 0:
#                     self.brk_ped -= 200 * self.dt
#                 else:
#                     self.brk_ped = 0
#             case 1:
#                 if self.brk_ped + 200 * self.dt <= 100:
#                     self.brk_ped += 200 * self.dt
#                 else:
#                     self.brk_ped = 100
#
#     def run_simulate(self, scenario_acc, scenario_brk, timing_acc, timing_brk):
#         iter_acc = 0
#         init_acc = False
#         len_acc = len(scenario_acc)
#         iter_brk = 0
#         init_brk = False
#         len_brk = len(scenario_brk)
#         while self.iter_t < self.t:
#
#             if timing_acc == self.iter_t:
#                 init_acc = True
#             if init_acc and iter_acc < len_acc:
#                 self.acceleration(scenario_acc[iter_acc])
#                 iter_acc += 1
#
#             if timing_brk == self.iter_t:
#                 init_brk = True
#             if init_brk and iter_brk < len_brk:
#                 self.braking(scenario_brk[iter_brk])
#                 iter_brk += 1
#
#             self.car.update(self.accr_ped, self.dt)
#             print(self.car.get_data())
#
#             self.iter_t += self.dt
#
#
#
# if __name__ == "__main__":
#     env = Environment(dt=0.01)
#     scenario_acc = [1 for i in range(50)]
#     scenario_brk = [1, 1, 1, 1, 1, 0, 0, 0]
#     timing_acc = 0
#     timing_brk = 0
#     env.run_simulate(scenario_acc=scenario_acc, scenario_brk=scenario_brk, timing_acc=timing_acc, timing_brk=timing_brk)
#     print(env.accr_ped, env.brk_ped)


if __name__ == "__main__":
    # Создание модели
    model = TransmissionModel()
    model.set_initial_state(800, 0)  # Обороты двигателя 800 RPM, скорость 0 м/с


    # Внешние условия
    # def engine_torque(t):
    #     """Характеристика двигателя"""
    #     return 200 + 100 * np.sin(0.5 * t)  # Пример переменного момента


    # def load_torque(t):
    #     """Улучшенная модель сопротивления движению"""
    #     speed = current_state[3]  # vehicle_speed в м/с
    #     rolling_resistance = 0.01 * vehicle_mass * 9.81
    #     air_resistance = 0.5 * 1.2 * 0.3 * 2.5 * speed ** 2
    #     grade_resistance = 0  # Можно добавить уклон
    #     total_force = rolling_resistance + air_resistance + grade_resistance
    #     return total_force * wheel_radius / 3.9


    def throttle(t):
        return 0.7 if t < 5 else 0.4


    # # Запуск симуляции
    # results = model.simulate(
    #     t_span=[0, 10],
    #     dt=0.05,
    #     engine_torque_func=engine_torque,
    #     #load_func=load_torque,
    #     throttle_func=throttle
    # )
    model = TransmissionModel()
    model.set_initial_state(800, 0)  # 800 RPM, скорость 0 м/с


    def engine_torque(t):
        """Характеристика двигателя"""
        return 250 if t < 5 else 200  # Постоянный момент


    results = model.simulate(
        t_span=[0, 10],
        dt=0.05,
        engine_torque_func=engine_torque,
        throttle_func=lambda t: 0.7
    )

    def analyze_results(results, model):
        """Анализ результатов симуляции"""
        print("\nДиагностика муфт:")
        for i, clutch in enumerate(model.clutches):
            info = clutch.get_debug_info()
            print(f"Муфта {i + 1}:")
            print(f"  Параметры: {info['params']}")
            print(f"  Последний момент: {info['torque']:.1f} Nm")

        print("\nСтатистика давлений:")
        avg_pressures = np.mean(np.array(results['pressures']), axis=0)
        max_pressures = np.max(np.array(results['pressures']), axis=0)
        for i, (avg, max_val) in enumerate(zip(avg_pressures, max_pressures)):
            print(f"Соленоид {i + 1}: Среднее = {avg:.2f}, Макс = {max_val:.2f}")

        print("\nПередачи:")
        gear_changes = np.where(np.diff(results['gear']))[0]
        print(f"Переключений: {len(gear_changes)}")
        for change in gear_changes:
            t = results['time'][change]
            gear = results['gear'][change]
            print(f"t={t:.2f}s -> Передача {gear + 1}")


    # После запуска симуляции
    analyze_results(results, model)

    # Визуализация результатов
    plt.figure(figsize=(12, 10))

    plt.subplot(3, 1, 1)
    plt.plot(results['time'], results['engine_rpm'], label='Engine RPM')
    plt.plot(results['time'], results['input_rpm'], label='Input RPM')
    plt.plot(results['time'], results['output_rpm'], label='Output RPM')
    plt.ylabel('Обороты (RPM)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(results['time'], results['vehicle_speed'], 'g-', label='Скорость')
    plt.ylabel('Скорость (м/с)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    for i in range(3):
        plt.plot(results['time'], [p[i] for p in results['pressures']], label=f'Муфта {i + 1}')
    plt.plot(results['time'], results['gear'], 'k--', label='Текущая передача')
    plt.xlabel('Время (с)')
    plt.ylabel('Давление/Передача')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('transmission_simulation.png', dpi=300)
    plt.show()

