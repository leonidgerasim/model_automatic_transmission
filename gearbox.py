import numpy as np
from scipy.integrate import solve_ivp
from torque_converter import TorqueConverter
from solenoid import Solenoid
from friction_clutch import FrictionClutch
import pandas as pd


class TransmissionModel:
    def __init__(self):
        # Параметры
        self.gear_ratios = [3.5, 2.0, 1.4, 1.0]  # Передаточные отношения
        self.current_gear = 0  # Текущая передача
        #self.shift_map = self.create_shift_map()

        # Создание компонентов
        self.tc = TorqueConverter(diameter=0.3)

        # Гидравлическая система
        self.system_pressure = 3e6  # 30 бар

        # Создание соленоидов
        self.solenoids = [
            Solenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            Solenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            Solenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            Solenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure)
        ]

        # Массив давлений
        self.pressures = [0.0] * len(self.solenoids)

        # Фрикционные муфты для каждой передачи
        self.clutches = [
            FrictionClutch(radius=0.15, area=0.015, n_pairs=5, mu_static=0.35, mu_kinetic=0.25),
            FrictionClutch(radius=0.14, area=0.014, n_pairs=5, mu_static=0.33, mu_kinetic=0.23),
            FrictionClutch(radius=0.13, area=0.013, n_pairs=5, mu_static=0.32, mu_kinetic=0.22),
            FrictionClutch(radius=0.12, area=0.012, n_pairs=5, mu_static=0.30, mu_kinetic=0.20)
        ]

        # Параметры динамики
        self.engine_inertia = 0.5  # кг*м^2
        self.input_inertia = 0.1  # кг*м^2
        self.output_inertia = 1  # кг*м^2
        self.vehicle_mass = 1500  # кг
        self.wheel_radius = 0.3  # м

        self.last_shift_time = 0
        self.shift_delay = 0.5

        # Состояние системы
        self.state = np.zeros(4)  # [engine_rpm, input_rpm, output_rpm, vehicle_speed]
        self.pressures = np.zeros(4)
        self.last_time = 0

    def set_initial_state(self, engine_rpm, vehicle_speed):
        wheel_rpm = (vehicle_speed * 60) / (2 * np.pi * self.wheel_radius)
        output_rpm = wheel_rpm * 3.9

        self.state = np.array([
            engine_rpm,  # engine_rpm
            engine_rpm * 0.8,  # input_rpm (начальное проскальзывание)
            output_rpm,  # output_rpm
            vehicle_speed  # vehicle_speed
        ], dtype=float)

    def _shift_gear(self, t, new_gear):
        if 0 <= new_gear < len(self.gear_ratios):
            print(f"Переключение с {self.current_gear + 1} на {new_gear + 1} при "
                  f"{self.current_speed_kmh:.1f} км/ч, {self.state[0]:.0f} RPM")

            self.current_gear = new_gear
            self.last_shift_time = t

            for sol in self.solenoids:
                sol.set_current(0, 0.05)

    def get_gear(self, throttle):
        vehicle_speed = self.state[3] * 3.6

        if vehicle_speed < 10:
            return [0, 0]
        elif vehicle_speed < 30:
            if throttle < 0.5:
                return [0, 1]
            else:
                return [0, 0]
        elif vehicle_speed < 50:
            if throttle < 0.5:
                return [1, 2]
            else:
                return [1, 1]
        elif vehicle_speed < 80:
            if throttle < 0.5:
                return [2, 3]
            else:
                return [2, 2]
        else:
            return [3, 3]

    def control_unit(self, t, throttle):
        """Улучшенная версия с диагностикой"""
        # Сохраняем положение дросселя
        self.throttle = throttle

        # Конвертация скорости
        vehicle_speed = self.state[3] * 3.6  # м/с → км/ч
        self.current_speed_kmh = vehicle_speed

        # Проверка задержки переключения
        if t - self.last_shift_time < self.shift_delay:
            return

        # Поиск ближайшей точки в карте переключений
        speed_key = round(vehicle_speed / 5) * 5
        throttle_key = round(throttle * 4) / 4

        # Получение рекомендаций по передачам
        #gear_options = self.shift_map.get((speed_key, throttle_key), [1, 2, 3, 4])

        # Текущие обороты двигателя
        engine_rpm = self.state[0]

        # Логика переключения
        if engine_rpm > 4500:
            #print(self.get_gear(throttle))
            self._shift_gear(t, max(self.get_gear(throttle)))

        elif engine_rpm < 1800:
            self._shift_gear(t, min(self.get_gear(throttle)))

        dt = t - self.last_time if t > self.last_time else 0.01
        # Диагностика
        for i, sol in enumerate(self.solenoids):
            # Активируем только текущую передачу
            target = 1.0 if i == self.current_gear else 0.0
            self.pressures[i] = sol.set_current(target, dt)

            # Диагностика
        # if t % 1.0 < 0.05:
        #     statuses = [f"{p / 1e5:.1f} bar" for p in self.pressures]
        #     print(f"t={t:.1f}s | Gear: {self.current_gear + 1} | Pressures: {statuses}")

        self.last_time = t

    def get_torque_e(self, throttle):
        rpm = self.state[0] // 500 * 500
        if rpm % 500 > 250:
            rpm += 500

        if rpm < 1000:
            rpm = 1000

        throttle *= 100
        if 10 <= throttle <= 20:
            throttle = 20
        elif throttle < 10:
            throttle = 0
        else:
            throttle = throttle // 10 * 10

        data_torque = pd.read_csv('data/torque_of_engine.csv')

        torque_e = data_torque.loc[data_torque['ped_pos'] == throttle][str(int(rpm))].tolist()[0]

        return torque_e

    def load_torque_func(self):
        # Параметры автомобиля
        mass = self.vehicle_mass
        g = 9.81
        rho = 1.225  # Плотность воздуха
        Cd = 0.3  # Коэффициент лобового сопротивления
        Af = 2.2  # Площадь лобовой поверхности
        Cr = 0.015  # Коэффициент сопротивления качению

        # Расчет сил
        rolling_resistance = Cr * mass * g
        air_resistance = 0.5 * rho * Cd * Af * self.state[3] ** 2
        total_force = rolling_resistance + air_resistance

        # Преобразование в момент на выходном валу
        wheel_torque = total_force * self.wheel_radius
        return wheel_torque / 3.9  # Главная передача

    def transmission_dynamics(self, t, state, engine_torque, load_torque, throttle):
        engine_rpm, input_rpm, output_rpm, vehicle_speed = state

        # 1. Расчет моментов ГДТ
        pump_torque, turbine_torque = self.tc.compute_torques(engine_rpm, input_rpm)

        self.control_unit(t, throttle)

        # 2. Уравнение ДВС
        engine_torque += pump_torque
        if engine_rpm + (engine_torque - pump_torque) / self.engine_inertia > 6000:
            d_engine = 6000 - engine_rpm
        elif engine_rpm + (engine_torque - pump_torque) / self.engine_inertia < 1000:
            d_engine = 1000 - engine_rpm
        else:
            d_engine = (engine_torque - pump_torque) / self.engine_inertia

        # 3. Уравнение входного вала
        clutch_torque = 0.0
        gear_ratio = 1.0

        if 0 <= self.current_gear < len(self.gear_ratios):
            gear_ratio = self.gear_ratios[self.current_gear]
            slip = input_rpm - output_rpm * gear_ratio

            pressure = self.pressures[self.current_gear]

            clutch_torque = self.clutches[self.current_gear].compute_torque(
                pressure, slip, t
            )

            # Диагностика
            # if t % 0.5 < 0.05:
            #     debug_info = self.clutches[self.current_gear].get_debug_info()
            #     print(clutch_torque, pressure, turbine_torque, engine_torque, pump_torque)
            #     print(f"Clutch {self.current_gear + 1}: P={pressure / 1e5:.1f} bar, "
            #           f"Slip={slip:.1f} RPM, T={clutch_torque:.1f} Nm")

        # 4. Уравнения динамики
        d_input = (turbine_torque - clutch_torque) / self.input_inertia

        d_output = (clutch_torque * gear_ratio - load_torque) / self.output_inertia

        # 5. Расчет скорости автомобиля
        d_wheel_rpm = d_output / 3.9  # Главная передача
        d_vehicle = d_wheel_rpm * (2 * np.pi * self.wheel_radius) / 60  # м/с

        return [d_engine, d_input, d_output, d_vehicle]

    def simulate(self, dt, throttle_func):

        current_state = np.asarray(self.state, dtype=np.float64)
        current_state = np.nan_to_num(current_state, nan=0.0)

        #t_eval = np.arange(t_span[0], t_span[1], dt)
        results = {
            'time': [],
            'engine_rpm': [],
            'input_rpm': [],
            'output_rpm': [],
            'vehicle_speed': [],
            'gear': [],
            'pressures': []
        }

        # Сохраняем начальное состояние как массив
        current_state = np.array(self.state, dtype=float)

        t = 0
        while self.state[3] * 3.6 < 100:
            load = self.load_torque_func()

            throttle = throttle_func(t)

            # Расчет момента двигателя
            engine_torque = self.get_torque_e(throttle)

            # Интегрирование
            sol = solve_ivp(
                fun=lambda t, y: self.transmission_dynamics(t, y, engine_torque, load, throttle),
                t_span=[t, t + dt],
                y0=current_state,
                t_eval=[t + dt],
                method='RK45'
            )

            # Обновление состояния с защитой
            if sol.success:
                new_state = sol.y[:, -1] if sol.y.ndim > 1 else sol.y
                # Физические ограничения
                new_state[0] = max(0, new_state[0])  # engine_rpm
                new_state[1] = max(0, new_state[1])  # input_rpm
                new_state[2] = max(0, new_state[2])  # output_rpm
                current_state = new_state
                self.state = current_state.copy()
            else:
                print(f"Ошибка интегрирования: {sol.message}")
                break

            # Сохранение результатов
            results['time'].append(t)
            results['engine_rpm'].append(current_state[0])
            results['input_rpm'].append(current_state[1])
            results['output_rpm'].append(current_state[2])
            results['vehicle_speed'].append(current_state[3])
            results['gear'].append(self.current_gear)
            results['pressures'].append(self.pressures.copy())
            #print(t)
            t += dt

        # for t in t_eval:
        #
        #     # Расчет нагрузки на основе текущей скорости
        #     load = self.load_torque_func()
        #
        #     throttle = throttle_func(t)
        #
        #     # Расчет момента двигателя
        #     engine_torque = self.get_torque_e(throttle)
        #
        #     # Интегрирование
        #     sol = solve_ivp(
        #         fun=lambda t, y: self.transmission_dynamics(t, y, engine_torque, load, throttle),
        #         t_span=[t, t + dt],
        #         y0=current_state,
        #         t_eval=[t + dt],
        #         method='RK45'
        #     )
        #
        #     # Обновление состояния с защитой
        #     if sol.success:
        #         new_state = sol.y[:, -1] if sol.y.ndim > 1 else sol.y
        #         # Физические ограничения
        #         new_state[0] = max(0, new_state[0])  # engine_rpm
        #         new_state[1] = max(0, new_state[1])  # input_rpm
        #         new_state[2] = max(0, new_state[2])  # output_rpm
        #         current_state = new_state
        #         self.state = current_state.copy()
        #     else:
        #         print(f"Ошибка интегрирования: {sol.message}")
        #         break
        #
        #     # Сохранение результатов
        #     results['engine_rpm'].append(current_state[0])
        #     results['input_rpm'].append(current_state[1])
        #     results['output_rpm'].append(current_state[2])
        #     results['vehicle_speed'].append(current_state[3])
        #     results['gear'].append(self.current_gear)
        #     results['pressures'].append(self.pressures.copy())
        #     print(t)

        return results
