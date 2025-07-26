import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from torque_converter import TorqueConverter
from engine import Engine
import pandas as pd
import time


class HydraulicSolenoid:
    """Реалистичная модель электрогидравлического соленоида"""

    def __init__(self, max_current=1.0, response_time=0.05, max_pressure=2e6, system_pressure=3e6):
        """
        :param max_current: Максимальный ток управления (А)
        :param response_time: Время отклика (с)
        :param max_pressure: Максимальное управляющее давление (Па)
        :param system_pressure: Давление в системе (Па)
        """
        self.max_current = max_current
        self.response_time = response_time
        self.max_pressure = max_pressure
        self.system_pressure = system_pressure

        # Текущее состояние
        self.current = 0.0
        self.pressure = 0.0
        self.target_current = 0.0

        # Параметры для гистерезиса
        self.hysteresis = 0.05 * max_current
        self.last_direction = 1

    def set_current(self, target_current, dt):
        """Установка целевого тока с учетом динамики и гистерезиса"""
        # Ограничение целевого значения
        target_current = np.clip(target_current, 0, self.max_current)

        # Определение направления изменения
        direction = 1 if target_current >= self.current else -1

        # Эффект гистерезиса
        if direction != self.last_direction:
            target_current += direction * self.hysteresis

        # Динамика 1-го порядка
        self.current += (target_current - self.current) * min(1.0, dt / self.response_time)

        # Преобразование тока в давление
        self._calculate_pressure()

        return self.pressure

    def _calculate_pressure(self):
        """Преобразование тока в давление (с нелинейной характеристикой)"""
        # Нормализованный ток (0-1)
        normalized_current = self.current / self.max_current

        # Нелинейная характеристика (пример)
        if normalized_current < 0.1:
            pressure_ratio = 0.0
        elif normalized_current < 0.5:
            pressure_ratio = 0.7 * normalized_current
        else:
            pressure_ratio = 0.3 + 0.7 * normalized_current

        # Ограничение и расчет давления
        pressure_ratio = np.clip(pressure_ratio, 0, 1)
        self.pressure = pressure_ratio * self.max_pressure

        # Учет давления в системе
        self.pressure = min(self.pressure, self.system_pressure)

    def get_status(self):
        """Возвращает текущее состояние"""
        return {
            'current': self.current,
            'pressure': self.pressure,
            'target': self.target_current
        }


class FrictionClutch:
    """Надежная модель фрикционной муфты"""

    def __init__(self, radius, area, n_pairs, mu_static, mu_kinetic):
        self.radius = radius
        self.area = area
        self.n_pairs = n_pairs
        self.mu_s = mu_static
        self.mu_k = mu_kinetic
        self.last_torque = 0.0

    def compute_torque(self, pressure, slip_velocity, dt):
        """Упрощенный и надежный расчет момента"""
        # 1. Проверка нулевого давления
        if pressure < 1e4:  # Минимум 0.1 бар
            self.last_torque = 0.0
            return 0.0

        # 2. Расчет нормальной силы
        normal_force = pressure * self.area

        # 3. Определение коэффициента трения
        abs_slip = abs(slip_velocity)
        if abs_slip < 10:  # Порог статического трения (RPM)
            friction_coeff = self.mu_s
        else:
            friction_coeff = self.mu_k

        # 4. Расчет момента
        torque = friction_coeff * normal_force * self.radius * self.n_pairs

        # 5. Учет направления проскальзывания
        if slip_velocity < 0:
            torque = -torque

        self.last_torque = torque
        return torque

    def get_debug_info(self):
        return {
            'torque': self.last_torque,
            'params': f"R={self.radius}, A={self.area}, n={self.n_pairs}, μ_s={self.mu_s}, μ_k={self.mu_k}"
        }


class TransmissionModel:
    """Интегральная модель АКПП"""

    def __init__(self):
        # Параметры
        self.gear_ratios = [3.5, 2.0, 1.4, 1.0]  # Передаточные отношения
        self.current_gear = 0  # Текущая передача (N)

        # Создание компонентов
        self.tc = TorqueConverter(diameter=0.3)

        # Гидравлическая система
        self.system_pressure = 3e6  # 30 бар

        # Создание соленоидов
        self.solenoids = [
            HydraulicSolenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            HydraulicSolenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            HydraulicSolenoid(max_current=1.0, response_time=0.08,
                              max_pressure=2e6, system_pressure=self.system_pressure),
            HydraulicSolenoid(max_current=1.0, response_time=0.08,
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

        # Состояние системы
        self.state = np.zeros(4)  # [engine_rpm, input_rpm, output_rpm, vehicle_speed]
        self.pressures = np.zeros(4)
        self.last_time = 0

    def set_initial_state(self, engine_rpm, vehicle_speed):
        """Физически корректная инициализация"""
        # Расчет output_rpm из скорости автомобиля
        wheel_rpm = (vehicle_speed * 60) / (2 * np.pi * self.wheel_radius)
        output_rpm = wheel_rpm * 3.9  # Главная передача

        self.state = np.array([
            engine_rpm,  # engine_rpm
            engine_rpm * 0.8,  # input_rpm (начальное проскальзывание)
            output_rpm,  # output_rpm
            vehicle_speed  # vehicle_speed
        ], dtype=float)

    # def shift_gear(self, target_gear):
    #     """Переключение передачи"""
    #     if 0 <= target_gear < len(self.gear_ratios):
    #         self.current_gear = target_gear

    def create_shift_map(self):
        """Создает подробную карту переключения передач"""
        shift_map = {}

        # Генерация карты для разных скоростей и положений дросселя
        for speed in range(0, 160, 5):  # Скорость в км/ч
            for throttle in [0.2, 0.4, 0.6, 0.8]:  # Положение дросселя
                # Определение передачи в зависимости от условий
                if speed < 10:
                    gears = [1, 1, 1, 1]
                elif speed < 30:
                    if throttle < 0.4:
                        gears = [1, 2, 2, 2]
                    else:
                        gears = [1, 1, 2, 2]
                elif speed < 50:
                    if throttle < 0.3:
                        gears = [2, 2, 3, 3]
                    elif throttle < 0.6:
                        gears = [2, 3, 3, 3]
                    else:
                        gears = [1, 2, 3, 3]
                elif speed < 80:
                    if throttle < 0.4:
                        gears = [3, 3, 4, 4]
                    elif throttle < 0.7:
                        gears = [3, 4, 4, 4]
                    else:
                        gears = [2, 3, 4, 4]
                else:
                    gears = [4, 4, 4, 4]

                shift_map[(speed, throttle)] = gears

        return shift_map

    def control_unit(self, t, throttle):
        """Улучшенное управление соленоидами"""
        # Логика переключения передач
        engine_rpm = self.state[0]
        #print(engine_rpm, self.current_gear, engine_rpm > 4500 and self.current_gear < len(self.gear_ratios) - 1)
        if engine_rpm > 4500 and self.current_gear < len(self.gear_ratios) - 1:
            self.current_gear += 1
        elif engine_rpm < 2000 and self.current_gear > 0:
            self.current_gear -= 1

        # Расчет dt для динамики
        dt = t - self.last_time if t > self.last_time else 0.01

        # Управление соленоидами
        for i, sol in enumerate(self.solenoids):
            # Активируем только текущую передачу
            target = 1.0 if i == self.current_gear else 0.0
            self.pressures[i] = sol.set_current(target, dt)

        # Диагностика
        if t % 1.0 < 0.05:
            statuses = [f"{p / 1e5:.1f} bar" for p in self.pressures]
            print(f"t={t:.1f}s | Gear: {self.current_gear + 1} | Pressures: {statuses}")

        self.last_time = t

    def transmission_dynamics(self, t, state, engine_torque, load_torque, throttle):
        engine_rpm, input_rpm, output_rpm, vehicle_speed = state


        # 1. Расчет моментов ГДТ
        pump_torque, turbine_torque = self.tc.compute_torques(engine_rpm, input_rpm)

        # 2. Уравнение ДВС
        d_engine = (engine_torque - pump_torque) / self.engine_inertia

        # 3. Уравнение входного вала
        clutch_torque = 0.0
        gear_ratio = 1.0

        if 0 <= self.current_gear < len(self.gear_ratios):
            gear_ratio = self.gear_ratios[self.current_gear]
            slip = input_rpm - output_rpm * gear_ratio

            # ИСПРАВЛЕНИЕ: используем давление напрямую (уже в Па)
            pressure = self.pressures[self.current_gear]

            clutch_torque = self.clutches[self.current_gear].compute_torque(
                pressure, slip, t
            )

            # Диагностика
            if t % 0.5 < 0.05:
                debug_info = self.clutches[self.current_gear].get_debug_info()
                print(clutch_torque, pressure)
                print(f"Clutch {self.current_gear + 1}: P={pressure / 1e5:.1f} bar, "
                      f"Slip={slip:.1f} RPM, T={clutch_torque:.1f} Nm")

        # 4. Уравнения динамики
        d_input = (turbine_torque - clutch_torque) / self.input_inertia
        #print(clutch_torque)
        d_output = (clutch_torque * gear_ratio - load_torque) / self.output_inertia

        # 5. Расчет скорости автомобиля
        wheel_rpm = output_rpm / 3.9  # Главная передача
        d_vehicle = wheel_rpm * (2 * np.pi * self.wheel_radius) / 60  # м/с

        return [d_engine, d_input, d_output, d_vehicle]

    def simulate(self, t_span, dt, engine_torque_func, throttle_func):
        """Запуск симуляции"""

        current_state = np.asarray(self.state, dtype=np.float64)
        current_state = np.nan_to_num(current_state, nan=0.0)

        t_eval = np.arange(t_span[0], t_span[1], dt)
        results = {
            'time': t_eval,
            'engine_rpm': [],
            'input_rpm': [],
            'output_rpm': [],
            'vehicle_speed': [],
            'gear': [],
            'pressures': []
        }

        # Сохраняем начальное состояние как массив
        current_state = np.array(self.state, dtype=float)

        def load_torque_func(t, vehicle_speed, model):
            """Реалистичная нагрузка с сопротивлением качению и воздухом"""
            # Параметры автомобиля
            mass = model.vehicle_mass
            g = 9.81
            rho = 1.225  # Плотность воздуха
            Cd = 0.3  # Коэффициент лобового сопротивления
            Af = 2.2  # Площадь лобовой поверхности
            Cr = 0.015  # Коэффициент сопротивления качению

            # Расчет сил
            rolling_resistance = Cr * mass * g
            air_resistance = 0.5 * rho * Cd * Af * vehicle_speed ** 2
            total_force = rolling_resistance + air_resistance

            # Преобразование в момент на выходном валу
            wheel_torque = total_force * model.wheel_radius
            return wheel_torque / 3.9  # Главная передача

        def engine_torque(t):
            """Типичная характеристика ДВС"""
            if t < 2.0:
                return 150  # Низкий момент при старте
            elif t < 5.0:
                return 300  # Средний момент
            else:
                return 250  # Установившийся режим


        for t in t_eval:
            vehicle_speed = current_state[3]

            # Расчет нагрузки на основе текущей скорости
            load = load_torque_func(t, vehicle_speed, self)
            throttle = throttle_func(t)

            # Расчет момента двигателя
            engine_torque = engine_torque_func(t)

            self.control_unit(t, throttle)
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
            # Внешние воздействия

            # throttle = throttle_func(t)
            # e_torque = engine_torque_func(t)
            # current_state[0] = np.clip(current_state[0], 0, 6000)  # Обороты двигателя
            # current_state[1] = np.clip(current_state[1], 0, 10000)  # Входной вал
            # current_state[2] = np.clip(current_state[2], 0, 10000)
            #
            # # Вызываем блок управления перед интегрированием
            # self.control_unit(t, throttle)
            #
            # # Интегрирование на шаге dt
            # sol = solve_ivp(
            #     fun=lambda t, y: self.transmission_dynamics(t, y, e_torque, load, throttle),
            #     t_span=[t, t + dt],
            #     y0=current_state,
            #     t_eval=[t + dt],
            #     method='RK45',
            #     atol=1e-2,  # Абсолютная точность
            #     rtol=1e-2  # Относительная точность
            # )
            #
            # # Правильное извлечение результатов интегрирования
            # if sol.success:
            #     # Извлекаем последнее значение состояния
            #     new_state = sol.y[:, -1] if sol.y.ndim > 1 else sol.y
            #     current_state = np.array(new_state, dtype=float)
            #     self.state = current_state.copy()
            # else:
            #     print(f"Интегрирование не удалось на шаге t={t}: {sol.message}")
            #     break

            # Сохранение результатов
            results['engine_rpm'].append(current_state[0])
            results['input_rpm'].append(current_state[1])
            results['output_rpm'].append(current_state[2])
            results['vehicle_speed'].append(current_state[3])
            results['gear'].append(self.current_gear)
            results['pressures'].append(self.pressures.copy())
            print(t)

        return results
