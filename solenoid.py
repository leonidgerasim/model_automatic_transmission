import numpy as np


class Solenoid:
    """Реалистичная модель электрогидравлического соленоида"""

    def __init__(self, max_current=1.0, response_time=0.05, max_pressure=2e6, system_pressure=3e6):
        """
        :param max_current: Максимальный ток управления (А)
        :param response_time:  Время отклика (с)
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
