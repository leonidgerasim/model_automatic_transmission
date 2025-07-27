class FrictionClutch:

    def __init__(self, radius, area, n_pairs, mu_static, mu_kinetic):
        self.radius = radius
        self.area = area
        self.n_pairs = n_pairs
        self.mu_s = mu_static
        self.mu_k = mu_kinetic
        self.last_torque = 0.0

    def compute_torque(self, pressure, slip_velocity, dt):
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

