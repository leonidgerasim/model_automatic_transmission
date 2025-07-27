import numpy as np
import matplotlib.pyplot as plt
from gearbox import TransmissionModel

if __name__ == "__main__":
    # Создание модели
    model = TransmissionModel()
    model.set_initial_state(800, 0)  # Обороты двигателя 800 RPM, скорость 0 м/с


    def throttle(t):
        if t < 5:
            return 0.8  # Интенсивный разгон
        elif t < 15:
            return 0.4  # Установившееся движение
        else:
            return 0.6

    model = TransmissionModel()
    model.set_initial_state(800, 0)  # 800 RPM, скорость 0 м/с

    results = model.simulate(
        dt=0.05,
        throttle_func=throttle
    )

    print(results['time'][-1])

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


    analyze_results(results, model)


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

