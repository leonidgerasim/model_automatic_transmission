from gearbox import TransmissionModel
import numpy as np
from solenoid import Solenoid
from friction_clutch import FrictionClutch
import matplotlib.pyplot as plt


class Individual:
    def __init__(self):
        self.chrom = [0.0 for i in range(34)]
        self.time_test = 0
        self.results = {}
        self.list_func = [
            lambda: np.random.uniform(3, 4),
            lambda: np.random.uniform(2, 3),
            lambda: np.random.uniform(1, 2),
            lambda: 1,
            lambda: 0.3,#np.random.uniform(0.1, 0.7),
            lambda: np.random.uniform(1.5e6, 3.5e6),
            lambda: np.random.uniform(0.5, 2),
            lambda: np.random.uniform(0.02, 0.1),
            lambda: np.random.uniform(1.2e6, 2.4e6),
            lambda: np.random.uniform(0.05, 0.5),
            lambda: np.random.uniform(0.005, 0.025),
            lambda: np.random.randint(1, 10),
            lambda: np.random.uniform(0.1, 1),
            lambda: np.random.uniform(0.1, 0.5),
            lambda: np.random.uniform(0.1, 1.5),
            lambda: np.random.uniform(0.01, 0.5),
            lambda: np.random.uniform(0.5, 3),
            lambda: np.random.randint(1500, 4000),
            lambda: np.random.uniform(0.2, 1),
        ]

    def throttle(self, t):
        if t < 5:
            return 0.8  # Интенсивный разгон
        elif t < 15:
            return 0.4  # Установившееся движение
        else:
            return 0.6

    def calc_time(self):
        model = TransmissionModel(self.chrom)
        model.set_initial_state(800, 0)  # 800 RPM, скорость 0 м/с

        results = model.simulate(
            dt=0.01,
            throttle_func=self.throttle
        )
        self.results = results
        self.time_test = results['time'][-1]

    def post_chrom(self, chrom):
        self.chrom = chrom

    def init_chrom(self):
        for i in range(len(self.chrom)):
            self.update_chrome(i)

        self.calc_time()

    def update_chrome(self, index):
        if 8 < index < 29:
            match index % 5:
                case 4:
                    self.chrom[index] = self.list_func[9]()
                case 0:
                    self.chrom[index] = self.list_func[10]()
                case 1:
                    self.chrom[index] = self.list_func[11]()
                case 2:
                    self.chrom[index] = self.list_func[12]()
                case 3:
                    self.chrom[index] = self.list_func[13]()
        elif index > 28:
            self.chrom[index] = self.list_func[index - 15]()
        else:
            self.chrom[index] = self.list_func[index]()

    def mutation(self):
        m = [i for i in range(len(self.chrom))]
        indexes = np.random.choice(m, 3, False)
        for i in indexes:
            self.update_chrome(i)

    def graph(self):
        plt.figure(figsize=(12, 10))

        plt.subplot(3, 1, 1)
        plt.plot(self.results['time'], self.results['engine_rpm'], label='Engine RPM')
        plt.plot(self.results['time'], self.results['input_rpm'], label='Input RPM')
        plt.plot(self.results['time'], self.results['output_rpm'], label='Output RPM')
        plt.ylabel('Обороты (RPM)')
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(self.results['time'], self.results['vehicle_speed'], 'g-', label='Скорость')
        plt.ylabel('Скорость (м/с)')
        plt.legend()
        plt.grid(True)

        # plt.subplot(3, 1, 3)
        # for i in range(3):
        #     plt.plot(results['time'], [p[i] for p in results['pressures']], label=f'Муфта {i + 1}')
        # plt.plot(results['time'], results['gear'], 'k--', label='Текущая передача')
        # plt.xlabel('Время (с)')
        # plt.ylabel('Давление/Передача')
        # plt.legend()
        # plt.grid(True)

        plt.tight_layout()
        plt.savefig('transmission_simulation.png', dpi=300)
        plt.show()


class GenAlgorithm:
    def __init__(self, m):
        self.generations = []
        population = []
        for i in range(m):
            ind = Individual()
            ind.init_chrom()
            #ind.graph()
            population.append(ind)

        self.generations.append(population)

    def prob_population(self, population):
        sum = 0
        res = np.zeros(len(population))
        for i in range(len(population)):
            sum += 1 / population[i].time_test

        for i in range(len(population)):
            res[i] = 1 / population[i].time_test / sum

        return res

    def get_par(self, list_prob):
        while True:
            for i in range(len(list_prob)):
                if np.random.random() < list_prob[i]:
                    return i

    def cross_par(self, par_1, par_2):
        indexes = [i for i in range(len(par_1.chrom))]
        np.random.shuffle(indexes)
        chrom = [0 for i in range(len(par_1.chrom))]
        ind = Individual()

        for i in range(len(indexes)):
            if i < 19:
                chrom[indexes[i]] = par_1.chrom[indexes[i]]
            else:
                chrom[indexes[i]] = par_2.chrom[indexes[i]]

        ind.post_chrom(chrom)
        ind.calc_time()

        return ind

    def selection(self, population):
        size = len(population)
        for i in range(size // 2):
            max_t = 0
            del_ind = 0
            for j in range(len(population)):
                if population[j].time_test > 2 and population[j].time_test > max_t:
                    max_t = population[j].time_test
                    del_ind = j

            del population[del_ind]

        return population

    def run(self, num_gen):
        population = self.generations[0]
        print('run')

        for i in range(num_gen):
            list_prob = self.prob_population(population)
            #print(list_prob)
            for j in range(len(list_prob)):
                ind_par_1 = self.get_par(list_prob)
                ind_par_2 = self.get_par(list_prob)

                while ind_par_1 == ind_par_2:
                    ind_par_2 = self.get_par(list_prob)

                par_1, par_2 = population[ind_par_1], population[ind_par_2]

                ind = self.cross_par(par_1, par_2)
                ind.mutation()

                population.append(ind)

            population = self.selection(population)
            self.generations.append(population)
            print("Gen: " + str(i))

    def get_decision(self):
        min_t = 60
        decision = Individual()
        for ind in self.generations[-1]:
            if ind.time_test < min_t:
                min_t = ind.time_test
                decision = ind

        print(decision.chrom)
        print(decision.time_test)
        decision.graph()
        return decision


if __name__ == "__main__":
    gen_algorithm = GenAlgorithm(5)
    gen_algorithm.run(10)
    decision = gen_algorithm.get_decision()
