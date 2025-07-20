import pandas as pd
import numpy as np
from numpy.f2py.crackfortran import privatepattern


class Environment:
    def __init__(self, road_slop=0, dt=0.1, t=10):
        self.accr_ped = 0
        self.brk_ped = 0
        self.road_slop = road_slop
        self.dt = dt
        self.iter_t = 0
        self.t = t

    def acceleration(self, press):
        match press:
            case 0:
                if self.accr_ped - 200 * self.dt >= 0:
                    self.accr_ped -= 200 * self.dt
                else:
                    self.accr_ped = 0
            case 1:
                if self.accr_ped + 200 * self.dt <= 100:
                    self.accr_ped += 200 * self.dt
                else:
                    self.accr_ped = 100

    def braking(self, press):
        match press:
            case 0:
                if self.brk_ped - 200 * self.dt >= 0:
                    self.brk_ped -= 200 * self.dt
                else:
                    self.brk_ped = 0
            case 1:
                if self.brk_ped + 200 * self.dt <= 100:
                    self.brk_ped += 200 * self.dt
                else:
                    self.brk_ped = 100

    def run_simulate(self, scenario_acc, scenario_brk, timing_acc, timing_brk):
        iter_acc = 0
        init_acc = False
        len_acc = len(scenario_acc)
        iter_brk = 0
        init_brk = False
        len_brk = len(scenario_brk)
        while self.iter_t < self.t:

            if timing_acc == self.iter_t:
                init_acc = True
            if init_acc and iter_acc < len_acc:
                self.acceleration(scenario_acc[iter_acc])
                iter_acc += 1

            if timing_brk == self.iter_t:
                init_brk = True
            if init_brk and iter_brk < len_brk:
                self.braking(scenario_brk[iter_brk])
                iter_brk += 1

            self.iter_t += self.dt



if __name__ == "__main__":
    env = Environment(dt=0.01)
    scenario_acc = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]
    scenario_brk = [1, 1, 1, 1, 1, 0, 0, 0]
    timing_acc = 0
    timing_brk = 0
    env.run_simulate(scenario_acc=scenario_acc, scenario_brk=scenario_brk, timing_acc=timing_acc, timing_brk=timing_brk)
    print(env.accr_ped, env.brk_ped)


