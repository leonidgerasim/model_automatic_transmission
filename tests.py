from run_simulate import Engine

eng = Engine()

dt = 0.01
for i in range(1000):
    eng.engine_update(50, 200, dt)
    print(eng.get())
