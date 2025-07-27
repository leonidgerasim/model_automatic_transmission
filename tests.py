import numpy as np

m = [i for i in range(10)]
np.random.shuffle(m)
print(m)

m[1:3] = [1, 2]
print(m)
