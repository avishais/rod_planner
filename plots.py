import numpy as np
import matplotlib.pyplot as plt 


m = [20, 50, 100, 500, 1000]
s1 = [2, 5, 9, 10, 10]
s2 = [1, 4, 10, 10, 10]

plt.plot(m, s1, '-', label='env. I')
plt.plot(m, s2, '-', label='env. II')
plt.legend()
plt.show()