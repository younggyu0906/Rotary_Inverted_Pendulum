import numpy as np

state = np.array([1, 2, 3, 4])
state = np.reshape(state, [1, 4, 1, 1])
history = np.zeros([1, 4, 10, 1])
for i in range(10):
    history = np.delete(history, 0, axis=2)
    history = np.append(history, state, axis=2)
history = np.reshape(history, [1, 4, 10, 1])
