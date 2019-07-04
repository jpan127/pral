import os
import matplotlib.pyplot as plt

DATA_FOLDER = "data"
DATA = "2d_kdtree.txt"

X = []
Y = []
D = []

with open(os.path.join(DATA_FOLDER, DATA)) as f:
    lines = f.readlines()
    for line in lines:
        x, y, d = line.strip('\n').split(",")
        x, y, d = int(x), int(y), int(d)
        X.append(x)
        Y.append(y)
        D.append((d, x if d == 0 else y))

for dimension, value in D:
    if dimension == 0:
        plt.axvline(value, color='k', linestyle='--')
    else:
        plt.axhline(value, color='k', linestyle='--')

plt.plot([50], [50], marker='o', markersize=6, color="red")
plt.scatter(X, Y)
plt.show()
