from matplotlib import pyplot as plt
import math

distances = [0.46,  0.668, 0.876, 1.084, 1.292, 1.5,  ] # Distances
human = (0.65, 2.35)
rel_points = [(human[0] + d*math.sqrt(2)/2, human[1] - d*math.sqrt(2)/2) for d in distances]
s = 380

for i, point in enumerate(rel_points):
    fig = plt.figure()
    plt.scatter(human[0], human[1], color="green", s=s, label="Human")
    plt.scatter(point[0], point[1], color="red", s=s, label="Robot")
    plt.xlim((0.0, 3.0))
    plt.ylim((0.0, 3.0))
    plt.title("Top view of the scene")
    plt.grid(True)
    plt.legend()
    fig.savefig(f"./{distances[i]}_matrix_plot.png")
    
