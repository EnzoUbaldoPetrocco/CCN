import numpy as np

# Generate 7 evenly spaced points between 0.46 and 2.1 (inclusive)
points = np.linspace(0.46, 1.5, 6)

# Apply a scale factor
#Scale X: 0.046153846153846156, Scale Y: 0.06276595744680852
scale_factor = 0.05 #(0.046153846153846156 + 0.06276595744680852)/2
scaled_points = points / scale_factor

# Output
print("Original Points:", points)
print("Scaled Points:", scaled_points)
