import numpy as np
from sklearn.linear_model import LinearRegression

def read_distance_values(file_path):
    """Read distance values from file into a Python array."""
    distances = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()  # Remove any whitespace/newlines
                if line:  # Make sure line isn't empty
                    try:
                        distance = float(line)  # Convert to float for better precision
                        distances.append(distance)
                    except ValueError:
                        print(f"Skipping invalid line: {line}")
        return np.array(distances)
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        return np.array([])

# === Load measured data from file ===
valuesPath = "/path/to/file/values.txt"
measured = read_distance_values(valuesPath)  # Your sensor measurements

# === Edit your reference data here ===
# These should match the same points where you took measurements
reference = np.array([10.5, 21.0, 29.5, 41.0])   # Real values (calibration points)

# Verify we have matching data points
if len(measured) != len(reference):
    print(f"Warning: Mismatched data points (measured: {len(measured)}, reference: {len(reference)})")
    # Use only the minimum available points
    min_points = min(len(measured), len(reference))
    measured = measured[:min_points]
    reference = reference[:min_points]

# === Compute linear regression ===
if len(measured) > 0:
    model = LinearRegression().fit(measured.reshape(-1, 1), reference)
    a = model.coef_[0]
    b = model.intercept_

    # Calculate R-squared value
    r_squared = model.score(measured.reshape(-1, 1), reference)
    
    print(f"\nCalibration results:")
    print(f"y = {a:.8f} * x + {b:.8f}")
    print(f"R-squared: {r_squared:.8f}")
    
    print(f"\nRaw vs Reference values:")
    for m, r in zip(measured, reference):
        print(f"{m:.8f} mm -> {r:.8f} mm")
    
    print("\nCalibration quality indicators:")
    residuals = reference - (a * measured + b)
    print(f"Max residual: {np.max(np.abs(residuals)):.8f} mm")
    print(f"Mean absolute error: {np.mean(np.abs(residuals)):.8f} mm")
    print(f"Standard deviation: {np.std(residuals):.8f} mm")
else:
    print("No valid data available for calibration")