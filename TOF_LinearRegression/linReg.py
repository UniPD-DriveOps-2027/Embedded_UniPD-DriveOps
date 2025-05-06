import numpy as np
from sklearn.linear_model import LinearRegression


# Sensor measurements
measured = np.array([
    12, 22, 24, 34, 39, 38, 43, 46, 52, 60,
    65, 66, 68, 77, 85, 87, 100, 106, 107, 107,
    119, 120, 126, 130, 135, 142, 145, 151, 156, 159,
    164, 169, 174, 176, 171
])
print(measured)
# Real values (calibration points)
init_cm = 3    # initial value in cm
end_cm = 20    # end value in cm
reference = np.arange(10 * init_cm, 10 * end_cm + 1, 5)
print(reference)

# === Must be same size ===
if len(measured) != len(reference):
    print(f"Warning: Mismatched data points (measured: {len(measured)}, reference: {len(reference)})")
else:
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
            print(f"{m:.1f} mm -> {r:.1f} mm")
        
        print("\nCalibration quality indicators:")
        residuals = reference - (a * measured + b)
        print(f"Max residual: {np.max(np.abs(residuals)):.8f} mm")
        print(f"Mean absolute error: {np.mean(np.abs(residuals)):.8f} mm")
        print(f"Standard deviation: {np.std(residuals):.8f} mm")
    else:
        print("No valid data available for calibration")