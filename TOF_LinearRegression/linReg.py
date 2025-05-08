import numpy as np
from sklearn.linear_model import LinearRegression

# Sensor 1 measurements
measured1_1 = np.array([
    12, 22, 24, 34, 39, 38, 43, 46, 52, 60,
    65, 66, 68, 77, 85, 87, 100, 106, 107, 107,
    119, 120, 126, 130, 135, 142, 145, 151, 156, 159,
    164, 169, 174, 176, 171
]) # from 3 to 20 cm
measured1_2 = np.array([
    7, 23, 11, 20, 29, 33, 35, 46, 51, 51, 57, 62, 65, 75, 77, 81, 89, 92, 98, 106,
    107, 110, 119, 123, 132, 136, 144, 148, 148, 155, 164, 163, 172
]) # from 3 to 19 cm
measured1_3 = np.array([
    8, 12, 14, 15, 18, 31, 32, 40, 46, 48, 56, 51, 64, 70, 76, 74, 83, 93, 100, 103,
    105, 112, 119, 123, 125, 133, 136, 147, 147, 151, 161, 155, 173, 180, 188, 183, 189
]) # from 2 to 20 cm
measured1 = np.concatenate((measured1_1, measured1_2, measured1_3))
#print(measured1)

# Sensor 2 measurements
measured2_1 = np.array([37, 48, 47, 53, 60, 64,
    72, 74, 79, 83, 91, 99, 102, 104, 104, 105,
    112, 119, 127, 130, 127, 146, 139, 148, 155,
    153, 159, 161, 168, 167, 174, 176, 176, 181, 200
]) # from 3 to 20 cm
measured2_2 = np.array([
    33, 65, 42, 48, 53, 59, 63, 72, 71, 77, 83, 89, 95, 96, 102, 108, 111, 117, 120,
    126, 124, 129, 134, 138, 150, 146, 158, 161, 167, 169, 171, 183, 181
]) # from 3 to 19 cm
measured2_3 = np.array([
    23, 22, 31, 37, 45, 47, 51, 58, 65, 68, 71, 80, 87, 90, 96, 97, 102, 112, 112, 117,
    125, 130, 136, 138, 146, 152, 159, 161, 164, 175, 170, 178, 183, 183, 255, 255, 255
]) # from 2 to 20 cm
measured2 = np.concatenate((measured2_1, measured2_2, measured2_3))
#print(measured2)

measured = measured1

# Real values (calibration points)
init_cm = 3    # initial value in cm
end_cm = 20    # end value in cm
reference_1 = np.arange(10 * init_cm, 10 * end_cm + 1, 5)
init_cm = 3    # initial value in cm
end_cm = 19    # end value in cm
reference_2 = np.arange(10 * init_cm, 10 * end_cm + 1, 5)
init_cm = 2    # initial value in cm
end_cm = 20    # end value in cm
reference_3 = np.arange(10 * init_cm, 10 * end_cm + 1, 5)
reference = np.concatenate((reference_1, reference_2, reference_3))
#print(reference)

#print(f"Length of measured2: {len(measured2)}")
#print(f"Length of reference: {len(reference)}")

# === Must be same size ===
if len(measured) != len(reference):
    print(f"Warning: Mismatched data points (measured: {len(measured)}, reference: {len(reference)})")
else:
    # === Compute linear regression ===
    if len(measured) > 0:
        # Linear Regression
        linear_model = LinearRegression().fit(measured.reshape(-1, 1), reference)
        a = linear_model.coef_[0]
        b = linear_model.intercept_

        # Calculate R-squared value
        r_squared = linear_model.score(measured.reshape(-1, 1), reference)
        
        print(f"\nLinear Calibration results:")
        print(f"y = {a:.8f} * x + {b:.8f}")
        print(f"R-squared: {r_squared:.8f}")
        
        print(f"\nRaw vs Reference values:")
        for m, r in zip(measured, reference):
            print(f"{m:.1f} mm -> {r:.1f} mm")
        
        print("\nLinear Calibration quality indicators:")
        residuals = reference - (a * measured + b)
        print(f"Max residual: {np.max(np.abs(residuals)):.8f} mm")
        print(f"Mean absolute error: {np.mean(np.abs(residuals)):.8f} mm")
        print(f"Standard deviation: {np.std(residuals):.8f} mm")
    else:
        print("No valid data available for calibration")