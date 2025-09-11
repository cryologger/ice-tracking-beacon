import numpy as np
from scipy import linalg
from matplotlib import pyplot as plt
import argparse
import sys
import os

# ---------------------------------------------------------------------------
# Magnetometer calibration via ellipsoid fit
# Corrected code by S. James Remington + enhancements for CLI integration
# ---------------------------------------------------------------------------

class Magnetometer(object):
    MField = 1000  # Arbitrary normalization factor (unit sphere target)

    def __init__(self, filename, outfile=None, F=MField):
        self.filename = filename
        # Default output name: input file name with .txt extension
        self.outfile = outfile if outfile else os.path.splitext(filename)[0] + "_out.txt"
        self.F = F
        self.b = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def run(self):
        # Load calibration data
        try:
            data = np.loadtxt(self.filename, delimiter=',')
        except Exception as e:
            print(f"ERROR: Unable to load '{self.filename}': {e}")
            sys.exit(1)

        print(f"Loaded {data.shape[0]} samples from '{self.filename}'")
        print("First 5 rows raw:\n", data[:5])

        # Ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # Calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

        print("\nSoft iron transformation matrix:\n", self.A_1)
        print("Hard iron bias:\n", self.b)

        # Apply calibration to input data for visualization
        result = []
        for row in data:
            xm_off = row[0] - self.b[0]
            ym_off = row[1] - self.b[1]
            zm_off = row[2] - self.b[2]
            xm_cal = xm_off * self.A_1[0, 0] + ym_off * self.A_1[0, 1] + zm_off * self.A_1[0, 2]
            ym_cal = xm_off * self.A_1[1, 0] + ym_off * self.A_1[1, 1] + zm_off * self.A_1[1, 2]
            zm_cal = xm_off * self.A_1[2, 0] + ym_off * self.A_1[2, 1] + zm_off * self.A_1[2, 2]
            result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]))

        result = result.reshape(-1, 3)

        # Visualization
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(result[:, 0], result[:, 1], result[:, 2], marker="o", color="g")
        plt.title("Calibrated Magnetometer Data")
        plt.show()

        print("First 5 rows calibrated:\n", result[:5])

        # Save corrected data to specified output file
        np.savetxt(self.outfile, result, fmt="%f", delimiter=" ,")
        print(f"Saved calibrated data to '{self.outfile}'")

        # Print paste-ready code for firmware
        b0, b1, b2 = self.b.ravel().tolist()
        print("\n*************************")
        print("Code to paste into IMU module:")
        print("*************************")
        print("static const float M_B[3] = { %.6f, %.6f, %.6f };" % (b0, b1, b2))
        print("\nstatic const float M_Ainv[3][3] = {")
        print("  { %.6f, %.6f, %.6f }," % (self.A_1[0, 0], self.A_1[0, 1], self.A_1[0, 2]))
        print("  { %.6f, %.6f, %.6f }," % (self.A_1[1, 0], self.A_1[1, 1], self.A_1[1, 2]))  # FIXED
        print("  { %.6f, %.6f, %.6f }" % (self.A_1[2, 0], self.A_1[2, 1], self.A_1[2, 2]))
        print("};\n")

    def __ellipsoid_fit(self, s):
        """Least-squares ellipsoid fit from a set of 3D points."""
        D = np.array([
            s[0]**2., s[1]**2., s[2]**2.,
            2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
            2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])
        ])
        S = np.dot(D, D.T)
        S_11 = S[:6, :6]
        S_12 = S[:6, 6:]
        S_21 = S[6:, :6]
        S_22 = S[6:, 6:]

        C = np.array([[-1, 1, 1, 0, 0, 0],
                      [1, -1, 1, 0, 0, 0],
                      [1, 1, -1, 0, 0, 0],
                      [0, 0, 0, -4, 0, 0],
                      [0, 0, 0, 0, -4, 0],
                      [0, 0, 0, 0, 0, -4]])

        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))
        E_w, E_v = np.linalg.eig(E)
        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0:
            v_1 = -v_1

        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)
        M = np.array([[v_1[0], v_1[5], v_1[4]],
                      [v_1[5], v_1[1], v_1[3]],
                      [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]], [v_2[1]], [v_2[2]]])
        d = v_2[3]
        return M, n, d

# ---------------------------------------------------------------------------
# CLI Entry
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Magnetometer calibration from raw CSV."
    )
    parser.add_argument(
        "filename",
        nargs="?",
        default="mag3_raw.csv",
        help="Path to CSV file (default: mag3_raw.csv)"
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file. Defaults to <input_basename>.txt"
    )
    args = parser.parse_args()

    Magnetometer(args.filename, outfile=args.output).run()
