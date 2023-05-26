
import math
import matplotlib.pyplot as plt
import csv
class KalmanFilter:

    cov = float('nan')
    x = float('nan')

    def __init__(self, R, Q):
        """
        Constructor

        :param R: Process Noise
        :param Q: Measurement Noise
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.R = R
        self.Q = Q

    def filter(self, measurement):
        """
        Filters a measurement

        :param measurement: The measurement value to be filtered
        :return: The filtered value
        """
        u = 0
        if math.isnan(self.x):
            self.x = (1 / self.C) * measurement
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        else:
            predX = (self.A * self.x) + (self.B * u)
            predCov = ((self.A * self.cov) * self.A) + self.R

            # Kalman Gain
            K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q));

            # Correction
            self.x = predX + K * (measurement - (self.C * predX));
            self.cov = predCov - (K * self.C * predCov);

        return self.x

    def last_measurement(self):
        """
        Returns the last measurement fed into the filter

        :return: The last measurement fed into the filter
        """
        return self.x

    def set_measurement_noise(self, noise):
        """
        Sets measurement noise

        :param noise: The new measurement noise
        """
        self.Q = noise

    def set_process_noise(self, noise):
        """
        Sets process noise

        :param noise: The new process noise
        """
        self.R = noise

# Create an instance of KalmanFilter
test = KalmanFilter(0.008, 0.1)

# Test data
testData =  [-58, -55, -56, -57, -66, -59, -58, -71, -56]

# Lists to store the data for plotting
x_values = []
filtered_values = []
# Lists to store the data for table generation and CSV export
data_rows = []
filtered_rows = []

# Iterate through the testData and apply the filter
for x in testData:
    filtered_x = test.filter(x)

    # Append the values to the respective lists
    x_values.append(x)
    filtered_values.append(filtered_x)
    data_rows.append([x])
    filtered_rows.append([filtered_x])
    # Print the data and filtered data
    print("Data:", x)
    print("Filtered Data:", filtered_x)

# Generate the CSV tables
with open('data_table.csv', 'w', newline='') as data_file:
    writer = csv.writer(data_file)
    writer.writerows(data_rows)

with open('filtered_data_table.csv', 'w', newline='') as filtered_data_file:
    writer = csv.writer(filtered_data_file)
    writer.writerows(filtered_rows)

# Plot the data and filtered data
plt.plot(x_values, label='Data')
plt.plot(filtered_values, label='Filtered Data')
plt.xlabel('Time')
plt.ylabel('Value')
plt.legend()
plt.show()
