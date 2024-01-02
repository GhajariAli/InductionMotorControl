import serial
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM6', 115200)

max_angle = 180  # Maximum angle to display
max_points = 100  # Maximum number of data points to display
angles = []
distances = []

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

try:
    while True:
        data = ser.readline().decode().strip()

        if data:
            parsed_data = data.split(',')
            for item in parsed_data:
                if '=' in item:
                    key, value = item.split('=')
                    if key == 'Distance':
                        distances.append(int(value))
                    elif key == 'ServoAngle':
                        angle = int(value)
                        scaled_angle = np.interp(angle, [250, 1250], [0, -180])
                        angles.append(scaled_angle)

            # Limit the number of stored points
            angles = angles[-max_points:]
            distances = distances[-max_points:]

            if len(angles) == len(distances):  # Check if dimensions match
                # Convert angles to radians for polar plot
                angles_rad = np.radians(angles)

                # Clear the previous plot
                ax.clear()

                # Plot the updated data within 180 degrees
                ax.plot(angles_rad, distances, marker='o')

                # Set the angular limit to 180 degrees
                ax.set_theta_zero_location('N')
                ax.set_theta_direction(-1)  # Set counterclockwise direction

                # Hide the bottom half of the plot
                ax.set_ylim(0, max(distances))
                ax.set_yticklabels([])  # Hide y-axis labels for the bottom half

                # Show the updated plot
                plt.pause(0.01)

except Exception as e:
    print(f"Exception occurred: {e}")

finally:
    ser.close()
    plt.show()  # Keep the plot window open after data collection ends
