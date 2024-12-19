import math
import numpy as np
import matplotlib.pyplot as plt


def forward_kinematics(theta1, theta2, l0, l1, l2):
    A = 2 * l1 * l2 * np.sin(theta2) - 2 * l1 * l2 * np.sin(theta1)
    B = 2 * l1 * l0 - 2 * l1 * l2 * np.cos(theta1) + 2 * l1 * l2 * np.cos(theta2)
    C = 2 * l1**2 + l0**2 - 2 * l1 * l2 * np.cos(theta2) * np.cos(theta1) - 2 * l1 * l2 * np.sin(theta2) * np.sin(theta1) + 2 * l1 * l0 * np.cos(theta2) - 2 * l1 * l0 * np.cos(theta1)
    B2 = 2 * math.atan((A + math.sqrt(A**2 + B**2 - C**2)) / (B - C))
    x = l0 + l1 * np.cos(theta2) + l2 * np.cos(B2)
    y = l1 * np.sin(theta2) + l2 * np.sin(B2)
    return x, y, B2


theta1 = np.deg2rad(135)
theta2 = np.pi / 4
l0 = 35  
l1 = 40  
l2 = 69  


x, y, B2 = forward_kinematics(theta1, theta2, l0, l1, l2)


base = (0, 0)
joint1 = (l0, 0)
joint2 = (l0 + l1 * np.cos(theta2), l1 * np.sin(theta2))
end_effector = (x, y)
additional_point = (0 - (np.cos(np.pi - theta1) * l1), np.sin(np.pi - theta1) * l1)


perpendicular_vector = np.array([0, 1])  
delta_x = end_effector[0] - additional_point[0]
delta_y = end_effector[1] - additional_point[1]
extended_vector = np.array([delta_x, delta_y])  


dot_product = np.dot(perpendicular_vector, extended_vector)


magnitude_perpendicular = np.linalg.norm(perpendicular_vector)
magnitude_extended = np.linalg.norm(extended_vector)


cos_angle = dot_product / (magnitude_perpendicular * magnitude_extended)


angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))
angle_deg = np.degrees(angle_rad)

print(f"The angle between the perpendicular line and the extended line from additional point to end effector is {angle_deg:.2f} degrees.")

# # Plotting
# plt.figure(figsize=(8, 8))

# # Plot the robot arm and other lines
# plt.plot([base[0], joint1[0], joint2[0], end_effector[0]],
#          [base[1], joint1[1], joint2[1], end_effector[1]], marker='o', linestyle='-', color='b', label="Robot Arm")
# plt.plot([additional_point[0], end_effector[0]],
#          [additional_point[1], end_effector[1]], linestyle='-', color='g', label="Extended Line (Additional Point to End Effector)")

# # Perpendicular line
# perpendicular_point = (end_effector[0], 0)
# plt.plot([end_effector[0], perpendicular_point[0]],
#          [end_effector[1], perpendicular_point[1]], linestyle='-', color='purple', label="Perpendicular Line")

# # Labels
# plt.text(base[0], base[1], "Base", fontsize=10, ha='right')
# plt.text(joint1[0], joint1[1], "Joint 1", fontsize=10, ha='right')
# plt.text(joint2[0], joint2[1], "Joint 2", fontsize=10, ha='right')
# plt.text(end_effector[0], end_effector[1], "End-Effector", fontsize=10, ha='right')
# plt.text(additional_point[0], additional_point[1], "Additional Point", fontsize=10, ha='right')
# plt.text(perpendicular_point[0], perpendicular_point[1], "Perpendicular Point", fontsize=10, ha='right')

# mid_point = [(additional_point[0] + end_effector[0]) / 2,
#              (additional_point[1] + end_effector[1]) / 2]
# plt.text(mid_point[0], mid_point[1], f"Angle: {angle_deg:.2f}°", fontsize=12, ha='center')

# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("Robot Arm with Extended Line and Perpendicular (Flipped)")
# plt.legend()
# plt.grid()
# plt.axis("equal")

# # Invert the y-axis without modifying coordinates
# plt.gca().invert_yaxis()

# plt.tight_layout()

# plt.show()
