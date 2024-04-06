import math

class ColinearTripteronSolution:
    def __init__(self, arm_length=100.0, arm_angle=30.0, alpha_tower_rotation=0.0, beta_tower_rotation=120.0, gamma_tower_rotation=240.0):
        self.arm_length = arm_length
        self.steps_per_revolution = 200  # Number of steps per revolution for stepper motor

        self.a_a = math.radians(arm_angle)
        self.a_t = math.tan(self.a_a)

        self.a_r = math.radians(alpha_tower_rotation)
        self.b_r = math.radians(beta_tower_rotation)
        self.g_r = math.radians(gamma_tower_rotation)

        self.a_x = math.sin(self.a_r) * self.arm_length
        self.a_y = math.cos(self.a_r) * self.arm_length
        self.b_x = math.sin(self.b_r) * self.arm_length
        self.b_y = math.cos(self.b_r) * self.arm_length
        self.g_x = math.sin(self.g_r) * self.arm_length
        self.g_y = math.cos(self.g_r) * self.arm_length

        self.d = self.a_y * self.b_x - self.g_y * self.b_x - self.a_x * self.b_y - self.a_y * self.g_x + self.b_y * self.g_x + self.a_x * self.g_y

    def cartesian_to_actuator(self, cartesian_mm):
        a_x, a_y = self.a_x, self.a_y
        b_x, b_y = self.b_x, self.b_y
        g_x, g_y = self.g_x, self.g_y

        alpha_stepper = (a_x * cartesian_mm[0] - a_y * cartesian_mm[1] + cartesian_mm[2]) / self.steps_per_revolution
        beta_stepper = (b_x * cartesian_mm[0] - b_y * cartesian_mm[1] + cartesian_mm[2]) / self.steps_per_revolution
        gamma_stepper = (g_x * cartesian_mm[0] - g_y * cartesian_mm[1] + cartesian_mm[2]) / self.steps_per_revolution

        return alpha_stepper, beta_stepper, gamma_stepper

    def actuator_to_cartesian(self, actuator_mm):
        a_x, a_y = self.a_x, self.a_y
        b_x, b_y = self.b_x, self.b_y
        g_x, g_y = self.g_x, self.g_y

        x = ((actuator_mm[0] * (g_y - b_y) + actuator_mm[1] * (a_y - g_y) + actuator_mm[2] * (b_y - a_y)) / self.d) * self.steps_per_revolution
        y = ((actuator_mm[0] * (g_x - b_x) + actuator_mm[1] * (a_x - g_x) + actuator_mm[2] * (b_x - a_x)) / self.d) * self.steps_per_revolution
        z = ((actuator_mm[0] * (b_y * g_x - b_x * g_y) + actuator_mm[1] * (a_x * g_y - a_y * g_x) + actuator_mm[2] * (a_y * b_x - a_x * b_y)) / self.d) * self.steps_per_revolution

        return x, y, z

# Example usage:
if __name__ == "__main__":
    kinematics = ColinearTripteronSolution(arm_length=100)  # Specify the arm length
    cartesian_coordinates = [100, 200, 50]  # Example Cartesian coordinates in millimeters
    actuator_coordinates = kinematics.cartesian_to_actuator(cartesian_coordinates)
    print("Actuator Coordinates:", actuator_coordinates)

    # Convert back to Cartesian coordinates
    converted_cartesian = kinematics.actuator_to_cartesian(actuator_coordinates)
    print("Converted Cartesian Coordinates:", converted_cartesian)
