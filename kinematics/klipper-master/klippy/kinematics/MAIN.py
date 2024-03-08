import tripteron
import toolhead

item = toolhead

# Example usage:
config = {
    "arm_angle": 30.0,
    "alpha_tower_rotation": 0.0,
    "beta_tower_rotation": 120.0,
    "gamma_tower_rotation": 240.0
}

# Initialize the kinematics object
solution = tripteron.ColinearTripteronKinematics(item, config)

# Example conversion from Cartesian to actuator coordinates
cartesian = [1.0, 2.0, 3.0]
actuator = tripteron.solution.cartesian_to_actuator(cartesian)
print("Actuator coordinates:", actuator)

# Example conversion from actuator to Cartesian coordinates
actuator_converted = [10.0, 20.0, 30.0]
cartesian_converted = tripteron.solution.actuator_to_cartesian(actuator_converted)
print("Converted Cartesian coordinates:", cartesian_converted)