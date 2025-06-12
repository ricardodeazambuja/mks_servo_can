# Implementing Custom Kinematics

While `RotaryKinematics` and `LinearKinematics` cover many common scenarios, some mechanical systems have a more complex, often non-linear, relationship between motor movement (encoder steps) and the physical output position or speed. For these cases, the `mks-servo-can` library allows you to define your own custom kinematics by creating a class that inherits from `BaseKinematics`.

## Prerequisites

* A solid understanding of the kinematics system. See [Using Kinematics](./kinematics.md).
* Knowledge of Python classes and object-oriented programming.
* The mathematical model describing the relationship between your motor's rotation and the desired physical output of your mechanism.

## The `BaseKinematics` Abstract Class

The foundation for all kinematics conversions is the `mks_servo_can.kinematics.base_kinematics.BaseKinematics` abstract base class. To create a custom kinematics model, you must subclass `BaseKinematics` and implement its abstract methods.

### Required Methods and Properties

Your custom kinematics class must implement the following:

1. **`units: str`**
   * A string property or attribute that defines the physical units your kinematics class works with (e.g., "mm", "degrees", "gantry_angle_rad"). This is used for display and logging.

2. **`get_info() -> str`**
   * A method that returns a descriptive string about the kinematics setup (e.g., "Custom SCARA Arm Kinematics - Joint 1 (degrees)").

3. **`steps_to_position(self, steps: int) -> float`**
   * Converts a raw motor encoder step count into a physical position in your defined `units`.
   * `steps`: Absolute motor position in encoder steps.
   * Returns: The corresponding physical position as a float.

4. **`position_to_steps(self, position: float) -> int`**
   * Converts a physical position (in your `units`) into the corresponding raw motor encoder step count.
   * `position`: Absolute physical position.
   * Returns: The corresponding motor position in encoder steps as an integer.

5. **`steps_sec_to_speed(self, steps_sec: float) -> float`**
   * Converts a motor speed from steps per second into a physical speed in your defined `units` per second.
   * `steps_sec`: Motor speed in encoder steps per second.
   * Returns: The corresponding physical speed.

6. **`speed_to_steps_sec(self, speed: float) -> float`**
   * Converts a physical speed (in your `units` per second) into motor speed in steps per second.
   * `speed`: Physical speed.
   * Returns: The corresponding motor speed in encoder steps per second.

### Optional Methods (Override if needed)

* **`steps_sec2_to_acceleration(self, steps_sec2: float) -> float`**: Converts motor acceleration (steps/s^2) to physical acceleration. By default, this uses the speed conversion logic, which is often sufficient if the relationship is linear or if acceleration units are directly analogous to speed units.
* **`acceleration_to_steps_sec2(self, acceleration: float) -> float`**: Converts physical acceleration to motor acceleration (steps/s^2). Also defaults to using speed conversion logic.

## Example: `EccentricKinematics`

The library includes `EccentricKinematics` as an example of a non-linear custom kinematics model. It describes a scenario where a rotary motor drives an eccentric mechanism, resulting in a non-linear relationship between motor angle and the linear position of an output.

Let's look at its structure (simplified):

```python
import math
from mks_servo_can.kinematics.base_kinematics import BaseKinematics
from mks_servo_can import const

class EccentricKinematics(BaseKinematics):
    def __init__(self,
                 steps_per_revolution: int = const.ENCODER_PULSES_PER_REVOLUTION,
                 eccentricity_mm: float = 5.0,
                 arm_length_mm: float = 20.0,
                 gear_ratio: float = 1.0,
                 offset_angle_deg: float = 0.0):
        super().__init__()
        self.steps_per_revolution = steps_per_revolution
        self.eccentricity = eccentricity_mm
        self.arm_length = arm_length_mm
        self.gear_ratio = gear_ratio # Motor revs per one eccentric mechanism rev
        self.offset_angle_rad = math.radians(offset_angle_deg)
        self._units = "mm"

    @property
    def units(self) -> str:
        return self._units

    def get_info(self) -> str:
        return (f"EccentricKinematics(e={self.eccentricity}{self.units}, "
                f"l={self.arm_length}{self.units}, gear={self.gear_ratio}, "
                f"offset={math.degrees(self.offset_angle_rad):.1f}deg)")

    def motor_angle_to_position(self, motor_angle_rad: float) -> float:
        # Actual math for the eccentric mechanism:
        # Effective angle of the eccentric part of the mechanism
        mechanism_angle_rad = (motor_angle_rad / self.gear_ratio) + self.offset_angle_rad
        # Law of cosines or other geometric calculation
        # x = e * cos(theta_m) + sqrt(L^2 - (e * sin(theta_m))^2)
        # This is a simplified conceptual placeholder for the actual geometric formula
        term_under_sqrt = self.arm_length**2 - (self.eccentricity * math.sin(mechanism_angle_rad))**2
        if term_under_sqrt < 0:
            # This can happen if eccentricity is too large for the arm length
            # Or if the angle results in an impossible configuration.
            # Handle appropriately, e.g., raise ValueError or return a boundary value.
            # For simplicity, let's assume valid configurations for this example.
            # A real implementation would need robust handling here.
            # This might indicate a design issue or that the motor angle is out of a valid range
            # for the desired linear motion.
            # One approach is to clamp to the nearest valid position.
            # For this example, we'll just note it.
            # print(f"Warning: Invalid configuration for angle {mechanism_angle_rad}")
            # A more robust solution would involve checking bounds or raising an error.
            # This example will proceed, potentially yielding math errors if term_under_sqrt is negative.
            # A production system would need to carefully define the valid range of motion.
            pass # Fallthrough, may lead to math domain error if not handled

        position = (self.eccentricity * math.cos(mechanism_angle_rad) +
                    math.sqrt(max(0, term_under_sqrt))) # max(0,..) to prevent math domain error
        return position


    def position_to_motor_angle(self, position_mm: float) -> float:
        # Inverse kinematics: more complex, may have multiple solutions or require iteration.
        # For this example, let's assume a simplified scenario or a lookup table if analytical is too hard.
        # This is highly dependent on the specific mechanism's geometry.
        # Placeholder: This would involve solving the equation from motor_angle_to_position for mechanism_angle_rad
        # e.g., using numerical methods or an analytical solution if available.
        # For this example, we'll return a conceptual value.
        # A real implementation is critical here.
        # For instance, if x = e * cos(theta) + L (simplified case if arm is aligned)
        # then cos(theta) = (x - L) / e => theta = acos((x - L) / e)
        # This is a gross oversimplification for the eccentric example.
        # The actual inverse for the given forward kinematics is non-trivial.
        
        # Due to the complexity of the true inverse for the example forward kinematics,
        # we'll use a conceptual, non-functional placeholder.
        # A real implementation would require solving:
        # position_mm = self.eccentricity * math.cos(mechanism_angle_rad) +
        #               math.sqrt(self.arm_length**2 - (self.eccentricity * math.sin(mechanism_angle_rad))**2)
        # for mechanism_angle_rad. This often requires numerical methods.

        # For demonstration, let's imagine a very simplified inverse relationship for a different mechanism
        # where motor angle is proportional to position after some offset and scaling.
        # This is NOT the inverse of the above `motor_angle_to_position`.
        if self.eccentricity == 0: # Avoid division by zero if e is used in a real inverse
            return 0.0 
        
        # Conceptual simplified inverse (NOT MATCHING THE FORWARD KINEMATICS ABOVE)
        # This is just to show the structure.
        # Assume mechanism_angle_rad is somehow derived from position_mm
        # For example, if it were a simple linear to rotary:
        # mechanism_angle_rad = (position_mm / some_scaling_factor) - self.offset_angle_rad
        # This is highly mechanism-dependent.
        # For the eccentric example, this is a placeholder.
        # A practical approach for complex non-linear kinematics might involve:
        # 1. Numerical solver (e.g., Newton-Raphson, bisection)
        # 2. Precomputed lookup table with interpolation
        # 3. Analytical solution if one exists (can be very complex)
        # For this placeholder, let's assume a fictional direct relationship for structure.
        # This will not work correctly with the forward kinematics provided.
        mechanism_angle_rad = math.acos( (position_mm - self.arm_length) / self.eccentricity ) if self.eccentricity !=0 else 0
        
        motor_angle_rad = (mechanism_angle_rad - self.offset_angle_rad) * self.gear_ratio
        return motor_angle_rad


    def steps_to_position(self, steps: int) -> float:
        motor_angle_rad = (steps / self.steps_per_revolution) * 2 * math.pi
        return self.motor_angle_to_position(motor_angle_rad)

    def position_to_steps(self, position: float) -> int:
        motor_angle_rad = self.position_to_motor_angle(position)
        return int((motor_angle_rad / (2 * math.pi)) * self.steps_per_revolution)

    # For speed and acceleration, if the relationship is non-linear,
    # the derivatives (dX/dTheta for speed, d^2X/dTheta^2 for accel) are needed.
    # By default, BaseKinematics uses a linear scaling based on position conversion,
    # which is an approximation for non-linear systems.
    # For accurate speed/accel in non-linear systems, these should also be overridden.

    def steps_sec_to_speed(self, steps_sec: float) -> float:
        # This is an approximation for non-linear systems if not overridden with derivatives.
        # It assumes d_pos/d_steps is locally constant.
        # For true non-linear speed, you'd use the derivative of position wrt motor angle.
        # d(pos)/dt = d(pos)/d(motor_angle) * d(motor_angle)/dt
        # d(motor_angle)/dt is (steps_sec / steps_per_rev) * 2 * pi
        # d(pos)/d(motor_angle) is the Jacobian of your mechanism.
        # For simplicity, the base class implementation does:
        # return self.steps_to_position(int(steps_sec)) - self.steps_to_position(0)
        # which is a linear approximation based on total steps in one second.
        # A more accurate way for EccentricKinematics would be to calculate the Jacobian.
        # However, for this example, we'll rely on the base class's approximation or implement a simple one.
        
        # Let's use the base class's default behavior for simplicity in this example,
        # acknowledging it's an approximation for non-linear cases.
        # If high accuracy speed conversion is needed for a non-linear system,
        # this method MUST be implemented using the derivative of the position function.
        motor_angular_velocity_rad_s = (steps_sec / self.steps_per_revolution) * 2 * math.pi
        
        # Placeholder for Jacobian calculation: J = d(position)/d(mechanism_angle_rad)
        # This requires differentiating the motor_angle_to_position formula.
        # For example, if position = f(mechanism_angle_rad), then J = f'(mechanism_angle_rad).
        # Then, physical_speed = J * (motor_angular_velocity_rad_s / self.gear_ratio).
        # This is complex and mechanism-specific.
        # For this example, we'll return a conceptual value based on motor angular velocity.
        # This is NOT a generally correct conversion for non-linear speed.
        return motor_angular_velocity_rad_s * self.eccentricity # Highly simplified, conceptual

    def speed_to_steps_sec(self, speed: float) -> float:
        # Inverse of the above, also an approximation for non-linear systems.
        # motor_angular_velocity_rad_s = physical_speed / J_approximated
        # steps_sec = (motor_angular_velocity_rad_s / (2 * math.pi)) * self.steps_per_revolution
        
        # Using a similar conceptual simplification as above.
        motor_angular_velocity_rad_s = speed / self.eccentricity if self.eccentricity !=0 else 0
        return (motor_angular_velocity_rad_s / (2 * math.pi)) * self.steps_per_revolution
```

## Key Takeaways from EccentricKinematics:

- It stores mechanism-specific parameters (eccentricity_mm, arm_length_mm).
- `steps_to_position` converts steps to a motor angle, then uses a geometric formula (`motor_angle_to_position`) to get the linear output.
- `position_to_steps` would ideally implement the inverse of that geometric formula. Note: The example above has a placeholder for `position_to_motor_angle` due to the complexity of the true inverse; a real implementation would need a robust solution.
- Speed and acceleration conversions for non-linear systems ideally require calculus (derivatives/Jacobians). The base class provides a linear approximation if these are not overridden. The EccentricKinematics example above also uses simplified speed conversions for brevity.

## Steps to Create Your Custom Kinematics

### 1. Model Your Mechanism:
- Derive the forward kinematics equation: physical_position = f(motor_steps_or_angle)
- Derive the inverse kinematics equation: motor_steps_or_angle = g(physical_position)
- If precise speed/acceleration control is needed, derive the derivatives (Jacobian).

### 2. Create a New Python Class:

```python
import math
from mks_servo_can.kinematics.base_kinematics import BaseKinematics

class MyCustomKinematics(BaseKinematics):
    def __init__(self, steps_per_motor_revolution: int, param1: float, param2: float, units_str: str = "my_units"):
        super().__init__()
        self.steps_per_rev = steps_per_motor_revolution
        self.param1 = param1 # Your mechanism-specific parameter
        self.param2 = param2 # Another parameter
        self._units = units_str
        # Initialize any other necessary attributes

    @property
    def units(self) -> str:
        return self._units

    def get_info(self) -> str:
        return f"MyCustomKinematics(param1={self.param1}, param2={self.param2}, units={self.units})"

    def steps_to_position(self, steps: int) -> float:
        motor_angle_rad = (steps / self.steps_per_rev) * 2 * math.pi
        # Apply your forward kinematics formula: f(motor_angle_rad, self.param1, self.param2)
        physical_position = (motor_angle_rad * self.param1) + self.param2 # Replace with your actual formula
        return physical_position

    def position_to_steps(self, position: float) -> int:
        # Apply your inverse kinematics formula: g(position, self.param1, self.param2)
        # This gives motor_angle_rad
        motor_angle_rad = (position - self.param2) / self.param1 # Replace with your actual formula
        steps = int((motor_angle_rad / (2 * math.pi)) * self.steps_per_rev)
        return steps

    def steps_sec_to_speed(self, steps_sec: float) -> float:
        # If linear, this might be:
        # motor_angular_speed_rad_s = (steps_sec / self.steps_per_rev) * 2 * math.pi
        # physical_speed = motor_angular_speed_rad_s * self.param1 # If param1 is a linear scaling factor
        # return physical_speed
        # For non-linear, implement using derivatives or accept BaseKinematics approximation.
        return super().steps_sec_to_speed(steps_sec) # Or your custom implementation

    def speed_to_steps_sec(self, speed: float) -> float:
        # If linear, this might be:
        # motor_angular_speed_rad_s = speed / self.param1
        # steps_per_sec = (motor_angular_speed_rad_s / (2 * math.pi)) * self.steps_per_rev
        # return steps_per_sec
        # For non-linear, implement using derivatives or accept BaseKinematics approximation.
        return super().speed_to_steps_sec(speed) # Or your custom implementation
```

### 3. Implement the Conversion Logic: 
Fill in the mathematical formulas specific to your mechanism in the methods. Pay special attention to the inverse kinematics (`position_to_steps`), which can sometimes be challenging to derive analytically and might require numerical methods or lookup tables for complex systems.

### 4. Handle Edge Cases and Errors: 
Consider what happens at the limits of motion, or if a position is unreachable. Your conversion functions should be robust (e.g., avoid math.domain errors by checking inputs).

## Using Your Custom Kinematics

Once your custom kinematics class is defined, you can instantiate it and assign it to an Axis object just like the built-in types:

```python
from mks_servo_can import Axis, CANInterface
import asyncio

# Assuming MyCustomKinematics is defined as above
async def use_custom_kinematics():
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()

    # Initialize your custom kinematics with its specific parameters
    my_kin_model = MyCustomKinematics(steps_per_motor_revolution=16384, param1=10.0, param2=5.0, units_str="widgets")
    
    motor_axis = Axis(can_if, can_id=1, name="CustomAxis", kinematics=my_kin_model)
    print(f"Initialized {motor_axis.name} with kinematics: {motor_axis.kinematics.get_info()}")

    try:
        await motor_axis.enable_motor()
        # Now, position and speed values are in "widgets"
        await motor_axis.move_absolute(target_position=100.0, speed=10.0) # 100 widgets, 10 widgets/s
        current_pos = await motor_axis.get_current_position()
        print(f"Moved to {current_pos:.2f} {motor_axis.kinematics.units}")
        await motor_axis.disable_motor()
    except Exception as e:
        print(f"Error during custom kinematics usage: {e}")
    finally:
        if can_if.is_connected:
            await can_if.disconnect()

asyncio.run(use_custom_kinematics())
```

By creating custom kinematics classes, you can adapt the mks-servo-can library to a wide variety of mechanical systems, keeping your application-level code clean and expressed in meaningful physical units rather than raw motor steps.