//  package frc.robot.subsystems.Templates;

// /*
//  * Control Methods Overview:
//  *
//  * The `PID_and_FF` subsystem offers three key control methods:
//  * - `setPosition`: Used for controlling precise positions with PID and feedforward.
//  *   Ideal for systems that need to reach a fixed location or angle, such as turrets or pivoting arms.
//  * - `setVelocity`: Designed to control speed directly, perfect for systems that require constant motion, like flywheels and drive motors. Ensures the motor maintains a specific speed.
//  * - `setProfiledPosition`: Combines motion profiling with maximum velocity and acceleration constraints for smooth and controlled movement. Ideal for systems like elevators and telescoping arms that need controlled, smooth motion between set points.
//  *
//  * System-by-System Control Recommendations:
//  *
//  * - **Turret**: Use `setPosition` for precise angular control. The turret requires exact positioning, and `setPosition` will smoothly move the turret to the desired angle using PID and feedforward.
//  *
//  * - **Flywheel**: Use `setVelocity` to maintain a consistent rotational speed. Flywheels must spin at a constant speed for optimal performance, making `setVelocity` the best method for this mechanism.
//  *
//  * - **Swerve Drive (Drive)**: Use `setVelocity` to control wheel speed. The drive system requires quick and consistent motion, so `setVelocity` allows you to set and maintain a specific wheel velocity for smooth driving.
//  *
//  * - **Swerve Drive (Rotation)**: Use `setPosition` for precise angular adjustments or `setProfiledPosition` for smoother, controlled rotations with speed and acceleration constraints. `setPosition` is ideal for simple positional adjustments, while `setProfiledPosition` is better for smooth, controlled movement.
//  *
//  * - **Compact Elevator**: Use `setProfiledPosition` for smooth, controlled movement between heights. Elevators often require a balance of speed and smoothness, and `setProfiledPosition` provides the best control, limiting velocity and acceleration.
//  *
//  * - **Telescoping Arms**: Use `setProfiledPosition` to smoothly extend or retract the arm. Like elevators, telescoping arms benefit from motion profiling, ensuring controlled and consistent movement while avoiding jerky motion.
//  *
//  * - **Pivots (Arm or Gripper)**: Use `setPosition` for precise angle control or `setProfiledPosition` for smoother movement with constraints on speed. Pivots generally need accurate positioning, but `setProfiledPosition` is helpful for smooth, controlled motion under speed/acceleration limits.
//  *
//  * - **Intake**: Use `setVelocity` to maintain a constant intake speed. Intakes require consistent speed to reliably gather or eject objects, and `setVelocity` is ideal for this type of system where speed consistency is key.
//  *
//  * Conclusion:
//  * - **Use `setPosition`** when precise position control is required (e.g., turrets, pivots).
//  * - **Use `setVelocity`** when the system requires constant speed (e.g., flywheels, drive motors, and intakes).
//  * - **Use `setProfiledPosition`** for systems that need smooth and controlled motion with constraints on speed and acceleration (e.g., elevators, telescoping arms, swerve rotation).
//  *
//  * By selecting the appropriate method based on the system’s requirements, you can achieve optimal control, smooth motion, and precision for each mechanism.
//  */

// // import edu.wpi.first.wpilibj.motorcontrol.TalonFX;  // Replace with your motor controller (TalonFX/TalonSRX)
// // import edu.wpi.first.wpilibj.Encoder;   // Encoder to measure the feedback (position or velocity)
// // import edu.wpi.first.wpilibj.PIDController; // For PID control
// // import edu.wpi.first.math.controller.SimpleMotorFeedforward; // For feedforward
// // import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for subsystems
// // import edu.wpi.first.wpilibj.Timer;  // Timer to simulate periodic updates

// // public class PID_and_FF extends SubsystemBase {
    
// //     // Motor controller and encoder initialization
// //     private final TalonFX motorController;  // TalonFX (or TalonSRX)
// //     private final Encoder encoder;  // Encoder for feedback

// //     // Feedforward and PID constants
// //     private static final double kS = 0.1;   // Static gain for feedforward
// //     private static final double kV = 0.2;   // Velocity gain for feedforward
// //     private static final double kA = 0.1;   // Acceleration gain for feedforward
// //     private static final double kP = 0.5;   // Proportional gain for PID controller
// //     private static final double kI = 0.0;   // Integral gain for PID controller
// //     private static final double kD = 0.1;   // Derivative gain for PID controller

// //     // PID controller and feedforward controller
// //     private final PIDController pidController;
// //     private final SimpleMotorFeedforward feedforward;

// //     public PID_and_FF() {
// //         // Initialize motor and encoder
// //         motorController = new TalonFX(1);  // Motor controller ID 1
// //         encoder = new Encoder(0, 1);  // Encoder channels 0 and 1
        
// //         // Initialize the PID controller
// //         pidController = new PIDController(kP, kI, kD);
        
// //         // Initialize feedforward controller
// //         feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        
// //         // Set encoder distance per pulse (this will depend on your motor/gearbox setup)
// //         encoder.setDistancePerPulse(1.0);  // Update this according to your system setup
// //     }
    
// //     // Method for position control (basic)
// //     public void setPosition(double setpoint) {
// //         // Get the current position from the encoder
// //         double currentPosition = encoder.getDistance();
        
// //         // Calculate the PID output for position control
// //         double pidOutput = pidController.calculate(currentPosition, setpoint);
        
// //         // For position control, calculate the feedforward based on velocity (not position)
// //         double targetVelocity = 1.0 * (setpoint - currentPosition);  // Simple velocity estimate
// //         double feedforwardOutput = feedforward.calculate(targetVelocity);
        
// //         // Combine PID and Feedforward outputs
// //         double motorOutput = pidOutput + feedforwardOutput;
        
// //         // Set the motor output as voltage (for voltage control)
// //         motorController.setVoltage(motorOutput);  // This sets the voltage directly
// //     }
    
// //     // Method for velocity control
// //     public void setVelocity(double velocitySetpoint) {
// //         // Get the current velocity from the encoder (if using velocity control)
// //         double currentVelocity = encoder.getRate();  // Get current velocity from the encoder
        
// //         // Calculate the PID output for velocity control
// //         double pidOutput = pidController.calculate(currentVelocity, velocitySetpoint);
        
// //         // Calculate the feedforward output for velocity control (directly use velocity setpoint)
// //         double feedforwardOutput = feedforward.calculate(velocitySetpoint);
        
// //         // Combine PID and Feedforward outputs
// //         double motorOutput = pidOutput + feedforwardOutput;
        
// //         // Set the motor output as voltage (for voltage control)
// //         motorController.setVoltage(motorOutput);  // This sets the voltage directly
// //     }
    
// //     // Method for motion profiling (with max velocity and max acceleration)
// //     public void setProfiledPosition(double setpoint, double maxVelocity, double maxAcceleration) {
// //         double currentPosition = encoder.getDistance();
// //         double currentVelocity = encoder.getRate();
        
// //         // Compute the time to reach the setpoint with constraints on max velocity and acceleration
// //         double distanceToGo = setpoint - currentPosition;
        
// //         // Calculate the required time to move based on max velocity and acceleration
// //         double timeToMaxSpeed = maxVelocity / maxAcceleration;  // Time to reach max velocity
// //         double distanceAtMaxSpeed = 0.5 * maxAcceleration * Math.pow(timeToMaxSpeed, 2); // Distance covered to reach max velocity
        
// //         // If the distance is less than the distance required to reach max speed, we limit it
// //         if (Math.abs(distanceToGo) < distanceAtMaxSpeed) {
// //             timeToMaxSpeed = Math.sqrt(Math.abs(2 * distanceToGo / maxAcceleration));  // Time to reach target position
// //             distanceAtMaxSpeed = 0.5 * maxAcceleration * Math.pow(timeToMaxSpeed, 2);
// //         }
        
// //         // Now we need to calculate the velocity profile (assuming trapezoidal motion)
// //         double velocityProfile = Math.signum(distanceToGo) * Math.min(maxVelocity, maxAcceleration * timeToMaxSpeed);
        
// //         // Feedforward calculation for this velocity profile
// //         double feedforwardOutput = feedforward.calculate(velocityProfile);
        
// //         // The motor's output is the combination of PID output and feedforward
// //         double motorOutput = feedforwardOutput;
        
// //         // Set the motor output as voltage
// //         motorController.setVoltage(motorOutput);  // This sets the voltage directly
// //     }

// //     @Override
// //     public void periodic() {
// //         // This method will be called once per scheduler run (typically every 20ms).
// //         // You can add any updates to your motor here if needed.
        
// //         // For testing purposes, we can just output the current encoder position to the SmartDashboard.
// //         // You can add debugging outputs here if necessary.
// //         System.out.println("Current Position: " + encoder.getDistance());
// //         System.out.println("Current Velocity: " + encoder.getRate());
// //     }
// // }


// import edu.wpi.first.math.controller.PIDController; // PID controller class for PID control
// import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Feedforward class for controlling motor
// import edu.wpi.first.math.trajectory.TrapezoidProfile; // TrapezoidProfile for motion profiling
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX; // For the motor controller
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // For CT
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class PID_and_FF extends SubsystemBase {
// //     // Motor and controller setup
// //     private final TalonFX motor;
// //     private final ProfiledPIDController pidController;
// //     private final SimpleMotorFeedforward feedforward;

// //     // Constants for PID and feedforward
// //     private static final double kP = 1.0;  // Proportional gain
// //     private static final double kI = 0.0;  // Integral gain
// //     private static final double kD = 0.0;  // Derivative gain
// //     private static final double kS = 0.1;  // Feedforward static gain
// //     private static final double kV = 0.2;  // Feedforward velocity gain
// //     private static final double kA = 0.3;  // Feedforward acceleration gain

// //     private static final double MAX_VELOCITY = 5.0; // Maximum velocity (units in radians per second)
// //     private static final double MAX_ACCELERATION = 3.0; // Maximum acceleration (in radians per second squared)

// //     // Constructor
// //     public PID_and_FF(int motorPort) {
// //         motor = new TalonFX(motorPort);

// //         // Set up the PID controller
// //         pidController = new ProfiledPIDController(kP, kI, kD,
// //             new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

// //         // Set up the feedforward controller
// //         feedforward = new SimpleMotorFeedforward(kS, kV, kA);
// //     }

// //     // Set velocity with feedforward and PID control
// //     public void setVelocity(double targetVelocity) {
// //         // Calculate the feedforward value
// //         double feedforwardValue = feedforward.calculate(targetVelocity);

// //         // Calculate the PID output (not really needed for velocity control but included for consistency)
// //         double pidOutput = pidController.calculate(getCurrentPosition(), targetVelocity);

// //         // Combine PID and feedforward to determine the motor output
// //         double motorOutput = pidOutput + feedforwardValue;

// //         // Apply the motor output (set the voltage to the motor)
// //         motor.setVoltage(motorOutput);

// //         // Log for debugging (optional)
// //         SmartDashboard.putNumber("Target Velocity", targetVelocity);
// //         SmartDashboard.putNumber("Motor Output", motorOutput);
// //     }

// //     // Set position with basic PID control (no profiling)
// //     public void setPosition(double targetPosition) {
// //         // Calculate the PID output based on position error
// //         double pidOutput = pidController.calculate(getCurrentPosition(), targetPosition);

// //         // Calculate the feedforward value for the position
// //         double feedforwardValue = feedforward.calculate(pidController.getSetpoint().velocity);

// //         // Combine PID and feedforward to determine the motor output
// //         double motorOutput = pidOutput + feedforwardValue;

// //         // Apply the motor output (set the voltage to the motor)
// //         motor.setVoltage(motorOutput);

// //         // Log for debugging (optional)
// //         SmartDashboard.putNumber("Target Position", targetPosition);
// //         SmartDashboard.putNumber("Motor Output", motorOutput);
// //     }

// // // Set position with motion profiling
// // public void setProfiledPosition(double targetPosition) {
// //     // Get current position and velocity
// //     double currentPosition = getCurrentPosition();
// //     double currentVelocity = motor.getSelectedSensorVelocity();  // Get current velocity from encoder

// //     // Set the target position and velocity (velocity is zero when stopping)
// //     TrapezoidProfile.State goal = new TrapezoidProfile.State(targetPosition, 0); // Target position, velocity = 0 (to stop)
// //     TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity); // Current state (position and velocity)

// //     // Set up the profile
// //     TrapezoidProfile profile = new TrapezoidProfile(
// //         new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION), goal);

// //     // Calculate the required motor output using the profiled PID controller
// //     double output = pidController.calculate(currentState, goal);  // This uses both position and velocity info

// //     // Compute the feedforward value using velocity from the profile
// //     double feedforwardValue = feedforward.calculate(pidController.getSetpoint().velocity);

// //     // Combine PID output and feedforward value to get the final motor output
// //     double motorOutput = output + feedforwardValue;

// //     // Apply the motor output (set voltage to motor)
// //     motor.setVoltage(motorOutput);

// //     // Log for debugging (optional)
// //     SmartDashboard.putNumber("Target Position", targetPosition);
// //     SmartDashboard.putNumber("Motor Output", motorOutput);
// // }


// //     private final PIDController pidController;
// //     private final SimpleMotorFeedforward feedforward;
// //     private final WPI_TalonFX motor;
    
// //     private final double MAX_VELOCITY = 10.0;  // Example maximum velocity (m/s)
// //     private final double MAX_ACCELERATION = 5.0;  // Example max acceleration (m/s^2)
// //     private final double GEAR_RATIO = 10.0; // Example gear ratio
// //     private final double RADIUS = 0.1; // Example radius in meters (tip of the mechanism)

// //     public PID_and_FF(WPI_TalonFX motor, PIDController pidController, SimpleMotorFeedforward feedforward) {
// //         this.motor = motor;
// //         this.pidController = pidController;
// //         this.feedforward = feedforward;
// //     }

// //     // Set position (with gear ratio consideration)
// //     public void setPosition(double targetPosition) {
// //         // Convert target position to motor encoder units, accounting for gear ratio
// //         double motorTargetPosition = targetPosition * GEAR_RATIO;

// //         // Get current position
// //         double currentPosition = getCurrentPosition();
        
// //         // Calculate the PID output based on current position and target position
// //         double output = pidController.calculate(currentPosition, motorTargetPosition);

// //         // Calculate feedforward value
// //         double feedforwardValue = feedforward.calculate(pidController.getSetpoint().velocity);

// //         // Apply motor output
// //         double motorOutput = output + feedforwardValue;
// //         motor.setVoltage(motorOutput);
// //     }

// //     // Set velocity (tip speed in meters per second)
// //     public void setVelocity(double tipSpeed) {
// //         // Convert the tip speed to the corresponding motor speed using gear ratio and radius
// //         double targetVelocity = tipSpeed / (RADIUS * GEAR_RATIO);  // Convert desired tip speed to motor speed (in encoder units)

// //         // Get the current motor velocity
// //         double currentVelocity = motor.getSelectedSensorVelocity() / GEAR_RATIO; // Get motor's current velocity in m/s

// //         // Calculate the PID output (difference between target and current motor velocity)
// //         double pidOutput = pidController.calculate(currentVelocity, targetVelocity);

// //         // Calculate the feedforward value (based on target velocity)
// //         double feedforwardValue = feedforward.calculate(targetVelocity);

// //         // Calculate the total motor output: PID output + feedforward value
// //         double motorOutput = pidOutput + feedforwardValue;

// //         // Apply the motor output (set voltage to motor)
// //         motor.setVoltage(motorOutput);
// //     }

// //     // Set profiled position with motion profiling (gear ratio consideration)
// //     public void setProfiledPosition(double targetPosition) {
// //         // Convert target position to motor encoder units
// //         double motorTargetPosition = targetPosition * GEAR_RATIO;

// //         // Get current position and velocity
// //         double currentPosition = getCurrentPosition();
// //         double currentVelocity = motor.getSelectedSensorVelocity();

// //         // Set the target position and velocity (zero velocity when stopping)
// //         TrapezoidProfile.State goal = new TrapezoidProfile.State(motorTargetPosition, 0);
// //         TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity);

// //         // Set up the motion profile with max velocity and acceleration constraints
// //         TrapezoidProfile profile = new TrapezoidProfile(
// //             new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION), goal);

// //         // Calculate PID output
// //         double output = pidController.calculate(currentState, goal);

// //         // Feedforward calculation
// //         double feedforwardValue = feedforward.calculate(pidController.getSetpoint().velocity);

// //         // Apply motor output
// //         double motorOutput = output + feedforwardValue;
// //         motor.setVoltage(motorOutput);
// //     }

// //     // Helper method to get current position
// //     private double getCurrentPosition() {
// //         return motor.getSelectedSensorPosition() / GEAR_RATIO; // Convert motor position to mechanism position
// //     }
// // }

// import edu.wpi.first.math.controller.PIDController; // PID controller class for PID control
// import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Feedforward class for controlling motor
// import edu.wpi.first.math.trajectory.TrapezoidProfile; // TrapezoidProfile for motion profiling
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // Talon FX motor controller class


//     private final PIDController pidController;
//     private final SimpleMotorFeedforward feedforward;
//     private final WPI_TalonFX motor;
    
//     private final double MAX_VELOCITY = 10.0;  // Example maximum velocity (m/s)
//     private final double MAX_ACCELERATION = 5.0;  // Example max acceleration (m/s^2)
//     private final double GEAR_RATIO = 10.0; // Example gear ratio
//     private final double RADIUS = 0.1; // Example radius in meters (tip of the mechanism)

//     public PID_and_FF(WPI_TalonFX motor, PIDController pidController, SimpleMotorFeedforward feedforward) {
//         this.motor = motor;
//         this.pidController = pidController;
//         this.feedforward = feedforward;
//     }

//     // Set position (with gear ratio consideration)
//     public void setPosition(double targetPosition) {
//         // Convert target position to motor encoder units, accounting for gear ratio
//         double motorTargetPosition = targetPosition * GEAR_RATIO;

//         // Get current position
//         double currentPosition = getCurrentPosition();
        
//         // Calculate the PID output based on current position and target position
//         double output = pidController.calculate(currentPosition, motorTargetPosition);

//         // Calculate feedforward value based on the target velocity (which we assume is 0 for position control)
//         double feedforwardValue = feedforward.calculate(0); // Zero velocity for position control

//         // Apply motor output (set the motor voltage)
//         double motorOutput = output + feedforwardValue;
//         motor.setVoltage(motorOutput);
//     }

//     // Set velocity (tip speed in meters per second)
//     public void setVelocity(double tipSpeed) {
//         // Convert the tip speed to the corresponding motor speed using gear ratio and radius
//         double targetVelocity = tipSpeed / (RADIUS * GEAR_RATIO);  // Convert desired tip speed to motor speed (in encoder units)

//         // Get the current motor velocity (converted to m/s)
//         double currentVelocity = motor.getSelectedSensorVelocity() / GEAR_RATIO;

//         // Calculate the PID output (difference between target and current motor velocity)
//         double pidOutput = pidController.calculate(currentVelocity, targetVelocity);

//         // Calculate the feedforward value based on the target velocity
//         double feedforwardValue = feedforward.calculate(targetVelocity);

//         // Calculate the total motor output: PID output + feedforward value
//         double motorOutput = pidOutput + feedforwardValue;

//         // Apply the motor output (set voltage to motor)
//         motor.setVoltage(motorOutput);
//     }

//     // Set profiled position with motion profiling (gear ratio consideration)
//     public void setProfiledPosition(double targetPosition) {
//         // Convert target position to motor encoder units
//         double motorTargetPosition = targetPosition * GEAR_RATIO;

//         // Get current position and velocity
//         double currentPosition = getCurrentPosition();
//         double currentVelocity = motor.getSelectedSensorVelocity();

//         // Set the target position and velocity (zero velocity when stopping)
//         TrapezoidProfile.State goal = new TrapezoidProfile.State(motorTargetPosition, 0);
//         TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity);

//         // Set up the motion profile with max velocity and acceleration constraints
//         TrapezoidProfile profile = new TrapezoidProfile(
//             new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION), goal);

//         // Get the target position and velocity from the profile
//         TrapezoidProfile.State profileState = profile.calculate(0.02); // Assume a 20ms loop time

//         // Calculate the PID output using the profile's target position
//         double output = pidController.calculate(currentState.position, profileState.position);

//         // Calculate the feedforward value based on the target velocity (from profile)
//         double feedforwardValue = feedforward.calculate(profileState.velocity);

//         // Apply motor output
//         double motorOutput = output + feedforwardValue;
//         motor.setVoltage(motorOutput);
//     }

//     // Helper method to get current position
//     private double getCurrentPosition() {
//         return motor.getSelectedSensorPosition() / GEAR_RATIO; // Convert motor position to mechanism position
//     }
// }



//     @Override
//     public void periodic() {
//         // This is where the control loop runs, and is called periodically
//         // You could call `setVelocity()`, `setPosition()`, or `setProfiledPosition()` with a target value
//     }
// }




// import edu.wpi.first.math.controller.PIDController; // PID controller class for PID control
// import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Feedforward class for controlling motor
// import edu.wpi.first.math.trajectory.TrapezoidProfile; // TrapezoidProfile for motion profiling
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // Talon FX motor controller class
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PID_and_FF extends SubsystemBase {
//     private final PIDController pidController;
//     private final SimpleMotorFeedforward feedforward;
//     private final WPI_TalonFX motor;

//     private final double MAX_VELOCITY = 10.0;  // Example maximum velocity (m/s)
//     private final double MAX_ACCELERATION = 5.0;  // Example max acceleration (m/s^2)
//     private final double GEAR_RATIO = 10.0; // Example gear ratio
//     private final double RADIUS = 0.1; // Example radius in meters (tip of the mechanism)

//     // Constructor for the subsystem
//     public PID_and_FF(WPI_TalonFX motor, PIDController pidController, SimpleMotorFeedforward feedforward) {
//         this.motor = motor;
//         this.pidController = pidController;
//         this.feedforward = feedforward;
//     }

//     // Set position (with gear ratio consideration)
//     public void setPosition(double targetPosition) {
//         // Convert target position to motor encoder units, accounting for gear ratio
//         double motorTargetPosition = targetPosition * GEAR_RATIO;

//         // Get current position
//         double currentPosition = getCurrentPosition();
        
//         // Calculate the PID output based on current position and target position
//         double output = pidController.calculate(currentPosition, motorTargetPosition);

//         // Calculate feedforward value based on the target velocity (which we assume is 0 for position control)
//         double feedforwardValue = feedforward.calculate(0); // Zero velocity for position control

//         // Apply motor output (set the motor voltage)
//         double motorOutput = output + feedforwardValue;
//         motor.setVoltage(motorOutput);
//     }

//     // Set velocity (tip speed in meters per second)
//     public void setVelocity(double tipSpeed) {
//         // Convert the tip speed to the corresponding motor speed using gear ratio and radius
//         double targetVelocity = tipSpeed / (RADIUS * GEAR_RATIO);  // Convert desired tip speed to motor speed (in encoder units)

//         // Get the current motor velocity (converted to m/s)
//         double currentVelocity = motor.getSelectedSensorVelocity() / GEAR_RATIO;

//         // Calculate the PID output (difference between target and current motor velocity)
//         double pidOutput = pidController.calculate(currentVelocity, targetVelocity);

//         // Calculate the feedforward value based on the target velocity
//         double feedforwardValue = feedforward.calculate(targetVelocity);

//         // Calculate the total motor output: PID output + feedforward value
//         double motorOutput = pidOutput + feedforwardValue;

//         // Apply the motor output (set voltage to motor)
//         motor.setVoltage(motorOutput);
//     }

//     // Set profiled position with motion profiling (gear ratio consideration)
//     public void setProfiledPosition(double targetPosition) {
//         // Convert target position to motor encoder units
//         double motorTargetPosition = targetPosition * GEAR_RATIO;

//         // Get current position and velocity
//         double currentPosition = getCurrentPosition();
//         double currentVelocity = motor.getSelectedSensorVelocity();

//         // Set the target position and velocity (zero velocity when stopping)
//         TrapezoidProfile.State goal = new TrapezoidProfile.State(motorTargetPosition, 0);
//         TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity);

//         // Set up the motion profile with max velocity and acceleration constraints
//         TrapezoidProfile profile = new TrapezoidProfile(
//             new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),goal);



//         // Get the target position and velocity from the profile
//         TrapezoidProfile.State profileState = profile.calculate(0.02); // Assume a 20ms loop time

//         // Calculate the PID output using the profile's target position
//         double output = pidController.calculate(currentState.position, profileState.position);

//         // Calculate the feedforward value based on the target velocity (from profile)
//         double feedforwardValue = feedforward.calculate(profileState.velocity);

//         // Apply motor output
//         double motorOutput = output + feedforwardValue;
//         motor.setVoltage(motorOutput);
//     }

//     // Helper method to get current position
//     private double getCurrentPosition() {
//         return motor.getSelectedSensorPosition() / GEAR_RATIO; // Convert motor position to mechanism position
//     }
// }
