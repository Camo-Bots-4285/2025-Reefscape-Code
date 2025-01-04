// package frc.robot.subsystems.Templates;

// import javax.swing.text.Position;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import frc.robot.subsystems.Drive.SwerveBase;
// import java.lang.module.Configuration;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// //import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.*;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.*;
// import frc.robot.Robot;



// public class FindMyPIDandFF extends SubsystemBase {

// public double Current_Speed;
// public double Current_Pos; 
// public double Desired_Pos; 
// public double Desired_Speed;

// public double kP, kI, kD, ks, kv, ka, kg, kdesV, kdesP, gearRat, wheelRad, kmax, kmin, MaxVel, minVel, maxAcc, allowedErr; 

// public PIDController PID = new PIDController(kP, kI, kD);
// public ElevatorFeedforward FF = new ElevatorFeedforward(ks , kg,kv , ka);



//      public FindMyPIDandFF(
//       int current_Velocity,
//       int current_Position

      


//       ){
       
        

// Current_Speed = current_Velocity;
// Current_Pos = current_Position;

// }  
// @Override
// public void periodic() {
//         //PID
//         double p = SmartDashboard.getNumber("PID_Helper/P Gain", 0);
//         double i = SmartDashboard.getNumber("PID_Helper/I Gain", 0);
//         double d = SmartDashboard.getNumber("PID_Helper/D Gain", 0);
//         //FF
//         double s = SmartDashboard.getNumber("PID_Helper/S Gain", 0);
//         double v = SmartDashboard.getNumber("PID_Helper/V Gain", 0);
//         double a = SmartDashboard.getNumber("PID_Helper/A Gain", 0);
//         double g = SmartDashboard.getNumber("PID_Helper/G Gain", 0);
//         //Set Points
//         double desV = SmartDashboard.getNumber("PID_Helper/Desired Velocity", 0);
//         double desP = SmartDashboard.getNumber("PID_Helper/Desired Position", 0);
//         //Physical
//         double gearR = SmartDashboard.getNumber("PID_Helper/Gear Ratio", 0);
//         double WheelR = SmartDashboard.getNumber("PID_Helper/Wheel Radius", 0);

//         //Constaibs
//         double max = SmartDashboard.getNumber("Max Output", 0);
//         double min = SmartDashboard.getNumber("Min Output", 0);
//         double maxV = SmartDashboard.getNumber("Max Velocity", 0);
//         double minV = SmartDashboard.getNumber("Min Velocity", 0);
//         double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
//         double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);


//     if((p != kP)) { arm_pid_Controller.setP(p); kP = p; }
//     if((i != kI)) { arm_pid_Controller.setI(i); kI = i; }
//     if((d != kD)) { arm_pid_Controller.setD(d); kD = d; }
//     if((max != kMaxOutput) || (min != kMinOutput)) { 
//       arm_pid_Controller.setOutputRange(min, max); 
//       kMinOutput = min; kMaxOutput = max; 
//     }
//     if((maxV != MaxVel)) { arm_pid_Controller.setSmartMotionMaxVelocity(maxV,0); MaxVel = maxV; }
//     if((minV != minVel)) { arm_pid_Controller.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
//     if((maxA != maxAcc)) { arm_pid_Controller.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
//     if((allE != allowedErr)) { arm_pid_Controller.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
// }

  
//     // Set velocity (tip speed in meters per second)
//     public double setVelocity(double tipSpeed) {
//         // Convert the tip speed to the corresponding motor speed using gear ratio and radius
//         double targetVelocity = tipSpeed / (wheelRad * gearRat);  // Convert desired tip speed to motor speed (in encoder units)

//         // Get the current motor velocity (converted to m/s)
//         double currentVelocity = Current_Speed / gearRat;

//         // Calculate the PID output (difference between target and current motor velocity)
//         double pidOutput = PID.calculate(currentVelocity, targetVelocity);

//         // Calculate the feedforward value based on the target velocity
//         double feedforwardValue = FF.calculate(targetVelocity);

//         // Calculate the total motor output: PID output + feedforward value
//         double motorOutput = pidOutput + feedforwardValue;

//         return motorOutput;
//     }

//     // Set profiled position with motion profiling (gear ratio consideration)
//     public double setProfiledPosition(double targetPosition) {
//         // Convert target position to motor encoder units
//         double motorTargetPosition = targetPosition * gearRat;

//         // Get current position and velocity
//         double currentPosition = Current_Pos();
//         double currentVelocity = Current_Speed;

//         // Set the target position and velocity (zero velocity when stopping)
//         TrapezoidProfile.State goal = new TrapezoidProfile.State(motorTargetPosition, 0);
//         TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity);

//         // Set up the motion profile with max velocity and acceleration constraints
//         TrapezoidProfile profile = new TrapezoidProfile(
//             new TrapezoidProfile.Constraints(MaxVel, maxAcc),goal);



//         // Get the target position and velocity from the profile
//         TrapezoidProfile.State profileState = profile.calculate(0.02); // Assume a 20ms loop time

//         // Calculate the PID output using the profile's target position
//         double output = PID.calculate(currentState.position, profileState.position);

//         // Calculate the feedforward value based on the target velocity (from profile)
//         double feedforwardValue = FF.calculate(profileState.velocity);

//         // Apply motor output
//         double motorOutput = output + feedforwardValue;
//        // motor.setVoltage(motorOutput);
//     }




// }

