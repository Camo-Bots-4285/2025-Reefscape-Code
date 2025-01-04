package frc.robot.subsystems.Drive;

import java.lang.module.Configuration;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.*;

// Old Modules from older phoenix
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import frc.robot.Robot;


public class SwerveModule extends SubsystemBase {
  
  /**
   * Class to represent and handle a swerve module
   * A module's state is measured by a CANCoder for the absolute position,
   * integrated CANEncoder for relative position
   * for both rotation and linear movement
   */


  private static SwerveBase swerveDrive;

  public PIDController testRotationController;

  public PIDController DrivePID;
  public SimpleMotorFeedforward DriveFF;

  public PIDController RotationPID;
  public SimpleMotorFeedforward RotationFF;

  private final TalonFX driveMotor;
  private final TalonFX rotationMotor;

  public double DriveSpeedDiff;

    private final int loopCount; // Number of loops for averaging velocity changes
    private double[] velocityHistory; // Array to store velocity data for each loop
    private int currentLoopIndex = 0; // Index for the current loop iteration
    private double lastTime; // To store the last timestamp of the loop for calculating delta-time
    public double timer;
    
    // For inverting D
    private double previousError = 0.0;  // Initialize the previous error to zero
    private double integral = 0.0;       // Initialize the integral to zero


  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  public TalonFX getRotationMotor() {
    return rotationMotor;
  }
  

  public final TalonFXConfigurator rotationEncoder;
  public final TalonFXConfigurator driveEncoder;

  private final CANcoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;

  private int Module_Running;
  
  public SwerveModule(
      int Module,

      int driveMotorId,
      PIDController drivePID,
      SimpleMotorFeedforward driveFF,

      int rotationMotorId,
      PIDController rotationPID,
      SimpleMotorFeedforward rotationFF,

      int canCoderId,
      double measuredOffsetRadians,
      SwerveBase swerveSubsystem) {

      this.loopCount = 3;//if you change need to change accleration
      this.velocityHistory = new double[loopCount];


     Module_Running = Module;

    swerveDrive = swerveSubsystem;
  
    //Defines what Talon to target
    driveMotor = new TalonFX(driveMotorId);

    //Get encoder for that Talon
    var driveConfig = new TalonFXConfiguration();

      // Turn on current limiter
      driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      // Enables the current limiting to prevent excessive current draw, which could trip the breaker or damage the system.

      // Ensures the motor can run continuously without exceeding the limits of the PDH breaker.
      driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // amps

      // Allows brief bursts of high current, such as during acceleration or when overcoming a heavy load.
      driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0; // amps
      // Peak current is set to 70 amps, matching the breaker limit. This allows the motor to draw up to 70 amps for a short period 
      // without triggering the breaker. Since FRC breakers handle high current bursts, this setting is safe for short-term acceleration needs.

      // Set the time (in seconds) the motor can run at peak current before limiting kicks in
      driveConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0; // second
      // This allows the motor to run at peak current for 1 second before limiting starts. 
      // 1 second is usually enough for the motor to handle short bursts of high current during acceleration or heavy loads without tripping the breaker.


    //Applies current limitor
    driveMotor.getConfigurator().apply(driveConfig);

    //Configures the encoder
    driveEncoder = driveMotor.getConfigurator();

/*When talking current limits - The following rules apply along with the motors physical limits
 * SupplyCurrenLimit should be =< the breacker on the robot
 * SupplyCurrentThreshold dependes on each motor but should not be more than twice the breacker
 * SupplyTimeThreshold is normal 1-2 seconds
 * 
 * Theses values are all in place to not trip breacker and make sure motor are not permently damaged
 * They will still get very warm when run contiusely
 */

    //Defines what Talon to target
    rotationMotor = new TalonFX(rotationMotorId);

    var rotationConfig = new TalonFXConfiguration();

    //Turns on current limitor
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set the supply current limit to 40 amps (continuous)
    rotationConfig.CurrentLimits.SupplyCurrentLimit = 35.0;

    // Set the supply peak current to 60 amps (short bursts)
    rotationConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0;

    // Set the time (in seconds) the motor can run at peak current before limiting kicks in
    rotationConfig.CurrentLimits.SupplyCurrentLowerLimit = 1.0; // 1 second

    //Applies current limitor
    rotationMotor.getConfigurator().apply(rotationConfig);

    //Configures the encoder
    rotationEncoder = rotationMotor.getConfigurator();


 
    //get canCoder
    canCoder = new CANcoder(canCoderId);
  

    //Set the offset to make the wheel go "staight"
    offset = new Rotation2d(measuredOffsetRadians);


    //Sets PID value for how the rotation motor willl reach it's target
    //You can lovwer this value to decress speed to help save wheels but will make the robot harder to drive
    testRotationController = new PIDController(0.5, 0.0, 0.0);
    testRotationController.enableContinuousInput(-Math.PI, Math.PI);


    RotationPID = rotationPID;
    RotationPID.enableContinuousInput(-Math.PI, Math.PI);
    // RotationPID.setIZone();
    // RotationPID.setIntegratorRange();
    // RotationPID.setTolerance(0.0,0.0);

    //double feedforwardOutput = DriveFF.calculate(0.0,0.0);


 
    RotationFF = rotationFF;

    DrivePID = drivePID;
    DriveFF = driveFF;
  









    





// // Set profiled position with motion profiling (gear ratio consideration)
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






    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
   
    // TODO - I cannot find where this is set in phoenix 
    //canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  

  
    }




  public void resetDistance() {

    driveMotor.setPosition(0.0);

  }

  public Double getDriveDistanceRadians() {

    return Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()) / SwerveConstants.driveGearRatio;

  }

  public Rotation2d getCanCoderAngle() {

    double unsignedAngle = (canCoder.getAbsolutePosition().getValueAsDouble() - offset.getRadians())
        % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getIntegratedAngle() {
    // Wass
    // double unsignedAngle = rotationMotor.getPosition().getValueAsDouble() % (2 * Math.PI);

    // if (unsignedAngle < 0)
    //   unsignedAngle += 2 * Math.PI;

    // return new Rotation2d(unsignedAngle);

    //Carlos
    return new Rotation2d(Units.rotationsToRadians(rotationMotor.getPosition().getValueAsDouble())/SwerveConstants.angleGearRatio);


  }


  public double getCurrentVelocityRadiansPerSecond() {

    return Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) / SwerveConstants.driveGearRatio;

  }

  public double getCurrentVelocityMetersPerSecond() {

    return Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) / SwerveConstants.driveGearRatio * (SwerveConstants.wheelDiameter / 2.0);

  }

    public double getCurrentAccelerationMetersPerSecond() {

    return Units.rotationsToRadians(driveMotor.getAcceleration().getValueAsDouble()) / SwerveConstants.driveGearRatio * (SwerveConstants.wheelDiameter / 2.0);
  }

  public double getCurrentDistanceMetersPerSecond() {
    return Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()) / SwerveConstants.driveGearRatio * (SwerveConstants.wheelDiameter / 2.0);
  }

  // unwraps a target angle to be [0,2π]
  public static double placeInAppropriate0To360Scope(double unwrappedAngle) {

    double modAngle = unwrappedAngle % (2.0 * Math.PI);

    if (modAngle < 0.0)
      modAngle += 2.0 * Math.PI;

    double wrappedAngle = modAngle;

    return wrappedAngle;

  }


  // initialize the integrated NEO encoder to the offset (relative to home
  // position)
  // measured by the CANCoder
  public void initRotationOffset() {

    rotationMotor.setPosition/*(Units.rotationsToRadians*/(getCanCoderAngle().getRotations()* SwerveConstants.angleGearRatio);
    //rotationEncoder.setPosition(getCanCoderAngle().getRadians());
  }

  /**
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of
   * writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */

// //Wass
//   public static SwerveModuleState optimize(
//       SwerveModuleState desiredState, Rotation2d currentAngle) {

//     double targetAngle = placeInAppropriate0To360Scope(desiredState.angle.getRadians());

//     double targetSpeed = desiredState.speedMetersPerSecond;
//     double delta = (targetAngle - currentAngle.getRadians());
//     if (Math.abs(delta) > (Math.PI / 2 )) {
//       //delta -= Math.PI * Math.signum(delta);
//       targetSpeed = -targetSpeed;
//       targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
//     }
// //Look where this was added
//     //double targetPosition = targetAngle + delta;
//    // return new SwerveModuleState(targetSpeed, new Rotation2d(targetPosition));
    
//     return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
//   }

  /**
   * Method to set the desired state of the swerve module
   * Parameter:
   * SwerveModuleState object that holds a desired linear and rotational setpoint
   * Uses PID and a feedforward to control the output
   */























































   public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
        stop();
    }

    // SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

    // SwerveModuleState optimizedDesiredState = unoptimizedDesiredState;
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(unoptimizedDesiredState, getState().angle);

    // double angularSetPoint = placeInAppropriate0To360Scope(
    //     optimizedDesiredState.angle.getRadians());
    double angularSetPoint = optimizedDesiredState.angle.getRadians();

    // Calculate deltaTime (time difference between current and previous loop)
    double deltaTime = timer - lastTime;
    lastTime = timer; // Update the lastTime for the next loop

    // Store the current velocity in the velocity history array
    velocityHistory[currentLoopIndex] = optimizedDesiredState.speedMetersPerSecond;

    // Increment the loop counter and wrap around if necessary
    currentLoopIndex = (currentLoopIndex + 1) % loopCount;

    // Calculate the average velocity change over the loop history
    double velocityChangeSum = 0;
    for (int i = 1; i < loopCount; i++) {
        int prevIndex = (currentLoopIndex - i + loopCount) % loopCount;
        velocityChangeSum += velocityHistory[prevIndex] - velocityHistory[(prevIndex - 1 + loopCount) % loopCount];
    }

    // Average the velocity changes and divide by deltaTime to get acceleration
    double averageVelocityChange = velocityChangeSum / loopCount;
    double acceleration = averageVelocityChange / deltaTime;

    

        PIDController rearRight_Drive_P = new PIDController(2.0, 0.0, 0.0); // 14.18//1.5//1.7//0.5, 0.0,0.
        PIDController rearRight_Drive_D = new PIDController(0.0, 0.0, 0.00683456); // 14.18//1.5//1.7//0.5, 0.0,0.
        PIDController rearRight_Drive_PID = new PIDController(1.0, 0.0, 0.0); // 14.18//1.5//1.7//0.5, 0.0,0.

        // Define variables
        double error = optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond();  // Calculate the error (desired speed - current speed)
        double derivative = (error - previousError) / deltaTime;  // Calculate the derivative (rate of change)
        integral = integral + error * deltaTime;  // Calculate the integral (sum of errors over time)

        // Reverse the effects of P and D
        double pOutput = rearRight_Drive_PID.getP() * error;  // P term
        double pOutput2 = rearRight_Drive_P.getP() * error;  // P term
        double dOutput = -rearRight_Drive_PID.getD() * derivative;  // Negate D term

        // Integral term remains unchanged
        double iOutput = rearRight_Drive_PID.getI() * integral;

        // Calculate the final output
        double output = pOutput + iOutput + dOutput;

        // Send the output to the motor (set the voltage, speed, or power)
        
 if (Module_Running == 3) { // Runs when FL is Running in the constructor
        // Debugging info to SmartDashboard
        System.out.println("optimizedDesiredState.speedMetersPerSecond: " + optimizedDesiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("bob", optimizedDesiredState.speedMetersPerSecond);
        DriveSpeedDiff = getCurrentVelocityMetersPerSecond();
        SmartDashboard.putNumber("optimizedDesiredState.speedMetersPerSecondsetpoint", getCurrentVelocityMetersPerSecond());
        SmartDashboard.putNumber("PID1", -rearRight_Drive_PID.calculate(optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond(), 0.0));
        SmartDashboard.putNumber("PID_P1", -rearRight_Drive_P.calculate(optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond(), 0.0));
        SmartDashboard.putNumber("PID_D1", -rearRight_Drive_D.calculate(optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond(), 0.0));
        SmartDashboard.putNumber("PID2", output);
        SmartDashboard.putNumber("PID_P2", pOutput);
        SmartDashboard.putNumber("PID_P3", pOutput2);
        SmartDashboard.putNumber("PID_D2", dOutput);
        SmartDashboard.putNumber("FF", DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond));
        SmartDashboard.putNumber("DriveSpeedDiff", optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond());
        SmartDashboard.putNumber("currentMetersPerSecond", DriveSpeedDiff);
        SmartDashboard.putNumber("Voltage", -rearRight_Drive_PID.calculate(optimizedDesiredState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond(), 0.0) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond));
    }
        // Optionally, update the D term and set the motor voltage based on the PID output
        // rearRight_Drive_PID.setD(-rearRight_Drive_PID.getD());
        // driveMotor.setVoltage((rearRight_Drive_PID.calculate(getCurrentVelocityMetersPerSecond(), optimizedDesiredState.speedMetersPerSecond) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond, acceleration)) * SwerveConstants.calibrationFactorSB);
rotationMotor.setVoltage(RotationPID.calculate(getIntegratedAngle().getRadians(), angularSetPoint));
driveMotor.setVoltage(output+DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond, acceleration));  // or driveMotor.setSpeed(output) depending on your API
        // Store the current error for use in the next iteration
        previousError = error;
   // }
}

























































//   public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
//     if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
//       stop();
//     }

//   // SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

//   //   double angularSetPoint = placeInAppropriate0To360Scope(
//   //       optimizedDesiredState.angle.getRadians());


//       //SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());
//     //SwerveModuleState optimizedDesiredState = unoptimizedDesiredState;
//     SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(unoptimizedDesiredState, getState().angle);
    





//     // double angularSetPoint = placeInAppropriate0To360Scope(
//     //     optimizedDesiredState.angle.getRadians());
//     double angularSetPoint = optimizedDesiredState.angle.getRadians();



//       //double currentTime = Timer.getFPGATimestamp(); // Get current time in seconds
      
//       // Calculate deltaTime (time difference between current and previous loop)
//       double deltaTime = timer - lastTime;
//       lastTime = timer; // Update the lastTime for the next loop
  
//       // Store the current velocity in the velocity history array
//       velocityHistory[currentLoopIndex] = optimizedDesiredState.speedMetersPerSecond;
  
//       // Increment the loop counter and wrap around if necessary
//       currentLoopIndex = (currentLoopIndex + 1) % loopCount;
  
//       // Calculate the average velocity change over the loop history
//       double velocityChangeSum = 0;
//       for (int i = 1; i < loopCount; i++) {
//         int prevIndex = (currentLoopIndex - i + loopCount) % loopCount;
//         velocityChangeSum += velocityHistory[prevIndex] - velocityHistory[(prevIndex - 1 + loopCount) % loopCount];
//       }
      
//       // Average the velocity changes and divide by deltaTime to get acceleration
//       double averageVelocityChange = velocityChangeSum / loopCount;
//       double acceleration = averageVelocityChange / deltaTime;
  
// if(Module_Running == 3){//Runs when FL is Running in the constuctor


//   PIDController rearRight_Drive_P = new PIDController(2.0, 0.0, 0.0);//14.18//1.5//1.7//0.5, 0.0,0.
//   PIDController rearRight_Drive_D = new PIDController(0.0, 0.0, 0.00683456);//14.18//1.5//1.7//0.5, 0.0,0.blic static SimpleMotorFeedforward rearRight_Drive_FF = new SimpleMotorFeedforward(0.1399,2.190,0.01);//0.1399,2.190,0.01
//    PIDController rearRight_Drive_PID = new PIDController(2.0, 0.0, 0.00683456);//14.18//1.5//1.7//0.5, 0.0,0.






// System.out.println("optimizedDesiredState.speedMetersPerSecond" + optimizedDesiredState.speedMetersPerSecond);
//     SmartDashboard.putNumber("bob",optimizedDesiredState.speedMetersPerSecond);
//     DriveSpeedDiff = getCurrentVelocityMetersPerSecond() ;
//       SmartDashboard.putNumber("optimizedDesiredState.speedMetersPerSecondsetpoint",getCurrentVelocityMetersPerSecond());
// //-DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond/*SwerveConstants.maxSpeed*/, getCurrentVelocityMetersPerSecond()) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed))*SwerveConstants.calibrationFactorSB);
//  SmartDashboard.putNumber("PID1",-rearRight_Drive_PID .calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0/*SwerveConstants.maxSpeed*/));
//  SmartDashboard.putNumber("PID_P1",-rearRight_Drive_P.calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0/*SwerveConstants.maxSpeed*/));
//  SmartDashboard.putNumber("PID_D1",-rearRight_Drive_D.calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0/*SwerveConstants.maxSpeed*/));
 
//  SmartDashboard.putNumber("PID2",rearRight_Drive_PID .calculate(getCurrentVelocityMetersPerSecond(),optimizedDesiredState.speedMetersPerSecond/*SwerveConstants.maxSpeed*/));
//  SmartDashboard.putNumber("PID_P2",rearRight_Drive_P.calculate(getCurrentVelocityMetersPerSecond(), optimizedDesiredState.speedMetersPerSecond/*SwerveConstants.maxSpeed*/));
//  SmartDashboard.putNumber("PID_D2",rearRight_Drive_D.calculate(getCurrentVelocityMetersPerSecond(),optimizedDesiredState.speedMetersPerSecond/*SwerveConstants.maxSpeed*/));


//  SmartDashboard.putNumber("FF", DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond));
// SmartDashboard.putNumber("DriveSpeedDiff", optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond());
// SmartDashboard.putNumber("current mpers",DriveSpeedDiff);

// //driveMotor.setVoltage((-DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond)) * SwerveConstants.calibrationFactorSB);
//  //SmartDashboard.putNumber("Averaged Acceleration " , acceleration);

//   SmartDashboard.putNumber("Voltage" , -DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0)+DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond,acceleration));
//  SmartDashboard.putNumber("Averaged Acceleration " , acceleration);

//  driveMotor.setVoltage((rearRight_Drive_PID.calculate(getCurrentVelocityMetersPerSecond(),optimizedDesiredState.speedMetersPerSecond/*SwerveConstants.maxSpeed*/) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond,acceleration)) * SwerveConstants.calibrationFactorSB);

//     }
//     //Was 
//      rotationMotor.setVoltage(RotationPID.calculate(getIntegratedAngle().getRadians(), angularSetPoint));
    //rotationMotor.setVoltage(RotationPID.calculate(getIntegratedAngle().getRadians(), angularSetPoint)+RotationFF.calculate(angularSetPoint));

// TrapezoidProfile.State goal = new TrapezoidProfile.State(motorTargetPosition, 0);
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

//     double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond /
//         (SwerveConstants.wheelDiameter / 2.0);

     
  //Was 
  //driveMotor.setVoltage(-DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed)*SwerveConstants.calibrationFactorSB);
  //DriveSpeedDiff = getCurrentVelocityMetersPerSecond() - optimizedDesiredState.speedMetersPerSecond;

//driveMotor.setVoltage((-DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond-getCurrentVelocityMetersPerSecond(),0.0) + DriveFF.calculate(optimizedDesiredState.speedMetersPerSecond,acceleration)) * SwerveConstants.calibrationFactorSB);

 
 //This should compart to our current to make a feed back might not work becasue it might take all swerves into acount insted of one
 // driveMotor.setVoltage(-DrivePID.calculate(optimizedDesiredState.speedMetersPerSecond,getCurrentVelocityMetersPerSecond())/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);
    
//}

  public void resetEncoders() {
    driveMotor.setPosition(0);
    rotationMotor.setPosition(0);
  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }


  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentVelocityRadiansPerSecond(), getIntegratedAngle());
  }



 
@Override
public void periodic(){


 SmartDashboard.putNumber("Time Teleop" , Robot.teleopTimeFinal);
     SmartDashboard.putNumber("time" , timer);
         // // Initialize the timer
      timer = Robot.Time;
}
}























// package frc.robot.subsystems.Drive;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.*;

// public class SwerveModule extends SubsystemBase {

//     private static SwerveBase swerveDrive;

//     public PIDController RotationPID; // PID Controller for rotation
//     public SimpleMotorFeedforward DriveFF; // Feedforward for drive
//     public PIDController DrivePID; // PID Controller for drive

//     private final TalonFX driveMotor; // TalonFX motor controller for the drive motor
//     private final TalonFX rotationMotor; // TalonFX motor controller for the rotation motor

//     private final CANCoder canCoder; // CANCoder to measure the current angle of the module
//     private final Rotation2d offset; // Offset for the angle to correct module alignment

//     // Constructor to initialize the swerve module with necessary parameters
//     public SwerveModule(int driveMotorId, PIDController drivePID, SimpleMotorFeedforward driveFF,
//                         int rotationMotorId, PIDController rotationPID, SimpleMotorFeedforward rotationFF,
//                         int canCoderId, double measuredOffsetRadians, SwerveBase swerveSubsystem) {

//         swerveDrive = swerveSubsystem;

//         // Initialize motors
//         driveMotor = new TalonFX(driveMotorId, "rio");
//         rotationMotor = new TalonFX(rotationMotorId, "rio");

//         // Initialize CANCoder for the module angle measurement
//         canCoder = new CANCoder(canCoderId);

//         // Set the offset to correct the initial orientation of the module
//         offset = new Rotation2d(measuredOffsetRadians);

//         // Initialize PID and Feedforward controllers
//         RotationPID = rotationPID;
//         RotationPID.enableContinuousInput(-Math.PI, Math.PI);

//         DrivePID = drivePID;
//         DriveFF = driveFF;

//         // Configure motor settings (e.g., current limits)
//         setupMotorAndEncoder(driveMotor, 40.0, 60.0, 1.0); 
//         setupMotorAndEncoder(rotationMotor, 40.0, 60.0, 1.0);
//     }

//     // Method to configure motor settings (e.g., current limits)
//     private void setupMotorAndEncoder(TalonFX motor, double supplyLimit, double supplyThreshold, double timeThreshold) {
//         TalonFXConfiguration motorConfig = new TalonFXConfiguration();
//         motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         motorConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
//         motorConfig.CurrentLimits.SupplyCurrentThreshold = supplyThreshold;
//         motorConfig.CurrentLimits.SupplyTimeThreshold = timeThreshold;
//         motor.getConfigurator().apply(motorConfig);
//     }

//     // Method to reset the distance traveled by the drive motor
//     public void resetDistance() {
//         driveMotor.setPosition(0.0);
//     }

//     // Method to reset the encoders of the drive and rotation motors
//     public void resetEncoders() {
//         driveMotor.setPosition(0);
//         rotationMotor.setPosition(0);
//     }

//     // Method to stop the motors (set speed to 0)
//     public void stop() {
//         driveMotor.set(0);
//         rotationMotor.set(0);
//     }

//     // Method to get the current integrated angle of the rotation motor
//     public Rotation2d getIntegratedAngle() {
//         return new Rotation2d(Units.rotationsToRadians(rotationMotor.getPosition().getValueAsDouble()) / SwerveConstants.angleGearRatio);
//     }

//     // Method to get the current angle from the CANCoder, considering the offset
//     public Rotation2d getCanCoderAngle() {
//         double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);
//         return new Rotation2d(unsignedAngle);
//     }

//     // Method to get the current velocity of the drive motor in radians per second
//     public double getCurrentVelocityRadiansPerSecond() {
//         return Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) / SwerveConstants.driveGearRatio;
//     }

//     // Method to get the current velocity of the drive motor in meters per second
//     public double getCurrentVelocityMetersPerSecond() {
//         return Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) / SwerveConstants.driveGearRatio * (SwerveConstants.wheelDiameter / 2.0);
//     }

//     // Method to set the desired state of the swerve module (speed and angle) in closed-loop control
//     public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

//         // If the speed is too low, stop the module to save power
//         if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
//             stop();
//             return;
//         }

//         // Optimize the desired state to align with the current module angle (field-oriented control)
//         SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getIntegratedAngle());

//         // Get the desired angle for the module
//         double angularSetpoint = optimizedState.angle.getRadians();

//         // Calculate the voltage needed to reach the desired angle using PID control
//         double rotationVoltage = RotationPID.calculate(getIntegratedAngle().getRadians(), angularSetpoint);

//         // Calculate the voltage needed to drive the module using PID control and feedforward
//         double driveVoltage = -DrivePID.calculate(optimizedState.speedMetersPerSecond - getCurrentVelocityMetersPerSecond(), 0.0)
//                               + DriveFF.calculate(optimizedState.speedMetersPerSecond);

//         // Set the rotation and drive motor voltages
//         rotationMotor.setVoltage(rotationVoltage);
//         driveMotor.setVoltage(driveVoltage);

//         // Optionally, output values to SmartDashboard for debugging
//         SmartDashboard.putNumber("Drive Voltage", driveVoltage);
//         SmartDashboard.putNumber("Rotation Voltage", rotationVoltage);
//         SmartDashboard.putNumber("Target Speed", optimizedState.speedMetersPerSecond);
//         SmartDashboard.putNumber("Target Angle", optimizedState.angle.getDegrees());
//     }

//     // Periodic method for updates to the subsystem, can be used for additional diagnostics or control logic
//     @Override
//     public void periodic() {
//         // Update SmartDashboard values for debugging
//         SmartDashboard.putNumber("Current Speed", getCurrentVelocityMetersPerSecond());
//         SmartDashboard.putNumber("Current Angle", getIntegratedAngle().getDegrees());
//     }
// }







// import com.ctre.phoenix.motorcontrol.*;

// //import com.ctre.phoenix.motorcontrol.can.TalonFXControlMode;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class SwerveModule extends SubsystemBase {

//     // Declare the TalonFX motor controllers for the drive and rotation motors
//     private TalonFX driveMotor;
//     private TalonFX rotationMotor;

//     // Constructor: Initializes TalonFX motor controllers and configures them
//     public SwerveModule(
//       int driveMotorId,
//       PIDController drivePID,
//       SimpleMotorFeedforward driveFF,

//       int rotationMotorId,
//       PIDController rotationPID,
//       SimpleMotorFeedforward rotationFF,

//       int canCoderId,
//       double measuredOffsetRadians,
//       SwerveBase swerveSubsystem) {
//         // Initialize TalonFX motor controllers with CAN IDs (drive and rotation motors)
//         driveMotor = new TalonFX(driveMotorId);
//         rotationMotor = new TalonFX(rotationMotorId,"rio");

//         // Configure the motors (drive motor gets 'true' for isDriveMotor, rotation motor gets 'false')
//         configureMotor(driveMotor, true);
//         configureMotor(rotationMotor, false);
//     }

//     // Method to configure a TalonFX motor controller
//     public void configureMotor(TalonFX motor, boolean isDriveMotor) {
//         // Step 1: Create a TalonFXConfiguration object to hold the settings
//         TalonFXConfiguration motorConfig = new TalonFXConfiguration();

//         // Step 2: Reset motor settings to factory defaults
//         motor.configFactoryDefault(); // Clears any existing settings and restores the factory defaults

//         // Step 3: Enable Voltage Compensation
//         motorConfig.voltageCompSaturation = 12.0; // Set the max voltage to 12V (use the battery voltage)
//         motor.configVoltageCompSaturation(motorConfig.voltageCompSaturation);
//         motor.enableVoltageCompensation(true); // Set 'true' to enable voltage compensation

//         // Step 4: Configure Current Limiting
//         // Create a current limit configuration and apply it to the motor
//         SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 1);
//         // Enable current limit, with continuous current of 35A, peak current of 40A for 1 second
//         motor.configSupplyCurrentLimit(currentLimit);

//         // Step 5: Set Neutral Mode
//         motor.setNeutralMode(NeutralMode.Brake); // Motor will stop immediately when neutral, helping to stop quicker

//         // Step 6: Configure Ramp Rate
//         motor.configOpenloopRamp(isDriveMotor ? 0.5 : 0.2); 
//         // Ramp rate: how fast the motor accelerates
//         // 0.5 seconds for drive motors, 0.2 seconds for rotation motors (typically rotation is slower)

//         // Step 7: Configure Feedback Sensor (integrated encoder in TalonFX)
//         motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
//         // This sets the motor to use the built-in integrated sensor for feedback (encoder)

//         // Step 8: Set PID (Proportional, Integral, Derivative) Gains for Closed-Loop Control
//         motor.config_kP(0, 1.0); // Proportional gain for position or velocity control
//         motor.config_kI(0, 0.0); // Integral gain (not often used)
//         motor.config_kD(0, 0.0); // Derivative gain (also rarely used in this case)
//         motor.config_kF(0, 0.0); // Feed-forward gain (used to offset motor behavior)

//         // Step 9: Set Sensor Phase
//         motor.setSensorPhase(true); // Ensures the feedback sensor's phase is correct (correct direction)

//         // Step 10: Set Soft Limits (optional)
//         motor.configForwardSoftLimitThreshold(10000); // Example: forward limit (in encoder units)
//         motor.configReverseSoftLimitThreshold(-10000); // Example: reverse limit (in encoder units)
//         motor.configForwardSoftLimitEnable(true); // Enable the forward soft limit
//         motor.configReverseSoftLimitEnable(true); // Enable the reverse soft limit

//         // Step 11: Control Mode Setup
//         if (isDriveMotor) {
//             motor.set(TalonFXControlMode.PercentOutput, 0); // Use open-loop control for drive motor (percentage output)
//         } else {
//             motor.set(TalonFXControlMode.Position, 0); // Set position control for the rotation motor (in encoder units)
//         }
//     }

//     // Method to control the drive motor's speed (for drive forward/backward)
//     public void drive(double speed) {
//         // Set the drive motor's speed based on input (-1.0 to 1.0)
//         driveMotor.set(TalonFXControlMode.PercentOutput, speed);
//     }

//     // Method to rotate the rotation motor to a specific position
//     public void rotateToPosition(int position) {
//         // Set the rotation motor to a specific position using position control mode
//         rotationMotor.set(TalonFXControlMode.Position, position);
//     }

//     // Method to get the current position (encoder value) of the rotation motor
//     public int getRotationPosition() {
//         // Get the current position of the rotation motor from its encoder
//         return rotationMotor.getSelectedSensorPosition();
//     }

//     // Method to stop both motors
//     public void stopMotors() {
//         // Set both motors to stop (0 speed)
//         driveMotor.set(TalonFXControlMode.PercentOutput, 0);
//         rotationMotor.set(TalonFXControlMode.PercentOutput, 0);
//     }
// }
