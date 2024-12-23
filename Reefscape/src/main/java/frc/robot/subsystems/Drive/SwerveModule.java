package frc.robot.subsystems.Drive;

import java.lang.module.Configuration;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;


public class SwerveModule extends SubsystemBase {
  
  /**
   * Class to represent and handle a swerve module
   * A module's state is measured by a CANCoder for the absolute position,
   * integrated CANEncoder for relative position
   * for both rotation and linear movement
   */


  private static SwerveBase swerveDrive;
  public PIDController testRotationController;
  public PIDController RotationController;
  public PIDController DriveController;
  public double DriveControllerkp;

  private final TalonFX driveMotor;
  private final TalonFX rotationMotor;


  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  public TalonFX getRotationMotor() {
    return rotationMotor;
  }
  

  public final TalonFXConfigurator rotationEncoder;
  public final TalonFXConfigurator driveEncoder;

  private final CANCoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;


  
  public SwerveModule(
      int driveMotorId,
      double drivemotorkP,
      int rotationMotorId,
      double rotationmotorkP,
      int canCoderId,
      double measuredOffsetRadians,
      SwerveBase swerveSubsystem) {

    swerveDrive = swerveSubsystem;
  
    //Defines what Talon to target
    driveMotor = new TalonFX(driveMotorId, "rio");

    //Get encoder for that Talon
    var driveConfig = new TalonFXConfiguration();

    //Turns on current limitor
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set the supply current limit to 40 amps (continuous)
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;//amps

    // Set the supply peak current to 60 amps (short bursts)
    driveConfig.CurrentLimits.SupplyCurrentThreshold = 70.0;//amps

    // Set the time (in seconds) the motor can run at peak current before limiting kicks in
    driveConfig.CurrentLimits.SupplyTimeThreshold = 1.0; //second

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
    rotationMotor = new TalonFX(rotationMotorId, "rio");

    var rotationConfig = new TalonFXConfiguration();

    //Turns on current limitor
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set the supply current limit to 40 amps (continuous)
    rotationConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    // Set the supply peak current to 60 amps (short bursts)
    rotationConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;

    // Set the time (in seconds) the motor can run at peak current before limiting kicks in
    rotationConfig.CurrentLimits.SupplyTimeThreshold = 1.0; // 1 second

    //Applies current limitor
    rotationMotor.getConfigurator().apply(rotationConfig);

    //Configures the encoder
    rotationEncoder = rotationMotor.getConfigurator();


 
    //get canCoder
    canCoder = new CANCoder(canCoderId);
  

    //Set the offset to make the wheel go "staight"
    offset = new Rotation2d(measuredOffsetRadians);


    //Sets PID value for how the rotation motor willl reach it's target
    //You can lovwer this value to decress speed to help save wheels but will make the robot harder to drive
    testRotationController = new PIDController(0.5, 0.0, 0.0);
    testRotationController.enableContinuousInput(-Math.PI, Math.PI);



    RotationController = new PIDController(rotationmotorkP, 0, 0.0);
    RotationController.enableContinuousInput(-Math.PI, Math.PI);

    DriveController = new PIDController(drivemotorkP /*+ Slope*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)*/ , 0.0, 0.0);
   // DriveControllerkp = drivemotorkP;

    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  

  public void resetDistance() {

    driveMotor.setPosition(0.0);

  }

  public Double getDriveDistanceRadians() {

    return Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()) / SwerveConstants.driveGearRatio;

  }

  public Rotation2d getCanCoderAngle() {

    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians())
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
      return;
    }

  // SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

  //   double angularSetPoint = placeInAppropriate0To360Scope(
  //       optimizedDesiredState.angle.getRadians());


    //SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());
    //SwerveModuleState optimizedDesiredState = unoptimizedDesiredState;
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(unoptimizedDesiredState, getState().angle);
    // double angularSetPoint = placeInAppropriate0To360Scope(
    //     optimizedDesiredState.angle.getRadians());
    double angularSetPoint = optimizedDesiredState.angle.getRadians();


    //Was 
    rotationMotor.setVoltage(RotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));



    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond /
        (SwerveConstants.wheelDiameter / 2.0);

     
  //Was 
  driveMotor.setVoltage(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed)*SwerveConstants.calibrationFactorSB);

 //This should compart to our current to make a feed back might not work becasue it might take all swerves into acount insted of one
 // driveMotor.setVoltage(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond,getCurrentVelocityMetersPerSecond())/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);
  
}

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

}
}
