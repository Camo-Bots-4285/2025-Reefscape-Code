package frc.robot.subsystems.Drive;

import java.lang.module.Configuration;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
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

  public double PositionConversionFactor = 2.0 * Math.PI / SwerveConstants.driveGearRatio;
  public double VelocityConversionFactor = 2.0 * Math.PI / 60 / SwerveConstants.driveGearRatio;
  public double AngleConversionFactor = 2.0 * Math.PI / SwerveConstants.angleGearRatio;

  private final TalonFX driveMotor;
  private final CurrentLimitsConfigs driveMotorLimiter;
  private final TalonFX rotationMotor;
  private final CurrentLimitsConfigs rotationMotorLimiter;

  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  public TalonFX getRotationMotor() {
    return rotationMotor;
  }
  
  public final TalonFXConfigurator driveEncoder;
  public final TalonFXConfigurator rotationEncoder;

  private final CANCoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;

  // private SparkPIDController rotationController;
  // private TalonFX driveController;
  
  public SwerveModule(
      int driveMotorId,
      double drivemotorkP,
      int rotationMotorId,
      double rotationmotorkP,
      int canCoderId,
      double measuredOffsetRadians,
      SwerveBase swerveSubsystem) {

    swerveDrive = swerveSubsystem;
  
    //Defines what spark to target
    //driveMotor = new TalonFX(driveMotorId, MotorType.kBrushless);
    driveMotor = new TalonFX(driveMotorId, "rio");
    driveMotorLimiter = new CurrentLimitsConfigs();
    driveMotor.getConfigurator().apply(driveMotorLimiter);
    

    rotationMotor = new TalonFX(rotationMotorId, "rio");
    rotationMotorLimiter = new CurrentLimitsConfigs();
    rotationMotor.getConfigurator().apply(rotationMotorLimiter);
    


    //Get encoder for that spark
    driveEncoder = driveMotor.getConfigurator();
    rotationEncoder = rotationMotor.getConfigurator();
 
    //get canCoder
    canCoder = new CANCoder(canCoderId);

    //Set the offset to make the wheel go "staight"
    offset = new Rotation2d(measuredOffsetRadians);

    //Sets the Idle mode to brake so robot can not be pushed when motor is not in use
    //driveMotor.setIdleMode(IdleMode.kBrake);
    //rotationMotor.setIdleMode(IdleMode.kBrake);

    //Sets PID controller to the SPARK
    // rotationController = rotationMotor.getPIDController();
    // driveController = driveMotor.getPIDController();

    //Sets the P value for the PID
    // rotationController.setP(rotationmotorkP);
    // driveController.setP(drivemotorkP);

    //Sets PID value for how the rotation motor willl reach it's target
    //You can lovwer this value to decress speed to help save wheels but will make the robot harder to drive
    testRotationController = new PIDController(0.5, 0.0, 0.0);
    testRotationController.enableContinuousInput(-Math.PI, Math.PI);



    RotationController = new PIDController(rotationmotorkP, 0, 0.0);
    //RotationController.enableContinuousInput(-Math.PI, Math.PI);

    DriveController = new PIDController(drivemotorkP /*+ Slope*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)*/ , 0.0, 0.0);
   // DriveControllerkp = drivemotorkP;


    // set the output of the drive encoder to be in radians for linear measurement

    // set the output of the drive encoder to be in radians per second for velocity
    // measurement


    // set the output of the rotation encoder to be in radians
//     rotationEncoder.SensorToMechancilRatio(2.0 * Math.PI / SwerveConstants.angleGearRatio);

//     rotationEncoder.Feedback.SensorToMechanismRatio = 1;

//     rotationMotor.getFeedback().setSensorToMechanismRatio(1);

//   // Set the sensor-to-mechanism ratio directly on the rotation motor
// rotationMotor.configSensorTerm(TalonFXFeedbackDevice.IntegratedSensor, 1.0); // Adjust based on your requirements

// // Example to configure feedback sensor
// rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

//    rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor);


    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  

  public void resetDistance() {

    driveEncoder.setPosition(0.0);

  }

  public Double getDriveDistanceRadians() {

    return driveMotor.getPosition().getValueAsDouble() * PositionConversionFactor;

  }

  public Rotation2d getCanCoderAngle() {

    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians())
        % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getIntegratedAngle() {
    // Wass
    // double unsignedAngle = rotationMotor.getPosition() % (2 * Math.PI);

    // if (unsignedAngle < 0)
    //   unsignedAngle += 2 * Math.PI;

    // return new Rotation2d(unsignedAngle);

    //Carlos
   // return new Rotation2d(rotationEncoder.getPosition());
    return new Rotation2d(rotationMotor.getPosition().getValueAsDouble()*AngleConversionFactor);

  }

  //     public FeedbackConfigs withSensorToMechanismRatio(double newSensorToMechanismRatio)
  //   {
  //       SensorToMechanismRatio = newSensorToMechanismRatio;
  //       return this;
  //  }


  public double getCurrentVelocityRadiansPerSecond() {

    return driveMotor.getVelocity().getValueAsDouble() * VelocityConversionFactor;

  }

  public double getCurrentVelocityMetersPerSecond() {

    return driveMotor.getVelocity().getValueAsDouble() * VelocityConversionFactor;

  }

  public double getCurrentDistanceMetersPerSecond() {
    return driveMotor.getPosition().getValueAsDouble() * PositionConversionFactor;
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

    rotationEncoder.setPosition(getCanCoderAngle().getRadians());

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

// Wass
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
    rotationMotor.set(RotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));
    //Change to make more consistent thought set voltage
    //rotationMotor.setVoltage(RotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint)/SwerveConstants.maxAngularVelocity);

    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond /
        (SwerveConstants.wheelDiameter / 2.0);

     
      
    //driveMotor.set(DriveController.calculate(optimizedDesiredState.speedMetersPerSecond)*SwerveConstants.calibrationFactorSB);
   // driveMotor.set(testRotationController.calculate(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed),optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed)*SwerveConstants.calibrationFactorSB);
    
   
   
   
  //Was 
  driveMotor.set(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond/SwerveConstants.maxSpeed)*SwerveConstants.calibrationFactorSB);
  
  //This soulde defently work
  //driveMotor.setVoltage(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond)/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);

 //This should compart to our current to make a feed back might not work becasue it might take all swerves into acount insted of one
 // driveMotor.setVoltage(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond,getCurrentVelocityMetersPerSecond())/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);
    System.out.print(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond)/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);

    //Sets the max amps the swerve base can draw
    //Need to be tested with krakens
    // if(RobotState.isAutonomous()){
    //   driveMotorLimiter.StatorCurrentLimit = 45;

    // }
    // else if(SwerveBase.needMoreAmps == true){
    //   driveMotorLimiter.StatorCurrentLimit = 45;
    // }
    // else if(SwerveBase.needMoreAmps == false){
    //   driveMotorLimiter.StatorCurrentLimit = 35;
    // }

    // driveMotorLimiter.StatorCurrentLimitEnable = true;
    // driveMotor.getConfigurator().apply(driveMotorLimiter);

  //   if (SwerveBase.FasterSwerve == true) {
  //     //System.out.println("Swerve is Fast");
  //     SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = 5.5;//Faster swerve speed
  //     }
  //  if (SwerveBase.SlowerSwerve == true) {
  //    //System.out.println("Swerve is Slow");
  //    SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = 0.125;//Slower swerve speed
  //  }
  //  if(SwerveBase.FasterSwerve == false & SwerveBase.SlowerSwerve == false){
  //   //System.out.println("Swerve is Normal 0.5");
  //    SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = .5;//Normal swerve speed
     
  //  }
}

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);
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
 // System.out.println(-DriveController.calculate(optimizedDesiredState.speedMetersPerSecond)/SwerveConstants.maxSpeed*SwerveConstants.calibrationFactorSB);
}
}
