package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPoint;


import frc.robot.Constants.*;

import java.sql.Driver;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import dev.doglog.DogLog;




public class SwerveBase extends SubsystemBase { 
  
  // Declare a PigeonIMU object and a motor for demonstration purposes
  private Pigeon2 pigeonIMU;

  // Declare variables to store IMU data
  private double[] imuData = new double[3]; // [yaw, pitch, roll]


  private double oldPigeonYaw = 0.0;
  public static boolean  isFieldRelative1 = true;

  public static boolean AllowMainDriving = true;
  public static double translation;
  public static double rotation;
  public static boolean needMoreAmps;
  public static int SwerveAmps;
  public static boolean GettingNote;

  public static double SwerveTuneingkP;


  private final int loopCount; // Number of loops for averaging velocity changes
  private double[] yawHistory; // Array to store velocity data for each loop
  private int currentLoopIndex = 0; // Index for the current loop iteration
  private double lastTime; // To store the last timestamp of the loop for calculating delta-time
  public double timer;

//Checks if setNeedMoreAmps is True of false and change need more
//amps based on if the command is being called
public void setNeedMoreAmps(boolean set) {
    needMoreAmps = set;
  }


public void setTeleOpMaxSwerveSpeed(double speed) {
  SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = speed;
  }


  public SwerveBase() {

    // Initialize the PigeonIMU connected to CAN ID 1 (adjust as necessary)
    pigeonIMU = new Pigeon2(Constants.SwerveConstants.PIGEON_SENSOR_ID);
    zeroPigeonYaw();

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();


    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    /*Change Driver Motor Derection */
    //if true inverts the derection of the drive motor
    rearRight.getDriveMotor().setInverted(true);
    rearLeft.getDriveMotor().setInverted(false);
    frontRight.getDriveMotor().setInverted(true);
    frontLeft.getDriveMotor().setInverted(false);

    //if true inverts the derection of the rotation motor should not need to be changed
    rearRight.getRotationMotor().setInverted(false);
    rearLeft.getRotationMotor().setInverted(false);
    frontRight.getRotationMotor().setInverted(false);
    frontLeft.getRotationMotor().setInverted(false);

    this.loopCount = 3;//if you change need to change accleration
    this.yawHistory = new double[loopCount];

  }
  
  /*
   * The following is the Method that are used to interact with the pigeon
   */

     private boolean needPigeonReset = false;
   //Is used to help zero pigeon when method is called
  public void setNeedPigeonReset(boolean set) {
    needPigeonReset = set;
  }

  //Zeros Pigeon sencor Yaw
  public void zeroPigeonYaw() {
    // Reset yaw to 0 degrees
    pigeonIMU.setYaw(0);  // Set yaw to 0 degrees to reset the heading
  }

    //Set a value Pigeon sencor Yaw
    public void setPigeonYaw(double Yaw_Degrees) {
      // Reset yaw to 0 degrees
      pigeonIMU.setYaw(Yaw_Degrees);  // Set yaw to 0 degrees to reset the heading
    }

// Method to get the yaw value (heading) from the IMU
public double getPigeonYaw() {
  // use pigeonIMU.getRotation3d() and put it in a variable named currentPosition
  Rotation3d currentPosition = pigeonIMU.getRotation3d();

  return currentPosition.getZ();
  

  // Old Code
  //return pigeonIMU.getAngle();  // Return the yaw value (heading in degrees)
  // pigeonIMU.(imuData);  // Fetch the latest yaw, pitch, and roll data from the IMU
  //return imuData[0];  // Return the yaw value (heading in degrees)
}

public double getPigeonYawRate(){
  // Calculate deltaTime (time difference between current and previous loop)
  double deltaTime = timer - lastTime;
  lastTime = timer; // Update the lastTime for the next loop

  // Store the current velocity in the velocity history array
  yawHistory[currentLoopIndex] = getPigeonYaw();

  // Increment the loop counter and wrap around if necessary
  currentLoopIndex = (currentLoopIndex + 1) % loopCount;

  // Calculate the average velocity change over the loop history
  double yawChangeSum = 0;
  for (int i = 1; i < loopCount; i++) {
      int prevIndex = (currentLoopIndex - i + loopCount) % loopCount;
      yawChangeSum += yawHistory[prevIndex] - yawHistory[(prevIndex - 1 + loopCount) % loopCount];
  }

  // Average the velocity changes and divide by deltaTime to get acceleration
  double averageYawRateChange = yawChangeSum / loopCount;
  double yawRate = averageYawRateChange / deltaTime;
  return yawRate;
}


// Method to get the pitch value from the IMU
public double getPigeonPitch() {
  // use pigeonIMU.getRotation3d() and put it in a variable named currentPosition
  Rotation3d currentPosition = pigeonIMU.getRotation3d();

  return currentPosition.getY();
  
  // Old Code
  //pigeonIMU.getYawPitchRoll(imuData);  // Fetch the latest yaw, pitch, and roll data from the IMU
  //return imuData[1];  // Return the pitch value (in degrees)
}

// Method to get the roll value from the IMU
public double getPigeonRoll() {
  // use pigeonIMU.getRotation3d() and put it in a variable named currentPosition
  Rotation3d currentPosition = pigeonIMU.getRotation3d(); 
  
  return currentPosition.getX();
  
  // Old Code
  //pigeonIMU.getYawPitchRoll(imuData);  // Fetch the latest yaw, pitch, and roll data from the IMU
  //return imuData[2];  // Return the roll value (in degrees)
}

public Rotation2d getGyroscopeRotation() {
  // WARNING!!! I am not sure if this is returning the correct value!

  // use pigeonIMU.getRotation3d() and put it in a variable named currentPosition
  Rotation3d currentPosition = pigeonIMU.getRotation3d(); 
  
  return currentPosition.toRotation2d();
   // Old Code
  //pigeonIMU.getYawPitchRoll(imuData);  // Fetch the latest yaw, pitch, and roll data
  //return Rotation2d.fromDegrees(imuData[0]);   // Return the yaw value which represents the compass heading
}

  //to be used for Driving the robot the heading whille be temparrly set to zero when driving in robot centric
  public Rotation2d getHeadingDrive() {
     if(isFieldRelative1 == true){
    return Rotation2d.fromDegrees(getPigeonYaw() + SwerveConstants.NormalPigeonOfSet);
    }
    if(RobotState.isAutonomous()){
      return Rotation2d.fromDegrees(getPigeonYaw() + SwerveConstants.NormalPigeonOfSet);
    }
    else{
      return Rotation2d.fromDegrees(SwerveConstants.NormalPigeonOfSet);
    }
  }

  /**
   * The following variable will be sent to the constructor in Swerve module to intilize the swerve module
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */
  public static final double frontLeftAngleOffset = Units.degreesToRadians(169.72);//239.94
  private static final double frontRightAngleOffset = Units.degreesToRadians(85.61);//15
  private static final double rearLeftAngleOffset = Units.degreesToRadians(158.47);//202.85
  private static final double rearRightAngleOffset = Units.degreesToRadians(275.36);//132.45


  /**
   * SwerveModule objects
   * Parameters:
   * drive motor CAN ID
   * drive motor PID value P
   * rotation motor can ID
   * rotation motor PID value P
   * external CANCoder can ID
   * measured CANCoder offset
   */

  public final SwerveModule frontLeft = new SwerveModule(
      2,
      SwerveConstants.frontLeftDriveMotorId,
      SwerveConstants.frontLeft_Drive_PID,
      SwerveConstants.frontLeft_Drive_FF,

      SwerveConstants.frontLeftRotationMotorId,
      SwerveConstants.frontLeft_Rotation_PID,
      SwerveConstants.frontLeft_Rotation_FF,

      SwerveConstants.frontLeftRotationEncoderId,
      frontLeftAngleOffset,
      this);

  public final SwerveModule frontRight = new SwerveModule(
       1,
      SwerveConstants.frontRightDriveMotorId,
      SwerveConstants.frontRight_Drive_PID,
      SwerveConstants.frontRight_Drive_FF,

      SwerveConstants.frontRightRotationMotorId,
      SwerveConstants.frontRight_Rotation_PID,
      SwerveConstants.frontRight_Rotation_FF,

      SwerveConstants.frontRightRotationEncoderId,
      frontRightAngleOffset,
      this);

  public final SwerveModule rearLeft = new SwerveModule(
      3,
      SwerveConstants.rearLeftDriveMotorId,
      SwerveConstants.rearLeft_Drive_PID,
      SwerveConstants.rearLeft_Drive_FF,

      SwerveConstants.rearLeftRotationMotorId,
      SwerveConstants.rearLeft_Rotation_PID,
      SwerveConstants.rearLeft_Rotation_FF,

      SwerveConstants.rearLeftRotationEncoderId,
      rearLeftAngleOffset,
      this);

  public final SwerveModule rearRight = new SwerveModule(
      4,
      SwerveConstants.rearRightDriveMotorId,
      SwerveConstants.rearRight_Drive_PID,
      SwerveConstants.rearRight_Drive_FF,

      SwerveConstants.rearRightRotationMotorId,
      SwerveConstants.rearRight_Rotation_PID,
      SwerveConstants.rearRight_Rotation_FF,

      SwerveConstants.rearRightRotationEncoderId,
      rearRightAngleOffset,
      this);



 
  

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
   */
  
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */


    // this is where feild relitive is activated was changed when trying to fix the pigeon yaw not going back to normal after
       if (needPigeonReset) {
      needPigeonReset = false;
      setPigeonYaw(oldPigeonYaw);
      isFieldRelative1 = true;
    }

     if (isFieldRelative == false) {
      isFieldRelative1 = false;
    }

     if(isFieldRelative == true){
      isFieldRelative1 = true;
    }


    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      forward, strafe, rotation, getHeadingDrive()
    );

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states);
 
  }
    // Set the wheels into an X formation to prevent movement
    //NOTE - has not been tested
    public void setX() {
      frontLeft.setDesiredStateClosedLoop(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      frontRight.setDesiredStateClosedLoop(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      rearLeft.setDesiredStateClosedLoop(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      rearRight.setDesiredStateClosedLoop(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }
  
    // reset the measured distance driven for each module
    public void resetDriveDistances() {
      frontLeft.resetDistance();
      frontRight.resetDistance();
      rearLeft.resetDistance();
      rearRight.resetDistance();
    }



    //Was used to make robot face the speaker in 2024 good example
  // public Rotation2d getAngleToSpeaker(){
  //   Translation2d robotPose = getPose().getTranslation();
  //  // Translation2d diffPose = RobotContainer.m_ArmPivotSubsystem.getSpeakerPose().getTranslation().minus(robotPose);
  //  // return new Rotation2d(Math.atan2(diffPose.getY(), diffPose.getX()));
  // }


  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

/**
 * The following is the periodic loop that record necisary varibales
 */
 @Override
  public void periodic() {
    //Runs DogLog to log values to USB
    DogLog();

    SmartDashboard.putNumber("CanCoderRR", rearRight.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderRL", rearLeft.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderFR", frontRight.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderFL", frontLeft.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorRR", rearRight.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorRL", rearLeft.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorFR", frontRight.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorFL", frontLeft.getIntegratedAngle().getRotations());

    // SmartDashboard.putString("Pigeon Rotation",
    // pigeonSensor.getRotation2d().toString());
    SmartDashboard.putNumber("Pigeon Yaw",
    getPigeonYaw());
    SmartDashboard.putString("Pigeon Compass",
    getGyroscopeRotation().toString());

    SmartDashboard.putString("FL Wheel Angle", frontLeft.getCanCoderAngle().toString());
    SmartDashboard.putString("FR Wheel Angle", frontRight.getCanCoderAngle().toString());
    SmartDashboard.putString("RL Wheel Angle", rearLeft.getCanCoderAngle().toString());
    SmartDashboard.putString("RR Wheel Angle", rearRight.getCanCoderAngle().toString());

    SmartDashboard.putNumber("FL Wheel Speed",  frontLeft.getCurrentVelocityRadiansPerSecond()/(2*Math.PI)*SwerveConstants.wheelCircumference);
    SmartDashboard.putNumber("FR Wheel Speed", frontRight.getCurrentVelocityRadiansPerSecond()/(2*Math.PI)*SwerveConstants.wheelCircumference);
    SmartDashboard.putNumber("RL Wheel Speed", rearLeft.getCurrentVelocityRadiansPerSecond()/(2*Math.PI)*SwerveConstants.wheelCircumference);
    SmartDashboard.putNumber("RR Wheel Speed", rearRight.getCurrentVelocityRadiansPerSecond()/(2*Math.PI)*SwerveConstants.wheelCircumference);

    SmartDashboard.putNumber("FL Wheel Speed2", Math.round(frontLeft.getCurrentVelocityMetersPerSecond()));
    SmartDashboard.putNumber("FR Wheel Speed2", Math.round(frontRight.getCurrentVelocityMetersPerSecond()));
    SmartDashboard.putNumber("RL Wheel Speed2", Math.round(rearLeft.getCurrentVelocityMetersPerSecond()));
    SmartDashboard.putNumber("RR Wheel Speed2", Math.round(rearRight.getCurrentVelocityMetersPerSecond()));
    
    timer = Robot.Time;
 }

 private void DogLog(){
    //Pigeon Data
    DogLog.log("/SwerveBase/Pigeon/Yaw", getPigeonYaw());
    DogLog.log("/SwerveBase/Pigeon/Pitch",getPigeonPitch());
    DogLog.log("/SwerveBase/Pigeon/Roll", getPigeonRoll());
    DogLog.log("/SwerveBase/Pigeon/Yaw_Rate", getPigeonYawRate());
    DogLog.log("/SwerveBase/Pigeon/Compass", getGyroscopeRotation());
    DogLog.log("/SwerveBase/Pigeon/Drive_Heading", getHeadingDrive());

    //CanCoder Angle
    DogLog.log("/SwerveBase/FL/CanCoderAngle", frontLeft.getCanCoderAngle());
    DogLog.log("/SwerveBase/FR/CanCoderAngle", frontRight.getCanCoderAngle());
    DogLog.log("/SwerveBase/RL/CanCoderAngle", rearLeft.getCanCoderAngle());
    DogLog.log("/SwerveBase/RR/CanCoderAngle", rearRight.getCanCoderAngle());

    //Vaule of wheels that are being ready
    DogLog.log("/SwerveBase/FL/Wheel_Angle", frontLeft.getIntegratedAngle());
    DogLog.log("/SwerveBase/FR/Wheel_Angle", frontRight.getIntegratedAngle());
    DogLog.log("/SwerveBase/RL/Wheel_Angle", rearLeft.getIntegratedAngle());
    DogLog.log("/SwerveBase/RR/Wheel_Angle", rearRight.getIntegratedAngle());

    //Value of wheel speeds
    DogLog.log("/SwerveBase/FL/Wheel_Speed", Math.round(frontLeft.getCurrentVelocityMetersPerSecond()));
    DogLog.log("/SwerveBase/FR/Wheel_Speed", Math.round(frontRight.getCurrentVelocityMetersPerSecond()));
    DogLog.log("/SwerveBase/RL/Wheel_Speed", Math.round(rearLeft.getCurrentVelocityMetersPerSecond()));
    DogLog.log("/SwerveBase/RR/Wheel_Speed", Math.round(rearRight.getCurrentVelocityMetersPerSecond()));

    //Logs same value as above but might be messy need to see
    DogLog.log("/SwerveBase/ModuleStates/ModuleStates",getModuleStates());

    //Values that help run diferent types of code
    DogLog.log("/SwerveBase/Values/isFeildRelative1",isFieldRelative1);
    DogLog.log("/SwerveBase/Values/AllowMainDriving",AllowMainDriving);

 } 


/**
 * This code does all the calculation for the swerve drive
 */


  public SwerveDriveKinematics getKinematics() {
    return SwerveConstants.kinematics;
  }



  // returns an array of SwerveModuleState
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
        new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return states;

  }

  // returns an array of SwerveModulePositions
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModulePosition(frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModulePosition(rearLeft.getCurrentDistanceMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModulePosition(rearRight.getCurrentDistanceMetersPerSecond(), rearRight.getIntegratedAngle())
    };

    return positions;

  }


  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  public void robotRelativeDrive(ChassisSpeeds speeds){
    setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds));
  }



















// /// Custom Sendable class for the swerve drive
// private class SwerveDriveSendable implements Sendable {
//   @Override
//   public void initSendable(SendableBuilder builder) {
//       builder.setSmartDashboardType("SwerveDrive");

//       // Use lambdas to access module properties
//       builder.addDoubleProperty("Front Left Angle", 
//           () -> frontLeft.getIntegratedAngle().getRadians(), null);
//       builder.addDoubleProperty("Front Left Velocity", 
//           () -> frontLeft.getCurrentVelocityMetersPerSecond(), null);

//       builder.addDoubleProperty("Front Right Angle", 
//           () -> frontRight.getIntegratedAngle().getRadians(), null);
//       builder.addDoubleProperty("Front Right Velocity", 
//           () -> frontRight.getCurrentVelocityMetersPerSecond(), null);

//       builder.addDoubleProperty("Rear Left Angle", 
//           () -> rearLeft.getIntegratedAngle().getRadians(), null);
//       builder.addDoubleProperty("Rear Left Velocity", 
//           () -> rearLeft.getCurrentVelocityMetersPerSecond(), null);

//       builder.addDoubleProperty("Rear Right Angle", 
//           () -> rearRight.getIntegratedAngle().getRadians(), null);
//       builder.addDoubleProperty("Rear Right Velocity", 
//           () -> rearRight.getCurrentVelocityMetersPerSecond(), null);

//       builder.addDoubleProperty("Robot Angle", 
//           () -> getHeadingDrive().getRadians(), null);
//   }}

}