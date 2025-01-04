package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.Constants.*;

import java.sql.Driver;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.PigeonIMU;
//import dev.doglog.DogLog;



public class SwerveBase extends SubsystemBase {  
  private PigeonIMU pigeonIMU;
  
  private double oldPigeonYaw = 0.0;
  public static boolean  isFieldRelative1 = true;

  public static boolean AllowMainDriving = true;
  public static double translation;
  public static double rotation;
  public static boolean needMoreAmps;
  public static int SwerveAmps;
  public static boolean GettingNote;

  public static double SwerveTuneingkP;

//Checks if setNeedMoreAmps is True of false and change need more
//amps based on if the command is being called
public void setNeedMoreAmps(boolean set) {
    needMoreAmps = set;
  }


public void setTeleOpMaxSwerveSpeed(double speed) {
  SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = speed;
  }


  public SwerveBase() {

    pigeonSensor = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_SENSOR_ID);
    pigeonConfig = new Pigeon2Configuration();
    pigeonSensor.configFactoryDefault();
    pigeonSensor.reset();
    zeroPigeon();

    pigeonSensor.getAllConfigs(pigeonConfig);

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

  }
  
  /*
   * The following is the Method that are used to interact with the pigeon
   */

  // Return Pigeon sensor
   public WPI_Pigeon2 getPigeonSensor() {
    return pigeonSensor;
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeonSensor.getCompassHeading());
  }

     private boolean needPigeonReset = false;
   //Is used to help zero pigeon when method is called
  public void setNeedPigeonReset(boolean set) {
    needPigeonReset = set;
  }

  //Zeros Pigeon sencor
  public void zeroPigeon() {
    pigeonSensor.reset();
  }

  //to be used for Driving the robot the heading whille be temparrly set to zero when driving in robot centric
  public Rotation2d getHeadingDrive() {
     if(isFieldRelative1 == true){
    return Rotation2d.fromDegrees(pigeonSensor.getYaw() + SwerveConstants.NormalPigeonOfSet);
    }
    if(RobotState.isAutonomous()){
      return Rotation2d.fromDegrees(pigeonSensor.getYaw() + SwerveConstants.NormalPigeonOfSet);
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
      pigeonSensor.setYaw(oldPigeonYaw);
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

    SmartDashboard.putString("Pigeon Rotation",
    pigeonSensor.getRotation2d().toString());
    SmartDashboard.putNumber("Pigeon Yaw",
    pigeonSensor.getYaw());
    SmartDashboard.putNumber("Pigeon Compass",
    pigeonSensor.getCompassHeading());

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
    

 }

 private void DogLog(){
    // //Pigeon Data
    // DogLog.log("/SwerveBase/Pigeon/Rotation2D", pigeonSensor.getRotation2d().toString());
    // DogLog.log("/SwerveBase/Pigeon/Yaw", pigeonSensor.getYaw());
    // DogLog.log("/SwerveBase/Pigeon/Pitch",pigeonSensor.getPitch());
    // DogLog.log("/SwerveBase/Pigeon/Roll", pigeonSensor.getRoll());
    // DogLog.log("/SwerveBase/Pigeon/Rate", pigeonSensor.getRate());
    // DogLog.log("/SwerveBase/Pigeon/Compass", pigeonSensor.getCompassHeading());
    // DogLog.log("/SwerveBase/Pigeon/Compass", getHeadingDrive());

    // //CanCoder Angle
    // DogLog.log("/SwerveBase/FL/CanCoderAngle", frontLeft.getCanCoderAngle());
    // DogLog.log("/SwerveBase/FR/CanCoderAngle", frontRight.getCanCoderAngle());
    // DogLog.log("/SwerveBase/RL/CanCoderAngle", rearLeft.getCanCoderAngle());
    // DogLog.log("/SwerveBase/RR/CanCoderAngle", rearRight.getCanCoderAngle());

    // //Vaule of wheels that are being ready
    // DogLog.log("/SwerveBase/FL/Wheel_Angle", frontLeft.getIntegratedAngle());
    // DogLog.log("/SwerveBase/FR/Wheel_Angle", frontRight.getIntegratedAngle());
    // DogLog.log("/SwerveBase/RL/Wheel_Angle", rearLeft.getIntegratedAngle());
    // DogLog.log("/SwerveBase/RR/Wheel_Angle", rearRight.getIntegratedAngle());

    // //Value of wheel speeds
    // DogLog.log("/SwerveBase/FL/Wheel_Speed", Math.round(frontLeft.getCurrentVelocityMetersPerSecond()));
    // DogLog.log("/SwerveBase/FR/Wheel_Speed", Math.round(frontRight.getCurrentVelocityMetersPerSecond()));
    // DogLog.log("/SwerveBase/RL/Wheel_Speed", Math.round(rearLeft.getCurrentVelocityMetersPerSecond()));
    // DogLog.log("/SwerveBase/RR/Wheel_Speed", Math.round(rearRight.getCurrentVelocityMetersPerSecond()));

    // //Logs same value as above but might be messy need to see
    // DogLog.log("/SwerveBase/ModuleStates/ModuleStates",getModuleStates());

    // //Values that help run diferent types of code
    // DogLog.log("/SwerveBase/Values/isFeildRelative1",isFieldRelative1);
    // DogLog.log("/SwerveBase/Values/AllowMainDriving",AllowMainDriving);

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

}

