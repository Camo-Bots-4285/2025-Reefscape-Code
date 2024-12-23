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



public class SwerveBase extends SubsystemBase {  
  public static WPI_Pigeon2 pigeonSensor;
  private Pigeon2Configuration pigeonConfig;
  private double oldPigeonYaw = 0.0;
  public static boolean  isFieldRelative1 = true;

  public static boolean AllowMainDriving = true;
  public static boolean GettingNote = false;
public static double translation;
public static double rotation;
public static boolean needMoreAmps;
public static int SwerveAmps;

public static double currentPoseX;
public static double currentPoseY;
public static double currentPoseRotation;

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


    odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

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

    double driveBaseRadius = Constants.SwerveConstants.mDriveRadius.getNorm();

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.525, 0.0, 0.0),//Translational   2.525
                    new PIDConstants(3.15, 0.0, 0.0),//Rotational  3.173
                    6.03504, //5.7912  module speed, in m/s
                    driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(false, true, 20 , 20) // 0.5,0.25 0.6 to high 0.4 too low 0.5 nice Default path replanning config. See the API for the options here   
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              Boolean alliance = Constants.isRed;
              if (alliance) {
                return alliance == Constants.isRed;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void zeroPigeon() {
    pigeonSensor.reset();
  }




  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
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

  public static Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
  private final double SCALE_X = -1/0.9;
  private final double SCALE_Y = -1/0.9;

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
      SwerveConstants.frontLeftDriveMotorId,
      SwerveConstants.frontLeft_Drive_kP,
      SwerveConstants.frontLeftRotationMotorId,
      SwerveConstants.frontLeft_Rotation_kP,
      SwerveConstants.frontLeftRotationEncoderId,
      frontLeftAngleOffset,
      this);

  public final SwerveModule frontRight = new SwerveModule(
      SwerveConstants.frontRightDriveMotorId,
      SwerveConstants.frontRight_Drive_kP,
      SwerveConstants.frontRightRotationMotorId,
      SwerveConstants.frontRight_Rotation_kP,
      SwerveConstants.frontRightRotationEncoderId,
      frontRightAngleOffset,
      this);

  public final SwerveModule rearLeft = new SwerveModule(
      SwerveConstants.rearLeftDriveMotorId,
      SwerveConstants.rearLeft_Drive_kP,
      SwerveConstants.rearLeftRotationMotorId,
      SwerveConstants.rearLeft_Rotation_kP,
      SwerveConstants.rearLeftRotationEncoderId,
      rearLeftAngleOffset,
      this);

  public final SwerveModule rearRight = new SwerveModule(
      SwerveConstants.rearRightDriveMotorId,
      SwerveConstants.rearRight_Drive_kP,
      SwerveConstants.rearRightRotationMotorId,
      SwerveConstants.rearRight_Rotation_kP,
      SwerveConstants.rearRightRotationEncoderId,
      rearRightAngleOffset,
      this);


  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, new Rotation2d(),
      getModulePositions(), new Pose2d());
  private boolean needPigeonReset = false;

 
  public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
  }

  public Pose3d getPose3d(){
    return new Pose3d(getOdometry().getEstimatedPosition());
  }
  
   //Is used to help zero pigeon when method is called
  public void setNeedPigeonReset(boolean set) {
    needPigeonReset = set;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("CanCoderRR", rearRight.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderRL", rearLeft.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderFR", frontRight.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("CanCoderFL", frontLeft.getCanCoderAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorRR", rearRight.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorRL", rearLeft.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorFR", frontRight.getIntegratedAngle().getRotations());
    SmartDashboard.putNumber("RotationMotorFL", frontLeft.getIntegratedAngle().getRotations());

    currentPoseX = getPose().getX();
    currentPoseY = getPose().getY();
    currentPoseRotation = getPose().getRotation().getDegrees() + 180;
    //System.out.println(currentPoseX);
    //System.out.println(currentPoseY);
    SmartDashboard.putNumber("RobotPoseX", currentPoseX);
    SmartDashboard.putNumber("RobotPoseY", currentPoseY);

    // update the odometry every 20ms
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putString("Robot pose",
        getPose().toString());
    SmartDashboard.putNumber("Bot Heading",
        getHeading().getDegrees());
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

  public Pose2d getScaledPose() {
    m_pose = getPose();
    final var translation = new Translation2d(m_pose.getX() * SCALE_X, m_pose.getY() * SCALE_Y);
    final var rotation = m_pose.getRotation().rotateBy(new Rotation2d(0));
    return new Pose2d(translation.getX(), translation.getY(), rotation);
  }

  public Rotation2d getGyroscopeRotation() {

    return Rotation2d.fromDegrees(pigeonSensor.getCompassHeading());
    
  }

  public void robotRelativeDrive(ChassisSpeeds speeds){
    setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds));
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

  public ChassisSpeeds getRobotRelativeChassisSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  // reset the current pose to a desired pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();
  }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {

    return Rotation2d.fromDegrees(pigeonSensor.getYaw() + SwerveConstants.NormalPigeonOfSet + RobotContainer.AutoPigeonOfSet);

  }
  //this is a duplicate of getHeading but is use to make the robot drive in robot centic
  //while evrything else like camera and odomatery tracking till use the normal pigeon heading
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

  public WPI_Pigeon2 getPigeonSensor() {
    return pigeonSensor;
  }

  public SwerveDriveKinematics getKinematics() {
    return SwerveConstants.kinematics;
  }


/*The following is all  code that was added to try to selfdrive in autp */
  // public Command goToNode(int apriltag, int node) {
  //   Rotation2d heading;
  //   Translation3d nodeTrans = Field.getNodeCoordinatesFieldRelative(apriltag, node);
  //   ChassisSpeeds currentSpeeds = getRobotRelativeChassisSpeeds();

  //   double linearVel =
  //       Math.sqrt(
  //           (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
  //               + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));


  //   Translation2d goal = new Translation2d(
  //       Field.fieldLayout.getTagPose(apriltag).get().getTranslation().getX() + Field.DIST_FROM_NODE_X_METERS,
  //   nodeTrans.getY());
  //   if (getPose().getY() > goal.getY()) {
  //     heading = Rotation2d.fromDegrees(-90);
  //   }
  //   else {
  //     heading = Rotation2d.fromDegrees(90);
  //   }

  //   PathPoint initialPoint = new PathPoint(
  //     getPose().getTranslation(), heading, getPose().getRotation(), linearVel);
  //   PathPlannerTrajectory trajToGoal = PathPlanner.generatePath(
  //       new PathConstraints(1, 1.5),
  //       //PathPoint.fromCurrentHolonomicState(getPose(), getChassisSpeeds()),
  //       initialPoint,
  //       new PathPoint(goal, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), -1)); // position, heading(direction of
  //                                                                                     // travel), holonomic rotation
  //   //return followTrajectoryCommand(trajToGoal, false);
  //  return AutoBuilder.followPath(trajToGoal);
  // }



  //     Different idea
  //       Pose3d currentPose = getPose3d();
  //       Pose3d tagPose = Vision.aprilTags.getTagPose(4).get();

  //       return PathPlanner.generatePath(
  //         PathConstraints(2.0, 4.0),
  //         PathPoint(currentPose.translation, currentPose.rotation),
  //         PathPoint(tagPose.translation.toTranslation2d() - Translation2d(2.0, 0.0), currentPose.rotation),
  //       );

}

