package frc.robot.subsystems.Drive;
import frc.robot.subsystems.ComputerVision.QuestNav;
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

// 2025 - commented out till pathplanner is fixed
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.config.*;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import java.sql.Driver;

//import com.ctre.phoenix.sensors.Pigeon2Configuration;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;



public class SwerveBasePose extends SwerveBase { 

    public static double currentPoseX;
    public static double currentPoseY;
    public static double currentPoseRotation;

    public SwerveBasePose() {

        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

    double driveBaseRadius = Constants.SwerveConstants.mDriveRadius.getNorm();


    // Old version
    // AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                 new PIDConstants(2.525, 0.0, 0.0),//Translational   2.525
    //                 new PIDConstants(3.15, 0.0, 0.0),//Rotational  3.173
    //                 6.03504, //5.7912  module speed, in m/s
    //                 driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //                 new ReplanningConfig(false, true, 20 , 20) // 0.5,0.25 0.6 to high 0.4 too low 0.5 nice Default path replanning config. See the API for the options here   
    //         ),
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           Boolean alliance = Constants.isRed;
    //           if (alliance) {
    //             return alliance == Constants.isRed;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );




    //Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    // RobotConfig config;
    // try{
    //   config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

    // Commented out till pathplanner is fixed
    // 2025 version
    // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> robotRelativeDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }
  public static Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
  private final double SCALE_X = -1/0.9;
  private final double SCALE_Y = -1/0.9;

    /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */

   private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, new Rotation2d(),
   getModulePositions(), new Pose2d());


    public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
    }

    // reset the current pose to a desired pose
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeadingPose(), getModulePositions(), pose);
    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeadingPose() {
        return Rotation2d.fromDegrees(getPigeonYaw() + SwerveConstants.NormalPigeonOfSet + RobotContainer.AutoPigeonOfSet);
      // Old Version
      //  return Rotation2d.fromDegrees(pigeonSensor.getYaw() + SwerveConstants.NormalPigeonOfSet + RobotContainer.AutoPigeonOfSet);
      }

    public Pose2d getScaledPose() {
        m_pose = getPose();
        final var translation = new Translation2d(m_pose.getX() * SCALE_X, m_pose.getY() * SCALE_Y);
        final var rotation = m_pose.getRotation().rotateBy(new Rotation2d(0));
        return new Pose2d(translation.getX(), translation.getY(), rotation);
    }

   /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  } 

      public Pose3d getPose3d(){
    return new Pose3d(getOdometry().getEstimatedPosition());
    }

@Override
public void periodic() {
    //Runs DogLog to log values to USB
    DogLog();


    currentPoseX = getPose().getX();
    currentPoseY = getPose().getY();
    currentPoseRotation = getPose().getRotation().getDegrees() + 180;
    //System.out.println(currentPoseX);
    //System.out.println(currentPoseY);
    SmartDashboard.putNumber("RobotPoseX", currentPoseX);
    SmartDashboard.putNumber("RobotPoseY", currentPoseY);

    // update the odometry every 20ms
    odometry.update(getHeadingPose(), getModulePositions());

    SmartDashboard.putString("Robot pose",
    getPose().toString());
SmartDashboard.putNumber("Bot Heading",
    getHeadingPose().getDegrees());
 }

  private void DogLog(){
        // //Published Pose Values
        // DogLog.log("/PoseEstimation/PublishedValue/Pose", getPose());
        // DogLog.log("/PoseEstimation/PublishedValue/Heading",  getHeadingPose());
        // DogLog.log("/PoseEstimation/PublishedValue/Odometry", odometry);

       
        // //Swerve Pose Reading
        // DogLog.log("/PoseEstimation/SwerveValue/Pose", getPose());
        // DogLog.log("/PoseEstimation/Swervevalue/Heading",  getHeadingPose());
        // DogLog.log("/PoseEstimation/SwerveValue/Odometry", odometry);
    }
}