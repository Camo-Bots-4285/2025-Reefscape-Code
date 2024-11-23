package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.sql.Driver;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.SwerveConstants;

public class SwerveBase extends SubsystemBase {

    public static boolean isFieldRelative1 = true;
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

    // private final GryroIO _io;
    // private final GryroIOInputsAutoLogged _inputs = new GryroIOInputsAutoLogged();

    private final GyroIO gyroIO;
    private final GyroIOInputs gyroInputs = new GyroIOInputs();
    private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;


    private final ModuleIO moduleIO;
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();


    // Checks if setNeedMoreAmps is True of false and change need more
    // amps based on if the command is being called
    public void setNeedMoreAmps(boolean set) {
        needMoreAmps = set;
    }

    public static boolean FasterSwerve;

    public void setFasterSwerve(boolean set) {
        FasterSwerve = set;
    }

    public static boolean SlowerSwerve;

    public void setSlowerSwerve(boolean set) {
        SlowerSwerve = set;
    }

    public SwerveBase(      
    RealGyroIO gyroIO,
    ModuleIO FLModuleIO,
    ModuleIO FRModuleIO,
    ModuleIO RLModuleIO,
    ModuleIO RRModuleIO) {
  this.gyroIO = gyroIO;
  modules[1] = new SwerveModule(FLModuleIO, 1);
  modules[2] = new SwerveModule(FRModuleIO, 2);
  modules[3] = new SwerveModule(RLModuleIO, 3);
  modules[4] = new SwerveModule(RRModuleIO, 4);


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

        /* Change Driver Motor Derection */
        // if true inverts the derection of the drive motor
        rearRight.getDriveMotor().setInverted(true);
        rearLeft.getDriveMotor().setInverted(false);
        frontRight.getDriveMotor().setInverted(true);
        frontLeft.getDriveMotor().setInverted(false);

        // if true inverts the derection of the rotation motor should not need to be
        // changed
        rearRight.getRotationMotor().setInverted(false);
        rearLeft.getRotationMotor().setInverted(false);
        frontRight.getRotationMotor().setInverted(false);
        frontLeft.getRotationMotor().setInverted(false);














 



















        double driveBaseRadius = Constants.SwerveConstants.mDriveRadius.getNorm();

        /*
         * Set PIDs for Path Planner
         * NOTE: you should alway use pathplanner to create the autos as of (8/14/2024)
         * becuase Choreo will not auto correct
         * NOTE: Thought it is not required it might help to make sure all gears on
         * swerve wheels are faceing inwards at the state of a match or auto
         * 
         * 1) Make two paths with Choreo or PathPlanner then make them into an auto
         * 1. Drive in a straight line from a know location on the feild (For about 2-3
         * meters or what ever lets you get up to max speed and back down)
         * 2. Make the same path as path one but make the robot rotate while driving
         * 2) Either hard code them in or add them to auto chooser
         * 3) Make out on the ground where the robot thought start and stop (peferable
         * to edge of the bumpers)
         * 4) Unplug all cameras set enableInitialreplanning and enableDynamicReplanning
         * to false now robot should drive with no correction software
         * 5) Run auto one adjust translation PID till satisfied
         * 6) Run auto two adjust rotation PID till satisfied
         * 7) Test a full auto path and make sure robot mantans correct position
         * 8) If robot can complete and auto with no auto correction system
         * enableDynamicReplanning run auto but use a spare robot to
         * bump it where it might hit another robot adjust DynamicReplanningError
         * Threshold till it can correct of a bump Range should be 0-100 check
         * pathplanner to see if value was changed
         * 9) If you wish enableInitialReplanning it will use cameras to correct
         * starting Pose (hopefullu the LEDs can do that)
         * 10) Add in camera and repeat bumptest to insure variable are right can do a
         * bigger hit and see if camera adjust for it
         * 
         */
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(2.525, 0.0, 0.0), // Translational 2.525
                        new PIDConstants(3.15, 0.0, 0.0), // Rotational 3.173
                        6.03504, // 5.7912 module speed, in m/s
                        driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(false, true, 20, 20) // 0.5,0.25 0.6 to high 0.4 too low 0.5 nice Default
                                                                  // path replanning config. See the API for the options
                                                                  // here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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
        gyroIO.zeroPigeon();
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
    public static final double frontLeftAngleOffset = Units.degreesToRadians(170.51);// 239.94
    private static final double frontRightAngleOffset = Units.degreesToRadians(84.73);// 15
    private static final double rearLeftAngleOffset = Units.degreesToRadians(158.38);// 202.85
    private static final double rearRightAngleOffset = Units.degreesToRadians(275.27);// 132.45

    public static Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
    private final double SCALE_X = -1 / 0.9;
    private final double SCALE_Y = -1 / 0.9;

    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * drive motor PID value P
     * rotation motor can ID
     * rotation motor PID value P
     * external CANCoder can ID
     * measured CANCoder offset
     */

    // public final SwerveModule frontLeft = new SwerveModule(
            
    //         Constants.SwerveConstants.frontLeftDriveMotorId,
    //         Constants.SwerveConstants.frontLeft_Drive_kP,
    //         Constants.SwerveConstants.frontLeftRotationMotorId,
    //         Constants.SwerveConstants.frontLeft_Rotation_kP,
    //         Constants.SwerveConstants.frontLeftRotationEncoderId,
    //         frontLeftAngleOffset,
    //         this);

    // public final SwerveModule frontRight = new SwerveModule(
    //         Constants.SwerveConstants.frontRightDriveMotorId,
    //         Constants.SwerveConstants.frontRight_Drive_kP,
    //         Constants.SwerveConstants.frontRightRotationMotorId,
    //         Constants.SwerveConstants.frontRight_Rotation_kP,
    //         Constants.SwerveConstants.frontRightRotationEncoderId,
    //         frontRightAngleOffset,
    //         this);

    // public final SwerveModule rearLeft = new SwerveModule(
        
    //         Constants.SwerveConstants.rearLeftDriveMotorId,
    //         Constants.SwerveConstants.rearLeft_Drive_kP,
    //         Constants.SwerveConstants.rearLeftRotationMotorId,
    //         Constants.SwerveConstants.rearLeft_Rotation_kP,
    //         Constants.SwerveConstants.rearLeftRotationEncoderId,
    //         rearLeftAngleOffset,
    //         this);

    // public final SwerveModule rearRight = new SwerveModule(

    //         Constants.SwerveConstants.rearRightDriveMotorId,
    //         Constants.SwerveConstants.rearRight_Drive_kP,
    //         Constants.SwerveConstants.rearRightRotationMotorId,
    //         Constants.SwerveConstants.rearRight_Rotation_kP,
    //         Constants.SwerveConstants.rearRightRotationEncoderId,
    //         rearRightAngleOffset,
    //         this);

    /**
     * odometry for the robot, measured in meters for linear motion and radians for
     * rotational motion
     * Takes in kinematics and robot angle for parameters
     */

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kinematics,
            new Rotation2d(),
            getModulePositions(), new Pose2d());
    private boolean needPigeonReset = false;

    public SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }

    public Pose3d getPose3d() {
        return new Pose3d(getOdometry().getEstimatedPosition());
    }

    // Is used to help zero pigeon when method is called
    public void setNeedPigeonReset(boolean set) {
        needPigeonReset = set;
    }

    @Override
    public void periodic() {

        currentPoseX = getPose().getX();
        currentPoseY = getPose().getY();
        currentPoseRotation = getPose().getRotation().getDegrees() + 180;

        // update the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putString("Robot pose",
                getPose().toString());
        SmartDashboard.putNumber("Bot Heading",
                getHeading().getDegrees());
        SmartDashboard.putNumber("Pigeon Rotation",
        gyroInputs._pigeonCompassHeading);
        SmartDashboard.putNumber("Pigeon Yaw",
        gyroInputs._pigeonSensorYaw);
        SmartDashboard.putNumber("Pigeon Compass",
        gyroInputs._pigeonCompassHeading);

        SmartDashboard.putString("FL Wheel Angle", moduleInputs._turnAbsolutePosition.toString());
        SmartDashboard.putString("FR Wheel Angle", frontRight.getCanCoderAngle().toString());
        SmartDashboard.putString("RL Wheel Angle", rearLeft.getCanCoderAngle().toString());
        SmartDashboard.putString("RR Wheel Angle", rearRight.getCanCoderAngle().toString());

        SmartDashboard.putNumber("FL Wheel Speed",
                frontLeft.getCurrentVelocityRadiansPerSecond() / (2 * Math.PI) * Constants.SwerveConstants.wheelCircumference);
        SmartDashboard.putNumber("FR Wheel Speed",
                frontRight.getCurrentVelocityRadiansPerSecond() / (2 * Math.PI) * Constants.SwerveConstants.wheelCircumference);
        SmartDashboard.putNumber("RL Wheel Speed",
                rearLeft.getCurrentVelocityRadiansPerSecond() / (2 * Math.PI) * Constants.SwerveConstants.wheelCircumference);
        SmartDashboard.putNumber("RR Wheel Speed",
                rearRight.getCurrentVelocityRadiansPerSecond() / (2 * Math.PI) * Constants.SwerveConstants.wheelCircumference);

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

        // this is where feild relitive is activated was changed when trying to fix the
        // pigeon yaw not going back to normal after
        if (needPigeonReset) {
            needPigeonReset = false;
            gyroIO.setHeadingForward();
            isFieldRelative1 = true;
        }

        if (isFieldRelative == false) {
            isFieldRelative1 = false;
        }

        if (isFieldRelative == true) {
            isFieldRelative1 = true;
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, rotation, getHeadingDrive());

        // use kinematics (wheel placements) to convert overall robot state to array of
        // individual module states
        SwerveModuleState[] states = Constants.SwerveConstants.kinematics.toSwerveModuleStates(speeds);

        setModuleStates(states);

    }

    public Pose2d getScaledPose() {
        m_pose = getPose();
        final var translation = new Translation2d(m_pose.getX() * SCALE_X, m_pose.getY() * SCALE_Y);
        final var rotation = m_pose.getRotation().rotateBy(new Rotation2d(0));
        return new Pose2d(translation.getX(), translation.getY(), rotation);
    }

    public Rotation2d getGyroscopeRotation() {

        return Rotation2d.fromDegrees(gyroInputs._pigeonCompassHeading);

    }

    public void robotRelativeDrive(ChassisSpeeds speeds) {
        setModuleStates(Constants.SwerveConstants.kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values
     * for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        // make sure the wheels don't try to spin faster than the maximum speed possible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.maxSpeed);
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
                new SwerveModulePosition(frontLeft.getCurrentDistanceMeters(), frontLeft.getIntegratedAngle()),
                new SwerveModulePosition(frontRight.getCurrentDistanceMeters(),
                        frontRight.getIntegratedAngle()),
                new SwerveModulePosition(rearLeft.getCurrentDistanceMeters(), rearLeft.getIntegratedAngle()),
                new SwerveModulePosition(rearRight.getCurrentDistanceMeters(), rearRight.getIntegratedAngle())
        };

        return positions;

    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
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

        return Rotation2d.fromDegrees(
                gyroInputs._pigeonSensorYaw + Constants.SwerveConstants.NormalPigeonOfSet + RobotContainer.AutoPigeonOfSet);

    }

    // this is a duplicate of getHeading but is use to make the robot drive in robot
    // centic
    // while evrything else like camera and odomatery tracking till use the normal
    // pigeon heading
    public Rotation2d getHeadingDrive() {
        if (isFieldRelative1 == true) {
            return Rotation2d.fromDegrees(gyroInputs._pigeonSensorYaw + Constants.SwerveConstants.NormalPigeonOfSet);
        }
        if (RobotState.isAutonomous()) {
            return Rotation2d.fromDegrees(gyroInputs._pigeonSensorYaw + Constants.SwerveConstants.NormalPigeonOfSet);
        } else {
            return Rotation2d.fromDegrees(Constants.SwerveConstants.NormalPigeonOfSet);
        }
    }

    // public Rotation2d getAngleToSpeaker(){
    // Translation2d robotPose = getPose().getTranslation();
    // // Translation2d diffPose =
    // RobotContainer.m_ArmPivotSubsystem.getSpeakerPose().getTranslation().minus(robotPose);
    // // return new Rotation2d(Math.atan2(diffPose.getY(), diffPose.getX()));
    // }

    public void stopModules() {
    moduleIO.stop();
    }



    public SwerveDriveKinematics getKinematics() {
        return Constants.SwerveConstants.kinematics;
    }

    // public Command goToNode(int apriltag, int node) {
    // Rotation2d heading;
    // Translation3d nodeTrans = Field.getNodeCoordinatesFieldRelative(apriltag,
    // node);
    // ChassisSpeeds currentSpeeds = getRobotRelativeChassisSpeeds();

    // double linearVel =
    // Math.sqrt(
    // (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
    // + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));

    // Translation2d goal = new Translation2d(
    // Field.fieldLayout.getTagPose(apriltag).get().getTranslation().getX() +
    // Field.DIST_FROM_NODE_X_METERS,
    // nodeTrans.getY());
    // if (getPose().getY() > goal.getY()) {
    // heading = Rotation2d.fromDegrees(-90);
    // }
    // else {
    // heading = Rotation2d.fromDegrees(90);
    // }

    // PathPoint initialPoint = new PathPoint(
    // getPose().getTranslation(), heading, getPose().getRotation(), linearVel);
    // PathPlannerTrajectory trajToGoal = PathPlanner.generatePath(
    // new PathConstraints(1, 1.5),
    // //PathPoint.fromCurrentHolonomicState(getPose(), getChassisSpeeds()),
    // initialPoint,
    // new PathPoint(goal, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180),
    // -1)); // position, heading(direction of
    // // travel), holonomic rotation
    // //return followTrajectoryCommand(trajToGoal, false);
    // return AutoBuilder.followPath(trajToGoal);
    // }

    // Different idea
    // Pose3d currentPose = getPose3d();
    // Pose3d tagPose = Vision.aprilTags.getTagPose(4).get();

    // return PathPlanner.generatePath(
    // PathConstraints(2.0, 4.0),
    // PathPoint(currentPose.translation, currentPose.rotation),
    // PathPoint(tagPose.translation.toTranslation2d() - Translation2d(2.0, 0.0),
    // currentPose.rotation),
    // );

}
