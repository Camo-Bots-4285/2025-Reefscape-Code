// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
 public final class Constants {
  public static boolean isRed= true;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AprilTagConstants {
  }

  public static class ArmPivotConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // 2 NEO Motor
    public static final int MOTOR_ARM_PIVOT_A = 20;
    public static final int MOTOR_ARM_PIVOT_B = 21;

    // PID Locations
     public static final int POSITION_STARTING = 0;
    public static final int POSITION_INTAKE_FLOOR = 1;
    public static final int POSITION_INTAKE_FEEDER = 2;
    public static final int POSITION_TRAVEL = 3;
    public static final int POSITION_AMP_SCORING = 4;
    public static final int POSITION_SHOOTING = 5;
    public static final int POSITION_HUMAN_FEEDER = 6;
    public static final int POSITION_STORE = 7;
    public static final int POSITION_ERRECTED = 8;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS = 9;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 10;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 11;
    public static final int POSITION_AMP_SCORING_POS = 12;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_N1 = 13;
    public static final int POSITION_SHOOTING_DEFENCE = 14;
    public static final int POSITION_LINE_SCORING = 15;
    public static final int POSITION_TRAP_SCORING = 16;
    public static final int POSITION_FAR_FEEDER = 17;

    public static final double POSITION_PID_STARTING = 0;// Robot Will go here on start
    public static final double POSITION_PID_INTAKE_FLOOR = POSITION_STARTING;
    public static final double POSITION_PID_INTAKE_FEEDER = 1.31;
    public static final double POSITION_PID_AMP_SCORING = -2.3333;
    public static final double POSITION_PID_HUMAN_FEEDER = -2.499999; //-2.595237 //-2.666665
    //-2.499999 Far Human Feeder
    public static final double POSITION_PID_STORE = 6.214284;
    public static final double POSITION_PID_ERRECTED = 0;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS =  2.1666 ;//2.1666   2.5   2.833331  2.880950
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 3.41; 
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 3.15;
    public static final double POSITION_PID_AMP_SCORING_POS = -1.9; //-1.738;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_N1 = 3.25; //1.07126
    public static final double POSITION_PID_SHOOTING_DEFENCE = .1;
    public static final double POSITION_PID_LINE_SCORING = 2.7;//2.85
    public static final double POSITION_PID_SHOOTING_TRAP = 1.761905;
    public static final double POSITION_PID_FAR_FEEDER = 2.833331;

    // 36 inch -- 1.31
    // 42 inch -- 1.336
    // 48 inch -- 1.37
    // 54 inch -- 1.35
    // 60 inch -- 1.36
    // 66 inch -- 1.382
    // 72 inch -- 1.388
    // 78 inch -- 1.395
    // 84 inch -- 1.42
    // 90 inch -- 1.44
    // 96 inch -- 1.45
    // 102 inch -- 1.47
    // 108 inch -- 1.48
    // 114 inch -- 1.48
    // 120 inch -- 1.485
    // 126 inch -- 1.49
    // 132 inch -- 1.495
    // 138 inch -- 1.495
    // 144 inch -- 1.50
    // 150 inch -- 1.50
    // 156 inch -- 1.505
    // 162 inch -- 1.51
    // 168 inch -- 1.515
    // 174 inch -- 1.53
    // 180 inch -- 1.5375
    // 186 inch -- 1.5425
    // 192 inch -- 1.54525
    // 198 inch -- 1.54
    // 204 inch -- 1.535
    // 216 inch --1.5375 This value shoot good disk high and bad disk low

    // public static final double POSITION_PID_INTAKE_FEEDER = 1.4;
    public static final double POSITION_PID_TRAVEL = 0.22;

  }

  public static class ClimberConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */
  }

  public static class IntakeConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_INTAKE_FLOOR = 13;
    public static final int MOTOR_INTAKE_FLOOR2 = 14;

  }

  public static class ShooterConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // Falcon Motor
    public static final int MOTOR_SHOOTER_A = 15;

    // Falcon Motor
    public static final int MOTOR_SHOOTER_B = 16;
  }

  public static class ShooterFeederConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    //public static final int MOTOR_SHOOTER_FEEDER = 11;

    //public static final int SHOOTER_FEEDER_STATE_FLOOR_FEEDING = 0;
    //public static final int SHOOTER_FEEDER_STATE_HUMAN_FEEDING = 1;
    //public static final int SHOOTER_FEEDER_REVERSE = 3;
    //public static final int SHOOTER_FEEDER_FAR_FEEDER = 4;

  }

  public static class SwerveConstants {

    /*Coding Swerve
    1) If controler is diffent from previes year change Drive Controls
    2) Set how far apart the wheels are in DriveTeain Constants
    3) Set the Diameter of the wheel in Wheel Diameter
    4) Get gear ratio off of Swerve Drive Specialties and set the in Gear Ratio
    5) Get max thoetical speeds from Swerve Drive Specialties best off of motor we are using
    6) Set all Sparks/Talons/Encoder/Pigeons to the correct ID
    7) If Pigeon is not mounted with the X-axis faceing the front of the bot change NormalPigeonOfSet
    8) Then set of "absolute encoder offsets" to zero deploy code and then use a STRAIGHT piece of hard something 
    to make the wheel all face forward with gear to the inside of the bot
    9) DO NOT PLACE BOT on the ground tip it up so you can see the wheels drive forward use 
    "Change Driver Motor Derection" to change derection as needed
    10) Set Max TeleOp Speed based on driver prefernce and Choreo
    11) At this point the robot should be driveable how ever it is very far from being a reliable drive base in auto
    or consistent estimating its POSE without apriltags view Swerve Calibration to make these thing happen


    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; // was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */// Measured from center of each module (wheel axis)
    public static final double trackWidth = Units.inchesToMeters(21.5); 
    public static final double wheelBase = Units.inchesToMeters(21.5);
    public static final double robotRotationFactor = Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase);

    /* Wheel Diameter *///Try to get as accurate as possible to reduce error 
    public static final double wheelDiameter = Units.inchesToMeters(3.8); 
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Gear Ratio */
    public static final double driveGearRatio = 5.14; // Mk4 drive ratio
    public static final double turnGearRatio = 12.8; // Mk4 steer ratio   
    
    /* Swerve Profiling Values */
    public static final double maxSpeed = 6.03504;//In meter/Second 
    public static final double maxAngularVelocity = robotRotationFactor*Math.PI/maxSpeed*2*Math.PI;
    
    /*Encoder Id's + Pigeon */
    public static final int frontLeftRotationEncoderId = 2;
    public static final int frontRightRotationEncoderId = 1;
    public static final int rearLeftRotationEncoderId = 3;
    public static final int rearRightRotationEncoderId = 4;
    
    public static final int PIGEON_SENSOR_ID = 0;

    /*Spark/Talon Id's */
    public static final int frontLeftRotationMotorId = 12;
    public static final int frontLeftDriveMotorId = 22;

    public static final int frontRightRotationMotorId = 11;
    public static final int frontRightDriveMotorId = 21;

    public static final int rearLeftRotationMotorId = 13;
    public static final int rearLeftDriveMotorId = 23;

    public static final int rearRightRotationMotorId = 14;
    public static final int rearRightDriveMotorId = 24;

    /* Set Normal Pigeon of Set */
    public static final int NormalPigeonOfSet = 180;

    /* TeleOp Swerve Constants *///Tune a little higher then what driver is confetable driving for Fast and normal
    public static double kTeleDriveMaxSpeedMetersPerSecondFast = 5.5;//Try getting this value from Choreo
    public static double kTeleDriveMaxSpeedMetersPerSecondNormal = 0.5; 
    public static double kTeleDriveMaxSpeedMetersPerSecondSlow = 0.125;   

    public static double kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecondNormal;  
    public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;//Try getting this value from Choreo
    
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.PI;//Try getting this value from Choreo
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0;//Try getting this value from Choreo


    /* Swerve Calibration
    Now after all that set up we need to make sure the Swerve Base gets calibrated correctly. The following goes over how
    to calibrate a bot who center of gravity is in the center of the robot

    NOTE: If Robots Center is gravity is not center of the wheels
    Thought technically weight should have nothing to do with the movement of a module, it can add friction. I have added a system 
    to acount for this called Weight/SwerveDriveModule. If you find conventianl tuneing is not working as well as you hoped it would.
    Uncomment everything that says "needed for Weight/SwerveDriveModule". Keep in mind this is all theoretical and has not been test 
    as of (8/14/2024) so system might need a few changes like changes in signs to become fully functioning.

    1) Edit(DrivePID1)/Make a button that uses the drive command at a set speed (perferably speed for auto) make sure calibrationFactorSB = 1.0
    Lift robot up and tune PIDs for free spin (PIDs for Indivual Motors) PID is tuned when SmartDashBoard ## wheel Speed is withing fice decimal
    places of the taget value

    2) Choose multiple other speeds and record taget(x) vs actual-target(y) without changing PIDS use this data to create an equation to 
    add to DriveController kP in swerve module

    Equation formate:
    Linear: Slope*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)
    Quadratic: A(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed) + B(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)

    3) When you are satisfied with the tracking place to robot on the ground and find the percent error and make the inverese equal to calibrationFactorSB 
   
    4) Now it is final time to start tuning auto!! Go to "Set PIDs for Path Planner" in Swerve Base.
    
    */

    /*PIDs for set Voltage*/
    // public static double frontLeft_Drive_kP = 0.3487;//0.3487//.38
    // public static double frontLeft_Rotation_kP = 0.5;

    // public static double frontRight_Drive_kP = 0.3464;//0.3464//.388
    // public static double frontRight_Rotation_kP = 0.5;

    // public static double rearLeft_Drive_kP = 0.3447;//0.3447//.38
    // public static double rearLeft_Rotation_kP = 0.5;

    // public static double rearRight_Drive_kP = 0.3473;//0.3473.3945
     //public static double rearRight_Rotation_kP = 0.5;

    /*PIDs for Indivual Motors */
    public static double frontLeft_Drive_kP = 1.004;
    public static double frontLeft_Rotation_kP = 0.5;

    public static double frontRight_Drive_kP = 0.992;
    public static double frontRight_Rotation_kP = 0.5;

    public static double rearLeft_Drive_kP =  0.9885;
    public static double rearLeft_Rotation_kP = 0.5;

    public static double rearRight_Drive_kP = 0.999;
    public static double rearRight_Rotation_kP = 0.5;

    /* Calibration Factor to Help offest for weight start with this at one*/
    public static double calibrationFactorSB = 1.17;//1.11


    //The following should not be touch 
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    public static Translation2d mDriveRadius = new Translation2d(trackWidth/2, wheelBase/ 2);
    
  }

  public static final class VisionConstants {

    //TODO Calculate and get these values, Units need to be in meters and radians
    /**
     * Physical location of the apriltag camera on the robot, relative to the center
     * of the robot.
     */

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
    //     new Translation3d(-0.063, -0.3125, 0.562),// Get from CAD Model In meters-0.063, -0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
    //     new Translation3d(0.063, -0.3125, 0.562),//0.063, -0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
    //     new Translation3d(0.063, 0.3125, 0.562),//0.063, 0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(-0.063, 0.3125, 0.562),//-0.063, 0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));
    
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
        new Translation3d(-0.063, 0.3125, 0.562),// Get from CAD Model In meters-0.063, -0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));//

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
        new Translation3d(0.063, -0.3125, 0.562),//0.063, -0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
        new Translation3d(-0.063, 0.3125, 0.562),//0.063, 0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
        new Translation3d(-0.063, -0.3125, 0.562),//-0.063, 0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));//-225
    //Lime Light 
    //          CAD             RealLife   OldVaules 
    // Height  0.6673348        0.684      0.669
    //Front    0.792804         0.072      0.059
    //Right    0.3028135        0.311      0.303
    //Angle    29.7169722 Deg   30 Deg     30 deg
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.3109483, 0.0631137, -0.567547072),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135.0)));
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_5 = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(30.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));
    
        //Main Note Camera Assumed to be center 
     public static final Transform3d Note_Camera_Main_To_Robot = new Transform3d(
        new Translation3d(0.18, 0.0, 0.38),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0),15.0, Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant1_To_Robot = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant2_To_Robot = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));
    
    
    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.2042;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
      new Translation2d(FIELD_LENGTH_METERS , FIELD_WIDTH_METERS),
      new Rotation2d(Math.PI)
    );

    

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    //Raise this value to reject less accurate poses 0.2 recommended by photon vision
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class LEDConstants {
      public static final int DIO_LED_IS_BLUE = 4;
      public static final int DIO_LED_IS_RED = 5;
      public static final int DIO_LED_WIN = 6;

      // public static int DIO_One = 1;
      // public static int DIO_Ten = 2;
      // public static int DIO_Hundred = 3;
      // public static int DIO_Thousand = 4;
      public static final Boolean DIO_ENABLE = false;
      public static final Boolean DIO_DISABLE = true;

      
    }


public static class LineBreakConstants {

    public static int DIO_BOTTOM_SENSOR = 8;
    public static int DIO_TOP_SENSOR = 7;

    public static final boolean LINEBREAK_BLOCKED = false;
    public static final boolean LINEBREAK_OPEN = true;
  }

  public static class BuildConstants {
    public static final String MAVEN_GROUP = "";
    public static final String MAVEN_NAME = "crescendo";
    public static final String VERSION = "unspecified";
    public static final int GIT_REVISION = 6;
    public static final String GIT_SHA = "cc013129e3c7396cb75b6440b939e763719c4f9a";
    public static final String GIT_DATE = "2024-05-28 11:17:52 EDT";
    public static final String GIT_BRANCH = "main";
    public static final String BUILD_DATE = "2024-07-16 15:15:26 EDT";
    public static final long BUILD_UNIX_TIME = 1721157326492L;
    public static final int DIRTY = 1;
  }

 }
