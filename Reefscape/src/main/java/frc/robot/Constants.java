// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;

 

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


  public static final class SwerveConstants {
    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; // was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */// Measured from center of each module (wheel axis)
    public static final double trackWidth = Units.inchesToMeters(22.625); 
    public static final double wheelBase = Units.inchesToMeters(22.625);
    public static final double robotRotationFactor = Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase);

    /* Wheel Diameter *///Try to get as accurate as possible to reduce error 
    public static final double wheelDiameter = Units.inchesToMeters(4); 
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Gear Ratio */
    public static final double driveGearRatio = 6.12; // Mk3 drive ratio
    public static final double angleGearRatio = 12.8; // Mk3 steer ratio   
    
    /* Swerve Profiling Values these value are all therotical if robot is not
     * tacking as you wish or not finctioing properly please lower these number
     * Also any time there is an update to the swerve base these value will need to be rcalibrated
     */
    public static final double maxSpeed = 5.21208;//In meter/Second  5.0292
    public static final double maxAngularVelocity = maxSpeed/robotRotationFactor; //meter/sec to radian/sec
    public static final double maxAcceleration = 8.59;

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
    public static final int NormalPigeonOfSet = 0;

    /* TeleOp Swerve Constants *///Tune a little higher then what driver is confetable driving for Fast and normal
    public static double kTeleDriveMaxSpeedMetersPerSecondFast = maxSpeed;//Try getting this value from Choreo
    public static double kTeleDriveMaxSpeedMetersPerSecondNormal = 2.0; 
    public static double kTeleDriveMaxSpeedMetersPerSecondSlow = 0.125;   

    public static double kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecondNormal;  
    public static double kTeleDriveMaxAccelerationUnitsPerSecond = maxAcceleration;//Try getting this value from Choreo
    
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxSpeedMetersPerSecond/robotRotationFactor;//Try getting this value from Choreo
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = maxAcceleration/robotRotationFactor;//Try getting this value from Choreo


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
    //  public static double rearRight_Rotation_kP = 0.5;

    /**PIDs for Indivual Motors 
     * 
     * While tuning first start with FF you can use the following links to 
     * help get a starting value try get get all variable as close as posible
     * 
     * Drive-https://www.reca.lc/drive
     *  When you use this cite please copy and pasta link to your project below so there may see the values you used
     *
     * Rotation- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html
     * You can read thought the following website and try to implment a FF contollor but as long as the
     * wheel turn quickly in the durection we want and stay there this is a little less import that does not mean
     * do not tak your time and make sure it is right.
     */
    public static PIDController frontLeft_Drive_PID = new PIDController(0.0, 0.0, 0.0);//14.1207
    public static PIDController frontRight_Drive_PID = new PIDController(0.0, 0.0, 0.0);//14.1125
    public static PIDController rearLeft_Drive_PID = new PIDController(0.0, 0.0, 0.0);//14.347
    // Changed to test
    public static PIDController rearRight_Drive_PID = new PIDController(0.0, 0.0, 0.0);
    //public static PIDController rearRight_Drive_PID = new PIDController(.219, 0.0, 0.00085432);//14.18//1.5//1.7//0.5, 0.0,0.00195052
   //0.00901141 to much
   //0.0090114 good ish  -6.5 2.31, 0.0, 0.0090114
   //0.0090112 to little
   
    //It seem that kp and kv should be the same value
    //How ever kd has to offset the uncertenty of kp so the wheel does not occilate. 0.0, 0.0, 0.0

    public static SimpleMotorFeedforward frontLeft_Drive_FF = new SimpleMotorFeedforward(0.13989,2.190,0.01);// 0.13989,2.190,0.01
    public static SimpleMotorFeedforward frontRight_Drive_FF = new SimpleMotorFeedforward(0.1399,2.178,0.01);// 0.1399,2.178,0.01
    public static SimpleMotorFeedforward rearLeft_Drive_FF = new SimpleMotorFeedforward(0.1399,2.220,0.01);// 0.1399,2.220,0.01
    public static SimpleMotorFeedforward rearRight_Drive_FF = new SimpleMotorFeedforward(0.1399,2.190,0.01);//0.1399,2.190,0.01

    public static PIDController frontLeft_Rotation_PID = new PIDController(5.0, 0.0, 0.0);
    public static PIDController frontRight_Rotation_PID = new PIDController(5.0, 0, 0);
    public static PIDController rearLeft_Rotation_PID = new PIDController(5.0, 0, 0);
    public static PIDController rearRight_Rotation_PID = new PIDController(5.0, 0, 0);

    public static SimpleMotorFeedforward frontLeft_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward frontRight_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward rearLeft_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward rearRight_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);


/*
 * Step-by-Step PID and FeedForward Tuning Plan for Robot Movement
 * 
 * 1. **Set Initial Values:**
 *    - Start with zero for all PID and FeedForward gains:
 *      - P = 0, I = 0, D = 0
 *      - FeedForward velocity = 0, FeedForward acceleration = 0
 *      - Static FeedForward (`ks`) = 0
 * 
 * 2. **Tune Static FeedForward (`ks`) for Low-Speed Movement:**
 *    - `ks` compensates for static friction and ensures the robot can overcome it and start moving.
 *    - Set `ks` to a small value (e.g., `ks = 0.01`), then gradually increase it if the robot hesitates or stalls at low speeds.
 *    - The goal is to get the robot to move smoothly at low speeds without stuttering or excessive torque.
 *    - Adjust `ks` until the robot starts moving from rest with smooth and controlled acceleration.
 * 
 * 3. **Tune FeedForward (Velocity and Acceleration):**
 *    - After static friction is overcome, tune the FeedForward velocity and acceleration gains.
 *    - Start by adjusting the FeedForward velocity term (e.g., from 0.1 to 0.2), testing the robot’s response to varying speeds.
 *    - Adjust FeedForward acceleration for smoother starts from a standstill.
 *    - The robot should accelerate and decelerate smoothly without oscillations or large delays.
 * 
 * 4. **Tune Proportional Gain (P):**
 *    - Set I = 0 and D = 0, and focus on tuning **P** first to control the robot’s responsiveness.
 *    - Start with a small P-gain (e.g., P = 0.1) and gradually increase it to eliminate sluggishness and improve speed accuracy.
 *    - Ensure there’s no overshoot or oscillation; if overshoot happens, reduce P.
 * 
 * 5. **Tune Integral Gain (I):**
 *    - After adjusting **P**, set D = 0 and tune **I** to minimize steady-state error (small persistent errors).
 *    - Start with a small value for **I** (e.g., I = 0.01) and increase it until any small drift is corrected.
 *    - Watch for wind-up (large I value), which could cause overshoot or instability.
 * 
 * 6. **Tune Derivative Gain (D):**
 *    - Set **I = 0** and focus on **D** to smooth out any overshoot or oscillations caused by **P**.
 *    - Gradually increase **D** to stabilize the motor response.
 *    - If the robot becomes sluggish or noisy, reduce **D** until smooth control is achieved.
 * 
 * 7. **Test on Ground and Adjust for Motor Interaction:**
 *    - After tuning off-ground, test the robot on the ground to account for real-world friction and weight distribution.
 *    - Fine-tune **FeedForward velocity** and **acceleration** as necessary for optimal on-ground performance.
 *    - If motors interfere with each other (e.g., one motor affects the other’s response), adjust the PID values to ensure balanced performance.
 * 
 * **To Make Tuning Values Work on the Ground:**
 *    - The **ground effect** changes motor behavior due to increased friction, weight distribution, and possible coupling between the wheels.
 *    - **Start with the off-ground tuned values** (such as `ks`, FeedForward, and PID gains) and test them on the ground.
 *    - **If the robot is too slow to start** or struggles to move, increase **`ks`** slightly to compensate for the additional friction of the ground.
 *    - **FeedForward adjustments** might also be necessary:
 *      - If the robot accelerates too slowly or drags, increase **FeedForward velocity** and **FeedForward acceleration**.
 *      - If the robot accelerates too quickly or jerks, slightly decrease the FeedForward values.
 *    - **PID gains** may also need slight adjustments:
 *      - If the robot oscillates or overshoots, reduce **P** or increase **D**.
 *      - If the robot drifts or struggles to hold a consistent speed, increase **I** to reduce steady-state error.
 *    - Keep an eye on **motor interaction**; if one motor is affecting the other (e.g., one motor is not reacting as expected due to ground friction or loading), adjust the PID parameters for each motor individually, if your system allows for that.
 *    - For **two-wheel drive robots** or other setups, sometimes adjusting the left and right motor PID terms separately is necessary to account for slight asymmetries in wheel friction or motor response.
 * 
 * 8. **Verify High-Speed Performance:**
 *    - After fine-tuning at low speeds, test the robot at higher speeds (e.g., 50%-100% of max speed).
 *    - Ensure the robot accelerates, decelerates, and holds a straight path without oscillations or excessive overshoot.
 * 
 * 9. **Repeat as Needed:**
 *    - Revisit any of the previous steps if issues arise in different conditions (e.g., varying surfaces or loads).
 * 
 * By following this approach, you start by tuning **static FeedForward (`ks`)** first, ensuring smooth low-speed movement before focusing on dynamic control with **PID** and **FeedForward** terms. The final tuning step of testing on the ground ensures that real-world factors such as friction, weight distribution, and motor interaction are accounted for.
 */




    /* Calibration Factor to Help offest for weight start with this at one*/
    public static double calibrationFactorSB = 1.0;//1.11


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
        new Translation3d(0.0, 0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));
    
        //Main Note Camera Assumed to be center 
     public static final Transform3d Note_Camera_Main_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0),15.0, Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant1_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant2_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
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

  public static final class ArmConstants {
    //Arm Stander Values
    public static final int Home = -1;

    // Coral Placing Values
    public static final int L1 = 1;
    public static final int L2 = 2;
    public static final int L3 = 3;
    public static final int L4 = 4;

    // Coral Intake Values
    public static final int Coral_Intake_Bot = 5;
    public static final int Coral_Intake_Feederstation = 6;

    //Algae Placing Values
    public static final int Barge = 7;
    public static final int Processor = 8;

    //Algae Intake Values
    public static final int Algae_1 = 9;
    public static final int Algae_2 = 10;
    public static final int Algae_Intake_Floor = 11;
    public static final int Algae_Intake_Robot = 12;



     //Arm Value Position
     public static final double Home_Pivot = 0.0;
     public static final double Home_Extention = 0.0;
     public static final double Home_Wrist = 0.0;



     public static final double L1_Pivot = 0.0;
     public static final double L1_Extention = 0.0;
     public static final double L1_Wrist = 0.0;

     public static final double L2_Pivot = 0.0;
     public static final double L2_Extention = 0.0;
     public static final double L2_Wrist = 0.0;

     public static final double L3_Pivot = 0.0;
     public static final double L3_Extention = 0.0;
     public static final double L3_Wrist = 0.0;

     public static final double L4_Pivot = 0.0;
     public static final double L4_Extention = 0.0;
     public static final double L4_Wrist = 0.0;



     public static final double Coral_Intake_Bot_Pivot = 0.0;
     public static final double Coral_Intake_Bot_Extention = 0.0;
     public static final double Coral_Intake_Bot_Wrist = 0.0;

     public static final double Coral_Intake_Feederstation_Pivot = 0.0;
     public static final double Coral_Intake_Feederstation_Extention = 0.0;
     public static final double Coral_Intake_Feederstation_Wrist = 0.0;



     public static final double Barge_Pivot = 0.0;
     public static final double Barge_Extention = 0.0;
     public static final double Barge_Wrist = 0.0;

     public static final double Processor_Pivot = 0.0;
     public static final double Processor_Extention = 0.0;
     public static final double Processor_Wrist = 0.0;



    public static final double Algae_1_Pivot = 0.0;
    public static final double Algae_1_Extention = 0.0;
    public static final double Algae_1_Wrist = 0.0;

    public static final double Algae_2_Pivot = 0.0;
    public static final double Algae_2_Extention = 0.0;
    public static final double Algae_2_Wrist = 0.0;

    public static final double Algae_Intake_Floor_Pivot = 0.0;
    public static final double Algae_Intake_Floor_Extention = 0.0;
    public static final double Algae_Intake_Floor_Wrist = 0.0;

    public static final double Algae_Intake_Robot_Pivot = 0.0;
    public static final double Algae_Intake_Robot_Extention = 0.0;
    public static final double Algae_Intake_Robot_Wrist = 0.0;




        //Physical Constants
        public static final double Pivot_GearRatio = 0.0;

        public static final double Extention_GearRatio = 0.0;

        public static final double Wrist_GearRatio = 0.0;

        public static final double Minipulator_GearRatio = 0.0;
        public static final double Minipulator_WheelRadius = 0.0;


  }

  public static final class IntakeConstants {
    //Like the example above set the game piece and and positon of intatke
    //I would recommend 0 for no prefernce 1 for coral and 2 for algae
    public static final int All_Floor = 1;    

    public static final int Coral_Floor = 1;
    public static final int Coral_Robot = 2;

    public static final int Algae_Floor = 3;
    public static final int Algae_Robot = 4;

    public static final int Home_Intake = 5;


    //Physical Constants gear ratios and Rdius should be used to make tip speeds match meanin the cames piece
    //shouldnot spin but some derectly into the robot
    public static final double MovementMotor_GearRatio = 0.0;

    public static final double CoralMotor_GearRatio = 0.0;
    public static final double CoralMotor_Radius = 0.0;

    public static final double MiddletMotor_GearRatio = 0.0;
    public static final double MiddletMotor_Radius = 0.0;

    public static final double AlgaeMotor_GearRatio = 0.0;
    public static final double AlgaeMotor_Radius = 0.0;




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
