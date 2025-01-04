/**OVERVIEW
 * 
 * This code brings in value from the Oculus using QuestNav source code
 * The convertes the odometry of the HeadSet to Robot odometry using where the HeadSet is locatied on the Robot
 * NOTE: There is a much simpler way of coding this but if you where to not mounted in 90 degree intervals from front the code
 * would not work
 * 
 * This code is broken up into the following sections.(Section 1 is copyeid code everything else is)
 * 
 * Section 1 (QuestNav): the original code that was taken from the project
 * however this pose was based on the headset not the pose sytem we have set up for the bot
 * 
 * Section 2 (QuestNav to RobotPose): This section of the code takes the location of the head
 * set and converts it into as pose
 * 
 * Section 3 (Resetting Pose): This code was made assuming that we can only get data from the oculus not send it.
 * So we can not rezero without haveing variables in the code to minuplate.
 * So section the breaks odometry up into translation and rotation and will let you set either or bot to a certian value
 * 
 * Section 4 (Loging): We are hoping to use Doglog for the upcoming 2025 season but if not this section should put the values in
 * smartdashboard and view them in advantage scope
 * 
 */

 /** TODO- 
 *  1) Test code for operation
 *  2) Move Constants to Constants
 *  3) Make more indepth comments
 */

 /** TESTING - needed to make operational
  * PreTest) Make sure that there is a pose being printed and that it moves aroud
 *  1) Test if zero oculus zeros the oculus
 *              I am assuming it does not so I wrote some code to do that with pose and headin sepratly.
 *  2) Test genral funciality of code does the pose to pose system work
 *              You can does this my first make sure the x and y axis that is defined in the code is correct
 *  3) Test setting postion allow the QuestNav to be rest and see it it does it correctly
 */



package frc.robot.subsystems.ComputerVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import dev.doglog.DogLog;


public class QuestNav extends SubsystemBase {


    /**Section 1 (Quest Nav)
    *The following vaible will be usesd to calulate the robor pose based of QuestNav
    */

   // Configure Network Tables topics (oculus/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables oculus data topics
  private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private boolean kGyroReversed = false;
  private double angleSetpoint = 0.0;

  // Zero the realative robot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
    angleSetpoint = 0.0;
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroOculus() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up oculus subroutine messages after processing on the headset
  public void cleanUpOculusMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Return the robot heading in degrees, between -180 and 180 degrees
  public double getHeading() {
    return Rotation2d.fromDegrees(getOculusYaw()).getDegrees();
  }

  // Get the rotation rate of the robot
  public double getTurnRate() {
    return getOculusYaw() * (kGyroReversed ? -1.0 : 1.0);
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

    private Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  private Pose2d getOculusPoseRaw() {
    var oculousPositionCompensated = getOculusPosition().minus(new Translation2d(QuestNavToCenterX, QuestNavToCenterY)); // 6.5
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }







 /** Section 2 (QuestNav to RobotPose)
   * The following method have been add because the QuestNav to robot does not work in the above code.
   * Getting the bots pose from the Oculus is way easier then getting the oculus pose from the robot
   * There are two methods that you can use for getting robot pose
   * I choose a more complex method because if I did not the code would have to be rewriten if it was
   * not mounted in 90 degree intervals from front.
   * I broke the code up into very small intervales for method insted of doing it all in one 
   * my hope is that this makes logging, proplem solving, and readbilty better.
   */

    // From the Quests Point of view how would it get to the center of the robot
    private double QuestNavToCenterX = 0.0;
    private double QuestNavToCenterY = 0.0;
    private double QuestNavToCenterZ = 0.0;

    //From the Quest Forward to the front of robot and pigeon x
    private double QuestNavToPigeon = 0.0;

  private double getOculusX() {
    float[] oculusPosition = questPosition.get();
    //This return satment might not be write but it sould be either the x or y
    return oculusPosition[2];
  }

    private double getOculusY() {
    float[] oculusPosition = questPosition.get();
    //This return satment might not be write but it sould be either the x or y
    return  -oculusPosition[0];
  }

    private double getOculusOnFeildX() {
    //Both sould work

    //This line should not need getOculusAdjustment
    //return  Math.sin(-QuestNavToPigeon)*(getOculusX()-QuestNavToCenterX) + Math.cos(-QuestNavToPigeon)*(getOculusY()-QuestNavToCenterY);

    //This line will need OculusAdjusment
    return  Math.sin(-QuestNavToPigeon)*getOculusX() + Math.cos(-QuestNavToPigeon)*getOculusY();
  }

    private double getOculusOnFeildY() {
    //Both sould work

    //This line should not need getOculusAdjustment
    //return  Math.cos(-QuestNavToPigeon)*(getOculusX()-QuestNavToCenterX) + Math.sin(-QuestNavToPigeon)*(getOculusY()-QuestNavToCenterY);

    //This line will need OculusAdjusment
    return  Math.cos(-QuestNavToPigeon)*getOculusX() + Math.sin(-QuestNavToPigeon)*getOculusY();
  }

  private Translation2d getOculusAdjustment(){
    double distance = Math.sqrt(QuestNavToCenterX*QuestNavToCenterX + QuestNavToCenterY*QuestNavToCenterY);
    double Xoffset = Math.cos(getOculusYaw()-QuestNavToPigeon)*distance;
    double Yoffset = Math.sin(getOculusYaw()-QuestNavToPigeon)*distance;

    return new Translation2d(Xoffset, Yoffset);
  }

     private Pose2d getHeadsetPoseToFeildOdometry(Pose2d HeadSet) {
    var oculousPositionCompensated = (new Translation2d(HeadSet.getX(), HeadSet.getY())).minus(getOculusAdjustment());
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }

   private Pose2d getHeadsetPoseToFeildOdometry() {
    var oculousPositionCompensated = (new Translation2d(getOculusOnFeildX(), getOculusOnFeildY())).minus(getOculusAdjustment());
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }

    private Pose2d getFeildPoseToHeadSetOdometry(Pose2d Feild){
    var oculousPositionCompensated = (new Translation2d(Feild.getX(), Feild.getY()))/* .plus(getOculusAdjustment()) this value was templarly remove for testing*/; 
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(Feild.getRotation().getDegrees()));
  }

  private Pose2d getFeildPoseToHeadSetOdometry(double PoseX, double PoseY, double RotationDegrees){
    var oculousPositionCompensated = (new Translation2d(PoseX, PoseY)).plus(getOculusAdjustment()); 
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(RotationDegrees));
  }







/** Section 3 (Resetting Pose)
 * 
 * 
 * 
 * getHeadsetPoseToFeildOdometry()
 * getHeadsetPoseToFeildOdometry(getOculusPoseRaw())
 */

private Translation2d resetTranslationValue = new Translation2d(0, 0);
  private Rotation2d resetRotationValue =  Rotation2d.fromDegrees(0);

  private Translation2d OculusZeroFactorTranslation = new Translation2d(0, 0);
  private Rotation2d OculusZeroFactorRotation =  Rotation2d.fromDegrees(0);


    // Reset the translation to the specified pose
  public void setZeroTranslationFactor(Pose2d pose) {
   OculusZeroFactorTranslation = getHeadsetPoseToFeildOdometry(new Pose2d(new Translation2d(getOculusPoseRaw().getX(), getOculusPoseRaw().getY()), Rotation2d.fromDegrees(0))).getTranslation(); 
  } 

    // Reset the rotation to the specified pose
  public void setZeroRotationFactor(Pose2d pose) {
   OculusZeroFactorRotation = pose.getRotation();
  } 

   public void zeroPose() {
    setZeroTranslationFactor(getHeadsetPoseToFeildOdometry(getOculusPoseRaw()));
   }

   public void zeroHeading2(){
    setZeroRotationFactor(getHeadsetPoseToFeildOdometry(getOculusPoseRaw()));
   }
  
   public void resetTranslation(Pose2d pose){
    zeroPose();
    resetTranslationValue = pose.getTranslation();
   }

   public void resetRotation(Pose2d pose){
    zeroHeading2();
    resetRotationValue = pose.getRotation();
   }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    resetTranslation(pose);
    resetRotation(pose);
  } 

  public Pose2d getRobotOdometry(){
    Translation2d Translation =  getHeadsetPoseToFeildOdometry(getOculusPoseRaw()).getTranslation().minus(OculusZeroFactorTranslation).plus(resetTranslationValue);
    Rotation2d Rotation = getHeadsetPoseToFeildOdometry(getOculusPoseRaw()).getRotation().minus(OculusZeroFactorRotation).plus(resetRotationValue);
    return new Pose2d(Translation, Rotation);
  }







//     private Pose2d getRobotPose() {
//     Pose2d OculusZero = new Pose2d(OculusZeroFactorTranslation,  OculusZeroFactorRotation);
//     Pose2d RobotPose = getHeadsetPoseToFeildOdometry(getOculusPoseRaw())- OculusZero;
//     return RobotPose;
//         //Make all of these feildPoses then do math
//         //QuestNav_Reading + Zeroed_QuestNav + Reading = pose
//         //Zeroed_QuestNav=-QuestNavReading
//     // var oculousPositionCompensated = getOculusPosition().minus(new Translation2d(0, 0.1651)); // 6.5
//     // return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
//   }










// Update odometry in the periodic block
  @Override
  public void periodic() {
   //Runs DogLog to log values to USB
   DogLog();

    //These two should be the same thing
    SmartDashboard.putString("RobotOdometry_FormOculus1",getHeadsetPoseToFeildOdometry().toString());
    SmartDashboard.putString("RobotOdometry_FormOculus2",getHeadsetPoseToFeildOdometry(getOculusPoseRaw()).toString());

    //These two should also be equal
    SmartDashboard.putString("RobotOdometry_OculusPose1",getFeildPoseToHeadSetOdometry(getHeadsetPoseToFeildOdometry(getOculusPoseRaw())).toString());
    SmartDashboard.putString("RobotOdometry_OculusPose2",getOculusPoseRaw().toString());

        SmartDashboard.putString("RobotOdometry_Final", getRobotOdometry().toString());


    // Logger.recordOutput("SwerveOdometry", odometry_QuestNav.getPoseMeters());
    // Logger.recordOutput("OculusAll", getOculusPose().minus(resetPosition));
    // Logger.recordOutput("OculusPositionFloats", questPosition.get());
    // Logger.recordOutput("OculusQuaternionFloats", questQuaternion.get());
    // float[] qqFloats = questQuaternion.get();
    // var qq = new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
    // Logger.recordOutput("OculusQuaternion", qq);
  }

  public void DogLog(){
  //  DogLog.log("/PoseEstimation/QuestNav/", TargetPoseX);
  //   DogLog.log("/SelfDriving/Position/TargetY", TargetPoseY);
  //   DogLog.log("/SelfDriving/Position/TargetRotation", TargetPoseRotation);

  }


  
}