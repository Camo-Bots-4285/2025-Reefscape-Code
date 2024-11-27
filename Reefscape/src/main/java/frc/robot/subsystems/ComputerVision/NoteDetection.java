// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ComputerVision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Main;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Swerve.MoveToNoteByPose;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;


public class NoteDetection extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


public static double XM;
public static double YM;
public static double AM;

public static double XA1;
public static double YA1;
public static double AA1;

public static double XA2;
public static double YA2;
public static double AA2;

//Was replace by advanced system
// public static double MainCameraY;
// public static double AssistantCamera1Y;
// public static double AssistantCamera2Y;

public static boolean seesTargetMain;
public static boolean seesTargetAssistant1;
public static boolean seesTargetAssistant2;

public static double MainCameraAnlgeNoteY;
public static double MainCameraAnlgeNoteX;
public static double MainCameraHeight;
public static double CameraMainRoll;
public static double CameraMainPitch;
public static double CameraMainYaw;
public static double MainCameraXOfSet;
public static double MainCameraYOfSet;

public static double Assistant1CameraAnlgeNoteY;
public static double Assistant1CameraAnlgeNoteX;
public static double Assistant1CameraHeight;
public static double CameraAssistant1Roll;
public static double CameraAssistant1Pitch;
public static double CameraAssistant1Yaw;
public static double Assistant1CameraXOfSet;
public static double Assistant1CameraYOfSet;

public static double Assistant2CameraAnlgeNoteY;
public static double Assistant2CameraAnlgeNoteX;
public static double Assistant2CameraHeight;
public static double CameraAssistant2Roll;
public static double CameraAssistant2Pitch;
public static double CameraAssistant2Yaw;
public static double Assistant2CameraXOfSet;
public static double Assistant2CameraYOfSet;

public static double MainXDistance;
public static double MainYDistance;
public static double MainNoteDistanceFromCenter;

public static double Assistant1XDistance;
public static double Assistant1YDistance;
public static double Assistant1NoteDistanceFromCenter;

public static double Assistant2XDistance;
public static double Assistant2YDistance;
public static double Assistant2NoteDistanceFromCenter;

public static double AngleM;
public static double AngleA1;
public static double AngleA2;

public static double MainNoteXRobot;
public static double MainNoteYRobot;

public static double Assistant1NoteXRobot;
public static double Assistant1NoteYRobot;

public static double Assistant2NoteXRobot;
public static double Assistant2NoteYRobot;

public static boolean NoteIsClose;
public static boolean JustSawNote;

public static double Time;
public static double LastTimeSawNote;

public static double NoteSpeedY;
public static double NoteSpeedX;
public static boolean CloseNoteInMainCamera;

public static double NoteSpeedAssistantRotation1;
public static double NoteSpeedAssistantRotation2;

public static boolean CloseNoteLeft;
public static boolean CloseNoteRight;


public NoteDetection() {
   
}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.

   // drive.drive(1.0, 0.0, 0.0, true);
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //System.out.println(MainXDistance);
    //System.out.println(MainXDistance);

    //  System.out.println(SwerveBase.TagetPoseX);
     //System.out.println(MAIN_CAMERA_NOTE_POSE);
     //System.out.println((90-40-VisionConstants.Note_Camera_Main_To_Robot.getRotation().getY()) + YM);
    //System.out.println(y);

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
 XM = tx.getDouble(0.0);
 YM = ty.getDouble(0.0);
 AM = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("Limelightx", XM);
SmartDashboard.putNumber("LimelightY", YM);
SmartDashboard.putNumber("LimelightArea", AM);

  // }
  // public void FindTheNotes() { 

// PhotonCamera note1 = new PhotonCamera("Note-Camera-1");

// // Query the latest result from PhotonVision
// var result = note1.getLatestResult();

// // Check if the latest result has any targets.
// boolean hasTargets = result.hasTargets();

// // Get a list of currently tracked targets.
// List<PhotonTrackedTarget> targets = result.getTargets();

// // Get the current best target.
// PhotonTrackedTarget target = result.getBestTarget();

// // Get information from target.
// double yaw = target.getYaw();
// double pitch = target.getPitch();
// double area = target.getArea();
// double skew = target.getSkew();
// // Transform2d pose = target.getCameraToTarget();
// // List<TargetCorner> corners = target.getCorners();

// //post to smart dashboard periodically
// SmartDashboard.putNumber("NoteYaw", yaw);
// SmartDashboard.putNumber("NotePitch", pitch);
// SmartDashboard.putNumber("Area", area);
// SmartDashboard.putNumber("NoteSkew", skew);
// // SmartDashboard.putNumber("NotePose", pose);
// // SmartDashboard.putNumber("NoteCorners", corners);

if (AM < 0.01){
  seesTargetMain = false;
  }
  else{
    seesTargetMain = true;
  }

  if (AA1 < 0.01){
  seesTargetAssistant1 = false;
  }
  else{
    seesTargetAssistant1 = true;
  }

  if (AA2 < 0.01){
   seesTargetAssistant2 = false;
  }
  else{
    seesTargetAssistant2 = true;
  }



 //Assistant Camera Ratio Setup
 //Was Replaced By Advanced Distance System

//  if(seesTargetMain == true){
//     MainCameraY = YM * 0.1;
//   }
//     else{
//       MainCameraY = 100;
//     }

//   if(seesTargetAssistant1 == true){
//   AssistantCamera1Y = YA1 * 0.1;
//   }
//     else{
//       AssistantCamera1Y = 100;
//     }

//   if(seesTargetAssistant2 == true){
//   AssistantCamera2Y = YA2 * 0.1;
//   }
//     else{
//       AssistantCamera2Y = 100;
//     }

//Main Camera Advaced Distance Estimation
MainCameraAnlgeNoteY = Units.degreesToRadians((90-40-VisionConstants.Note_Camera_Main_To_Robot.getRotation().getY()) + YM);
MainCameraAnlgeNoteX = VisionConstants.Note_Camera_Main_To_Robot.getRotation().getZ()+Units.degreesToRadians(XM);
MainCameraXOfSet = VisionConstants.Note_Camera_Main_To_Robot.getX();
MainCameraYOfSet = VisionConstants.Note_Camera_Main_To_Robot.getY();
MainCameraHeight = VisionConstants.Note_Camera_Main_To_Robot.getZ();


MainXDistance = Math.tan(MainCameraAnlgeNoteY)*MainCameraHeight + MainCameraXOfSet;
MainYDistance = Math.tan(MainCameraAnlgeNoteX)*Math.sqrt(MainXDistance*MainXDistance+MainCameraHeight*MainCameraHeight) + MainCameraYOfSet;
MainNoteDistanceFromCenter = Math.sqrt(MainXDistance*MainXDistance+MainYDistance*MainYDistance);

AngleM = Math.atan(MainXDistance/MainYDistance) + SelfDriving.PoseDifferenceRotationRaw;
MainNoteXRobot = MainNoteDistanceFromCenter*Math.sin(AngleM);
MainNoteYRobot = MainNoteDistanceFromCenter*Math.cos(AngleM);



//Assistant 1 Camera Advaced Distance Estimation
Assistant1CameraAnlgeNoteX = VisionConstants.Note_Camera_Assistant1_To_Robot.getRotation().getY()+Units.degreesToRadians(YA1)+90;
Assistant1CameraAnlgeNoteY = VisionConstants.Note_Camera_Assistant1_To_Robot.getRotation().getZ()+Units.degreesToRadians(XA1);
Assistant1CameraXOfSet = VisionConstants.Note_Camera_Assistant1_To_Robot.getX();
Assistant1CameraYOfSet = VisionConstants.Note_Camera_Assistant1_To_Robot.getX();
Assistant1CameraHeight = VisionConstants.Note_Camera_Assistant1_To_Robot.getZ();


Assistant1XDistance = Math.tan(Assistant1CameraAnlgeNoteX)*Assistant1CameraHeight + Assistant1CameraXOfSet;
Assistant1YDistance = Math.tan(Assistant1CameraAnlgeNoteY)*Assistant1XDistance + Assistant1CameraYOfSet;
Assistant1NoteDistanceFromCenter = Math.sqrt(Assistant1XDistance*Assistant1XDistance+Assistant1YDistance*Assistant1YDistance);

AngleA1 = Math.atan(Assistant1XDistance/Assistant1YDistance) + SelfDriving.PoseDifferenceRotationRaw;
Assistant1NoteXRobot = Assistant1NoteDistanceFromCenter*Math.sin(AngleA1);
Assistant1NoteYRobot = Assistant1NoteDistanceFromCenter*Math.cos(AngleA1);



//Assistant 2 Camera Advaced Distance Estimation
Assistant2CameraAnlgeNoteX = VisionConstants.Note_Camera_Assistant2_To_Robot.getRotation().getY()+Units.degreesToRadians(YA2)+90;
Assistant2CameraAnlgeNoteY = VisionConstants.Note_Camera_Assistant2_To_Robot.getRotation().getZ()+Units.degreesToRadians(XA2);
Assistant2CameraXOfSet = VisionConstants.Note_Camera_Assistant2_To_Robot.getX();
Assistant2CameraYOfSet = VisionConstants.Note_Camera_Assistant2_To_Robot.getX();
Assistant2CameraHeight = VisionConstants.Note_Camera_Assistant2_To_Robot.getZ();


Assistant2XDistance = Math.tan(Assistant2CameraAnlgeNoteX)*Assistant2CameraHeight + Assistant2CameraXOfSet;
Assistant2YDistance = Math.tan(Assistant2CameraAnlgeNoteY)*Assistant2XDistance + Assistant2CameraYOfSet;
Assistant2NoteDistanceFromCenter = Math.sqrt(Assistant2XDistance*Assistant2XDistance+Assistant2YDistance*Assistant2YDistance);

AngleA2 = Math.atan(Assistant2XDistance/Assistant2YDistance) + SelfDriving.PoseDifferenceRotationRaw;
Assistant2NoteXRobot = Assistant2NoteDistanceFromCenter*Math.sin(AngleA2);
Assistant2NoteYRobot = Assistant2NoteDistanceFromCenter*Math.cos(AngleA2);

// if (MainXDistance < 0.5 && MainYDistance <0.1) {
//   NoteIsClose = true;
// }

if(seesTargetMain){
  LastTimeSawNote = Time;
}

if(LastTimeSawNote - 2 < Time){
  JustSawNote = true;
}

//Close Notes is in main Camera FOV
if(NoteDetection.seesTargetMain == true && NoteDetection.seesTargetAssistant1 == false && NoteDetection.seesTargetAssistant2 == false){
  CloseNoteInMainCamera = true;
  CloseNoteLeft = false;
  CloseNoteRight = false;
}
if(NoteDetection.seesTargetMain == false && NoteDetection.seesTargetAssistant1 == true && NoteDetection.seesTargetAssistant2 == false){
  CloseNoteInMainCamera = false;
  CloseNoteLeft = true;
  CloseNoteRight = false;
}
if(NoteDetection.seesTargetMain == false && NoteDetection.seesTargetAssistant1 == false && NoteDetection.seesTargetAssistant2 == true){
  CloseNoteInMainCamera = false;
  CloseNoteLeft = false;
  CloseNoteRight = true;
}
 
if(NoteDetection.seesTargetMain == true && NoteDetection.seesTargetAssistant1 == true && NoteDetection.seesTargetAssistant2 == false){
  if(NoteDetection.MainNoteDistanceFromCenter < NoteDetection.Assistant1NoteDistanceFromCenter){
        CloseNoteInMainCamera = true;
        CloseNoteLeft = false;
        CloseNoteRight = false;
      }
  if(NoteDetection.MainNoteDistanceFromCenter > NoteDetection.Assistant1NoteDistanceFromCenter){
        CloseNoteInMainCamera = false;
        CloseNoteLeft = true;
        CloseNoteRight = false;
      } 
  }

if(NoteDetection.seesTargetMain == true && NoteDetection.seesTargetAssistant1 == false && NoteDetection.seesTargetAssistant2 == true){
    if(NoteDetection.MainNoteDistanceFromCenter < NoteDetection.Assistant2NoteDistanceFromCenter){
        CloseNoteInMainCamera = true;
        CloseNoteLeft = false;
        CloseNoteRight = false;
      }
   if(NoteDetection.MainNoteDistanceFromCenter > NoteDetection.Assistant2NoteDistanceFromCenter){
        CloseNoteInMainCamera = false;
        CloseNoteLeft = false;
        CloseNoteRight = true;
      } 
 }
if(NoteDetection.seesTargetMain == false && NoteDetection.seesTargetAssistant1 == true && NoteDetection.seesTargetAssistant2 == true){
  if(NoteDetection.Assistant1NoteDistanceFromCenter < NoteDetection.Assistant2NoteDistanceFromCenter){
        CloseNoteInMainCamera = false;
        CloseNoteLeft = true;
        CloseNoteRight = false;
      }
   if(NoteDetection.Assistant1NoteDistanceFromCenter > NoteDetection.Assistant2NoteDistanceFromCenter){
        CloseNoteInMainCamera = false;
        CloseNoteLeft = false;
        CloseNoteRight = true;
      } 
}
if(NoteDetection.seesTargetMain == true && NoteDetection.seesTargetAssistant1 == true && NoteDetection.seesTargetAssistant2 == true){

  if(NoteDetection.MainNoteDistanceFromCenter < NoteDetection.Assistant1NoteDistanceFromCenter && NoteDetection.MainNoteDistanceFromCenter < NoteDetection.Assistant2NoteDistanceFromCenter){
    CloseNoteInMainCamera = true;
    CloseNoteLeft = false;
    CloseNoteRight = false;
  } 
  //Close Notes is in assistent 1 FOV
  if(NoteDetection.MainNoteDistanceFromCenter > NoteDetection.Assistant1NoteDistanceFromCenter&& NoteDetection.Assistant1NoteDistanceFromCenter< NoteDetection.Assistant2NoteDistanceFromCenter){
    CloseNoteInMainCamera = false;
    CloseNoteLeft = true;
    CloseNoteRight = false;
  }
  //Close Notes is in assistent 2 FOV
  if(NoteDetection.Assistant2NoteDistanceFromCenter < NoteDetection.Assistant1NoteDistanceFromCenter&& NoteDetection.MainNoteDistanceFromCenter > NoteDetection.Assistant2NoteDistanceFromCenter){
    CloseNoteInMainCamera = false;
    CloseNoteLeft = false;
    CloseNoteRight = true;
  }
}


// if(NoteDetection.JustSawNote == true){
// drive.drive(0.1,0.0, 0.0, false);
// }

// if(CloseNoteInMainCamera == true){
// SwerveBase.TagetPoseX = VisionConstants.Note_Camera_Main_To_Robot.getX();
// SwerveBase.TagetPoseY = VisionConstants.Note_Camera_Main_To_Robot.getY();
// }
// if(CloseNoteLeft == true){
// SwerveBase.TagetPoseX = VisionConstants.Note_Camera_Assistant1_To_Robot.getX();
// SwerveBase.TagetPoseY = VisionConstants.Note_Camera_Assistant1_To_Robot.getY();
// }
// if(CloseNoteRight == true){
// SwerveBase.TagetPoseX = VisionConstants.Note_Camera_Assistant2_To_Robot.getX();
// SwerveBase.TagetPoseY = VisionConstants.Note_Camera_Assistant2_To_Robot.getY();
// }


//public static Pose2d MAIN_CAMERA_NOTE_POSE = new Pose2d(0, 0, new Rotation2d());
// Pose2d MAIN_CAMERA_NOTE_POSE = new Pose2d(
//   new Translation2d(NoteDetection.MainNoteXRobot + SwerveBase.currentPoseX, NoteDetection.MainNoteYRobot + SwerveBase.currentPoseY),
//   new Rotation2d(Math.PI)
// );

// public static final Pose2d Assistant_CAMERA_1_NOTE_POSE = new Pose2d(
//   new Translation2d(NoteDetection.Assistant1NoteXRobot + SwerveBase.currentPoseX, NoteDetection.Assistant1NoteYRobot + SwerveBase.currentPoseY),
//   new Rotation2d(Math.PI)
// );

// public static final Pose2d Assistant_CAMERA_2_NOTE_POSE = new Pose2d(
//   new Translation2d(NoteDetection.Assistant2NoteXRobot + SwerveBase.currentPoseX, NoteDetection.Assistant2NoteYRobot + SwerveBase.currentPoseY),
//   new Rotation2d(Math.PI)
// );
}





//   public void LimelightinAuto() {
   
//     System.out.println(x);

// NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
// NetworkTableEntry tx = table.getEntry("tx");
// NetworkTableEntry ty = table.getEntry("ty");
// NetworkTableEntry ta = table.getEntry("ta");

// //read values periodically
//  x = tx.getDouble(0.0);
// double y = ty.getDouble(0.0);
// double area = ta.getDouble(0.0);

// //post to smart dashboard periodically
// SmartDashboard.putNumber("Limelightx", x);
// SmartDashboard.putNumber("LimelightY", y);
// SmartDashboard.putNumber("LimelightArea", area);
 // }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
     

  }
  
}
