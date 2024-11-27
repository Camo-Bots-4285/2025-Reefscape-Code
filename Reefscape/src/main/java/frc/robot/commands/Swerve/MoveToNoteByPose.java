// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
;

/** An example command that uses an example subsystem. */
public class MoveToNoteByPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private NoteDetection m_note_Detection;
 
  private final SwerveBase drive;
  public static double NoteSpeedY;
  public static double NoteSpeedX;
  public static boolean CloseNoteInMainCamera;

  public static double NoteSpeedAssistantRotation1;
  public static double NoteSpeedAssistantRotation2;

  public static boolean CloseNoteLeft;
  public static boolean CloseNoteRight;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToNoteByPose(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
public static final Pose2d NOTE_POSE = new Pose2d(
      new Translation2d(SelfDriving.TargetPoseX,SelfDriving.TargetPoseY),
      new Rotation2d(Math.PI)
    );

  @Override
  public void execute() {
  


//Close Notes is in main Camera FOV
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

// if(NoteDetection.JustSawNote == true){
// drive.drive(0.1,0.0, 0.0, false);
// }

if(CloseNoteInMainCamera == true){
SelfDriving.TargetPoseX = VisionConstants.Note_Camera_Main_To_Robot.getX();
SelfDriving.TargetPoseY = VisionConstants.Note_Camera_Main_To_Robot.getY();
}
if(CloseNoteLeft == true){
SelfDriving.TargetPoseX = VisionConstants.Note_Camera_Assistant1_To_Robot.getX();
SelfDriving.TargetPoseY = VisionConstants.Note_Camera_Assistant1_To_Robot.getY();
}
if(CloseNoteRight == true){
SelfDriving.TargetPoseX = VisionConstants.Note_Camera_Assistant2_To_Robot.getX();
SelfDriving.TargetPoseY = VisionConstants.Note_Camera_Assistant2_To_Robot.getY();
}





}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //    if (LineBreak.HasNote1 == true){
        return false;
  //   }
  //   else{
  //       return true;
  //   }
  // }
}
}
