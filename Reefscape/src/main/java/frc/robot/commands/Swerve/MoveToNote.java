// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
;

/** An example command that uses an example subsystem. */
public class MoveToNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private NoteDetection m_note_Detection;
 
  private final SwerveBase drive;
  public static double NoteSpeedTranslationMain;
  public static double NoteSpeedRotationMain;
 

  public static double NoteSpeedAssistantRotation1;
  public static double NoteSpeedAssistantRotation2;

 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToNote(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveBase.AllowMainDriving = false;
    SwerveBase.GettingNote = true;
    SelfDriving.DriveCalculationNote();

//Takes how far off the note is from the desired state puts a PID controler on it
//The PID controler will give a value that will tell the robot how fast to drive
// NoteSpeedTranslationMain = SelfDriving.NoteTranslation.calculate(NoteDetection.MainXDistance, 0.0);


// NoteSpeedRotationMain = SelfDriving.NoteRotation.calculate(NoteDetection.MainYDistance, 0.0);

// NoteSpeedAssistantRotation1 = SelfDriving.NoteRotation.calculate(NoteDetection.XA1, 30);
// NoteSpeedAssistantRotation2 = SelfDriving.NoteRotation.calculate(NoteDetection.XA2, -30);


// NoteSpeedTranslationMain




//   if (NoteDetection.seesTargetMain == true && NoteDetection.CloseNoteInMainCamera == true){
      drive.drive(SelfDriving.XSpeedFinal, 0.0,SelfDriving.RotationSpeedFinal, false);
      //XSpeed = -SelfDriving.NoteTranslation.calculate(NoteDetection.MainXDistance, 0.4);
      // RotationSpeed = SelfDriving.NoteRotation.calculate(NoteDetection.MainYDistance, 0.0);
//     }

//   if (NoteDetection.CloseNoteLeft == true){
//     drive.drive(0.0, 0.0, NoteSpeedAssistantRotation1, false);
//   }

//   if (NoteDetection.CloseNoteRight == true){
//     drive.drive(0.0, 0.0, -NoteSpeedAssistantRotation2, false);
//   }

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveBase.AllowMainDriving = true;
    SwerveBase.GettingNote = false;
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
