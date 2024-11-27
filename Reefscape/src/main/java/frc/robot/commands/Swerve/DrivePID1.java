// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BiFunction;


/** An example command that uses an example subsystem. */
public class DrivePID1 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SelfDriving m_self_Driving;
 
  private final SwerveBase drive;
  
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivePID1(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
drive.drive(1.0, 0.0, 0.0, false);
//drive.drive(SelfDriving.XSpeed, SelfDriving.YSpeed, 0.0, true);
SwerveBase.AllowMainDriving = false;

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       SwerveBase.AllowMainDriving = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (SelfDriving.CloseEnough == true){
        return false;  
    
    //     }
    
    // else{
    //     return true;
    //     }
    }   
}
