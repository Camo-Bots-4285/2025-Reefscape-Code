package frc.robot.commands.Auto;

import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BiFunction;


/** An example command that uses an example subsystem. */
public class AutoInTele extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SelfDriving m_self_Driving;
 
  private final SwerveBase drive;
  
  private Command m_autonomousdrivepath;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoInTele(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    //This number should be a vaible that is linked to the path number
    m_autonomousdrivepath = SelfDriving.getAutoDrivingPath(1);

    // schedule the autonomous command (example)
    if (m_autonomousdrivepath != null) {
      m_autonomousdrivepath.schedule();
    }

SwerveBase.AllowMainDriving = false;
SelfDriving.DriveCalculationPose();
// SelfDriving.LaneLogicOut(); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (m_autonomousdrivepath != null) {
        m_autonomousdrivepath.cancel();
      }

    SwerveBase.AllowMainDriving = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

        return false;  

    }   
}
