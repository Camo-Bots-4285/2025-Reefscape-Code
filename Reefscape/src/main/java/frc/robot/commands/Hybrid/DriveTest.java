package frc.robot.commands.Hybrid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Swerve.MoveToNote;
import frc.robot.subsystems.Drive.SwerveBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class DriveTest extends SequentialCommandGroup{
// public static FeedToShoot3 repeatingSequence(){
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveBase drive;
 //FeedToShoot3 repeats = FeedToShoot3.repeatedly();
 //public FeedToShoot3 repeatedly(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak){
  public DriveTest(SwerveBase m_swerveBase) {
    drive = m_swerveBase;
 addRequirements(drive);
   
    addCommands(

       new ParallelDeadlineGroup(
                new MoveToNote(m_swerveBase)
                )
    );
   
            }

     
  }
  