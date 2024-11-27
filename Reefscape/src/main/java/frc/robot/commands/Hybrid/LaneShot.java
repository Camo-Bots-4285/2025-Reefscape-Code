package frc.robot.commands.Hybrid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPivotConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote1;
import frc.robot.commands.Swerve.MoveToNote;
import frc.robot.commands.Swerve.MoveToPose;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
// public class LaneShot extends SequentialCommandGroup{
// // public static FeedToShoot3 repeatingSequence(){
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ShooterSubsystem shoot;
//   private final ArmPivotSubsystem aim;
//   private final ShooterFeederSubsystem feed;
//   private final LineBreak breakline;
//   private final SelfDriving selfdrive; 
//   private final SwerveBase drive;
//   private final IntakeSubsystem intake;
//  //FeedToShoot3 repeats = FeedToShoot3.repeatedly();
//  //public FeedToShoot3 repeatedly(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak){
//   public LaneShot(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,
//   LineBreak m_lineBreak, SelfDriving m_selfDriving, SwerveBase m_swerveBase,IntakeSubsystem m_intake) {
//     shoot = m_shooter;
//     aim = m_ArmPivotSubsystem;
//     feed = m_shooterFeeder;
//     breakline = m_lineBreak;
//     selfdrive = m_selfDriving;
//     drive = m_swerveBase;
//     intake = m_intake;
//  addRequirements(shoot,aim,feed,breakline,selfdrive,drive,intake);
   
//     addCommands(

//        new ParallelDeadlineGroup(
//         //variable that ends when sees note
//                //new RunCommand(() -> selfdrive.LaneLogicOut()),
//                 new RunCommand(() -> selfdrive.setTargetPose(10)),
//                 new MoveToPose(drive), 
//                 new RunCommand(() -> selfdrive.DriveCalculationPose()), 
//                 new ArmPivotErrected(m_ArmPivotSubsystem)   
//             ),
  
//         //The following Group auto picks up note
//             new ParallelDeadlineGroup(
//                 new HasNote1(m_lineBreak),
//                 new MoveToNote(m_swerveBase),
//                 new ShooterFeederPickUp(m_shooterFeeder),
//                 new FloorFeederTest (m_intake),
//                 new ArmPivotErrected(m_ArmPivotSubsystem)
//             ),

//         // The following sequence is to drive back and shot the note
//             new ParallelDeadlineGroup(
//             //Add command (like has note) that says it may now move the shooter could be aline on the feild 
//                 new ShooterFeederPickUp(m_shooterFeeder),
//                 // new RunCommand(() -> selfdrive.LaneLogicIn()),
//                 new RunCommand(() -> selfdrive.setTargetPose(10)),
//                  new RunCommand(() -> selfdrive.DriveCalculationPose()), 
//                 new MoveToPose(drive)
//             ),

//              new ParallelDeadlineGroup(
//                 new SequentialCommandGroup(
//                   //Add ready to shoot and close enought so when both are true it will move on and shot  
//                 ),
//                 new ShootingWithoutCameras(aim),
//                 new ShooterTest (m_shooter),
//                 // new RunCommand(() -> selfdrive.LaneLogicIn()),
//                 new RunCommand(() -> selfdrive.setTargetPose(10)),
//                 new RunCommand(() -> selfdrive.DriveCalculationPose()),
//                 new MoveToPose(drive)
//             ),

//               new ParallelCommandGroup(
//                 new ShootingWithoutCameras(aim),
//                 new ShooterTest (m_shooter),
//                 new ShooterFeederFire(m_shooterFeeder)
//             ).withTimeout(0.25)




           
      
//     );
   
//             }

     
//   }
  