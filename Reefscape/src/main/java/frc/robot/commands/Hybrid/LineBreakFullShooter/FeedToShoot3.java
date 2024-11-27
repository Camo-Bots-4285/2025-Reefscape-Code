package frc.robot.commands.Hybrid.LineBreakFullShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPivotConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SwerveBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class FeedToShoot3 extends SequentialCommandGroup{
// public static FeedToShoot3 repeatingSequence(){
 // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}
//   private final ShooterSubsystem shoot;
//   private final ArmPivotSubsystem aim;
//   private final ShooterFeederSubsystem feed;
//   private final LineBreak breakline;
  
//  //FeedToShoot3 repeats = FeedToShoot3.repeatedly();
//  //public FeedToShoot3 repeatedly(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak){
//   public FeedToShoot3(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak) {
//     shoot = m_shooter;
//     aim = m_ArmPivotSubsystem;
//     feed = m_shooterFeeder;
//     breakline = m_lineBreak;
//  addRequirements(shoot,aim,feed,breakline);
  
//     addCommands(
//             new ParallelDeadlineGroup(
//             new HasNote1(breakline),
//             new ArmPivotHumanFeeder(aim),
//             new ShooterFeederHuman(feed)
//        ),
//       // 2. Then continues intaking while moving to shot position
//                new ParallelCommandGroup(
//                    new ShootingWithoutCameras(aim),
//                     new ShooterFeederHuman(feed)
//                    ).withTimeout(0.5),
//        // 
//                // 3. Contine Moving to fire postion and revs shooter
//                new ParallelCommandGroup(
//                    new ShootingWithoutCameras(aim),
//                    new ShooterCrossField(shoot)
//                ).withTimeout(0.5),
              
//                // 4. Shots Note
//                new ParallelCommandGroup(
//                    new ShootingWithoutCameras(aim),
//                    new ShooterCrossField(shoot),
//                    new ShooterFeederFire(feed)
//     //                new AutoTest(
//     // m_shooter,
//     // m_ArmPivotSubsystem,
//     // m_intake,
//     // m_shooterFeeder,
//     // m_swerveBase,
//     // m_lineBreak
//                ).withTimeout(0.25)
    
    //);
   
            }

     
 // }
  