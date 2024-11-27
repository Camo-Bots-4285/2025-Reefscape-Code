
// //Copyed from team 4829
// //
// //


package frc.robot.commands.Auto;


import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote1;
import frc.robot.commands.Swerve.AlignPoseSpeaker;
import frc.robot.commands.Swerve.MoveToNote;
import frc.robot.subsystems.*;
/**
 * This class is for the 5 Ball Auto Command
 */
// public class AutoTest extends SequentialCommandGroup //{
   // public class FeedToShoot extends repeatingSequenceCommandGroup{
//   public AutoTest(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem,
//       IntakeSubsystem m_intake, ShooterFeederSubsystem m_shooterFeeder,SwerveBase m_swerveBase,LineBreak m_lineBreak) {

//      addCommands(
        
//         // 1. intake till linebreck is broken
//         new ParallelDeadlineGroup(
//             new HasNote1(m_lineBreak),
//             new ArmPivotErrected(m_ArmPivotSubsystem),
//             new FloorFeederTest(m_intake),
//             new ShooterFeederPickUp(m_shooterFeeder),
//             new MoveToNote(m_swerveBase)
//         ),
        
//         // 2. Then continues intaking while moving to shot position
//         new ParallelCommandGroup(
//             new ShooterFeederPickUp(m_shooterFeeder)
//         ).withTimeout(1.0),

//         // 3. Contine Moving to fire postion and revs shooter
//         new ParallelCommandGroup(
//             new AlignPoseSpeaker(m_swerveBase),
//             new ArmPivotShooting(m_ArmPivotSubsystem),
//             new ShooterTest (m_shooter)
//         ).withTimeout(2.0),
       
//         // 4. Shots Note
//         new ParallelCommandGroup(
//             new ArmPivotShooting(m_ArmPivotSubsystem),
//             new ShooterTest (m_shooter),
//             new ShooterFeederFire(m_shooterFeeder)
//         ).withTimeout(0.5)
//     );
//   }
// }
