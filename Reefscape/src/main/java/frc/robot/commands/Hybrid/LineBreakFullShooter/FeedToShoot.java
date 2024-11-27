// //Copyed from team 4829
// //
// //


package frc.robot.commands.Hybrid.LineBreakFullShooter;


import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;


public class FeedToShoot extends Command {

   // public class FeedToShoot extends repeatingSequenceCommandGroup{
//   public FeedToShoot(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak) {
//      shoot = m_shooter;
//      aim = m_ArmPivotSubsystem;
//      feed = m_shooterFeeder;
//      breakline = m_lineBreak;
     
//   addRequirements(shoot,aim,feed,breakline);
//       }

//       @Override
//       public void execute () {
//     new SequentialCommandGroup(
        
//         // 1. intake till linebreck is broken
//         new ParallelDeadlineGroup(
//             new HasNote1(breakline),
//             new ArmPivotFarHumanFeeder(aim),
//             new ShootingFarHumanFeeder(shoot)
//         ),
        
//         // 2. Then continues intaking while moving to shot position
//         new ParallelCommandGroup(
//             new ShootingWithoutCameras(aim),
//             new ShootingFarHumanFeeder(shoot)
//         ).withTimeout(0.5),

//         // 3. Contine Moving to fire postion and revs shooter
//         new ParallelCommandGroup(
//             new ShootingWithoutCameras(aim),
//             new ShooterCrossField(shoot)
//         ).withTimeout(0.5),
       
//         // 4. Shots Note
//         new ParallelCommandGroup(
//             new ShootingWithoutCameras(aim),
//             new ShooterCrossField(shoot),
//             new ShooterFeederFire(feed)
//         ).withTimeout(0.25)
//     );
   
//       }

  
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
