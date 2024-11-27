// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hybrid.LineBreakFullShooter;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.LineBreakConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ComputerVision.NoteDetection;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Drive.SwerveBase;
import frc.robot.Constants.LEDConstants;

/** An example command that uses an example subsystem. */
// public class FeedToShoot2 extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ShooterSubsystem shoot;
//   private final ArmPivotSubsystem aim;
//   private final ShooterFeederSubsystem feed;
//   private final LineBreak breakline;
//   private final SwerveBase drive;



//  // public class FeedToShoot extends repeatingSequenceCommandGroup{
// public FeedToShoot2(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak, SwerveBase m_swerveBase) {
//    shoot = m_shooter;
//    aim = m_ArmPivotSubsystem;
//    feed = m_shooterFeeder;
//    breakline = m_lineBreak;
//    drive = m_swerveBase;
// addRequirements(shoot,aim,feed,breakline,drive);
//     }


  // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
 
   // new ParallelDeadlineGroup(
            //new HasNote1(breakline),
            // new ArmPivotFarHumanFeeder(aim);
            // new ShootingFarHumanFeeder(shoot);
            // new m_shooter.ShootingFarHumanFeeder();
       // );
        //System.out.println("hi");
        
      
//         // 2. Then continues intaking while moving to shot position
//         new ParallelCommandGroup(
//             new ShootingWithoutCameras(aim),
//             new ShootingFarHumanFeeder(shoot)
//         );//.withTimeout(0.5);
// // 
        // // 3. Contine Moving to fire postion and revs shooter
        // new ParallelCommandGroup(
        //     new ShootingWithoutCameras(aim),
        //     new ShooterCrossField(shoot)
        // ).withTimeout(0.5);
       
        // // 4. Shots Note
        // new ParallelCommandGroup(
        //     new ShootingWithoutCameras(aim),
        //     new ShooterCrossField(shoot),
        //     new ShooterFeederFire(feed)
        // ).withTimeout(0.25);
 // }

  // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
   
//   }

 
  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//      return false;
//   }
// }
