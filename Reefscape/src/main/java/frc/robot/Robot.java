// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.Socket;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import frc.robot.subsystems.PowerDistributionPanel;
import frc.robot.subsystems.ComputerVision.AprilTagSubsystem;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SwerveBase;
import frc.robot.subsystems.Drive.SwerveModule;






/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

public static RobotContainer m_robotContainer;


  // Timer object to track elapsed time
 private Timer Timer = new Timer();
 public static double Time;
 public static double autoTime;
 public static double autoTimeFinal;
 public static double teleopTime;
 public static double teleopTimeFinal;
 public static double startTime;
 public static double currentTime;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer(this);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
     m_robotContainer.SmartDashboardtoCommands();
     //Clean up oculas messages
     //m_robotContainer.m_quest_nav.cleanUpOculusMessages();
     
   

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  
  }

  @Override
  public void disabledPeriodic() {


  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
   // m_robotContainer.m_ArmPivotSubsystem.setGoalPose();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    startTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    currentTime = Timer.getFPGATimestamp();
    autoTime = currentTime - startTime;

      m_robotContainer.m_aprilTag.updatedPoseFromTagAuto();
  

     autoTimeFinal = Math.abs(15-autoTime);
     NoteDetection.Time = autoTimeFinal;
 
     SmartDashboard.putNumber("Time", Math.round(autoTimeFinal));
    }


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  startTime = Timer.getFPGATimestamp();

   // m_robotContainer.m_ArmPivotSubsystem.setGoalPose();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   m_robotContainer.m_aprilTag.updatedPoseFromTagTeleOp();
   //m_robotContainer.m_aprilTag.MegaTag2();

    currentTime = Timer.getFPGATimestamp();
    teleopTime = currentTime - startTime;
   
  teleopTimeFinal = 135-teleopTime;
  NoteDetection.Time = teleopTimeFinal;
  SmartDashboard.putNumber("Time",  teleopTimeFinal);
  //System.out.println(SwerveBase.current9);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  // to bring back arm pivot
  // public ArmPivotSubsystem getArmPivotSubsystem() {
  //   return m_armPivot;
  // }


}
