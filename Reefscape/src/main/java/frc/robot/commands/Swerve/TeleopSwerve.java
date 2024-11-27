package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.ComputerVision.NoteDetection;
import frc.robot.subsystems.Drive.SelfDriving;
import frc.robot.subsystems.Drive.SwerveBase;

public class TeleopSwerve extends Command {
  /*
   * Teleoperated Swerve Drive Command
   * ---------------------------------
   * 
   * This command hooks up to the Swerve Drive subsystem
   * and passes in our joystick inputs into it.
   */

  private final SwerveBase drive;

  /*
   * Joysticks return DoubleSuppliers when the get methods are called
   * This is so that joystick getter methods can be passed in as a parameter but
   * will continuously update,
   * versus using a double which would only update when the constructor is called
   */
  private final DoubleSupplier forwardX;
  private final DoubleSupplier forwardY;
  private final DoubleSupplier rotation;
  private final DoubleSupplier speedchange;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Supplier<Boolean> isFieldOriented;
  public static double fwdX;
  public static double fwdY;
  public static double rot;
  public static double AdjustDriveSpeed; 

  public TeleopSwerve(
      SwerveBase subsystem,
      DoubleSupplier fwdX,
      DoubleSupplier fwdY,
      DoubleSupplier rot,
      DoubleSupplier AdjustDriveSpeed,
      Supplier<Boolean> isFieldOriented) {

    drive = subsystem;
    forwardX = fwdX;
    forwardY = fwdY;
    rotation = rot;
    speedchange = AdjustDriveSpeed;

    this.isFieldOriented = isFieldOriented;

    this.xLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
        // System.out.println(fwdX);
        // System.out.println(fwdY);
if (SwerveBase.AllowMainDriving == true){
    AdjustDriveSpeed = speedchange.getAsDouble();
    /*
     * Units are given in meters per second radians per second. Since joysticks give output
     * from -1 to 1, we multiply the outputs by the max speed. Otherwise, our max speed
     * would be only 1 meter per second and 1 radian per second.
     */
   
    fwdX = forwardX.getAsDouble();
    fwdY = forwardY.getAsDouble();
    rot = rotation.getAsDouble();
    
    // 2. Apply deadband/deadzone, can edit this later to have smoother behavior
    //If velocity is less then number it will be set to zero need to tune these value with driver
    fwdX = Math.abs(fwdX) > 0.1 ? fwdX : 0.0; 
    fwdY = Math.abs(fwdY) > 0.04 ? fwdY : 0.0;//0.1
    rot = Math.abs(rot) > .30 ? rot : 0.0;

    // 3. Make the driving smoother this will set max velocity in teleop 
    //There should be three setting that are programed in normal and other two are activated by buttons 
    //superfast and superslow(should be pared with high amps if push needed)
    fwdX = xLimiter.calculate(fwdX) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond*AdjustDriveSpeed;
    fwdY = yLimiter.calculate(fwdY) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond*AdjustDriveSpeed;
    rot = turningLimiter.calculate(rot) * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond*AdjustDriveSpeed;
 
    drive.drive(
        -fwdX,
        -fwdY,
        rot,
        isFieldOriented.get()
      );
      System.out.println(fwdX);
}
  }
}
