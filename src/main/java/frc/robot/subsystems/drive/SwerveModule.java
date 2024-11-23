package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class SwerveModule extends SubsystemBase {

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position,
     * integrated CANEncoder for relative position
     * for both rotation and linear movement
     */

    private PIDController _rotationController;
    private PIDController _driveController;


    private final ModuleIO moduleIO;
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();
    private final int index;


    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;


   public SwerveModule(ModuleIO io, int index) {
    this.moduleIO = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        turnFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        turnFeedback = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

 
  public void setBrakeMode(boolean enabled) {
    moduleIO.setDriveBrakeMode(enabled);
    moduleIO.setTurnBrakeMode(enabled);
  }




    // public SwerveModule(
    //         ModuleIO io,
    //         int driveMotorId,
    //         double drivemotorkP,
    //         int rotationMotorId,
    //         double rotationmotorkP,
    //         int canCoderId,
    //         double measuredOffsetRadians) {

    //     _io = io;

    //     _rotationController = new PIDController(rotationmotorkP, 0, 0.0);
    //     _rotationController.enableContinuousInput(-Math.PI, Math.PI);

    //     _driveController = new PIDController(drivemotorkP, 0.0, 0.0);
    // }

    public void resetDistance() {
        moduleIO.resetDriveEncoder();
    }

    public Double getDriveDistanceRadians() {
        return moduleInputs._drivePositionRad;
    }

    public Rotation2d getCanCoderAngle() {
        double unsignedAngle = (moduleInputs._turnPosition.getRadians()) % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);
    }

    public Rotation2d getIntegratedAngle() {
        return new Rotation2d((moduleInputs._turnPosition.getRadians()) % (2 * Math.PI));
    }

    public double getCurrentVelocityRadiansPerSecond() {
        return moduleInputs._driveVelocityRadPerSec;
    }

    public double getCurrentVelocityMetersPerSecond() {
        return moduleInputs._driveVelocityRadPerSec * SwerveConstants.wheelDiameter; // v = w*r
    }

    public double getCurrentDistanceMeters() {
        return moduleInputs._drivePositionRad * SwerveConstants.wheelDiameter;
    }

    // unwraps a target angle to be [0,2π]
    public static double placeInAppropriate0To360Scope(double unwrappedAngle) {

        double modAngle = unwrappedAngle % (2.0 * Math.PI);

        if (modAngle < 0.0)
            modAngle += 2.0 * Math.PI;

        double wrappedAngle = modAngle;

        return wrappedAngle;
    }

    public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
        if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(unoptimizedDesiredState, getState().angle);
        double angularSetPoint = optimizedDesiredState.angle.getRadians();

        moduleIO.setTurnVoltage(_rotationController.calculate(getIntegratedAngle().getRadians(), angularSetPoint)
                / SwerveConstants.maxAngularVelocity);

        moduleIO.setDriveVoltage(-_driveController.calculate(optimizedDesiredState.speedMetersPerSecond)
                / SwerveConstants.maxSpeed * SwerveConstants.calibrationFactorSB);

        // TODO: I wouldn't suggest setting configs for the drive controller in something running for a 20ms cycle. Configs can really slow down cycle time. You should do this once in your constructor (see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html)
        // TODO: Also these currents may be too low. I'd suggest tuning them according to https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html 
        // if (RobotState.isAutonomous()) {
        //     _driveMotorLimiter.StatorCurrentLimit = 45;

        // } else if (SwerveBase.needMoreAmps == true) {
        //     _driveMotorLimiter.StatorCurrentLimit = 45;
        // } else if (SwerveBase.needMoreAmps == false) {
        //     _driveMotorLimiter.StatorCurrentLimit = 35;
        // }

        // _driveMotorLimiter.StatorCurrentLimitEnable = true;
        // _io.setDriveConfig(_driveMotorLimiter);

        if (SwerveBase.FasterSwerve == true) {
            SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = 5.5;// Faster swerve speed
        }
        if (SwerveBase.SlowerSwerve == true) {
            SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = 0.125;// Slower swerve speed
        }
        if (SwerveBase.FasterSwerve == false & SwerveBase.SlowerSwerve == false) {
            SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond = .5;// Normal swerve speed
        }
    }

    public void stop() {
        moduleIO.setDriveVoltage(0);
        moduleIO.setTurnVoltage(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentVelocityRadiansPerSecond(), getIntegratedAngle());
    }

}
