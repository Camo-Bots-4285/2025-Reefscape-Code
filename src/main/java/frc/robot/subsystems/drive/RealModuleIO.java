package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class RealModuleIO implements ModuleIO {
    private TalonFX _driveMotor;
    private TalonFX _rotateMotor;
    private CANcoder _cancoder;
    private Rotation2d _encoderOffset;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final boolean isTurnMotorInverted = false;

    // public RealModuleIO(int driveMotorId, int rotateMotorId, Rotation2d encoderOffset) {
    //     _driveMotor = new TalonFX(driveMotorId, "rio");
    //     _rotateMotor = new TalonFX(rotateMotorId, "rio");
    //     _encoderOffset = encoderOffset;
    // }

    public RealModuleIO(int index) {
    switch (index) {
      case 0:
        _driveMotor = new TalonFX(0,"rio");
        _rotateMotor = new TalonFX(1,"rio");
        _cancoder = new CANcoder(2,"rio");
        _encoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        _driveMotor = new TalonFX(3,"rio");
        _rotateMotor = new TalonFX(4,"rio");
        _cancoder = new CANcoder(5,"rio");
        _encoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        _driveMotor = new TalonFX(6,"rio");
        _rotateMotor = new TalonFX(7,"rio");
        _cancoder = new CANcoder(8,"rio");
        _encoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        _driveMotor = new TalonFX(9);
        _rotateMotor = new TalonFX(10);
        _cancoder = new CANcoder(11);
        _encoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var _driveConfig = new TalonFXConfiguration();
    _driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    _driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    _driveMotor.getConfigurator().apply(_driveConfig);
    setDriveBrakeMode(true);
   

    var _turnConfig = new TalonFXConfiguration();
    _turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    _turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    _rotateMotor.getConfigurator().apply(_turnConfig);
     setTurnBrakeMode(true);
   // setTurnBrakeMode(true);

    _cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = _driveMotor.getPosition();
    driveVelocity = _driveMotor.getVelocity();
    driveAppliedVolts = _driveMotor.getMotorVoltage();
    driveCurrent = _driveMotor.getSupplyCurrent();
    

    turnAbsolutePosition = _cancoder.getAbsolutePosition();
    turnPosition = _rotateMotor.getPosition();
    turnVelocity = _rotateMotor.getVelocity();
    turnAppliedVolts = _rotateMotor.getMotorVoltage();
    turnCurrent = _rotateMotor.getSupplyCurrent();
   

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    _driveMotor.optimizeBusUtilization();
    _rotateMotor.optimizeBusUtilization();
  }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);



    inputs._drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / Constants.SwerveConstants.driveGearRatio;
    inputs._driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / Constants.SwerveConstants.driveGearRatio;
    inputs._driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs._driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs._turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(_encoderOffset);
    inputs._turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / Constants.SwerveConstants.turnGearRatio);
    inputs._turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / Constants.SwerveConstants.turnGearRatio;
    inputs._turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs._turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    }


      @Override
  public void setDriveVoltage(double volts) {
    _driveMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
   _rotateMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    _driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    _rotateMotor.getConfigurator().apply(config);
  }

    @Override
  public void stop() {
    _driveMotor.stopMotor();
  }
    // /** Run the drive motor at the specified voltage. */
    // public void setDriveVoltage(double volts) {
    //     _driveMotor.setVoltage(volts);
    // }

    // /** Run the turn motor at the specified voltage. */
    // public void setTurnVoltage(double volts) {
    //     _rotateMotor.setVoltage(volts);
    // }

    // /** Enable or disable brake mode on the drive motor. */
    // public void setDriveConfig(CurrentLimitsConfigs config) {
    //     _driveMotor.getConfigurator().apply(config);
    // }

    // public void setRotateConfig(CurrentLimitsConfigs config) {
    //     _rotateMotor.getConfigurator().apply(config);
    // }

    // public void setDriveInverted(boolean inverted) {
    //     _driveMotor.setInverted(inverted);
    // }

    // public void setRotationInverted(boolean inverted) {
    //     _rotateMotor.setInverted(inverted);
    // }

    public void resetDriveEncoder() {
        _driveMotor.setPosition(0);
    }
}
