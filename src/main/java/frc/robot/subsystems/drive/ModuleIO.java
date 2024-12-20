// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import frc.robot.Constants.SwerveConstants;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double _drivePositionRad = 0.0;
    public double _driveVelocityRadPerSec = 0.0;
    public double _driveAppliedVolts = 0.0;
    public double[] _driveCurrentAmps = new double[] {};

    public Rotation2d _turnAbsolutePosition = new Rotation2d();
    public Rotation2d _turnPosition = new Rotation2d();
    public double _turnVelocityRadPerSec = 0.0;
    public double _turnAppliedVolts = 0.0;
    public double[] _turnCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  public default void stop() {}

  public default void resetDriveEncoder() {}
}
