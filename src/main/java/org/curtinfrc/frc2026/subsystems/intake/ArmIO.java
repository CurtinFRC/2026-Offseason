package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double motorTemperature;
    public double appliedVoltage;
    public double currentAmps;
    public double angularVelocity;
    public double motorPosition;
  }

  public default void setArmVoltage(double volts) {}

  public default void setArmPosition(double rotations) {}

  public default void updateInputs(ArmIOInputs inputs) {}
}
