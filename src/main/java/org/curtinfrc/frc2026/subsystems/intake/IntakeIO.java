package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorTemperature;
    public double appliedVoltage;
    public double currentAmps;
    public double angularVelocity;
    public double rollerMotorPosition;
  }

  public default void setRollerVoltage(double volts) {}

  public default void setRollerVelocity(double velocity) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
