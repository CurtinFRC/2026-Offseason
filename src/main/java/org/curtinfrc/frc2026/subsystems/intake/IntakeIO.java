package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // what we are getting from the motors
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double angularVelocity = 0.0;
    public double rollerMotorPosition = 0.0;
  }

  // what we are sending to the motors
  public default void setRollerVoltage(double volts) {}

  public default void setRollerVelocity(double velocity) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
