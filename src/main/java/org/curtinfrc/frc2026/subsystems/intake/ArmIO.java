package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    // what we are getting from the motors

    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double angularVelocity = 0.0;
    public double motorPosition = 0.0;
  }

  // what we are sending to the motors

  public default void setArmVoltage(double volts) {}

  public default void setArmPosition(double rotations) {}

  public default void updateInputs(ArmIOInputs inputs) {}
}
