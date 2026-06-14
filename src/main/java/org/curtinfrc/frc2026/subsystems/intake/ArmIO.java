package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    // what we are getting from the motors

    public double armMotorAppliedVoltage = 0.0;
    public double armMotorCurrentAmps = 0.0;
    public double armMotorAngularVelocity = 0.0;
    public double armMotorPosition = 0.0;
  }

  // what we are sending to the motors

  public default void setArmVoltage(double volts) {}

  public default void setArmPosition(double rotations) {}

  public default void updateInputs(ArmIOInputs inputs) {}
}
