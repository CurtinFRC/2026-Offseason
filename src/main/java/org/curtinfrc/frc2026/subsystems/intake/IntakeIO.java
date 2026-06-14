package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // what we are getting from the motors
    public double rollerMotorAppliedVoltage = 0.0;
    public double rollerMotorCurrentAmps = 0.0;
    public double rollerMotorAngularVelocity = 0.0;
    public double rollerMotorPosition = 0.0;

    public double armMotorAppliedVoltage = 0.0;
    public double armMotorCurrentAmps = 0.0;
    public double armMotorAngularVelocity = 0.0;
    public double armMotorPosition = 0.0;
  }

  // what we are sending to the motors
  public default void setRollerVoltage(double volts) {}

  public default void setArmVoltage(double volts) {}

  public default void setRollerVelocity(double velocity) {}

  public default void setArmPosition(double degrees) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
