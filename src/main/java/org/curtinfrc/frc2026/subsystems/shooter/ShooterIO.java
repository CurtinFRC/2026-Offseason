package org.curtinfrc.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotationsPerSecond;
    public double accelerationRotationsPerSecondPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setAngularVelocity(double velocityRotationsPerSecond) {}
}
