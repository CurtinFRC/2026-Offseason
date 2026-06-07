package org.curtinfrc.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean[] motorsConnected = new boolean[4];
    public double[] motorTemperatures = new double[4];
    public double velocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocityRotationsPerSecond) {}
}
