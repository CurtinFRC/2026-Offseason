package org.curtinfrc.frc2026.subsystems.hopperindexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerRollerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double positionRotations;
    public double velocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
