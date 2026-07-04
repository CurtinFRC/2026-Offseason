package org.curtinfrc.frc2026.subsystems.hopperindexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerRollerIO {
  @AutoLog
  public static class IndexerRollerIOInputs {
    public double positionRotations;
    public double velocityRotationsPerSecond;
    public double statorCurrentAmps;
    public double supplyCurrentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(IndexerRollerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
