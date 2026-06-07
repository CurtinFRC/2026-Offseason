package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean[] motorsConnected = new boolean[2];
    public double[] motorTemperatures = new double[2];
    public double positionRotations;
    public double velocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
