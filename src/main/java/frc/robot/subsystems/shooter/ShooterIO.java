package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShootIOInputs {
    public boolean[] motorsConnected = new boolean[2];
    public double[] motorTemperatures = new double[2];
    public double velocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(ShootIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocityRotationsPerSecond) {}
}
