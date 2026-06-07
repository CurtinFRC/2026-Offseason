package org.curtinfrc.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public static final double ROLLER_DIAMETER = 0;

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
  }

  public Command setVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public Command setVelocity(double angularVelocityRotationsPerSecond) {
    return run(() -> shooterIO.setVelocity(angularVelocityRotationsPerSecond));
  }
}
