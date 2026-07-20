package org.curtinfrc.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static final double ROLLER_DIAMETER = 0.082;
  public static final double MOTOR_WARNING_TEMP = 60;

  public static final double OPTIMAL_SHOOTING_DISTANCE = 3.5; // TODO! tune this value

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private double targetVelocityRotationsPerSecond = 0;
  private double velocityToleranceRotationsPerSecond = 1;
  private double maxAccelerationRotationsPerSecondPerSecond = 5;

  public final Trigger readyToIndex =
      new Trigger(
          () ->
              (Math.abs(shooterInputs.velocityRotationsPerSecond - targetVelocityRotationsPerSecond)
                      <= velocityToleranceRotationsPerSecond
                  && Math.abs(shooterInputs.accelerationRotationsPerSecondPerSecond)
                      <= maxAccelerationRotationsPerSecondPerSecond));

  private final Alert[] shooterMotorTempAlerts = new Alert[4];

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;

    for (int motor = 0; motor < 4; motor++) {
      shooterMotorTempAlerts[motor] =
          new Alert(
              "Shooter motor "
                  + String.valueOf(motor)
                  + " temperature above "
                  + MOTOR_WARNING_TEMP
                  + "°C.",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.recordOutput("Shooter/readyToShoot", readyToIndex.getAsBoolean());

    for (int motor = 0; motor < 4; motor++) {
      shooterMotorTempAlerts[motor].set(
          shooterInputs.motorTemperatures[motor] > MOTOR_WARNING_TEMP);
    }
  }

  public Command setVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public Command setAngularVelocity(double angularVelocityRotationsPerSecond) {
    return run(
        () -> {
          shooterIO.setAngularVelocity(angularVelocityRotationsPerSecond);
          targetVelocityRotationsPerSecond = angularVelocityRotationsPerSecond;
        });
  }
}
