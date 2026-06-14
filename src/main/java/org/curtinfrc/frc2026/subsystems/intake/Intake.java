package org.curtinfrc.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  // These variables are used for the voltage and velocity//
  private static final double STOP_MOTOR_VOLTAGE = 0;
  // consume means intake//
  private final double CONSUME_VEL_RPS = 8;
  // vel means velocity//
  private static final double CONSUME_VOLTS = 8;
  private static final double IDLE_VOLTS = 2;
  private static final double IDLE_VEL_RPS = 2;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // Stops roller by setting voltage to 0
  public Command stopRoller() {
    return run(() -> io.setRollerVoltage(STOP_MOTOR_VOLTAGE)).withName("stopRoller");
  }

  // Sets roller voltage
  public Command setRollerVoltage(double volts) {
    return run(() -> io.setRollerVoltage(volts)).withName("setRollerVoltage");
  }

  // Sets roller velocity to Consume_VEL_RPS
  public Command consumeRollerVelocity() {
    return run(() -> io.setRollerVelocity(CONSUME_VEL_RPS)).withName("consumeRollerVelocity");
  }

  // Sets roller velocity to IDLE_VEL_RPS
  public Command idleRollerVelocity() {
    return run(() -> io.setRollerVelocity(IDLE_VEL_RPS)).withName("idleRollerVelocity");
  }
}
