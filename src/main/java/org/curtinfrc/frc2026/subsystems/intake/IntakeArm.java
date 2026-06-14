package org.curtinfrc.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final ArmIO armIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public IntakeArm(IntakeIO intakeIO, ArmIO armIO) {
    this.intakeIO = intakeIO;
    this.armIO = armIO;
  }

  // These variables are used for the voltage and velocity//
  private static final double STOP_MOTOR_VOLTAGE = 0;
  // consume means intake//
  private final double CONSUME_VEL_RPS = 8;
  // vel means velocity//
  private static final double IDLE_VEL_RPS = 2;

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    armIO.updateInputs(armInputs);
    Logger.processInputs("Intake", intakeInputs);
    Logger.processInputs("Intake", armInputs);
  }

  // Stops roller by setting voltage to 0
  public Command stopRoller() {
    return run(() -> intakeIO.setRollerVoltage(STOP_MOTOR_VOLTAGE)).withName("stopRoller");
  }

  // Sets roller voltage
  public Command setRollerVoltage(double volts) {
    return run(() -> intakeIO.setRollerVoltage(volts)).withName("setRollerVoltage");
  }

  // Sets roller velocity to Consume_VEL_RPS
  public Command consumeRollerVelocity() {
    return run(() -> intakeIO.setRollerVelocity(CONSUME_VEL_RPS)).withName("consumeRollerVelocity");
  }

  // Sets roller velocity to IDLE_VEL_RPS
  public Command idleRollerVelocity() {
    return run(() -> intakeIO.setRollerVelocity(IDLE_VEL_RPS)).withName("idleRollerVelocity");
  }

  // Set arm rotation

  public Command setIntakeArmPosition() {
    return run(() -> armIO.setArmPosition(5)).withName("armPosition");
  }
}
