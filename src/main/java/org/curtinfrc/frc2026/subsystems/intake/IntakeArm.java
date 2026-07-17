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

  // These variables are used for the voltage and velocity
  private static final double INTAKE_VOLTAGE = 8;
  private static final double IDLE_VOLTAGE = 2;

  // Arm position for game piece intake (in rotations)
  private static final double MIN_ARM_POSITION_ROTATIONS = 0.0; // Make LOWER
  private static final double MAX_ARM_POSITION_ROTATIONS = 24.0;

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    armIO.updateInputs(armInputs);
    Logger.processInputs("Intake", intakeInputs);
    Logger.processInputs("Arm", armInputs);
  }

  public Command setIntakeArmPosition() {
    return run(() -> armIO.setArmPosition(MIN_ARM_POSITION_ROTATIONS))
        .withName("intakeArmPosition");
  }

  public Command push() {
    return run(
        () -> {
          armIO.setArmPosition(MAX_ARM_POSITION_ROTATIONS);
          intakeIO.setRollerVoltage(IDLE_VOLTAGE);
        });
  }

  public Command intake() {
    return run(
        () -> {
          armIO.setArmPosition(MIN_ARM_POSITION_ROTATIONS);
          intakeIO.setRollerVoltage(INTAKE_VOLTAGE);
        });
  }
}
