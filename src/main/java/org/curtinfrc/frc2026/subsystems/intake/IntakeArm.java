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

  // Roller velocity setpoints in mechanism rotations/sec.
  // Derived from the previous 8V/2V open-loop values (~58 RPS free speed at 12V); tune on robot.
  private static final double INTAKE_VELOCITY_RPS = 40;
  private static final double IDLE_VELOCITY_RPS = 10;

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
          intakeIO.setRollerVelocity(IDLE_VELOCITY_RPS);
        });
  }

  public Command intake() {
    return run(
        () -> {
          armIO.setArmPosition(MIN_ARM_POSITION_ROTATIONS);
          intakeIO.setRollerVelocity(INTAKE_VELOCITY_RPS);
        });
  }
}
