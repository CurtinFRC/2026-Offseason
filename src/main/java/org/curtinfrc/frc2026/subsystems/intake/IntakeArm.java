package org.curtinfrc.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.curtinfrc.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final ArmIO armIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  private final Alert intakeMotorTempAlert;
  private final Alert armMotorTempAlert;

  public IntakeArm(IntakeIO intakeIO, ArmIO armIO) {
    this.intakeIO = intakeIO;
    this.armIO = armIO;

    intakeMotorTempAlert =
        new Alert(
            "Intake motor temperature above " + Constants.MOTOR_WARNING_TEMP + "°C.",
            AlertType.kWarning);
    armMotorTempAlert =
        new Alert(
            "Arm motor temperature above " + Constants.MOTOR_WARNING_TEMP + "°C.",
            AlertType.kWarning);
  }

  private static final double INTAKE_VELOCITY_RPS = 100;
  private static final double IDLE_VELOCITY_RPS = 0;
  public static final double MIN_ARM_POSITION_ROTATIONS = -28.0;
  private static final double MAX_ARM_POSITION_ROTATIONS = -0.1;

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    armIO.updateInputs(armInputs);
    Logger.processInputs("Intake", intakeInputs);
    Logger.processInputs("Arm", armInputs);

    intakeMotorTempAlert.set(intakeInputs.motorTemperature > Constants.MOTOR_WARNING_TEMP);
    armMotorTempAlert.set(armInputs.motorTemperature > Constants.MOTOR_WARNING_TEMP);
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

  public Command outake() {
    return run(
        () -> {
          armIO.setArmPosition(MIN_ARM_POSITION_ROTATIONS);
          intakeIO.setRollerVelocity(-INTAKE_VELOCITY_RPS);
        });
  }
}
