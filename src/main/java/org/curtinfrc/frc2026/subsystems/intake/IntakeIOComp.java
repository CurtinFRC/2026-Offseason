package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOComp implements IntakeIO {

  // Two separate motors
  private final TalonFX rollerMotor = new TalonFX(46); // TODO: correct ID
  private final TalonFX armMotor = new TalonFX(47); // TODO: correct ID

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Signals for roller motor
  private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = rollerMotor.getStatorCurrent();
  private final StatusSignal<Angle> rollerPosition = rollerMotor.getPosition();
  private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();

  // Signals for arm motor
  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Angle> armPosition = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> armVelocitySignal = armMotor.getVelocity();

  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));

  public IntakeIOComp() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kD = 0;
    slot0Configs.kI = 0;
    slot0Configs.kP = 0.01;

    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> armMotor.getConfigurator().apply(config));

    rollerMotor.getConfigurator().apply(slot0Configs);
    armMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorAppliedVoltage = rollerVoltage.getValueAsDouble();
    inputs.rollerMotorCurrentAmps = rollerCurrent.getValueAsDouble();
    inputs.rollerMotorAngularVelocity = rollerVelocity.getValueAsDouble();
    inputs.rollerMotorPosition = rollerPosition.getValueAsDouble();

    inputs.armMotorAppliedVoltage = armVoltage.getValueAsDouble();
    inputs.armMotorCurrentAmps = armCurrent.getValueAsDouble();
    inputs.armMotorAngularVelocity = armVelocitySignal.getValueAsDouble();
    inputs.armMotorPosition = armPosition.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setArmVoltage(double volts) {
    armMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollerMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setArmPosition(double degrees) {
    // TODO set PID
  }
}
