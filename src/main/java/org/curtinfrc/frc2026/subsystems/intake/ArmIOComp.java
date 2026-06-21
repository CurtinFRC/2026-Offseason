package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class ArmIOComp implements ArmIO {

  public final TalonFX motor = new TalonFX(47); // TODO: correct ID

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public static final double GEAR_RATIO = 1;

  // Signals for arm motor
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));

  public ArmIOComp() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kD = 0;
    slot0Configs.kI = 0;
    // This kP is used by both voltage and position control requests
    // You'll probably want a higher kP for position control, tune on the real robot
    slot0Configs.kP = 0.01;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    motor.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, velocity, voltage, current);
    motor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, voltage, velocity, voltage, current);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
    inputs.motorPosition = position.getValueAsDouble();
  }

  @Override
  public void setArmVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setArmPosition(double rotations) {
    // Send the position request to the arm motor
    // The motor's internal PID handles getting there smoothly
    motor.setControl(positionRequest.withPosition(rotations));
  }
}
