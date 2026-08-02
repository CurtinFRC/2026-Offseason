package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import java.time.temporal.Temporal;

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
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class ArmIOComp implements ArmIO {
  public final TalonFX motor = new TalonFX(7);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public static final double GEAR_RATIO = 1;

  // Signals for arm motor
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();

  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(15).withStatorCurrentLimit(30));

  public ArmIOComp() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    motor.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, velocity, position, temperature);
    motor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, voltage, velocity, position, current, temperature);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
    inputs.motorPosition = position.getValueAsDouble();
    inputs.motorTemperature = temperature.getValueAsDouble();
  }

  @Override
  public void setArmVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setArmPosition(double rotations) {
    motor.setControl(positionRequest.withPosition(rotations));
  }
}
