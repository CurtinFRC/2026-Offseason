package org.curtinfrc.frc2026.subsystems.hopperindexer;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class IndexerRollerIOComp implements IndexerRollerIO {
  protected final TalonFX rollerMotor;
  protected final TalonFXConfiguration motorConfig;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<AngularVelocity> angularVelocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> motorTemperature;

  public IndexerRollerIOComp(
      int motorID,
      InvertedValue invertedValue,
      double supplyCurrentLimit,
      double statorCurrentLimit) {
    rollerMotor = new TalonFX(motorID);
    motorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(invertedValue))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(supplyCurrentLimit)
                    .withStatorCurrentLimit(statorCurrentLimit));

    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(motorConfig));

    voltage = rollerMotor.getMotorVoltage();
    angularVelocity = rollerMotor.getVelocity();
    position = rollerMotor.getPosition();
    statorCurrent = rollerMotor.getStatorCurrent();
    supplyCurrent = rollerMotor.getSupplyCurrent();
    motorTemperature = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, statorCurrent, supplyCurrent, position, angularVelocity, motorTemperature);
    PhoenixUtil.registerSignals(
        false, voltage, statorCurrent, supplyCurrent, angularVelocity, position, motorTemperature);
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerRollerIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotationsPerSecond = angularVelocity.getValueAsDouble();
    inputs.motorTemperature = motorTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }
}
