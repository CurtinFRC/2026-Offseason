package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
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
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class IntakeIOComp implements IntakeIO {

  public final TalonFX rollerMotor = new TalonFX(46); // TODO: correct ID

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  // This is new — a request that tells the motor to go to a specific position
  public static final double GEAR_RATIO = 1;

  // Signals for roller motor
  private final StatusSignal<Voltage> voltage = rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> current = rollerMotor.getStatorCurrent();
  private final StatusSignal<Angle> position = rollerMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = rollerMotor.getVelocity();

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
    // This kP is used by both voltage and velocity control requests
    // Tune on the real robot for optimal performance
    slot0Configs.kP = 0.01;

    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));

    rollerMotor.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, velocity, voltage, current);
    rollerMotor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, voltage, velocity, voltage, current);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorAppliedVoltage = voltage.getValueAsDouble();
    inputs.rollerMotorCurrentAmps = current.getValueAsDouble();
    inputs.rollerMotorAngularVelocity = velocity.getValueAsDouble();
    inputs.rollerMotorPosition = position.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollerMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
