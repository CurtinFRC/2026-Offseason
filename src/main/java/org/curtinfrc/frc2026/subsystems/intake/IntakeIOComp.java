package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
  public final TalonFX motor = new TalonFX(8);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public static final double GEAR_RATIO = 1.67;

  // Signals for roller motor
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO));

  public IntakeIOComp() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kD = 0;
    slot0Configs.kI = 0;
    // This kP is used by both voltage and velocity control requests
    // Tune on the real robot for optimal performance
    slot0Configs.kP = 1;
    // Velocity feedforward: 12V / (~58 RPS mechanism free speed). Tune on robot.
    slot0Configs.kV = 0.21;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    motor.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, velocity, position, current);
    motor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, voltage, velocity, position, current);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
    inputs.rollerMotorPosition = position.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }
}
