package org.curtinfrc.frc2026.subsystems.hopperindexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerRollerIOSim extends IndexerRollerIOComp {
  private static final double DT = 0.02;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public IndexerRollerIOSim(int motorID, InvertedValue invertedValue, double moiJKgMetresSquared) {
    super(motorID, invertedValue);

    motorSim = rollerMotor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, moiJKgMetresSquared, 1.0), motorType);

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(DT);
  }

  public void updateSim() {
    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(DT);

    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
    motorSim.setRotorVelocity(motorSimModel.getAngularVelocityRPM() / 60);
  }

  @Override
  public void updateInputs(IndexerRollerIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
