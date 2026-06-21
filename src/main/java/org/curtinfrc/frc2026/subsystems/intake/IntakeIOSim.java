package org.curtinfrc.frc2026.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim extends IntakeIOComp {
  private static final double DT = 0.02;
  private static final double INTAKE_JKG = 0.1986; // TODO Fix this

  private final TalonFXSimState intakeSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(3);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public IntakeIOSim() {
    super();
    intakeSim = rollerMotor.getSimState();
    intakeSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, INTAKE_JKG, GEAR_RATIO), motorType);

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(DT);
  }

  public void updateSim() {
    double motorVolts = intakeSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(DT);

    intakeSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    intakeSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
    intakeSim.setRotorVelocity(motorSimModel.getAngularVelocityRPM() / 60.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
