package org.curtinfrc.frc2026.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIOComp {
  private static final double DT = 0.02;
  private static final double ARM_JKG = 0.036555;
  // Derived: 24 rotor rotations (IntakeArm.MAX_ARM_POSITION_ROTATIONS) over the full
  // 50 deg arm travel -> 172.8:1. Tune against real hardware measurements.
  private static final double ARM_GEAR_RATIO = 172.8;
  private static final double ARM_LENGTH_METERS = 0.340313;
  private static final double MIN_ANGLE_RADS = -0.8726646;
  private static final double MAX_ANGLE_RADS = 0.0;
  private static final double STARTING_ANGLE_RADS = -0.8726646;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(3);
  private final SingleJointedArmSim motorSimModel;
  private final Notifier simNotifier;

  public ArmIOSim() {
    super();
    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(motorType, ARM_JKG, ARM_GEAR_RATIO),
            motorType,
            ARM_GEAR_RATIO,
            ARM_LENGTH_METERS,
            MIN_ANGLE_RADS,
            MAX_ANGLE_RADS,
            false,
            STARTING_ANGLE_RADS);

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(DT);
  }

  public void updateSim() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(DT);

    double motorRotations =
        Units.radiansToRotations(motorSimModel.getAngleRads() - STARTING_ANGLE_RADS)
            * ARM_GEAR_RATIO;
    double motorRPS =
        Units.radiansToRotations(motorSimModel.getVelocityRadPerSec()) * ARM_GEAR_RATIO;

    motorSim.setRawRotorPosition(motorRotations);
    motorSim.setRotorVelocity(motorRPS);
  }
}
