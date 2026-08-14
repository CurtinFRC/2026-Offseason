package org.curtinfrc.frc2026;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.subsystems.Autos;
import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.drive.GyroIO;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIO;
import org.curtinfrc.frc2026.subsystems.hopperindexer.HopperIndexer;
import org.curtinfrc.frc2026.subsystems.hopperindexer.IndexerRollerIO;
import org.curtinfrc.frc2026.subsystems.intake.ArmIO;
import org.curtinfrc.frc2026.subsystems.intake.IntakeArm;
import org.curtinfrc.frc2026.subsystems.intake.IntakeIO;
import org.curtinfrc.frc2026.subsystems.shooter.Shooter;
import org.curtinfrc.frc2026.subsystems.shooter.ShooterIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Reproduces auto-start crashes by scheduling the real routines against empty IO layers. */
public class AutosTest {
  private Autos autos;

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    Shooter shooter = new Shooter(new ShooterIO() {});
    IntakeArm intakeArm = new IntakeArm(new IntakeIO() {}, new ArmIO() {});
    HopperIndexer hopperIndexer =
        new HopperIndexer(new IndexerRollerIO() {}, new IndexerRollerIO() {});
    AutoFactory autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
    autos = new Autos(autoFactory, drive, hopperIndexer, shooter, intakeArm);

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Runs the scheduler with stepped sim time so trajectory timers actually elapse. */
  private void runFor(double seconds) {
    SimHooks.pauseTiming();
    try {
      for (int i = 0; i < (int) (seconds / 0.02); i++) {
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(0.02);
      }
    } finally {
      SimHooks.resumeTiming();
    }
  }

  /** Schedules the routine and simulates a full auto period, failing if anything throws. */
  private void assertRoutineRuns(Supplier<AutoRoutine> routine) {
    assertDoesNotThrow(
        () -> {
          var cmd = routine.get().cmd();
          CommandScheduler.getInstance().schedule(cmd);
          runFor(30.0);
        });
  }

  @Test
  void doubleGreedyRoutineRightStarts() {
    assertRoutineRuns(autos::doubleGreedyRoutineRight);
  }

  @Test
  void doubleGreedyRoutineLeftStarts() {
    assertRoutineRuns(autos::doubleGreedyRoutineLeft);
  }

  @Test
  void singleGreedyRoutineRightStarts() {
    assertRoutineRuns(autos::singleGreedyRoutineRight);
  }

  @Test
  void singleGreedyRoutineLeftStarts() {
    assertRoutineRuns(autos::singleGreedyRoutineLeft);
  }
}
