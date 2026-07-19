package org.curtinfrc.frc2026.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.hopperindexer.HopperIndexer;
import org.curtinfrc.frc2026.subsystems.intake.IntakeArm;
import org.curtinfrc.frc2026.subsystems.intake.IntakeIO;
import org.curtinfrc.frc2026.subsystems.shooter.Shooter;

public class Autos {
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final IntakeArm intake;
  private final HopperIndexer hopperIndexer;
  private final Shooter shooter;

  public Autos(AutoFactory autoFactory, Drive drive, IntakeArm intake, HopperIndexer hopperIndexer, Shooter shooter) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.intake = intake;
    this.hopperIndexer = hopperIndexer;
    this.shooter = shooter;
  }

  public Command singleSideGreedy() {
    return trajectoryAuto("singleSideGreedy");
  }

  public Command testAutoSafe() {
    return trajectoryAuto("testAutoSafe");
  }

  public AutoRoutine singleSideGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("singleSideGreedy");
    AutoTrajectory singleSideGreedy = routine.trajectory("singleSideGreedy");

    routine.active().onTrue(Commands.sequence(intake.intake()));
    return routine;
  }

  /** Resets odometry to the trajectory's start pose, follows it, then stops. */
  private Command trajectoryAuto(String trajName) {
    return autoFactory
        .resetOdometry(trajName)
        .andThen(autoFactory.trajectoryCmd(trajName))
        .andThen(drive.runOnce(drive::stop));
  }
}
