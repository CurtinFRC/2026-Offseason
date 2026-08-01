package org.curtinfrc.frc2026.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.hopperindexer.HopperIndexer;
import org.curtinfrc.frc2026.subsystems.intake.IntakeArm;
import org.curtinfrc.frc2026.subsystems.shooter.Shooter;

public class Autos {
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final IntakeArm intake;
  private final HopperIndexer hopperIndexer;
  private final Shooter shooter;

  public Autos(
      AutoFactory autoFactory,
      Drive drive,
      IntakeArm intake,
      HopperIndexer hopperIndexer,
      Shooter shooter) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.intake = intake;
    this.hopperIndexer = hopperIndexer;
    this.shooter = shooter;
  }

  public Command singleGreedy() {
    return trajectoryAuto("SingleGreedy");
  }

  public Command testAutoSafe() {
    return trajectoryAuto("testAutoSafe");
  }

  public Command DoubleGreedy1() {
    return trajectoryAuto("DoubleGreedyPart1");
  }

  public Command DoubleGreedy2() {
    return trajectoryAuto("DoubleGreedyPart2");
  }

  public Command DoubleGreedy1_Test() {
    return trajectoryAuto("DoubleGreedyPart1_Test");
  }

  public AutoRoutine singleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("SingleGreedy");
    AutoTrajectory singleGreedy = routine.trajectory("SingleGreedy");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                singleGreedy.resetOdometry(), singleGreedy.cmd().deadlineFor(intake.intake())));
    return routine;
  }

  public AutoRoutine doubleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("DoubleGreedyRoutine");
    AutoTrajectory DoubleGreedy1 = routine.trajectory("DoubleGreedyPart1");
    AutoTrajectory DoubleGreedy2 = routine.trajectory("DoubleGreedyPart2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                DoubleGreedy1.resetOdometry(),
                DoubleGreedy1.cmd().deadlineFor(intake.intake()),
                Commands.parallel(shooter.setAngularVelocity(30), drive.alignToHub())));
    //         .withTimeout(4),
    //     DoubleGreedy2.cmd().deadlineFor(intake.intake()),
    //     Commands.parallel(shooter.setAngularVelocity(30), drive.alignToHub()))
    // .withTimeout(4));

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
