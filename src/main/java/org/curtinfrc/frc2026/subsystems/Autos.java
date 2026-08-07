package org.curtinfrc.frc2026.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.hopperindexer.HopperIndexer;
import org.curtinfrc.frc2026.subsystems.shooter.Shooter;

public class Autos {
  /** Matches the teleop shoot binding in Robot. */
  private static final double SHOOTER_SPEED_RPS = 30;

  private static final double FEED_VOLTS = 6;
  private static final double SHOOT_SECONDS = 4;

  private final AutoFactory autoFactory;
  private final Drive drive;
  private final HopperIndexer hopperIndexer;
  private final Shooter shooter;

  public Autos(AutoFactory autoFactory, Drive drive, HopperIndexer hopperIndexer, Shooter shooter) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.hopperIndexer = hopperIndexer;
    this.shooter = shooter;
  }

  public Command singleGreedy() {
    return trajectoryAuto("SingleGreedy");
  }

  public Command testAutoSafe() {
    return trajectoryAuto("testAutoSafe");
  }

  public Command doubleGreedy1() {
    return trajectoryAuto("DoubleGreedyPart1");
  }

  public Command doubleGreedy2() {
    return trajectoryAuto("DoubleGreedyPart2");
  }

  public Command doubleGreedy1Test() {
    return trajectoryAuto("DoubleGreedyPart1_Test");
  }

  public AutoRoutine singleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("SingleGreedyRoutine");
    AutoTrajectory singleGreedy = routine.trajectory("SingleGreedy");

    routine
        .active()
        .onTrue(Commands.sequence(singleGreedy.resetOdometry(), singleGreedy.cmd(), shoot()));
    return routine;
  }

  public AutoRoutine doubleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("DoubleGreedyRoutine");
    AutoTrajectory part1 = routine.trajectory("DoubleGreedyPart1");
    AutoTrajectory part2 = routine.trajectory("DoubleGreedyPart2");

    routine
        .active()
        .onTrue(
            Commands.sequence(part1.resetOdometry(), part1.cmd(), shoot(), part2.cmd(), shoot()));
    return routine;
  }

  /**
   * Spins up the shooter, feeds once it is at speed, and times out after {@link #SHOOT_SECONDS}.
   * Shoots from wherever the preceding trajectory ended: the paths are drawn to end back-to-hub at
   * shooting distance, so the drivetrain is intentionally left alone (no alignment lunge between
   * split trajectories).
   */
  private Command shoot() {
    return Commands.parallel(
            shooter.setAngularVelocity(SHOOTER_SPEED_RPS),
            Commands.waitUntil(shooter.readyToShoot)
                .andThen(hopperIndexer.setAllRollerVoltage(FEED_VOLTS)))
        .withTimeout(SHOOT_SECONDS);
  }

  /** Resets odometry to the trajectory's start pose, follows it, then stops. */
  private Command trajectoryAuto(String trajName) {
    return autoFactory
        .resetOdometry(trajName)
        .andThen(autoFactory.trajectoryCmd(trajName))
        .andThen(drive.runOnce(drive::stop));
  }
}
