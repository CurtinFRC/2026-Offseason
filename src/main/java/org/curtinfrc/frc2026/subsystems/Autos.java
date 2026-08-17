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
  /** Matches the teleop shoot binding in Robot. */
  private static final double SHOOTER_SPEED_RPS = 32.5;

  private static final double FEED_VOLTS = 6;
  private static final double SHOOT_SECONDS = 4;

  /** Aim-settling time before the shooter and indexer start. */
  private static final double ALIGN_SECONDS = 1;

  /** Delay after the feed starts before the intake arm pushes. */
  private static final double PUSH_DELAY_SECONDS = 1;

  private final AutoFactory autoFactory;
  private final Drive drive;
  private final HopperIndexer hopperIndexer;
  private final Shooter shooter;
  private final IntakeArm intakeArm;

  public Autos(
      AutoFactory autoFactory,
      Drive drive,
      HopperIndexer hopperIndexer,
      Shooter shooter,
      IntakeArm intakeArm) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.hopperIndexer = hopperIndexer;
    this.shooter = shooter;
    this.intakeArm = intakeArm;
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

  public Command disrupt() {
    return trajectoryAuto("Disrupt");
  }

  public AutoRoutine singleGreedyRoutineRight() {
    AutoRoutine routine = autoFactory.newRoutine("Right Single Greedy");
    AutoTrajectory part1 = routine.trajectory("DoubleGreedyPart1");

    // The sequence holds the intakeArm requirement for its whole duration, so the default
    // intake() command can't run during the trajectories -- deploy the intake explicitly or
    // the arm never leaves stowed and push() during shoot(4) has nothing to do.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine singleGreedyRoutineLeft() {
    AutoRoutine routine = autoFactory.newRoutine("Left Single Greedy");
    AutoTrajectory part1 = routine.trajectory("LDoubleGreedyPart1");

    // The sequence holds the intakeArm requirement for its whole duration, so the default
    // intake() command can't run during the trajectories -- deploy the intake explicitly or
    // the arm never leaves stowed and push() during shoot(4) has nothing to do.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine shootTest() {
    AutoRoutine routine = autoFactory.newRoutine("Shoot Test");
    routine.active().onTrue(shoot(4));
    return routine;
  }

  public AutoRoutine doubleGreedyRoutineRight() {
    AutoRoutine routine = autoFactory.newRoutine("Right Double Greedy");
    AutoTrajectory part1 = routine.trajectory("DoubleGreedyPart1");
    AutoTrajectory part2 = routine.trajectory("DoubleGreedyPart2");

    // The sequence holds the intakeArm requirement for its whole duration, so the default
    // intake() command can't run during the trajectories -- deploy the intake explicitly or
    // the arm never leaves stowed and push() during shoot(4) has nothing to do.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4),
                Commands.deadline(part2.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine doubleGreedyRoutineLeft() {
    AutoRoutine routine = autoFactory.newRoutine("Left Double Greedy");
    AutoTrajectory part1 = routine.trajectory("LDoubleGreedyPart1");
    AutoTrajectory part2 = routine.trajectory("LDoubleGreedyPart2");

    // The sequence holds the intakeArm requirement for its whole duration, so the default
    // intake() command can't run during the trajectories -- deploy the intake explicitly or
    // the arm never leaves stowed and push() during shoot(4) has nothing to do.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4),
                Commands.deadline(part2.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine doubleHalfRoutineRight() {
    AutoRoutine routine = autoFactory.newRoutine("Right Double Half");
    AutoTrajectory part1 = routine.trajectory("DoubleHalfPart1");
    AutoTrajectory part2 = routine.trajectory("DoubleHalfPart2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4),
                Commands.deadline(part2.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine doubleHalfRoutineLeft() {
    AutoRoutine routine = autoFactory.newRoutine("Left Double Half");
    AutoTrajectory part1 = routine.trajectory("LDoubleHalfPart1");
    AutoTrajectory part2 = routine.trajectory("LDoubleHalfPart2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4),
                Commands.deadline(part2.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  public AutoRoutine doubleTwoHalvesRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("Right Double Two Halves");
    AutoTrajectory part1 = routine.trajectory("DoubleTwoHalvesPart1");
    AutoTrajectory part2 = routine.trajectory("DoubleTwoHalvesPart2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intakeAuto()),
                shoot(4),
                Commands.deadline(part2.cmd(), intakeArm.intakeAuto()),
                shoot(4)));
    return routine;
  }

  /**
   * Aims at the hub for {@link #ALIGN_SECONDS}, then spins the shooter and indexer together, then
   * pushes with the intake arm {@link #PUSH_DELAY_SECONDS} later. Aiming runs throughout. Times out
   * after {@link #SHOOT_SECONDS}.
   */
  private Command shoot(double timeout) {
    return Commands.parallel(
            // No drive::stop here: alignToHub already owns the drive, and a second drive
            // command in the same parallel throws at construction.
            drive.alignToHub(),
            // Fixed timing rather than waiting on shooter.readyToShoot: that trigger compares
            // against the shooter's target velocity, which is 0 while idle, so it reads true
            // at rest and gates nothing.
            Commands.waitSeconds(ALIGN_SECONDS)
                .andThen(
                    // Feed commands are runEnd and never finish, so push must run alongside the
                    // feed (andThen after it would never be reached).
                    Commands.parallel(
                        shooter.setAngularVelocity(SHOOTER_SPEED_RPS),
                        hopperIndexer.setAllRollerVoltage(FEED_VOLTS),
                        Commands.waitSeconds(1)
                            .andThen(
                                Commands.repeatingSequence(
                                    intakeArm.push().withTimeout(0.5),
                                    intakeArm.intakeAuto().withTimeout(0.5))))))
        // Required: nothing in the parallel above ever finishes on its own, so without this
        // the routine's sequence hangs here and the rollers feed forever.
        .withTimeout(timeout);
  }

  /** Resets odometry to the trajectory's start pose, follows it, then stops. */
  private Command trajectoryAuto(String trajName) {
    return autoFactory
        .resetOdometry(trajName)
        .andThen(autoFactory.trajectoryCmd(trajName))
        .andThen(drive.runOnce(drive::stop));
  }
}
