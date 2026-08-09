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

  public AutoRoutine singleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("SingleGreedyRoutine");
    AutoTrajectory singleGreedy = routine.trajectory("SingleGreedy");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                singleGreedy.resetOdometry(),
                Commands.deadline(singleGreedy.cmd(), intakeArm.intake()),
                shoot()));
    return routine;
  }

  public AutoRoutine doubleGreedyRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("DoubleGreedyRoutine");
    AutoTrajectory part1 = routine.trajectory("DoubleGreedyPart1");
    AutoTrajectory part2 = routine.trajectory("DoubleGreedyPart2");

    // The sequence holds the intakeArm requirement for its whole duration, so the default
    // intake() command can't run during the trajectories -- deploy the intake explicitly or
    // the arm never leaves stowed and push() during shoot() has nothing to do.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.deadline(part1.cmd(), intakeArm.intake()),
                shoot(),
                Commands.deadline(part2.cmd(), intakeArm.intake()),
                shoot()));
    return routine;
  }

  /**
   * Aims at the hub while spinning up the shooter, feeds once it is at speed, and times out after
   * {@link #SHOOT_SECONDS}. Mirrors the teleop shoot binding in Robot.
   */
  private Command shoot() {
    return Commands.parallel(
            // No drive::stop here: alignToHub already owns the drive, and a second drive
            // command in the same parallel throws at construction.
            drive.alignToHub(),
            shooter.setAngularVelocity(SHOOTER_SPEED_RPS),
            // Feed commands are runEnd and never finish, so push must run alongside the
            // feed (andThen after it would never be reached).
            Commands.waitUntil(shooter.readyToShoot)
                .andThen(
                    Commands.parallel(
                        hopperIndexer.setAllRollerVoltage(FEED_VOLTS), intakeArm.push())))
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

// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣤⣤⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⡿⠛⠉⠙⠛⠛⠛⠛⠻⢿⣿⣷⣤⡀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⠋⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠈⢻⣿⣿⡄⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣸⣿⡏⠀⠀⠀⣠⣶⣾⣿⣿⣿⠿⠿⠿⢿⣿⣿⣿⣄⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠁⠀⠀⢰⣿⣿⣯⠁⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣷⡄⠀
// ⠀⠀⣀⣤⣴⣶⣶⣿⡟⠀⠀⠀⢸⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀
// ⠀⢰⣿⡟⠋⠉⣹⣿⡇⠀⠀⠀⠘⣿⣿⣿⣿⣷⣦⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⠀
// ⠀⢸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀
// ⠀⣸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠉⠻⠿⣿⣿⣿⣿⡿⠿⠿⠛⢻⣿⡇⠀⠀
// ⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣧⠀⠀
// ⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
// ⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
// ⠀⢿⣿⡆⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⠀
// ⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀
// ⠀⠀⠛⢿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⣰⣿⣿⣷⣶⣶⣶⣶⠶⠀⢠⣿⣿⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⣽⣿⡏⠁⠀⠀⢸⣿⡇⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⢹⣿⡆⠀⠀⠀⣸⣿⠇⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⢿⣿⣦⣄⣀⣠⣴⣿⣿⠁⠀⠈⠻⣿⣿⣿⣿⡿⠏⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠈⠛⠻⠿⠿⠿⠿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
