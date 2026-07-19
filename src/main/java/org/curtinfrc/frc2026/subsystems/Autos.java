package org.curtinfrc.frc2026.subsystems;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import org.curtinfrc.frc2026.subsystems.drive.Drive;

public class Autos {
  private final AutoFactory autoFactory;
  private final Drive drive;

  public Autos(AutoFactory autoFactory, Drive drive) {
    this.autoFactory = autoFactory;
    this.drive = drive;
  }

  public Command singleSideGreedy() {
    return trajectoryAuto("singleSideGreedy");
  }

  public Command testAutoSafe() {
    return trajectoryAuto("testAutoSafe");
  }

  /** Resets odometry to the trajectory's start pose, follows it, then stops. */
  private Command trajectoryAuto(String trajName) {
    return autoFactory
        .resetOdometry(trajName)
        .andThen(autoFactory.trajectoryCmd(trajName))
        .andThen(drive.runOnce(drive::stop));
  }
}
