package org.curtinfrc.frc2026;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static boolean tuningMode = false;

  /** Mode used when running on the desktop: SIM for physics sim, REPLAY for log replay. */
  public static final Mode simMode = Mode.SIM;

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : simMode;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY;
  }
}
