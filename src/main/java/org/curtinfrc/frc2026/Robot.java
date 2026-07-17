// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.curtinfrc.frc2026;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.drive.GyroIO;
import org.curtinfrc.frc2026.subsystems.drive.GyroIOPigeon2;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIO;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIOSim;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.curtinfrc.frc2026.subsystems.drive.TunerConstants;
import org.curtinfrc.frc2026.subsystems.intake.ArmIO;
import org.curtinfrc.frc2026.subsystems.intake.ArmIOComp;
import org.curtinfrc.frc2026.subsystems.intake.ArmIOSim;
import org.curtinfrc.frc2026.subsystems.intake.IntakeArm;
import org.curtinfrc.frc2026.subsystems.intake.IntakeIO;
import org.curtinfrc.frc2026.subsystems.intake.IntakeIOComp;
import org.curtinfrc.frc2026.subsystems.intake.IntakeIOSim;
import org.curtinfrc.frc2026.subsystems.shooter.Shooter;
import org.curtinfrc.frc2026.subsystems.shooter.ShooterIO;
import org.curtinfrc.frc2026.subsystems.shooter.ShooterIOComp;
import org.curtinfrc.frc2026.subsystems.shooter.ShooterIOSim;
import org.curtinfrc.frc2026.util.GameState;
import org.curtinfrc.frc2026.util.PhoenixUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Drive drive;
  private Shooter shooter;
  private IntakeArm intakeArm;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final Alert controllerDisconnected =
      new Alert("Driver controller disconnected!", AlertType.kError);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robotType) {
        case COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          shooter = new Shooter(new ShooterIOComp());
          intakeArm = new IntakeArm(new IntakeIOComp(), new ArmIOComp());
        }
        case SIM -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          shooter = new Shooter(new ShooterIOSim());
          intakeArm = new IntakeArm(new IntakeIOSim(), new ArmIOSim());
        }
      }
    } else {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      shooter = new Shooter(new ShooterIO() {});
      intakeArm = new IntakeArm(new IntakeIO() {}, new ArmIO() {});
    }

    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.b().whileTrue(shooter.setVoltage(5)).onFalse(shooter.setVoltage(0));
    intakeArm.setDefaultCommand(intakeArm.intake());
    controller.rightBumper().whileTrue(intakeArm.push());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    PhoenixUtil.refreshAll();
    CommandScheduler.getInstance().run();
    GameState.periodic();
    controllerDisconnected.set(!controller.isConnected());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
