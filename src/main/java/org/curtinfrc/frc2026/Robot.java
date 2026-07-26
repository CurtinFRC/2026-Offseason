// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.curtinfrc.frc2026;

import static org.curtinfrc.frc2026.subsystems.vision.Vision.cameraConfigs;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.curtinfrc.frc2026.subsystems.Autos;
import org.curtinfrc.frc2026.subsystems.drive.Drive;
import org.curtinfrc.frc2026.subsystems.drive.GyroIO;
import org.curtinfrc.frc2026.subsystems.drive.GyroIOPigeon2;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIO;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIOSim;
import org.curtinfrc.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.curtinfrc.frc2026.subsystems.drive.TunerConstants;
import org.curtinfrc.frc2026.subsystems.hopperindexer.HopperIndexer;
import org.curtinfrc.frc2026.subsystems.hopperindexer.IndexerRollerIO;
import org.curtinfrc.frc2026.subsystems.hopperindexer.IndexerRollerIOComp;
import org.curtinfrc.frc2026.subsystems.hopperindexer.IndexerRollerIOSim;
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
import org.curtinfrc.frc2026.subsystems.vision.Vision;
import org.curtinfrc.frc2026.subsystems.vision.VisionIO;
import org.curtinfrc.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.curtinfrc.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.curtinfrc.frc2026.util.AutoChooser;
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
  private Vision vision;
  private Shooter shooter;
  private IntakeArm intakeArm;
  private HopperIndexer hopperIndexer;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final Autos autos;

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

    switch (Constants.getMode()) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(cameraConfigs[0].name(), cameraConfigs[0].robotToCamera()),
                new VisionIOPhotonVision(
                    cameraConfigs[1].name(), cameraConfigs[1].robotToCamera()));
        shooter = new Shooter(new ShooterIOComp());
        intakeArm = new IntakeArm(new IntakeIOComp(), new ArmIOComp());
        hopperIndexer =
            new HopperIndexer(
                new IndexerRollerIOComp(
                    HopperIndexer.indexerRollerID, InvertedValue.CounterClockwise_Positive),
                new IndexerRollerIOComp(
                    HopperIndexer.hopperIndexerRollersID, InvertedValue.CounterClockwise_Positive));
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    cameraConfigs[0].name(), cameraConfigs[0].robotToCamera(), drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraConfigs[1].name(), cameraConfigs[1].robotToCamera(), drive::getPose));
        shooter = new Shooter(new ShooterIOSim());
        intakeArm = new IntakeArm(new IntakeIOSim(), new ArmIOSim());
        hopperIndexer =
            new HopperIndexer(
                new IndexerRollerIOSim(
                    HopperIndexer.indexerRollerID,
                    InvertedValue.Clockwise_Positive,
                    HopperIndexer.INDEXER_ROLLER_JKG),
                new IndexerRollerIOSim(
                    HopperIndexer.hopperIndexerRollersID,
                    InvertedValue.Clockwise_Positive,
                    HopperIndexer.HOPPER_ROLLERS_JKG));
      }
      case REPLAY -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intakeArm = new IntakeArm(new IntakeIO() {}, new ArmIO() {});
        hopperIndexer = new HopperIndexer(new IndexerRollerIO() {}, new IndexerRollerIO() {});
      }
    }

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
    autos = new Autos(autoFactory, drive, intakeArm, hopperIndexer, shooter);

    autoChooser = new AutoChooser();
    autoChooser.addCmd("Test Auto Safe", autos::testAutoSafe);
    autoChooser.addCmd("Single Side Greedy", autos::singleSideGreedy);
    autoChooser.addRoutine("Single Side Greedy Routine", autos::singleSideGreedyRoutine);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.b().whileTrue(shooter.setVoltage(5)).onFalse(shooter.setVoltage(0));
    // intakeArm.setDefaultCommand(intakeArm.intake());
    controller.rightBumper().whileTrue(intakeArm.push());
    controller.a().whileTrue(hopperIndexer.setAllRollerVoltage(6)).onFalse(hopperIndexer.stopAll());
    controller
        .leftBumper()
        .whileTrue(drive.TrenchAlign(() -> -controller.getLeftY(), () -> -controller.getLeftX()));
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
  public void disabledPeriodic() {
    autoChooser.periodic();
  }

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
