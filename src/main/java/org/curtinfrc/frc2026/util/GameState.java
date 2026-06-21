package org.curtinfrc.frc2026.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GameState {
  public static final double TELEOP_GAME_LENGTH = 140.0;
  public static final double AUTONOMOUS_PERIOD_LENGTH = 20.0;
  public static final double TRANSITION_PERIOD_LENGTH = 10.0;
  public static final double MATCH_SHIFT_LENGTH = 25.0;

  public static Optional<DriverStation.Alliance> alliance = Optional.empty();
  public static Optional<DriverStation.Alliance> inactiveFirst = Optional.empty();

  private static Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private static Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  public static final Trigger activeShift = new Trigger(GameState::isHubActive);

  public static void updateAlliance() {
    noAllianceAlert.set(false);
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    if (alliance.isEmpty() && readAlliance.isPresent()) {
      alliance = readAlliance;
    }

    if (alliance.isEmpty()) {
      noAllianceAlert.set(true);
    }
  }

  public static void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      inactiveFirst = Optional.empty();
      noGameDataAlert.set(true);
      return;
    }

    char inactiveAlliance = gameData.charAt(0);
    if (inactiveAlliance == 'B') {
      inactiveFirst = Optional.of(DriverStation.Alliance.Blue);
      noGameDataAlert.set(false);
    } else if (inactiveAlliance == 'R') {
      inactiveFirst = Optional.of(DriverStation.Alliance.Red);
      noGameDataAlert.set(false);
    } else {
      inactiveFirst = Optional.empty();
      noGameDataAlert.set(true);
    }
  }

  public static double getMatchTime() {
    double gameTime = DriverStation.getMatchTime();
    if (DriverStation.isTeleopEnabled()) {
      gameTime = TELEOP_GAME_LENGTH - gameTime;
    } else {
      gameTime = AUTONOMOUS_PERIOD_LENGTH - gameTime;
    }
    return gameTime;
  }

  // Returns the number of game periods that has passed starting from auto as -1
  public static int getGamePeriodNumber() {
    double gameTime = getMatchTime();

    int gamePeriodNumber;
    if (!DriverStation.isTeleopEnabled()) { // auto
      gamePeriodNumber = -1;
    } else {
      if (gameTime <= 10) {
        gamePeriodNumber = 0;
      } else {
        gamePeriodNumber =
            Math.min(
                (int) Math.ceil((gameTime - TRANSITION_PERIOD_LENGTH) / MATCH_SHIFT_LENGTH), 5);
      }
    }

    return gamePeriodNumber;
  }

  public static boolean isHubActive() {
    if (DriverStation.isAutonomous()) {
      return true;
    }

    if (!(inactiveFirst.isPresent() && alliance.isPresent())) {
      return false;
    }

    int shiftDiscriminant = (inactiveFirst.get() == alliance.get()) ? 1 : 0;
    int gamePeriodNumber = getGamePeriodNumber();
    boolean isActive = false;
    if (gamePeriodNumber == 0) {
      isActive = true;
    } else if (gamePeriodNumber < 5) {
      isActive = !(gamePeriodNumber % 2 == shiftDiscriminant);
    } else {
      isActive = true;
    }
    return isActive;
  }

  public static boolean isHubInactive() {
    return !isHubActive();
  }

  public static double getRemainingShiftTime() {
    double shiftEndTime;
    double gameTime = getMatchTime();
    double shiftNumber = getGamePeriodNumber();
    if (shiftNumber == -1) {
      shiftEndTime = AUTONOMOUS_PERIOD_LENGTH;
    } else if (shiftNumber == 0) {
      shiftEndTime = TRANSITION_PERIOD_LENGTH;
    } else {
      shiftEndTime = shiftNumber * MATCH_SHIFT_LENGTH + TRANSITION_PERIOD_LENGTH;
      shiftEndTime = (shiftNumber < 5) ? shiftEndTime : shiftEndTime + 5; // checking for endgame
    }

    return shiftEndTime - gameTime;
  }

  public static void periodic() {
    updateGameData();
    updateAlliance();

    Logger.recordOutput("GameState/gameState", GameState.isHubActive());
    Logger.recordOutput("GameState/remainingShiftTime", GameState.getRemainingShiftTime());
  }
}
