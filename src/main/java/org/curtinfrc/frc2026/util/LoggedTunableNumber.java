package org.curtinfrc.frc2026.util;

import static org.curtinfrc.frc2026.Constants.tuningMode;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (tuningMode) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return tuningMode ? dashboardNumber.get() : defaultValue;
    }
  }

  public boolean hasChanged(int id) {
    if (!tuningMode) return false;
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public void runUpdate(Consumer<Double> update) {
    if (hasChanged(update.hashCode())) {
      update.accept(get());
    }
  }
}
