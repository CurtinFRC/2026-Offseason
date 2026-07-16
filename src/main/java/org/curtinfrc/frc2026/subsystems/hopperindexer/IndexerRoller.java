package org.curtinfrc.frc2026.subsystems.hopperindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerRoller extends SubsystemBase {
  private final IndexerRollerIO indexerRollerIO;
  private final IndexerRollerIOInputsAutoLogged indexerRollerInputs =
      new IndexerRollerIOInputsAutoLogged();
  private final String logName;

  public IndexerRoller(IndexerRollerIO indexerIO, String logName) {
    this.indexerRollerIO = indexerIO;
    this.logName = logName;
  }

  @Override
  public void periodic() {
    indexerRollerIO.updateInputs(indexerRollerInputs);
    Logger.processInputs("HopperIndexer/" + logName, indexerRollerInputs);
  }

  public Command setVoltage(double voltage) {
    return run(() -> indexerRollerIO.setVoltage(voltage));
  }
}
