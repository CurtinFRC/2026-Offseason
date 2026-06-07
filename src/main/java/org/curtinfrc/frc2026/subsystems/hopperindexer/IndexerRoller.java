package org.curtinfrc.frc2026.subsystems.hopperindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerRoller extends SubsystemBase {
  private final IndexerRollerIO indexerRollerIO;
  private final IndexerIOInputsAutoLogged indexerRollerInputs = new IndexerIOInputsAutoLogged();

  public IndexerRoller(IndexerRollerIO indexerIO) {
    this.indexerRollerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerRollerIO.updateInputs(indexerRollerInputs);
  }

  public Command setVoltage(double voltage) {
    return (run(() -> indexerRollerIO.setVoltage(voltage)));
  }
}
