package org.curtinfrc.frc2026.subsystems.hopperindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperIndexer extends SubsystemBase {
  public static final int indexerRollerID = 0;
  public static final int hopperIndexerRollersID = 0;

  private IndexerRoller indexerRoller;
  private IndexerRoller hopperIndexerRollers;

  public HopperIndexer(IndexerRollerIO indexerRollerIO, IndexerRollerIO hopperIndexerRollers) {
    this.indexerRoller = new IndexerRoller(indexerRollerIO);
    this.hopperIndexerRollers = new IndexerRoller(hopperIndexerRollers);
  }

  public Command setVoltage(double voltage) {
    return (run(
        () -> {
          indexerRoller.setVoltage(voltage);
          hopperIndexerRollers.setVoltage(voltage);
        }));
  }
}
