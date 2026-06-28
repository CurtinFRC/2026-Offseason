package org.curtinfrc.frc2026.subsystems.hopperindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperIndexer extends SubsystemBase {
  // temp
  public static final int indexerRollerID = 15;
  public static final int hopperIndexerRollersID = 16;

  private IndexerRoller indexerRoller;
  private IndexerRoller hopperIndexerRollers;

  public HopperIndexer(IndexerRollerIO indexerRollerIO, IndexerRollerIO hopperIndexerRollers) {
    this.indexerRoller = new IndexerRoller(indexerRollerIO, "indexerRoller");
    this.hopperIndexerRollers = new IndexerRoller(hopperIndexerRollers, "hopperIndexerRollers");
  }

  public Command setHopperIndexerVoltage(double voltage) {
    return Commands.parallel(
        indexerRoller.setVoltage(voltage), hopperIndexerRollers.setVoltage(voltage));
  }

  public Command setIndexerRollerVoltage(double voltage) {
    return indexerRoller.setVoltage(voltage);
  }

  public Command setHopperRollerVoltage(double voltage) {
    return hopperIndexerRollers.setVoltage(voltage);
  }
}
