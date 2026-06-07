package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
  }

  public Command setVoltage(double voltage) {
    return (run(() -> indexerIO.setVoltage(voltage)));
  }
}
