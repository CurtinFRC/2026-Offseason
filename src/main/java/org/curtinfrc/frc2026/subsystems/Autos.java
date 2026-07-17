package org.curtinfrc.frc2026.subsystems;

import org.curtinfrc.frc2026.subsystems.drive.Drive;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    private final AutoFactory autoFactory;
    private final Drive drive;

    public Autos(AutoFactory autoFactory, Drive drive) {
        this.autoFactory = autoFactory;
        this.drive = drive;
        }

        public Command testAuto1() {
            return autoFactory.trajectoryCmd("testAuto1").andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
        }

        public Command testAutoSafe() {
            return autoFactory.trajectoryCmd("testAutoSafe").andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
        }

}
