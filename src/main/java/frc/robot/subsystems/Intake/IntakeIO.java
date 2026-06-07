package frc.robot.subsystems.Intake;
package org.curtinfrc.frc2026.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  //what we are getting from the motors
  public static class IntakeIOInputs {
    double rollerMotor1AppliedVoltage;
    double rollerMotor1CurrentAmps;
    double rollerMotor1AngularVelocity;
    double rollerMotor1Position;

    double armMotorAppliedVoltage;
    double armMotorCurrentAmps;
    double armMotorAngularVelocity;
    double armMotorPosition;

    
  }//what we are sending to the motors
  public default void setVoltage(double Volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVelocity(double Velocity) {}

  
}
