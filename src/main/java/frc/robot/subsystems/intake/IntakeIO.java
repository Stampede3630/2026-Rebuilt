package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void runVelocity(double perc) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void stop() {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;

    // intake motor
    public double intakePosition = 0.0;
    public double intakeVelocity = 0.0;
    public double intakeTorqueCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeStatorCurrent = 0.0;
    public double intakeSupplyCurrent = 0.0;
    public double intakeTemp = 0.0;

    // flip motor
    public double flipPosition = 0.0;
    public double flipVelocity = 0.0;
    public double flipTorqueCurrent = 0.0;
    public double flipVoltage = 0.0;
    public double flipStatorCurrent = 0.0;
    public double flipSupplyCurrent = 0.0;
    public double flipTemp = 0.0;
  }
}
