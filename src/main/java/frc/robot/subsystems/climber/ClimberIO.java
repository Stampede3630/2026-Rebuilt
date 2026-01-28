package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  // default void runVelocity(double vel) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  // default boolean setCoastMode(boolean enabled) {
  //   return true;
  // }

  default void runDutyCycleLeft(double dutyCycle) {}

  default void stopLeft() {}

  default void runDutyCycleRight(double dutyCycle) {}

  default void stopRight() {}

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class ClimberIOInputs {
    public boolean connected = false;

    // right motor
    public double rightPosition = 0.0;
    public double rightVelocity = 0.0;
    public double rightTorqueCurrent = 0.0;
    public double rightVoltage = 0.0;
    public double rightStatorCurrent = 0.0;
    public double rightSupplyCurrent = 0.0;
    public double rightTemp = 0.0;

    // left motor
    public double leftPosition = 0.0;
    public double leftVelocity = 0.0;
    public double leftTorqueCurrent = 0.0;
    public double leftVoltage = 0.0;
    public double leftStatorCurrent = 0.0;
    public double leftSupplyCurrent = 0.0;
    public double leftTemp = 0.0;
  }
}
