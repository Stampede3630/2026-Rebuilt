package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  // default void runVelocity(double vel) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  // default boolean setCoastMode(boolean enabled) {
  //   return true;
  // }

  default void runDutyCycleHook(double dutyCycle) {}

  default void stopHook() {}

  default void runDutyCycleElevator(double dutyCycle) {}

  default void stopElevator() {}

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class ClimberIOInputs {
    public boolean connected = false;

    // elevator motor
    public double elevatorPosition = 0.0;
    public double elevatorVelocity = 0.0;
    public double elevatorTorqueCurrent = 0.0;
    public double elevatorVoltage = 0.0;
    public double elevatorStatorCurrent = 0.0;
    public double elevatorSupplyCurrent = 0.0;
    public double elevatorTemp = 0.0;

    // hook motor
    public double hookPosition = 0.0;
    public double hookVelocity = 0.0;
    public double hookTorqueCurrent = 0.0;
    public double hookVoltage = 0.0;
    public double hookStatorCurrent = 0.0;
    public double hookSupplyCurrent = 0.0;
    public double hookTemp = 0.0;
  }
}
