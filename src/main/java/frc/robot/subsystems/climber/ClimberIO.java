package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  // default void runVelocity(double vel) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default boolean setElevCoastMode(boolean enabled) {
    return true;
  }

  default void runDutyCycleHook(double dutyCycle) {}

  default void stopHook() {}

  default void runDutyCycleElevator(double dutyCycle) {}

  default void stopElevator() {}

  /**
   * Sets the elevator to target a position
   *
   * @param pos The position to target, in rotations
   * @param slot The config slot to use (0: up, 1: down)
   */
  default void runElevPos(double pos, int slot) {}

  /**
   * Sets the hook to target a position
   *
   * @param pos The position to target, in rotations
   */
  default void runHookPos(Angle pos) {}

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
    public double elevatorSetpoint = 0.0;

    // hook motor
    public double hookPosition = 0.0;
    public double hookVelocity = 0.0;
    public double hookTorqueCurrent = 0.0;
    public double hookVoltage = 0.0;
    public double hookStatorCurrent = 0.0;
    public double hookSupplyCurrent = 0.0;
    public double hookTemp = 0.0;
    public double hookSetpoint = 0.0;
  }
}
