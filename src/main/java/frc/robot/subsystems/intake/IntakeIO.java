package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  // default void runVelocity(double vel) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  // default boolean setCoastMode(boolean enabled) {
  //   return true;
  // }

  default void runDutyCycleFlip(double dutyCycle) {}

  default void stopFlip() {}

  default void runDutyCycle(double dutyCycle) {}

  default void stop() {}

  default boolean isRunning() {
    return true;
  }

  default void setFlipPosition(Angle pos) {}

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
