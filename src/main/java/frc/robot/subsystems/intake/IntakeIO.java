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

    // flipLeft motor
    public double flipLeftPosition = 0.0;
    public double flipLeftVelocity = 0.0;
    public double flipLeftTorqueCurrent = 0.0;
    public double flipLeftVoltage = 0.0;
    public double flipLeftStatorCurrent = 0.0;
    public double flipLeftSupplyCurrent = 0.0;
    public double flipLeftTemp = 0.0;

    // flipRight motor
    public double flipRightPosition = 0.0;
    public double flipRightVelocity = 0.0;
    public double flipRightTorqueCurrent = 0.0;
    public double flipRightVoltage = 0.0;
    public double flipRightStatorCurrent = 0.0;
    public double flipRightSupplyCurrent = 0.0;
    public double flipRightTemp = 0.0;
  }
}
