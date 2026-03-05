package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
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

  default void resetFlipPosition(Angle pos) {}

  default void runFlipCurrent(Current current) {}

  default void runFlipsVoltage(Voltage volts) {}

  default void stopFlips() {}

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;

    // intake motor
    public double intakePosition = 0.0;
    public AngularVelocity intakeVelocity;
    public double intakeTorqueCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public Current intakeStatorCurrent;
    public Current intakeSupplyCurrent;
    public double intakeTemp = 0.0;

    // flipLeft motor
    public double flipLeftPosition = 0.0;
    public AngularVelocity flipLeftVelocity;
    public double flipLeftTorqueCurrent = 0.0;
    public double flipLeftVoltage = 0.0;
    public Current flipLeftStatorCurrent;
    public Current flipLeftSupplyCurrent;
    public double flipLeftTemp = 0.0;

    // flipRight motor
    public double flipRightPosition = 0.0;
    public AngularVelocity flipRightVelocity;
    public double flipRightTorqueCurrent = 0.0;
    public double flipRightVoltage = 0.0;
    public Current flipRightStatorCurrent;
    public Current flipRightSupplyCurrent;
    public double flipRightTemp = 0.0;
  }
}
