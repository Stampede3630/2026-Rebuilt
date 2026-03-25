package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void runVelocity(AngularVelocity vel) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  // default boolean setCoastMode(boolean enabled) {
  //   return true;
  // }

  default void runDutyCycle(double dutyCycle) {}

  default void stop() {}

  default boolean isRunning() {
    return true;
  }

  default void setFlipPosition(Angle pos) {}

  default void resetFlipPosition(Angle pos) {}

  default void runFlipsVoltage(Voltage volts) {}

  default void stopFlips() {}

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean intakeConnected = false;
    public boolean flipLeftConnected = false;
    public boolean flipRightConnected = false;

    // intake motor
    public Angle intakePosition = Rotations.of(0.0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public Current intakeTorqueCurrent = Amps.of(0.0);
    public Voltage intakeVoltage = Volts.of(0.0);
    public Current intakeStatorCurrent = Amps.of(0.0);
    public Current intakeSupplyCurrent = Amps.of(0.0);
    public Temperature intakeTemp = Celsius.of(0.0);
    public double intakeSetpoint = 0.0;

    // flipLeft motor
    public Angle flipLeftPosition = Rotations.of(0.0);
    public AngularVelocity flipLeftVelocity = RotationsPerSecond.of(0.0);
    public Current flipLeftTorqueCurrent = Amps.of(0.0);
    public Voltage flipLeftVoltage = Volts.of(0.0);
    public Current flipLeftStatorCurrent = Amps.of(0.0);
    public Current flipLeftSupplyCurrent = Amps.of(0.0);
    public Temperature flipLeftTemp = Celsius.of(0.0);
    public double flipLeftSetpoint = 0.0;
    public boolean flipLeftStalling = false;

    // flipRight motor
    public Angle flipRightPosition = Rotations.of(0.0);
    public AngularVelocity flipRightVelocity = RotationsPerSecond.of(0.0);
    public Current flipRightTorqueCurrent = Amps.of(0.0);
    public Voltage flipRightVoltage = Volts.of(0.0);
    public Current flipRightStatorCurrent = Amps.of(0.0);
    public Current flipRightSupplyCurrent = Amps.of(0.0);
    public Temperature flipRightTemp = Celsius.of(0.0);
    public double flipRightSetpoint = 0.0;
    public boolean flipRightStalling = false;

    public Angle flipSetpoint = Rotations.of(0.0);
    public double intakeDutyCycle = 0.0;
  }
}
