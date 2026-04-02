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

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean intakeConnected = false;

    // intake motor
    public Angle intakePosition = Rotations.of(0.0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public Current intakeTorqueCurrent = Amps.of(0.0);
    public Voltage intakeVoltage = Volts.of(0.0);
    public Current intakeStatorCurrent = Amps.of(0.0);
    public Current intakeSupplyCurrent = Amps.of(0.0);
    public Temperature intakeTemp = Celsius.of(0.0);
    public double intakeSetpoint = 0.0;

    public double intakeDutyCycle = 0.0;
  }
}
