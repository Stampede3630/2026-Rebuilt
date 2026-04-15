package frc.robot.subsystems.kicker;

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
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  default void runVelocity(AngularVelocity vel) {}

  default void updateInputs(KickerIOInputs inputs) {}

  default void runDutyCycle(double dutyCycle) {}

  default void stop() {}

  @AutoLog
  class KickerIOInputs {
    public boolean kickerConnected = false;

    // kicker motor
    public Angle kickerPosition = Rotations.of(0.0);
    public AngularVelocity kickerVelocity = RotationsPerSecond.of(0.0);
    public Current kickerTorqueCurrent = Amps.of(0.0);
    public Voltage kickerVoltage = Volts.of(0.0);
    public Current kickerStatorCurrent = Amps.of(0.0);
    public Current kickerSupplyCurrent = Amps.of(0.0);
    public Temperature kickerTemp = Celsius.of(0.0);
    public double kickerSetpoint = 0.0;

    public double kickerDutyCycle = 0.0;
  }
}
