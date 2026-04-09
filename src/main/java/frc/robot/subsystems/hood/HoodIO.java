package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void updateInputs(HoodIOInputs inputs) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default void stopHood() {}

  default void setHoodPos(double pos) {}

  default void setMicroseconds(int max) {}

  default void setPosition(Angle pos) {}

  default boolean isAtSetpoint(Angle tol) {
    return true;
  }

  @AutoLog
  class HoodIOInputs {
    // public boolean connected = false;

    // hood motor
    public double positionEstimateOld = 0.0;
    public double positionSetpointOld = 0.0;

    public boolean connected = false;

    public Angle position = Degrees.of(0.0);
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public Current torqueCurrent = Amps.of(0.0);
    public Voltage voltage = Volts.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Temperature temp = Celsius.of(0.0);
    public Angle setpoint = Degrees.of(0.0);
  }
}
