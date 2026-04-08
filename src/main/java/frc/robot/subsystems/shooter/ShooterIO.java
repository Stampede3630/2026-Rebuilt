package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  default void runVelocity(double vel) {}

  default void updateInputs(ShooterIOInputs inputs) {}

  default void stop() {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default void setShooterMotorsControl(ControlRequest control) {}

  @AutoLog
  class ShooterIOInputs {
    public boolean topRightConnected = false;
    public boolean bottomRightConnected = false;
    public boolean topLeftConnected = false;
    public boolean bottomLeftConnected = false;

    // topRight motor
    // leader btw
    public Angle topRightPosition = Rotations.of(0.0);
    public AngularVelocity topRightVelocity = RotationsPerSecond.of(0.0);
    public Current topRightTorqueCurrent = Amps.of(0.0);
    public Voltage topRightVoltage = Volts.of(0.0);
    public Current topRightStatorCurrent = Amps.of(0.0);
    public Current topRightSupplyCurrent = Amps.of(0.0);
    public Temperature topRightTemp = Celsius.of(0.0);

    // bottomRight motor
    public Angle bottomRightPosition = Rotations.of(0.0);
    public AngularVelocity bottomRightVelocity = RotationsPerSecond.of(0.0);
    public Current bottomRightTorqueCurrent = Amps.of(0.0);
    public Voltage bottomRightVoltage = Volts.of(0.0);
    public Current bottomRightStatorCurrent = Amps.of(0.0);
    public Current bottomRightSupplyCurrent = Amps.of(0.0);
    public Temperature bottomRightTemp = Celsius.of(0.0);

    // topLeft motor
    public Angle topLeftPosition = Rotations.of(0.0);
    public AngularVelocity topLeftVelocity = RotationsPerSecond.of(0.0);
    public Current topLeftTorqueCurrent = Amps.of(0.0);
    public Voltage topLeftVoltage = Volts.of(0.0);
    public Current topLeftStatorCurrent = Amps.of(0.0);
    public Current topLeftSupplyCurrent = Amps.of(0.0);
    public Temperature topLeftTemp = Celsius.of(0.0);

    // bottomLeft motor
    public Angle bottomLeftPosition = Rotations.of(0.0);
    public AngularVelocity bottomLeftVelocity = RotationsPerSecond.of(0.0);
    public Current bottomLeftTorqueCurrent = Amps.of(0.0);
    public Voltage bottomLeftVoltage = Volts.of(0.0);
    public Current bottomLeftStatorCurrent = Amps.of(0.0);
    public Current bottomLeftSupplyCurrent = Amps.of(0.0);
    public Temperature bottomLeftTemp = Celsius.of(0.0);

    // setpoint should be shared
    public AngularVelocity velSetpoint = RotationsPerSecond.of(0.0);
    public boolean promoteFollower = false;
  }
}
