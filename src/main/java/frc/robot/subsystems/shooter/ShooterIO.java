package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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

  default double getShooterSpeed() {
    return 0.0;
  }

  default void setShooterMotorsControl(ControlRequest control) {}

  default AngularVelocity getSpeedSetpoint() {
    return RadiansPerSecond.of(0);
  }

  default AngularVelocity getSpeedReal() {
    return RadiansPerSecond.of(0);
  }

  @AutoLog
  class ShooterIOInputs {
    public boolean connected = false;

    // leader motor
    public Angle leaderPosition = Rotations.of(0.0);
    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0.0);
    public Current leaderTorqueCurrent = Amps.of(0.0);
    public Voltage leaderVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0.0);
    public Current leaderSupplyCurrent = Amps.of(0.0);
    public Temperature leaderTemp = Celsius.of(0.0);

    // follower motor
    public Angle followerPosition = Rotations.of(0.0);
    public AngularVelocity followerVelocity = RotationsPerSecond.of(0.0);
    public Current followerTorqueCurrent = Amps.of(0.0);
    public Voltage followerVoltage = Volts.of(0.0);
    public Current followerStatorCurrent = Amps.of(0.0);
    public Current followerSupplyCurrent = Amps.of(0.0);
    public Temperature followerTemp = Celsius.of(0.0);

    // setpoint should be shared
    public AngularVelocity velSetpoint = RotationsPerSecond.of(0.0);
  }
}
