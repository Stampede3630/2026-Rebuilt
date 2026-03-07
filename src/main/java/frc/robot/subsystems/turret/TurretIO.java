package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void stopTurret() {}

  default void setTurretAngle(Angle angle) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default void resetAnglePos(Angle newAngle) {}

  default boolean isInitSet() {
    return false;
  }

  default void updateInitSet(boolean set) {}

  default void setTurretMotorControl(VoltageOut volts) {}

  /**
   * A method to be used in case turret auto aim is disabled
   *
   * @param speed The speed to turn at [-1, 1]
   */
  default void runTurret(double speed) {}

  default void setTurretAngleTorqueCurrent(Angle angle) {}

  default void setNeutralMode(NeutralModeValue val) {}

  default boolean isAtSetpoint(Angle tol) {
    return false;
  }

  @AutoLog
  class TurretIOInputs {
    public boolean connected = false;

    // turret motor
    public Angle position = Rotations.of(0.0);
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public Current torqueCurrent = Amps.of(0.0);
    public Voltage voltage = Volts.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Temperature temp = Celsius.of(0.0);
    public Angle setpoint = Rotations.of(0.0);
  }
}
