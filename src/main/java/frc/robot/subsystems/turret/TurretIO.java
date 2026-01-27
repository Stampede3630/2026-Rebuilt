package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void stopTurret() {}

  default void setAngle(Angle angle) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  /**
   * @return Get the current angle of the turret, in robot-relative coordinates
   */
  default Angle getTurretAngle() {
    return Radians.of(0);
  }

  default void setHoodAngle(Angle angle) {}

  default void resetAnglePos(Angle newAngle) {}

  default boolean isInitSet() {
    return false;
  }

  default void updateInitSet(boolean set) {}

  /**
   * @return Get the current angle of the turret's hood
   */
  default Angle getHoodAngle() {
    return Radians.of(0);
  }

  default void setTurretMotorControl(VoltageOut volts) {}

  @AutoLog
  class TurretIOInputs {
    public boolean connected = false;
    // public boolean shootReady = false;

    // public MotorInputs turret = new MotorInputs();
    // public MotorInputs hood = new MotorInputs();

    // turret motor
    public double turretPosition = 0.0;
    public double turretVelocity = 0.0;
    public double turretTorqueCurrent = 0.0;
    public double turretVoltage = 0.0;
    public double turretStatorCurrent = 0.0;
    public double turretSupplyCurrent = 0.0;
    public double turretTemp = 0.0;
    public double turretSetpoint = 0.0;

    // hood motor
    public double hoodPosition = 0.0;
    public double hoodVelocity = 0.0;
    public double hoodTorqueCurrent = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodStatorCurrent = 0.0;
    public double hoodSupplyCurrent = 0.0;
    public double hoodTemp = 0.0;
    public double hoodSetpoint = 0.0;
  }
}
