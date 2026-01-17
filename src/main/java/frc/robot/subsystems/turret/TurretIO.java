package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void stop() {}

  default void setAngle(Angle angle) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default double getTurretAngle() {
    return 0.0;
  }

  default void setHoodAngle(Angle angle) {}

  default void setAngleInit(double newAngle) {}

  default boolean isInitSet() {
    return false;
  }

  default void updateInitSet(boolean set) {}

  default Angle getHoodAngle() {
    return null;
  }

  default void setTurretMotorControl(VoltageOut volts) {}

  @AutoLog
  class TurretIOInputs {
    public boolean connected = false;

    // turret motor
    public double turretPosition = 0.0;
    public double turretVelocity = 0.0;
    public double turretTorqueCurrent = 0.0;
    public double turretVoltage = 0.0;
    public double turretStatorCurrent = 0.0;
    public double turretSupplyCurrent = 0.0;
    public double turretTemp = 0.0;

    // hood motor
    public double hoodPosition = 0.0;
    public double hoodVelocity = 0.0;
    public double hoodTorqueCurrent = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodStatorCurrent = 0.0;
    public double hoodSupplyCurrent = 0.0;
    public double hoodTemp = 0.0;
  }
}
