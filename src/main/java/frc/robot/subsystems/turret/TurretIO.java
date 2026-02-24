package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void stopTurret() {}

  default void setTurretAngle(Angle angle) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  /**
   * @return The current angle of the turret, in robot-relative coordinates
   */
  default Angle getTurretAngle() {
    return Radians.of(0);
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

  /**
   * @return The current angular velocity of the turret
   */
  default AngularVelocity getAngularVelocity() {
    return RadiansPerSecond.of(0.0);
  }

  default void setTurretAngleTorqueCurrent(Angle angle) {}

  @AutoLog
  class TurretIOInputs {
    public boolean connected = false;

    // turret motor
    public double position = 0.0;
    public double velocity = 0.0;
    public double torqueCurrent = 0.0;
    public double voltage = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double temp = 0.0;
    public double setpoint = 0.0;
  }
}
