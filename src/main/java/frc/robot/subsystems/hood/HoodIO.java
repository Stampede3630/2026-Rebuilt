package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void updateInputs(HoodIOInputs inputs) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default void setHoodAngle(Angle angle) {}

  /**
   * @return The current angle of the turret's hood
   */
  default Angle getHoodAngle() {
    return Radians.of(0);
  }

  default void resetAnglePos(Angle newAngle) {}

  /**
   * A method to be used in case turret auto aim is disabled
   *
   * @param speed The speed to turn at [-1, 1]
   */
  default void runHood(double speed) {}

  default void stopHood() {}

  default double getHoodPos() {
    return 0.0;
  }

  default void setHoodPos(double pos) {}

  @AutoLog
  class HoodIOInputs {
    // public boolean connected = false;

    // hood motor
    public double position = 0.0;
    public double angle = 0.0;
    public double positionSetpoint = 0.0;
    public double angleSetpoint = 0.0;

    // public double velocity = 0.0;
    // // public double torqueCurrent = 0.0;
    // // public double voltage = 0.0;
    // // public double statorCurrent = 0.0;
    // // public double supplyCurrent = 0.0;
    // // public double temp = 0.0;
    // // public double setpoint = 0.0;
  }
}
