package frc.robot.subsystems.hood;

import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void updateInputs(HoodIOInputs inputs) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default void stopHood() {}

  default double getHoodPos() {
    return 0.0;
  }

  default void setHoodPos(double pos) {}

  default void setMicroseconds(int max) {}

  @AutoLog
  class HoodIOInputs {
    // public boolean connected = false;

    // hood motor
    public double positionEstimate = 0.0;
    // public double angle = 0.0;
    public double positionSetpoint = 0.0;
    // public double angleSetpoint = 0.0;

    // public double velocity = 0.0;
    // // public double torqueCurrent = 0.0;
    // // public double voltage = 0.0;
    // // public double statorCurrent = 0.0;
    // // public double supplyCurrent = 0.0;
    // // public double temp = 0.0;
    // // public double setpoint = 0.0;
  }
}
