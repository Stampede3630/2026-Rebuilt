package frc.robot.subsystems.toftimer;

import org.littletonrobotics.junction.AutoLog;

public interface TofTimerIO {

  default void updateInputs(TofTimerIOInputs inputs) {}

  @AutoLog
  class TofTimerIOInputs {
    public boolean connected = false;

    public boolean ballShot = false;
    public boolean hubTriggered = false;
  }
}
