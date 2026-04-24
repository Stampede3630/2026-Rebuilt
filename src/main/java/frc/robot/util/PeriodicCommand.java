package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PeriodicCommand extends FunctionalCommand {
  private double lastTime = 0.0;
  private double periodSec;

  public PeriodicCommand(double periodSec, Runnable run, Subsystem... requirements) {
    super(
        () -> {},
        () -> {
          // if (isReady()) {

          // };
        },
        interrupted -> {},
        () -> false,
        requirements);
  }

  public boolean isReady() {
    return Timer.getFPGATimestamp() + periodSec > lastTime;
  }
}
