package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class TimedSubsystem extends SubsystemBase {
  private final String epochName;

  public TimedSubsystem(String epochName) {
    this.epochName = epochName;
  }

  @Override
  public final void periodic() {
    double startTime = Timer.getFPGATimestamp();
    timedPeriodic();
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("PeriodicTimes/" + epochName + " ms", (now - startTime) * 1000.0);
  }

  public abstract void timedPeriodic();
}
