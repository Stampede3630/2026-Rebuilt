package frc.robot.subsystems.toftimer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.toftimer.TofTimer.Shot.ShotState;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class TofTimer extends SubsystemBase {
  private TofTimerIO io;

  private final TofTimerIOInputsAutoLogged inputs = new TofTimerIOInputsAutoLogged();
  private State state = State.READY;
  private Queue<Shot> activeShots = new LinkedList<>();
  private Queue<Shot> finishedShots = new LinkedList<>();

  public Queue<Shot> getActiveShots() {
    return activeShots;
  }

  public Queue<Shot> getFinishedShots() {
    return finishedShots;
  }

  private double timeoutThreshold = 100000;

  public TofTimer(TofTimerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    boolean hubTriggered = inputs.hubTriggered;
    boolean ballShot = inputs.ballShot;
    Logger.processInputs("TofTimer", inputs);
    if (activeShots.peek() != null) {
      Logger.recordOutput(
          "TofTimer/latestTime", Timer.getFPGATimestamp() - activeShots.peek().shotTime);
    }
    Logger.recordOutput("TofTimer/state", state);
    if (state == State.READY) {
      if (hubTriggered
          && activeShots.size() == 0) { // if hub is triggered but there are no shots in progress
        state = State.NOT_READY;
        return;
      }

      if (ballShot) {
        activeShots.add(new Shot(Timer.getFPGATimestamp()));
      }

      if (hubTriggered) {
        Shot landed = activeShots.poll();
        if (landed != null) {
          landed.landTime = Timer.getFPGATimestamp();
          landed.state = ShotState.LANDED;
          finishedShots.add(landed);
        }
      }
      activeShots.removeIf(
          s -> {
            if (Timer.getFPGATimestamp() - s.shotTime > timeoutThreshold) {
              s.state = ShotState.TIMED_OUT;
              return true;
            }
            return false;
          });
    } else {
      if (!hubTriggered
          && activeShots.size() == 0) { // if hub is ready and there are no shots in progress
        state = State.READY;
        return;
      }
    }
  }

  public class Shot {
    private double shotTime;
    private double landTime;
    private ShotState state;

    public enum ShotState {
      SHOT,
      LANDED,
      TIMED_OUT
    }

    Shot(double shotTime) {
      this.shotTime = shotTime;
      state = ShotState.SHOT;
    }
  }

  public enum State {
    READY,
    NOT_READY
  }
}
