package frc.robot.subsystems.toftimer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.toftimer.TofTimer.Shot.ShotState;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class TofTimer extends SubsystemBase {
  private TofTimerIO io;

  private final TofTimerIOInputsAutoLogged inputs = new TofTimerIOInputsAutoLogged();
  private State state = State.READY;
  private Queue<Shot> activeShots = new LinkedList<>();
  private Queue<Shot> finishedShots = new LinkedList<>();

  private List<Consumer<Shot>> ballShotConsumers = new ArrayList<>();
  private List<Consumer<Shot>> ballLandedConsumers = new ArrayList<>();

  public void registerShotCallback(Consumer<Shot> callback) {
    if (ballShotConsumers.contains(callback)) {
      throw new IllegalArgumentException("Consumer already exists");
    }
    ballShotConsumers.add(callback);
  }

  public void registerLandedCallback(Consumer<Shot> callback) {
    if (ballLandedConsumers.contains(callback)) {
      throw new IllegalArgumentException("Consumer already exists");
    }
    ballLandedConsumers.add(callback);
  }

  public Queue<Shot> getActiveShots() {
    return activeShots;
  }

  public Queue<Shot> getFinishedShots() {
    return finishedShots;
  }

  private double timeoutThreshold = 30;

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
        Shot active = new Shot(Timer.getFPGATimestamp());
        activeShots.add(active);
        ballShotConsumers.forEach(c -> c.accept(active));
        state = State.NOT_READY;
      }

      if (hubTriggered) {
        Shot landed = activeShots.poll();
        if (landed != null) {
          landed.landTime = Timer.getFPGATimestamp();
          landed.state = ShotState.LANDED;
          finishedShots.add(landed);
          ballLandedConsumers.forEach(c -> c.accept(landed));
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
      // if hub is ready and there are no shots in progress OR if there is an active shot and the
      // sensor is ready again
      if (!hubTriggered && activeShots.size() == 0 || !ballShot && activeShots.size() > 0) {
        state = State.READY;
        return;
      }
    }
  }

  public class Shot {
    private double shotTime;
    private double landTime;

    public double getShotTime() {
      return shotTime;
    }

    public void setShotTime(double shotTime) {
      this.shotTime = shotTime;
    }

    public double getLandTime() {
      return landTime;
    }

    public void setLandTime(double landTime) {
      this.landTime = landTime;
    }

    public ShotState getState() {
      return state;
    }

    public void setState(ShotState state) {
      this.state = state;
    }

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

    public double getTof() {
      return landTime - shotTime;
    }
  }

  public enum State {
    READY,
    NOT_READY
  }
}
