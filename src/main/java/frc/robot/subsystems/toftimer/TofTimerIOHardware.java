package frc.robot.subsystems.toftimer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class TofTimerIOHardware implements TofTimerIO {
  private DigitalInput hubInput;
  private CANrange canRange;
  private final Debouncer connDebouncer = new Debouncer(0.5);
  private StatusSignal<Boolean> ballShotSignal;

  public TofTimerIOHardware() {
    hubInput = new DigitalInput(Constants.HUB_PORT);
    canRange = new CANrange(Constants.SHOOTER_CANRANGE);
    ballShotSignal = canRange.getIsDetected();
  }

  @Override
  public void updateInputs(TofTimerIOInputs inputs) {
    boolean connected = BaseStatusSignal.refreshAll(ballShotSignal).isOK();
    inputs.ballShot = ballShotSignal.getValue();
    inputs.connected = connDebouncer.calculate(connected);
    inputs.hubTriggered = hubInput.get();
  }
}
