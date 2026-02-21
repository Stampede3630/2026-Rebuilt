package frc.robot.subsystems.toftimer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class TofTimerIOHardware implements TofTimerIO {
  private DigitalInput hubInput;
  private CANrange canRange;
  private final Debouncer connDebouncer = new Debouncer(0.5);
  private StatusSignal<Boolean> ballShotSignal;
  private final CANrangeConfiguration configuration = new CANrangeConfiguration();

  public TofTimerIOHardware() {
    hubInput = new DigitalInput(Constants.HUB_PORT);
    canRange = new CANrange(Constants.SHOOTER_CANRANGE);
    configuration.withProximityParams(
        new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(1000)
            .withProximityHysteresis(0.001)
            .withProximityThreshold(0.1));
    canRange.getConfigurator().apply(configuration);
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
