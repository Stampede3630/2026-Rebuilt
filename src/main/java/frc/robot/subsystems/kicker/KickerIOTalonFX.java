package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class KickerIOTalonFX implements KickerIO {

  private final TalonFX kicker;
  // private final CANcoder encoder;

  private final Debouncer kickerConnDebouncer = new Debouncer(0.5);

  // kicker motor
  private final StatusSignal<Angle> kickerPosition;
  private final StatusSignal<AngularVelocity> kickerVelocity;
  private final StatusSignal<Current> kickerTorqueCurrent;
  private final StatusSignal<Voltage> kickerVoltage;
  private final StatusSignal<Current> kickerStatorCurrent;
  private final StatusSignal<Current> kickerSupplyCurrent;
  private final StatusSignal<Temperature> kickerTemp;
  private final StatusSignal<Double> kickerSetpoint;

  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double kickerDutyCycle = 0.0;

  public KickerIOTalonFX() {
    // init kicker motor
    kicker = new TalonFX(Constants.V2_KICK_ID, Constants.SWERVE_BUS);
    kickerPosition = kicker.getPosition();
    kickerVelocity = kicker.getVelocity();
    kickerTorqueCurrent = kicker.getTorqueCurrent();
    kickerVoltage = kicker.getMotorVoltage();
    kickerStatorCurrent = kicker.getStatorCurrent();
    kickerSupplyCurrent = kicker.getSupplyCurrent();
    kickerTemp = kicker.getDeviceTemp();
    kickerSetpoint = kicker.getClosedLoopReference();
    kickerConfig.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));
    kicker.getConfigurator().apply(kickerConfig);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    boolean kickerConnected =
        BaseStatusSignal.refreshAll(
                kickerPosition,
                kickerVelocity,
                kickerTorqueCurrent,
                kickerVoltage,
                kickerStatorCurrent,
                kickerSupplyCurrent,
                kickerTemp,
                kickerSetpoint)
            .isOK();

    inputs.kickerConnected = kickerConnDebouncer.calculate(kickerConnected);

    // kicker
    inputs.kickerPosition = kickerPosition.getValue();
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.kickerTorqueCurrent = kickerTorqueCurrent.getValue();
    inputs.kickerVoltage = kickerVoltage.getValue();
    inputs.kickerStatorCurrent = kickerStatorCurrent.getValue();
    inputs.kickerSupplyCurrent = kickerSupplyCurrent.getValue();
    inputs.kickerTemp = kickerTemp.getValue();
    inputs.kickerSetpoint = kickerSetpoint.getValue();

    inputs.kickerDutyCycle = kickerDutyCycle;
  }

  @Override
  public void runVelocity(AngularVelocity vel) {
    kicker.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public void runDutyCycle(double dutyCycle) {
    kicker.set(dutyCycle);
    kickerDutyCycle = dutyCycle;
  }

  @Override
  public void stop() {
    kicker.stopMotor();
    kickerDutyCycle = 0.0;
  }
}
