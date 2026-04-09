package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX spin;
  private final TalonFX chute;
  // private final CANcoder encoder;

  private final Debouncer spinConnDebouncer = new Debouncer(0.5);
  private final Debouncer chuteConnDebouncer = new Debouncer(0.5);

  // intake motor
  private final TalonFXConfiguration spinConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> spinPosition;
  private final StatusSignal<AngularVelocity> spinVelocity;
  private final StatusSignal<Current> spinTorqueCurrent;
  private final StatusSignal<Voltage> spinVoltage;
  private final StatusSignal<Current> spinStatorCurrent;
  private final StatusSignal<Current> spinSupplyCurrent;
  private final StatusSignal<Temperature> spinTemp;

  // chute motor
  private final TalonFXConfiguration chuteConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> chutePosition;
  private final StatusSignal<AngularVelocity> chuteVelocity;
  private final StatusSignal<Current> chuteTorqueCurrent;
  private final StatusSignal<Voltage> chuteVoltage;
  private final StatusSignal<Current> chuteStatorCurrent;
  private final StatusSignal<Current> chuteSupplyCurrent;
  private final StatusSignal<Temperature> chuteTemp;

  private double chuteDutyCycle = 0.0;
  private double spinDutyCycle = 0.0;

  public IndexerIOTalonFX() {
    // init spin motor
    spin = new TalonFX(Constants.INDEXER_SPIN_ID, Constants.SWERVE_BUS);
    spinPosition = spin.getPosition();
    spinVelocity = spin.getVelocity();
    spinTorqueCurrent = spin.getTorqueCurrent();
    spinVoltage = spin.getMotorVoltage();
    spinStatorCurrent = spin.getStatorCurrent();
    spinSupplyCurrent = spin.getSupplyCurrent();
    spinTemp = spin.getDeviceTemp();
    spinConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(100.0))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));
    spin.getConfigurator().apply(spinConfig);
    // add spinConfig here

    // init chute motor
    chute = new TalonFX(Constants.INDEXER_CHUTE_ID, Constants.SWERVE_BUS);
    chutePosition = chute.getPosition();
    chuteVelocity = chute.getVelocity();
    chuteTorqueCurrent = chute.getTorqueCurrent();
    chuteVoltage = chute.getMotorVoltage();
    chuteStatorCurrent = chute.getStatorCurrent();
    chuteSupplyCurrent = chute.getSupplyCurrent();
    chuteTemp = chute.getDeviceTemp();
    chuteConfig.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));
    chute.getConfigurator().apply(chuteConfig);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean spinConnected =
        BaseStatusSignal.refreshAll(
                spinPosition,
                spinVelocity,
                spinTorqueCurrent,
                spinVoltage,
                spinStatorCurrent,
                spinSupplyCurrent,
                spinTemp)
            .isOK();

    boolean chuteConnected =
        BaseStatusSignal.refreshAll(
                chutePosition,
                chuteVelocity,
                chuteTorqueCurrent,
                chuteVoltage,
                chuteStatorCurrent,
                chuteSupplyCurrent,
                chuteTemp)
            .isOK();

    inputs.spinConnected = spinConnDebouncer.calculate(spinConnected);
    inputs.chuteConnected = chuteConnDebouncer.calculate(chuteConnected);

    // spin
    inputs.spinPosition = spinPosition.getValue();
    inputs.spinVelocity = spinVelocity.getValue();
    inputs.spinTorqueCurrent = spinTorqueCurrent.getValue();
    inputs.spinVoltage = spinVoltage.getValue();
    inputs.spinStatorCurrent = spinStatorCurrent.getValue();
    inputs.spinSupplyCurrent = spinSupplyCurrent.getValue();
    inputs.spinTemp = spinTemp.getValue();

    // chute
    inputs.chutePosition = chutePosition.getValue();
    inputs.chuteVelocity = chuteVelocity.getValue();
    inputs.chuteTorqueCurrent = chuteTorqueCurrent.getValue();
    inputs.chuteVoltage = chuteVoltage.getValue();
    inputs.chuteStatorCurrent = chuteStatorCurrent.getValue();
    inputs.chuteSupplyCurrent = chuteSupplyCurrent.getValue();
    inputs.chuteTemp = chuteTemp.getValue();

    inputs.spinDutyCycle = spinDutyCycle;
    inputs.chuteDutyCycle = chuteDutyCycle;
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  // runVelocity(getValue(dist));
  // }

  // @Override
  // public void runVelocity(double vel) {
  // spin.setControl(velocityRequest.withVelocity(vel));
  // }

  @Override
  public void runDutyCycleChute(double dutyCycle) {
    chute.set(dutyCycle);
    chuteDutyCycle = dutyCycle;
  }

  @Override
  public void stopChute() {
    chute.stopMotor();
    chuteDutyCycle = 0.0;
  }

  @Override
  public void runDutyCycleSpin(double dutyCycle) {
    // System.out.println("spinning at " + dutyCycle);
    spin.set(dutyCycle);
    spinDutyCycle = dutyCycle;
  }

  @Override
  public void stopSpin() {
    spin.stopMotor();
    spinDutyCycle = 0.0;
  }
}
