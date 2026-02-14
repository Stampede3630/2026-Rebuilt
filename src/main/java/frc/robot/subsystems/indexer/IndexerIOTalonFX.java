package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private final Debouncer connDebouncer = new Debouncer(0.5);

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

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

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
    // add chuteConfig here

    // encoder = new CANcoder(Constants.INDEXER_ENCODER_ID);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                spinPosition,
                spinVelocity,
                spinTorqueCurrent,
                spinVoltage,
                spinStatorCurrent,
                spinSupplyCurrent,
                spinTemp,
                chutePosition,
                chuteVelocity,
                chuteTorqueCurrent,
                chuteVoltage,
                chuteStatorCurrent,
                chuteSupplyCurrent,
                chuteTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // spin
    inputs.spinPosition = spinPosition.getValueAsDouble();
    inputs.spinVelocity = spinVelocity.getValueAsDouble();
    inputs.spinTorqueCurrent = spinTorqueCurrent.getValueAsDouble();
    inputs.spinVoltage = spinVoltage.getValueAsDouble();
    inputs.spinStatorCurrent = spinStatorCurrent.getValueAsDouble();
    inputs.spinSupplyCurrent = spinSupplyCurrent.getValueAsDouble();
    inputs.spinTemp = spinTemp.getValueAsDouble();

    // chute
    inputs.chutePosition = chutePosition.getValueAsDouble();
    inputs.chuteVelocity = chuteVelocity.getValueAsDouble();
    inputs.chuteTorqueCurrent = chuteTorqueCurrent.getValueAsDouble();
    inputs.chuteVoltage = chuteVoltage.getValueAsDouble();
    inputs.chuteStatorCurrent = chuteStatorCurrent.getValueAsDouble();
    inputs.chuteSupplyCurrent = chuteSupplyCurrent.getValueAsDouble();
    inputs.chuteTemp = chuteTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  // @Override
  // public void runVelocity(double vel) {
  //   spin.setControl(velocityRequest.withVelocity(vel));
  // }

  @Override
  public void runDutyCycleChute(double dutyCycle) {
    chute.set(dutyCycle);
  }

  @Override
  public void stopChute() {
    chute.stopMotor();
  }

  @Override
  public void runDutyCycleSpin(double dutyCycle) {
    System.out.println("spinning at " + dutyCycle);
    spin.set(dutyCycle);
  }

  @Override
  public void stopSpin() {
    spin.stopMotor();
  }

  // @Override
  // public boolean isRunning() {
  //   return spin.getVelocity().getValueAsDouble() > 0;
  // }

  // @Override
  // public void setShooterMotorsControl(VoltageOut volts) {
  //   spin.setControl(volts);
  //   chute.setControl(volts);
  // }
}
