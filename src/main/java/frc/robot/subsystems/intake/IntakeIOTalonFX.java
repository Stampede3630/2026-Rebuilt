package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intake;
  // private final CANcoder encoder;

  private final Debouncer intakeConnDebouncer = new Debouncer(0.5);

  // intake motor
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeTorqueCurrent;
  private final StatusSignal<Voltage> intakeVoltage;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Temperature> intakeTemp;
  private final StatusSignal<Double> intakeSetpoint;

  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double intakeDutyCycle = 0.0;

  public IntakeIOTalonFX() {
    // init intake motor
    intake = new TalonFX(Constants.INTAKE_ID, Constants.SWERVE_BUS);
    intakePosition = intake.getPosition();
    intakeVelocity = intake.getVelocity();
    intakeTorqueCurrent = intake.getTorqueCurrent();
    intakeVoltage = intake.getMotorVoltage();
    intakeStatorCurrent = intake.getStatorCurrent();
    intakeSupplyCurrent = intake.getSupplyCurrent();
    intakeTemp = intake.getDeviceTemp();
    intakeSetpoint = intake.getClosedLoopReference();
    intakeConfig
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(new Slot0Configs().withKP(0.1).withKS(0.35).withKV(0.12));
    intake.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean intakeConnected =
        BaseStatusSignal.refreshAll(
                intakePosition,
                intakeVelocity,
                intakeTorqueCurrent,
                intakeVoltage,
                intakeStatorCurrent,
                intakeSupplyCurrent,
                intakeTemp,
                intakeSetpoint)
            .isOK();

    inputs.intakeConnected = intakeConnDebouncer.calculate(intakeConnected);

    // intake
    inputs.intakePosition = intakePosition.getValue();
    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeTorqueCurrent = intakeTorqueCurrent.getValue();
    inputs.intakeVoltage = intakeVoltage.getValue();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValue();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();
    inputs.intakeTemp = intakeTemp.getValue();
    inputs.intakeSetpoint = intakeSetpoint.getValue();

    inputs.intakeDutyCycle = intakeDutyCycle;
  }

  @Override
  public void runVelocity(AngularVelocity vel) {
    intake.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public void runDutyCycle(double dutyCycle) {
    intake.set(dutyCycle);
    intakeDutyCycle = dutyCycle;
  }

  @Override
  public void stop() {
    intake.stopMotor();
    intakeDutyCycle = 0.0;
  }
}
