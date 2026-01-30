package frc.robot.subsystems.intake;

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

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX flip;
  private final TalonFX intake;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // intake motor
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeTorqueCurrent;
  private final StatusSignal<Voltage> intakeVoltage;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Temperature> intakeTemp;

  // flip motor
  private final TalonFXConfiguration flipConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> flipPosition;
  private final StatusSignal<AngularVelocity> flipVelocity;
  private final StatusSignal<Current> flipTorqueCurrent;
  private final StatusSignal<Voltage> flipVoltage;
  private final StatusSignal<Current> flipStatorCurrent;
  private final StatusSignal<Current> flipSupplyCurrent;
  private final StatusSignal<Temperature> flipTemp;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public IntakeIOTalonFX() {
    // init intake motor
    intake = new TalonFX(Constants.INTAKE_ID);
    intakePosition = intake.getPosition();
    intakeVelocity = intake.getVelocity();
    intakeTorqueCurrent = intake.getTorqueCurrent();
    intakeVoltage = intake.getMotorVoltage();
    intakeStatorCurrent = intake.getStatorCurrent();
    intakeSupplyCurrent = intake.getSupplyCurrent();
    intakeTemp = intake.getDeviceTemp();
    // add intakeConfig here

    // init flip motor
    flip = new TalonFX(Constants.INTAKE_FLIP_ID);
    flipPosition = flip.getPosition();
    flipVelocity = flip.getVelocity();
    flipTorqueCurrent = flip.getTorqueCurrent();
    flipVoltage = flip.getMotorVoltage();
    flipStatorCurrent = flip.getStatorCurrent();
    flipSupplyCurrent = flip.getSupplyCurrent();
    flipTemp = flip.getDeviceTemp();
    // add flipConfig here

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                intakePosition,
                intakeVelocity,
                intakeTorqueCurrent,
                intakeVoltage,
                intakeStatorCurrent,
                intakeSupplyCurrent,
                intakeTemp,
                flipPosition,
                flipVelocity,
                flipTorqueCurrent,
                flipVoltage,
                flipStatorCurrent,
                flipSupplyCurrent,
                flipTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // intake
    inputs.intakePosition = intakePosition.getValueAsDouble();
    inputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    inputs.intakeTorqueCurrent = intakeTorqueCurrent.getValueAsDouble();
    inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeTemp = intakeTemp.getValueAsDouble();

    // flip
    inputs.flipPosition = flipPosition.getValueAsDouble();
    inputs.flipVelocity = flipVelocity.getValueAsDouble();
    inputs.flipTorqueCurrent = flipTorqueCurrent.getValueAsDouble();
    inputs.flipVoltage = flipVoltage.getValueAsDouble();
    inputs.flipStatorCurrent = flipStatorCurrent.getValueAsDouble();
    inputs.flipSupplyCurrent = flipSupplyCurrent.getValueAsDouble();
    inputs.flipTemp = flipTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  // @Override
  // public void runVelocity(double vel) {
  //   intake.setControl(velocityRequest.withVelocity(vel));
  // }

  @Override
  public void runDutyCycleFlip(double dutyCycle) {
    flip.set(dutyCycle);
  }

  @Override
  public void stopFlip() {
    flip.stopMotor();
  }

  @Override
  public void runDutyCycle(double dutyCycle) {
    intake.set(dutyCycle);
  }

  @Override
  public void stop() {
    intake.stopMotor();
  }

  @Override
  public boolean isRunning() {
    return intake.getVelocity().getValueAsDouble() > 0;
  }

  // @Override
  // public double getShooterSpeed() {
  //   return intake.getVelocity().getValueAsDouble() * Intake.WHEEL_RADIUS_METERS;
  // }

  // @Override
  // public void setShooterMotorsControl(VoltageOut volts) {
  //   intake.setControl(volts);
  //   flip.setControl(volts);
  // }
}
