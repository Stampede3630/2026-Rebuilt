package frc.robot.subsystems.climber;

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

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX left;
  private final TalonFX right;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // right motor
  private final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Voltage> rightVoltage;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Temperature> rightTemp;

  // left motor
  private final TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Voltage> leftVoltage;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Temperature> leftTemp;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ClimberIOTalonFX() {
    // init right motor
    right = new TalonFX(Constants.CLIMBER_RIGHT_ID);
    rightPosition = right.getPosition();
    rightVelocity = right.getVelocity();
    rightTorqueCurrent = right.getTorqueCurrent();
    rightVoltage = right.getMotorVoltage();
    rightStatorCurrent = right.getStatorCurrent();
    rightSupplyCurrent = right.getSupplyCurrent();
    rightTemp = right.getDeviceTemp();
    // add rightConfig here

    // init left motor
    left = new TalonFX(Constants.CLIMBER_LEFT_ID);
    leftPosition = left.getPosition();
    leftVelocity = left.getVelocity();
    leftTorqueCurrent = left.getTorqueCurrent();
    leftVoltage = left.getMotorVoltage();
    leftStatorCurrent = left.getStatorCurrent();
    leftSupplyCurrent = left.getSupplyCurrent();
    leftTemp = left.getDeviceTemp();
    // add leftConfig here

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightTorqueCurrent,
                rightVoltage,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightTemp,
                leftPosition,
                leftVelocity,
                leftTorqueCurrent,
                leftVoltage,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // right
    inputs.rightPosition = rightPosition.getValueAsDouble();
    inputs.rightVelocity = rightVelocity.getValueAsDouble();
    inputs.rightTorqueCurrent = rightTorqueCurrent.getValueAsDouble();
    inputs.rightVoltage = rightVoltage.getValueAsDouble();
    inputs.rightStatorCurrent = rightStatorCurrent.getValueAsDouble();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTemp = rightTemp.getValueAsDouble();

    // left
    inputs.leftPosition = leftPosition.getValueAsDouble();
    inputs.leftVelocity = leftVelocity.getValueAsDouble();
    inputs.leftTorqueCurrent = leftTorqueCurrent.getValueAsDouble();
    inputs.leftVoltage = leftVoltage.getValueAsDouble();
    inputs.leftStatorCurrent = leftStatorCurrent.getValueAsDouble();
    inputs.leftSupplyCurrent = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTemp = leftTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  // @Override
  // public void runVelocity(double vel) {
  //   right.setControl(velocityRequest.withVelocity(vel));
  // }

  @Override
  public void runDutyCycleLeft(double dutyCycle) {
    left.set(dutyCycle);
  }

  @Override
  public void stopLeft() {
    left.stopMotor();
  }

  @Override
  public void runDutyCycleRight(double dutyCycle) {
    right.set(dutyCycle);
  }

  @Override
  public void stopRight() {
    right.stopMotor();
  }

  // @Override
  // public double getShooterSpeed() {
  //   return right.getVelocity().getValueAsDouble() * right.WHEEL_RADIUS_METERS;
  // }

  // @Override
  // public void setShooterMotorsControl(VoltageOut volts) {
  //   right.setControl(volts);
  //   left.setControl(volts);
  // }
}
