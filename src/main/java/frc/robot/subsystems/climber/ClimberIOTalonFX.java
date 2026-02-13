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
  private final TalonFX hook;
  private final TalonFX elevator;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // elevator motor
  private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Current> elevatorTorqueCurrent;
  private final StatusSignal<Voltage> elevatorVoltage;
  private final StatusSignal<Current> elevatorStatorCurrent;
  private final StatusSignal<Current> elevatorSupplyCurrent;
  private final StatusSignal<Temperature> elevatorTemp;

  // hook motor
  private final TalonFXConfiguration hookConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> hookPosition;
  private final StatusSignal<AngularVelocity> hookVelocity;
  private final StatusSignal<Current> hookTorqueCurrent;
  private final StatusSignal<Voltage> hookVoltage;
  private final StatusSignal<Current> hookStatorCurrent;
  private final StatusSignal<Current> hookSupplyCurrent;
  private final StatusSignal<Temperature> hookTemp;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ClimberIOTalonFX() {
    // init elevator motor
    elevator = new TalonFX(Constants.CLIMBER_ELEVATOR_ID, Constants.SWERVE_BUS); // need to refactor
    elevatorPosition = elevator.getPosition();
    elevatorVelocity = elevator.getVelocity();
    elevatorTorqueCurrent = elevator.getTorqueCurrent();
    elevatorVoltage = elevator.getMotorVoltage();
    elevatorStatorCurrent = elevator.getStatorCurrent();
    elevatorSupplyCurrent = elevator.getSupplyCurrent();
    elevatorTemp = elevator.getDeviceTemp();
    // add elevatorConfig here

    // init hook motor
    hook = new TalonFX(Constants.CLIMBER_HOOK_ID, Constants.SWERVE_BUS);
    hookPosition = hook.getPosition();
    hookVelocity = hook.getVelocity();
    hookTorqueCurrent = hook.getTorqueCurrent();
    hookVoltage = hook.getMotorVoltage();
    hookStatorCurrent = hook.getStatorCurrent();
    hookSupplyCurrent = hook.getSupplyCurrent();
    hookTemp = hook.getDeviceTemp();
    // add hookConfig here

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                elevatorPosition,
                elevatorVelocity,
                elevatorTorqueCurrent,
                elevatorVoltage,
                elevatorStatorCurrent,
                elevatorSupplyCurrent,
                elevatorTemp,
                hookPosition,
                hookVelocity,
                hookTorqueCurrent,
                hookVoltage,
                hookStatorCurrent,
                hookSupplyCurrent,
                hookTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // elevator
    inputs.elevatorPosition = elevatorPosition.getValueAsDouble();
    inputs.elevatorVelocity = elevatorVelocity.getValueAsDouble();
    inputs.elevatorTorqueCurrent = elevatorTorqueCurrent.getValueAsDouble();
    inputs.elevatorVoltage = elevatorVoltage.getValueAsDouble();
    inputs.elevatorStatorCurrent = elevatorStatorCurrent.getValueAsDouble();
    inputs.elevatorSupplyCurrent = elevatorSupplyCurrent.getValueAsDouble();
    inputs.elevatorTemp = elevatorTemp.getValueAsDouble();

    // hook
    inputs.hookPosition = hookPosition.getValueAsDouble();
    inputs.hookVelocity = hookVelocity.getValueAsDouble();
    inputs.hookTorqueCurrent = hookTorqueCurrent.getValueAsDouble();
    inputs.hookVoltage = hookVoltage.getValueAsDouble();
    inputs.hookStatorCurrent = hookStatorCurrent.getValueAsDouble();
    inputs.hookSupplyCurrent = hookSupplyCurrent.getValueAsDouble();
    inputs.hookTemp = hookTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  // @Override
  // public void runVelocity(double vel) {
  //   elevator.setControl(velocityRequest.withVelocity(vel));
  // }

  @Override
  public void runDutyCycleHook(double dutyCycle) {
    hook.set(dutyCycle);
  }

  @Override
  public void stopHook() {
    hook.stopMotor();
  }

  @Override
  public void runDutyCycleElevator(double dutyCycle) {
    elevator.set(dutyCycle);
  }

  @Override
  public void stopElevator() {
    elevator.stopMotor();
  }

  // @Override
  // public double getShooterSpeed() {
  //   return elevator.getVelocity().getValueAsDouble() * elevator.WHEEL_RADIUS_METERS;
  // }

  // @Override
  // public void setShooterMotorsControl(VoltageOut volts) {
  //   elevator.setControl(volts);
  //   hook.setControl(volts);
  // }
}
