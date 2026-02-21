package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final MotionMagicExpoVoltage positionRequest =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true);

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  private double elevSetpoint = 0.0;
  private double hookSetpoint = 0.0;

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

    elevatorConfig
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2)
                .withKG(0)) /* set upwards PID */
        .withSlot1(
            new Slot1Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2)
                .withKG(0)) /* set upwards PID */
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(0.0)
                .withMotionMagicExpo_kV(0.0)
                .withMotionMagicAcceleration(0.0)
                .withMotionMagicCruiseVelocity(0.0))
        .withSoftwareLimitSwitch(
            /* disabled for now */ new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(30 /* 30 rot */)
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(0 /* 0 rot */)); /* set PID */
    elevator.getConfigurator().apply(elevatorConfig);
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

    hookConfig
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2)); /* set PID */
    hook.getConfigurator().apply(hookConfig);
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
    inputs.elevatorSetpoint = elevSetpoint;

    // hook
    inputs.hookPosition = hookPosition.getValueAsDouble();
    inputs.hookVelocity = hookVelocity.getValueAsDouble();
    inputs.hookTorqueCurrent = hookTorqueCurrent.getValueAsDouble();
    inputs.hookVoltage = hookVoltage.getValueAsDouble();
    inputs.hookStatorCurrent = hookStatorCurrent.getValueAsDouble();
    inputs.hookSupplyCurrent = hookSupplyCurrent.getValueAsDouble();
    inputs.hookTemp = hookTemp.getValueAsDouble();
    inputs.hookSetpoint = hookSetpoint;
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

  /**
   * Sets the elevator to target a position
   *
   * @param pos The position to target, in rotations
   * @param slot The config slot to use (0: up, 1: down)
   */
  @Override
  public void runElevPos(double pos, int slot) {
    elevSetpoint = pos;
    elevator.setControl(positionRequest.withPosition(pos).withSlot(slot));
  }

  @Override
  public boolean setElevCoastMode(boolean enabled) {
    return elevator
        .setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake)
        .isOK();
  }

  @Override
  public void runHookPos(double pos) {
    hookSetpoint = pos;
    hook.setControl(positionRequest.withPosition(pos).withSlot(0));
  }
}
