package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
  // left (climber) side of robot
  private final TalonFX flipLeft;
  // right side of robot
  private final TalonFX flipRight;
  private final TalonFX intake;
  // private final CANcoder encoder;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // intake motor
  // private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeTorqueCurrent;
  private final StatusSignal<Voltage> intakeVoltage;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Temperature> intakeTemp;

  private final TalonFXConfiguration flipConfig = new TalonFXConfiguration();

  // flipLeft motor
  private final StatusSignal<Angle> flipLeftPosition;
  private final StatusSignal<AngularVelocity> flipLeftVelocity;
  private final StatusSignal<Current> flipLeftTorqueCurrent;
  private final StatusSignal<Voltage> flipLeftVoltage;
  private final StatusSignal<Current> flipLeftStatorCurrent;
  private final StatusSignal<Current> flipLeftSupplyCurrent;
  private final StatusSignal<Temperature> flipLeftTemp;

  // flipRight motor
  private final StatusSignal<Angle> flipRightPosition;
  private final StatusSignal<AngularVelocity> flipRightVelocity;
  private final StatusSignal<Current> flipRightTorqueCurrent;
  private final StatusSignal<Voltage> flipRightVoltage;
  private final StatusSignal<Current> flipRightStatorCurrent;
  private final StatusSignal<Current> flipRightSupplyCurrent;
  private final StatusSignal<Temperature> flipRightTemp;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);

  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

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
    // add intakeConfig here

    // init flipLeft motor
    flipLeft = new TalonFX(Constants.INTAKE_FLIP_LEFT_ID, Constants.SWERVE_BUS);
    flipLeftPosition = flipLeft.getPosition();
    flipLeftVelocity = flipLeft.getVelocity();
    flipLeftTorqueCurrent = flipLeft.getTorqueCurrent();
    flipLeftVoltage = flipLeft.getMotorVoltage();
    flipLeftStatorCurrent = flipLeft.getStatorCurrent();
    flipLeftSupplyCurrent = flipLeft.getSupplyCurrent();
    flipLeftTemp = flipLeft.getDeviceTemp();

    // init flipRight motor
    flipRight = new TalonFX(Constants.INTAKE_FLIP_RIGHT_ID, Constants.SWERVE_BUS);
    flipRightPosition = flipRight.getPosition();
    flipRightVelocity = flipRight.getVelocity();
    flipRightTorqueCurrent = flipRight.getTorqueCurrent();
    flipRightVoltage = flipRight.getMotorVoltage();
    flipRightStatorCurrent = flipRight.getStatorCurrent();
    flipRightSupplyCurrent = flipRight.getSupplyCurrent();
    flipRightTemp = flipRight.getDeviceTemp();

    flipConfig
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(23.0)) //  15.899412
        .withSlot0(
            new Slot0Configs()
                .withKP(20.0)
                .withKS(5.0)
                .withKG(0.72) // 0.72
                .withKV(2.25) // 2.25
                .withKA(0.19) // 0.19
                .withGravityType(GravityTypeValue.Arm_Cosine))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(0.25)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0.05));
    flipLeft.getConfigurator().apply(flipConfig);
    flipRight
        .getConfigurator()
        .apply(
            flipConfig.withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive)));

    // flipLeft.setPosition(0.337402);
    // flipRight.setControl(new Follower(Constants.INTAKE_FLIP_LEFT_ID,
    // MotorAlignmentValue.Opposed));
    flipLeft.setPosition(Degrees.of(90));
    flipRight.setPosition(Degrees.of(90));
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
                flipLeftPosition,
                flipLeftVelocity,
                flipLeftTorqueCurrent,
                flipLeftVoltage,
                flipLeftStatorCurrent,
                flipLeftSupplyCurrent,
                flipLeftTemp,
                flipRightPosition,
                flipRightVelocity,
                flipRightTorqueCurrent,
                flipRightVoltage,
                flipRightStatorCurrent,
                flipRightSupplyCurrent,
                flipRightTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // intake
    inputs.intakePosition = intakePosition.getValueAsDouble();
    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeTorqueCurrent = intakeTorqueCurrent.getValueAsDouble();
    inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValue();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();
    inputs.intakeTemp = intakeTemp.getValueAsDouble();

    // flipLeft
    inputs.flipLeftPosition = flipLeftPosition.getValueAsDouble();
    inputs.flipLeftVelocity = flipLeftVelocity.getValue();
    inputs.flipLeftTorqueCurrent = flipLeftTorqueCurrent.getValueAsDouble();
    inputs.flipLeftVoltage = flipLeftVoltage.getValueAsDouble();
    inputs.flipLeftStatorCurrent = flipLeftStatorCurrent.getValue();
    inputs.flipLeftSupplyCurrent = flipLeftSupplyCurrent.getValue();
    inputs.flipLeftTemp = flipLeftTemp.getValueAsDouble();

    // flipRight
    inputs.flipRightPosition = flipRightPosition.getValueAsDouble();
    inputs.flipRightVelocity = flipRightVelocity.getValue();
    inputs.flipRightTorqueCurrent = flipRightTorqueCurrent.getValueAsDouble();
    inputs.flipRightVoltage = flipRightVoltage.getValueAsDouble();
    inputs.flipRightStatorCurrent = flipRightStatorCurrent.getValue();
    inputs.flipRightSupplyCurrent = flipRightSupplyCurrent.getValue();
    inputs.flipRightTemp = flipRightTemp.getValueAsDouble();
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
    flipLeft.set(dutyCycle);
  }

  @Override
  public void stopFlip() {
    flipLeft.stopMotor();
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

  @Override
  public void resetFlipPosition(Angle pos) {
    flipLeft.setPosition(pos);
    flipRight.setPosition(pos);
  }

  @Override
  public void setFlipPosition(Angle pos) {
    flipLeft.setControl(positionRequest.withPosition(pos));
    flipRight.setControl(positionRequest.withPosition(pos));
  }

  // @Override
  // public void runFlipCurrent(Current current) {
  //   flipLeft.setControl(torqueRequest.withOutput(current));
  // }

  @Override
  public void runFlipsVoltage(Voltage volts) {
    flipLeft.setControl(voltageRequest.withOutput(volts));
    flipRight.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stopFlips() {
    flipLeft.stopMotor();
    flipRight.stopMotor();
  }

  // @Override
  // public double getShooterSpeed() {
  //   return intake.getVelocity().getValueAsDouble() * Intake.WHEEL_RADIUS_METERS;
  // }

  // @Override
  // public void setShooterMotorsControl(VoltageOut volts) {
  //   intake.setControl(volts);
  //   flipLeft.setControl(volts);
  // }
}
