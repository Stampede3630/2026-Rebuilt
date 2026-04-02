package frc.robot.subsystems.flips;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlipsIOTalonFX implements FlipsIO {
  // left (climber) side of robot
  private final TalonFX flipLeft;
  // right side of robot
  private final TalonFX flipRight;
  // private final CANcoder encoder;

  private final Debouncer flipLeftConnDebouncer = new Debouncer(0.5);
  private final Debouncer flipRightConnDebouncer = new Debouncer(0.5);

  private Debouncer flipLeftStallDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private Debouncer flipRightStallDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private Debouncer flipLeftGoingDownDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private Debouncer flipRightGoingDownDebouncer = new Debouncer(0.5, DebounceType.kRising);

  // intake motor
  // private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  private final TalonFXConfiguration flipConfig = new TalonFXConfiguration();

  // flipLeft motor
  private final StatusSignal<Angle> flipLeftPosition;
  private final StatusSignal<AngularVelocity> flipLeftVelocity;
  private final StatusSignal<Current> flipLeftTorqueCurrent;
  private final StatusSignal<Voltage> flipLeftVoltage;
  private final StatusSignal<Current> flipLeftStatorCurrent;
  private final StatusSignal<Current> flipLeftSupplyCurrent;
  private final StatusSignal<Temperature> flipLeftTemp;
  private final StatusSignal<Double> flipLeftSetpoint;

  // flipRight motor
  private final StatusSignal<Angle> flipRightPosition;
  private final StatusSignal<AngularVelocity> flipRightVelocity;
  private final StatusSignal<Current> flipRightTorqueCurrent;
  private final StatusSignal<Voltage> flipRightVoltage;
  private final StatusSignal<Current> flipRightStatorCurrent;
  private final StatusSignal<Current> flipRightSupplyCurrent;
  private final StatusSignal<Temperature> flipRightTemp;
  private final StatusSignal<Double> flipRightSetpoint;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private Angle setpoint = Rotations.of(0.0);

  public FlipsIOTalonFX() {

    // init flipLeft motor
    flipLeft = new TalonFX(Constants.INTAKE_FLIP_LEFT_ID, Constants.SWERVE_BUS);
    flipLeftPosition = flipLeft.getPosition();
    flipLeftVelocity = flipLeft.getVelocity();
    flipLeftTorqueCurrent = flipLeft.getTorqueCurrent();
    flipLeftVoltage = flipLeft.getMotorVoltage();
    flipLeftStatorCurrent = flipLeft.getStatorCurrent();
    flipLeftSupplyCurrent = flipLeft.getSupplyCurrent();
    flipLeftTemp = flipLeft.getDeviceTemp();
    flipLeftSetpoint = flipLeft.getClosedLoopReference();

    // init flipRight motor
    flipRight = new TalonFX(Constants.INTAKE_FLIP_RIGHT_ID, Constants.SWERVE_BUS);
    flipRightPosition = flipRight.getPosition();
    flipRightVelocity = flipRight.getVelocity();
    flipRightTorqueCurrent = flipRight.getTorqueCurrent();
    flipRightVoltage = flipRight.getMotorVoltage();
    flipRightStatorCurrent = flipRight.getStatorCurrent();
    flipRightSupplyCurrent = flipRight.getSupplyCurrent();
    flipRightTemp = flipRight.getDeviceTemp();
    flipRightSetpoint = flipRight.getClosedLoopReference();

    flipConfig
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(20.0)) //  15.899412
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
                .withReverseSoftLimitThreshold(0.05))
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(50));
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
  public void updateInputs(FlipsIOInputs inputs) {
    boolean leftFlipConnected =
        BaseStatusSignal.refreshAll(
                flipLeftPosition,
                flipLeftVelocity,
                flipLeftTorqueCurrent,
                flipLeftVoltage,
                flipLeftStatorCurrent,
                flipLeftSupplyCurrent,
                flipLeftTemp,
                flipLeftSetpoint)
            .isOK();
    boolean rightFlipConnected =
        BaseStatusSignal.refreshAll(
                flipRightPosition,
                flipRightVelocity,
                flipRightTorqueCurrent,
                flipRightVoltage,
                flipRightStatorCurrent,
                flipRightSupplyCurrent,
                flipRightTemp,
                flipRightSetpoint)
            .isOK();
    inputs.flipLeftConnected = flipLeftConnDebouncer.calculate(leftFlipConnected);
    inputs.flipRightConnected = flipRightConnDebouncer.calculate(rightFlipConnected);

    // flipLeft
    inputs.flipLeftPosition = flipLeftPosition.getValue();
    inputs.flipLeftVelocity = flipLeftVelocity.getValue();
    inputs.flipLeftTorqueCurrent = flipLeftTorqueCurrent.getValue();
    inputs.flipLeftVoltage = flipLeftVoltage.getValue();
    inputs.flipLeftStatorCurrent = flipLeftStatorCurrent.getValue();
    inputs.flipLeftSupplyCurrent = flipLeftSupplyCurrent.getValue();
    inputs.flipLeftTemp = flipLeftTemp.getValue();
    inputs.flipLeftSetpoint = flipLeftSetpoint.getValue();
    inputs.flipLeftStalling =
        flipLeftStallDebouncer.calculate(inputs.flipLeftStatorCurrent.abs(Amps) > 10);

    // flipRight
    inputs.flipRightPosition = flipRightPosition.getValue();
    inputs.flipRightVelocity = flipRightVelocity.getValue();
    inputs.flipRightTorqueCurrent = flipRightTorqueCurrent.getValue();
    inputs.flipRightVoltage = flipRightVoltage.getValue();
    inputs.flipRightStatorCurrent = flipRightStatorCurrent.getValue();
    inputs.flipRightSupplyCurrent = flipRightSupplyCurrent.getValue();
    inputs.flipRightTemp = flipRightTemp.getValue();
    inputs.flipRightSetpoint = flipRightSetpoint.getValue();
    inputs.flipRightStalling =
        flipRightStallDebouncer.calculate(inputs.flipRightStatorCurrent.abs(Amps) > 10);

    inputs.flipSetpoint = setpoint;

    if (inputs.flipLeftStalling
        && flipLeftGoingDownDebouncer.calculate(
            inputs.flipLeftSetpoint
                < inputs.flipLeftPosition.in(
                    Rotations))) { // if stalling and going down, then turn off
      flipLeft.stopMotor();
    }
    if (inputs.flipRightStalling
        && flipRightGoingDownDebouncer.calculate(
            inputs.flipRightSetpoint
                < inputs.flipRightPosition.in(
                    Rotations))) { // if stalling and going down, then turn off
      flipRight.stopMotor();
    }
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  @Override
  public void resetFlipPosition(Angle pos) {
    flipLeft.setPosition(pos);
    flipRight.setPosition(pos);
  }

  @Override
  public void setFlipPosition(Angle pos) {
    flipLeft.setControl(positionRequest.withPosition(pos));
    flipRight.setControl(positionRequest.withPosition(pos));
    setpoint = pos;
  }

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
