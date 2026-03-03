package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15.899412))
        .withSlot0(new Slot0Configs().withKS(3));
    flipLeft.getConfigurator().apply(flipConfig);
    flipRight.getConfigurator().apply(flipConfig);

    // flipLeft.setPosition(0.337402);
    flipRight.setControl(new Follower(Constants.INTAKE_FLIP_LEFT_ID, MotorAlignmentValue.Opposed));
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
    inputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    inputs.intakeTorqueCurrent = intakeTorqueCurrent.getValueAsDouble();
    inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeTemp = intakeTemp.getValueAsDouble();

    // flipLeft
    inputs.flipLeftPosition = flipLeftPosition.getValueAsDouble();
    inputs.flipLeftVelocity = flipLeftVelocity.getValueAsDouble();
    inputs.flipLeftTorqueCurrent = flipLeftTorqueCurrent.getValueAsDouble();
    inputs.flipLeftVoltage = flipLeftVoltage.getValueAsDouble();
    inputs.flipLeftStatorCurrent = flipLeftStatorCurrent.getValueAsDouble();
    inputs.flipLeftSupplyCurrent = flipLeftSupplyCurrent.getValueAsDouble();
    inputs.flipLeftTemp = flipLeftTemp.getValueAsDouble();

    // flipRight
    inputs.flipRightPosition = flipRightPosition.getValueAsDouble();
    inputs.flipRightVelocity = flipRightVelocity.getValueAsDouble();
    inputs.flipRightTorqueCurrent = flipRightTorqueCurrent.getValueAsDouble();
    inputs.flipRightVoltage = flipRightVoltage.getValueAsDouble();
    inputs.flipRightStatorCurrent = flipRightStatorCurrent.getValueAsDouble();
    inputs.flipRightSupplyCurrent = flipRightSupplyCurrent.getValueAsDouble();
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
  public void setFlipPosition(Angle pos) {
    intake.setControl(new PositionVoltage(pos));
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
