package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFXV2 implements ShooterIO {
  private final TalonFX topRightLeader;
  private final TalonFX bottomRight;
  private final TalonFX topLeft;
  private final TalonFX bottomLeft;

  // debounce connection seperately for each motor
  private final Debouncer topRightConnDebouncer = new Debouncer(0.5);
  private final Debouncer bottomRightConnDebouncer = new Debouncer(0.5);
  private final Debouncer topLeftConnDebouncer = new Debouncer(0.5);
  private final Debouncer bottomLeftConnDebouncer = new Debouncer(0.5);

  // motor configs - same for all for
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // topRight motor
  private final StatusSignal<Angle> topRightPosition;
  private final StatusSignal<AngularVelocity> topRightVelocity;
  private final StatusSignal<Current> topRightTorqueCurrent;
  private final StatusSignal<Voltage> topRightVoltage;
  private final StatusSignal<Current> topRightStatorCurrent;
  private final StatusSignal<Current> topRightSupplyCurrent;
  private final StatusSignal<Temperature> topRightTemp;

  // bottomRight motor
  private final StatusSignal<Angle> bottomRightPosition;
  private final StatusSignal<AngularVelocity> bottomRightVelocity;
  private final StatusSignal<Current> bottomRightTorqueCurrent;
  private final StatusSignal<Voltage> bottomRightVoltage;
  private final StatusSignal<Current> bottomRightStatorCurrent;
  private final StatusSignal<Current> bottomRightSupplyCurrent;
  private final StatusSignal<Temperature> bottomRightTemp;

  // topLeft motor
  private final StatusSignal<Angle> topLeftPosition;
  private final StatusSignal<AngularVelocity> topLeftVelocity;
  private final StatusSignal<Current> topLeftTorqueCurrent;
  private final StatusSignal<Voltage> topLeftVoltage;
  private final StatusSignal<Current> topLeftStatorCurrent;
  private final StatusSignal<Current> topLeftSupplyCurrent;
  private final StatusSignal<Temperature> topLeftTemp;

  // bottomLeft motor
  private final StatusSignal<Angle> bottomLeftPosition;
  private final StatusSignal<AngularVelocity> bottomLeftVelocity;
  private final StatusSignal<Current> bottomLeftTorqueCurrent;
  private final StatusSignal<Voltage> bottomLeftVoltage;
  private final StatusSignal<Current> bottomLeftStatorCurrent;
  private final StatusSignal<Current> bottomLeftSupplyCurrent;
  private final StatusSignal<Temperature> bottomLeftTemp;

  private final SlewRateLimiter velLimit =
      new SlewRateLimiter(5.0); // for checking if motor meets setpoint

  private double velSetpoint = 0.0;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ShooterIOTalonFXV2() {
    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(15.0 / 30.0)) // 2:1 ratio - V1
        .withSlot0(
            new Slot0Configs() // these PID numbers are from V1
                .withKS(5.5)
                .withKV(0.05)
                .withKA(0.0)
                .withKP(8.0)
                .withKI(0.0)
                .withKD(0.0))
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(100));

    // init regardless because you need at least one
    // init topRightLeader motor
    topRightLeader = new TalonFX(Constants.V2_SHOOTER_TOP_RIGHT_ID, Constants.SHOOTER_BUS);

    // init signals
    topRightPosition = topRightLeader.getPosition();
    topRightVelocity = topRightLeader.getVelocity();
    topRightTorqueCurrent = topRightLeader.getTorqueCurrent();
    topRightVoltage = topRightLeader.getMotorVoltage();
    topRightStatorCurrent = topRightLeader.getStatorCurrent();
    topRightSupplyCurrent = topRightLeader.getSupplyCurrent();
    topRightTemp = topRightLeader.getDeviceTemp();

    // apply configs
    topRightLeader.getConfigurator().apply(config);

    // only init if exists to save wasted time looking for nonexistent motor
    if (Constants.V2_SHOOTER_BOTTOM_RIGHT_ON) {
      // init bottomRight motor
      bottomRight = new TalonFX(Constants.V2_SHOOTER_BOTTOM_RIGHT_ID, Constants.SHOOTER_BUS);

      // init signals
      bottomRightPosition = bottomRight.getPosition();
      bottomRightVelocity = bottomRight.getVelocity();
      bottomRightTorqueCurrent = bottomRight.getTorqueCurrent();
      bottomRightVoltage = bottomRight.getMotorVoltage();
      bottomRightStatorCurrent = bottomRight.getStatorCurrent();
      bottomRightSupplyCurrent = bottomRight.getSupplyCurrent();
      bottomRightTemp = bottomRight.getDeviceTemp();

      // apply configs
      bottomRight.getConfigurator().apply(config);

      // set control
      bottomRight.setControl(
          new Follower(Constants.V2_SHOOTER_TOP_RIGHT_ID, MotorAlignmentValue.Aligned));
    } else {
      bottomRight = null;

      bottomRightPosition = null;
      bottomRightVelocity = null;
      bottomRightTorqueCurrent = null;
      bottomRightVoltage = null;
      bottomRightStatorCurrent = null;
      bottomRightSupplyCurrent = null;
      bottomRightTemp = null;
    }

    // only init if exists to save wasted time looking for nonexistent motor
    if (Constants.V2_SHOOTER_TOP_LEFT_ON) {
      // init topLeft motor
      topLeft = new TalonFX(Constants.V2_SHOOTER_TOP_LEFT_ID, Constants.SHOOTER_BUS);

      // init signals
      topLeftPosition = topLeft.getPosition();
      topLeftVelocity = topLeft.getVelocity();
      topLeftTorqueCurrent = topLeft.getTorqueCurrent();
      topLeftVoltage = topLeft.getMotorVoltage();
      topLeftStatorCurrent = topLeft.getStatorCurrent();
      topLeftSupplyCurrent = topLeft.getSupplyCurrent();
      topLeftTemp = topLeft.getDeviceTemp();

      // apply configs
      topLeft.getConfigurator().apply(config);

      // set control
      topLeft.setControl(
          new Follower(Constants.V2_SHOOTER_TOP_LEFT_ID, MotorAlignmentValue.Opposed));
    } else {
      topLeft = null;

      topLeftPosition = null;
      topLeftVelocity = null;
      topLeftTorqueCurrent = null;
      topLeftVoltage = null;
      topLeftStatorCurrent = null;
      topLeftSupplyCurrent = null;
      topLeftTemp = null;
    }

    // only init if exists to save wasted time looking for nonexistent motor
    if (Constants.V2_SHOOTER_BOTTOM_LEFT_ON) {
      // init bottomLeft motor
      bottomLeft = new TalonFX(Constants.V2_SHOOTER_BOTTOM_LEFT_ID, Constants.SHOOTER_BUS);

      // init signals
      bottomLeftPosition = bottomLeft.getPosition();
      bottomLeftVelocity = bottomLeft.getVelocity();
      bottomLeftTorqueCurrent = bottomLeft.getTorqueCurrent();
      bottomLeftVoltage = bottomLeft.getMotorVoltage();
      bottomLeftStatorCurrent = bottomLeft.getStatorCurrent();
      bottomLeftSupplyCurrent = bottomLeft.getSupplyCurrent();
      bottomLeftTemp = bottomLeft.getDeviceTemp();

      // apply configs
      bottomLeft.getConfigurator().apply(config);

      // set control
      bottomLeft.setControl(
          new Follower(Constants.V2_SHOOTER_BOTTOM_LEFT_ID, MotorAlignmentValue.Opposed));
    } else {
      bottomLeft = null;

      bottomLeftPosition = null;
      bottomLeftVelocity = null;
      bottomLeftTorqueCurrent = null;
      bottomLeftVoltage = null;
      bottomLeftStatorCurrent = null;
      bottomLeftSupplyCurrent = null;
      bottomLeftTemp = null;
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean topRightConnected =
        BaseStatusSignal.refreshAll(
                topRightPosition,
                topRightVelocity,
                topRightTorqueCurrent,
                topRightVoltage,
                topRightStatorCurrent,
                topRightSupplyCurrent,
                topRightTemp)
            .isOK();

    // topRightLeader
    inputs.topRightConnected = topRightConnDebouncer.calculate(topRightConnected);

    inputs.topRightPosition = topRightPosition.getValue();
    inputs.topRightVelocity = topRightVelocity.getValue();
    inputs.topRightTorqueCurrent = topRightTorqueCurrent.getValue();
    inputs.topRightVoltage = topRightVoltage.getValue();
    inputs.topRightStatorCurrent = topRightStatorCurrent.getValue();
    inputs.topRightSupplyCurrent = topRightSupplyCurrent.getValue();
    inputs.topRightTemp = topRightTemp.getValue();

    if (Constants.V2_SHOOTER_BOTTOM_RIGHT_ON) {
      boolean bottomRightConnected =
          BaseStatusSignal.refreshAll(
                  bottomRightPosition,
                  bottomRightVelocity,
                  bottomRightTorqueCurrent,
                  bottomRightVoltage,
                  bottomRightStatorCurrent,
                  bottomRightSupplyCurrent,
                  bottomRightTemp)
              .isOK();

      // bottomRight
      inputs.bottomRightConnected = bottomRightConnDebouncer.calculate(bottomRightConnected);

      inputs.bottomRightPosition = bottomRightPosition.getValue();
      inputs.bottomRightVelocity = bottomRightVelocity.getValue();
      inputs.bottomRightTorqueCurrent = bottomRightTorqueCurrent.getValue();
      inputs.bottomRightVoltage = bottomRightVoltage.getValue();
      inputs.bottomRightStatorCurrent = bottomRightStatorCurrent.getValue();
      inputs.bottomRightSupplyCurrent = bottomRightSupplyCurrent.getValue();
      inputs.bottomRightTemp = bottomRightTemp.getValue();
    }

    if (Constants.V2_SHOOTER_TOP_LEFT_ON) {
      boolean topLeftConnected =
          BaseStatusSignal.refreshAll(
                  topLeftPosition,
                  topLeftVelocity,
                  topLeftTorqueCurrent,
                  topLeftVoltage,
                  topLeftStatorCurrent,
                  topLeftSupplyCurrent,
                  topLeftTemp)
              .isOK();

      // topLeftLeader
      inputs.topLeftConnected = topLeftConnDebouncer.calculate(topLeftConnected);

      inputs.topLeftPosition = topLeftPosition.getValue();
      inputs.topLeftVelocity = topLeftVelocity.getValue();
      inputs.topLeftTorqueCurrent = topLeftTorqueCurrent.getValue();
      inputs.topLeftVoltage = topLeftVoltage.getValue();
      inputs.topLeftStatorCurrent = topLeftStatorCurrent.getValue();
      inputs.topLeftSupplyCurrent = topLeftSupplyCurrent.getValue();
      inputs.topLeftTemp = topLeftTemp.getValue();
    }

    if (Constants.V2_SHOOTER_BOTTOM_LEFT_ON) {
      boolean bottomLeftConnected =
          BaseStatusSignal.refreshAll(
                  bottomLeftPosition,
                  bottomLeftVelocity,
                  bottomLeftTorqueCurrent,
                  bottomLeftVoltage,
                  bottomLeftStatorCurrent,
                  bottomLeftSupplyCurrent,
                  bottomLeftTemp)
              .isOK();

      // bottomLeft
      inputs.bottomLeftConnected = bottomLeftConnDebouncer.calculate(bottomLeftConnected);

      inputs.bottomLeftPosition = bottomLeftPosition.getValue();
      inputs.bottomLeftVelocity = bottomLeftVelocity.getValue();
      inputs.bottomLeftTorqueCurrent = bottomLeftTorqueCurrent.getValue();
      inputs.bottomLeftVoltage = bottomLeftVoltage.getValue();
      inputs.bottomLeftStatorCurrent = bottomLeftStatorCurrent.getValue();
      inputs.bottomLeftSupplyCurrent = bottomLeftSupplyCurrent.getValue();
      inputs.bottomLeftTemp = bottomLeftTemp.getValue();
    }

    // sorry promotFollower :(

    // if (promoteFollowerDebouncer.calculate(
    //     inputs.followerVelocity.minus(inputs.topRightVelocity).abs(RotationsPerSecond)
    //         > 50)) // if two velocities are not close to each other then they have become
    // decoupled
    // {
    //   // attempt to swap topRightLeader and follower
    //   inputs.promoteFollower = true;
    //   promoteFollower = true;
    //   topRightLeader.setControl(
    //       new Follower(Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FOLLOWER_ALIGNMENT));
    // }

    inputs.velSetpoint = RotationsPerSecond.of(velSetpoint);
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  @Override
  public void runVelocity(double vel) {
    velSetpoint = vel;
    // if (promoteFollower) {
    // follower.setControl(velocityRequest.withVelocity(vel));

    // } else
    topRightLeader.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public void setShooterMotorsControl(ControlRequest control) {
    // if (promoteFollower) {
    // follower.setControl(control);

    // } else
    topRightLeader.setControl(control);
  }

  @Override
  public void stop() {
    // if (promoteFollower) follower.stopMotor();
    topRightLeader.stopMotor();
  }
  // add AutoLogOutputs
}
