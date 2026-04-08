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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFXV1 implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final Debouncer leaderConnDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnDebouncer = new Debouncer(0.5);

  // motor configs
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // leader motor
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Temperature> leaderTemp;

  // follower motor
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final SlewRateLimiter velLimit =
      new SlewRateLimiter(5.0); // for checking if motor meets setpoint

  private double velSetpoint = 0.0;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ShooterIOTalonFXV1() {
    // init leader motor
    leader = new TalonFX(Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_BUS);
    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderVoltage = leader.getMotorVoltage();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderTemp = leader.getDeviceTemp();

    // init follower motor
    follower = new TalonFX(Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_BUS);
    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerVoltage = follower.getMotorVoltage();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    config
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    15.0 / 30.0)) // 1:2 ratio     no bad caleb other way 2:1
        .withSlot0(
            new Slot0Configs()
                .withKS(5.5) // 6.5
                .withKV(0.05) // 0.01
                .withKA(0.0) // 0.0
                .withKP(8.0) // 11.5
                .withKI(0.0) // 0.0
                .withKD(0.0))
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(100)); // 0.0
    leader.getConfigurator().apply(config);
    follower
        .getConfigurator()
        .apply(
            config.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

    // also switched ids
    // off since missing leader
    follower.setControl(
        new Follower(Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ALIGNMENT));
  }

  private boolean promoteFollower = false;

  private Debouncer promoteFollowerDebouncer = new Debouncer(0.5);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean leaderConnected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderTorqueCurrent,
                leaderVoltage,
                leaderStatorCurrent,
                leaderSupplyCurrent,
                leaderTemp)
            .isOK();
    boolean followerConnected =
        BaseStatusSignal.refreshAll(
                followerPosition,
                followerVelocity,
                followerTorqueCurrent,
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemp)
            .isOK();

    inputs.topRightConnected = leaderConnDebouncer.calculate(leaderConnected);
    inputs.topLeftConnected = followerConnDebouncer.calculate(followerConnected);

    // leader
    inputs.topRightPosition = leaderPosition.getValue();
    inputs.topRightVelocity = leaderVelocity.getValue();
    inputs.topRightTorqueCurrent = leaderTorqueCurrent.getValue();
    inputs.topRightVoltage = leaderVoltage.getValue();
    inputs.topRightStatorCurrent = leaderStatorCurrent.getValue();
    inputs.topRightSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.topRightTemp = leaderTemp.getValue();

    // follower
    inputs.topLeftPosition = followerPosition.getValue();
    inputs.topLeftVelocity = followerVelocity.getValue();
    inputs.topLeftTorqueCurrent = followerTorqueCurrent.getValue();
    inputs.topLeftVoltage = followerVoltage.getValue();
    inputs.topLeftStatorCurrent = followerStatorCurrent.getValue();
    inputs.topLeftSupplyCurrent = followerSupplyCurrent.getValue();
    inputs.topLeftTemp = followerTemp.getValue();

    if (promoteFollowerDebouncer.calculate(
        inputs.topLeftVelocity.minus(inputs.topRightVelocity).abs(RotationsPerSecond)
            > 50)) // if two velocities are not close to each other then they have become decoupled
    {
      // attempt to swap leader and follower
      inputs.promoteFollower = true;
      promoteFollower = true;
      leader.setControl(
          new Follower(Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FOLLOWER_ALIGNMENT));
    }

    inputs.velSetpoint = RotationsPerSecond.of(velSetpoint);
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  @Override
  public void runVelocity(double vel) {
    velSetpoint = vel;
    if (promoteFollower) {
      follower.setControl(velocityRequest.withVelocity(vel));

    } else leader.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public void setShooterMotorsControl(ControlRequest control) {
    if (promoteFollower) {
      follower.setControl(control);

    } else leader.setControl(control);
  }

  @Override
  public void stop() {
    if (promoteFollower) follower.stopMotor();
    else leader.stopMotor();
  }
  // add AutoLogOutputs
}
