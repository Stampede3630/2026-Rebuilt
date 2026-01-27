package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // leader motor
  private final TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Temperature> leaderTemp;

  // follower motor
  private final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private double velSetpoint = 0.0;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ShooterIOTalonFX() {
    // init leader motor
    leader = new TalonFX(Constants.SHOOTER_LEADER_ID);
    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderVoltage = leader.getMotorVoltage();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderTemp = leader.getDeviceTemp();

    leaderConfig
        .withMotorOutput(new MotorOutputConfigs())
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2));
    leader.getConfigurator().apply(leaderConfig);

    // init follower motor
    follower = new TalonFX(Constants.SHOOTER_FOLLOWER_ID);
    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerVoltage = follower.getMotorVoltage();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    followerConfig
        .withMotorOutput(new MotorOutputConfigs())
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2));
    follower.getConfigurator().apply(followerConfig);

    follower.setControl(
        new Follower(Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ALIGNMENT));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderTorqueCurrent,
                leaderVoltage,
                leaderStatorCurrent,
                leaderSupplyCurrent,
                leaderTemp,
                followerPosition,
                followerVelocity,
                followerTorqueCurrent,
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // leader
    inputs.leaderPosition = leaderPosition.getValueAsDouble();
    inputs.leaderVelocity = leaderVelocity.getValueAsDouble();
    inputs.leaderTorqueCurrent = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTemp = leaderTemp.getValueAsDouble();

    // follower
    inputs.followerPosition = followerPosition.getValueAsDouble();
    inputs.followerVelocity = followerVelocity.getValueAsDouble();
    inputs.followerTorqueCurrent = followerTorqueCurrent.getValueAsDouble();
    inputs.followerVoltage = followerVoltage.getValueAsDouble();
    inputs.followerStatorCurrent = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTemp = followerTemp.getValueAsDouble();

    inputs.velSetpoint = velSetpoint;
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  @Override
  public void runVelocity(double vel) {
    velSetpoint = vel;
    leader.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public double getShooterSpeed() {
    return leader.getVelocity().getValueAsDouble() * Shooter.WHEEL_RADIUS_METERS;
  }

  @Override
  public void setShooterMotorsControl(VoltageOut volts) {
    leader.setControl(volts);
    follower.setControl(volts);
  }
}
