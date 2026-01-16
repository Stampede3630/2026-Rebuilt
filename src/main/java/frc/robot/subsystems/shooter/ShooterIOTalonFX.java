package frc.robot.subsystems.shooter;

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

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotorLeader;
  private final TalonFX shooterMotorFollower;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // shooterLeader motor
  private final TalonFXConfiguration shooterLeaderConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> shooterLeaderPosition;
  private final StatusSignal<AngularVelocity> shooterLeaderVelocity;
  private final StatusSignal<Current> shooterLeaderTorqueCurrent;
  private final StatusSignal<Voltage> shooterLeaderVoltage;
  private final StatusSignal<Current> shooterLeaderStatorCurrent;
  private final StatusSignal<Current> shooterLeaderSupplyCurrent;
  private final StatusSignal<Temperature> shooterLeaderTemp;

  // shooterFollower motor
  private final TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> shooterFollowerPosition;
  private final StatusSignal<AngularVelocity> shooterFollowerVelocity;
  private final StatusSignal<Current> shooterFollowerTorqueCurrent;
  private final StatusSignal<Voltage> shooterFollowerVoltage;
  private final StatusSignal<Current> shooterFollowerStatorCurrent;
  private final StatusSignal<Current> shooterFollowerSupplyCurrent;
  private final StatusSignal<Temperature> shooterFollowerTemp;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ShooterIOTalonFX() {
    // init shooterLeader motor
    shooterMotorLeader = new TalonFX(32);
    shooterLeaderPosition = shooterMotorLeader.getPosition();
    shooterLeaderVelocity = shooterMotorLeader.getVelocity();
    shooterLeaderTorqueCurrent = shooterMotorLeader.getTorqueCurrent();
    shooterLeaderVoltage = shooterMotorLeader.getMotorVoltage();
    shooterLeaderStatorCurrent = shooterMotorLeader.getStatorCurrent();
    shooterLeaderSupplyCurrent = shooterMotorLeader.getSupplyCurrent();
    shooterLeaderTemp = shooterMotorLeader.getDeviceTemp();
    // add shooterLeaderConfig here

    // init shooterFollower motor
    shooterMotorFollower = new TalonFX(33);
    shooterFollowerPosition = shooterMotorFollower.getPosition();
    shooterFollowerVelocity = shooterMotorFollower.getVelocity();
    shooterFollowerTorqueCurrent = shooterMotorFollower.getTorqueCurrent();
    shooterFollowerVoltage = shooterMotorFollower.getMotorVoltage();
    shooterFollowerStatorCurrent = shooterMotorFollower.getStatorCurrent();
    shooterFollowerSupplyCurrent = shooterMotorFollower.getSupplyCurrent();
    shooterFollowerTemp = shooterMotorFollower.getDeviceTemp();
    // add shooterFollowerConfig here
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                shooterLeaderPosition,
                shooterLeaderVelocity,
                shooterLeaderTorqueCurrent,
                shooterLeaderVoltage,
                shooterLeaderStatorCurrent,
                shooterLeaderSupplyCurrent,
                shooterLeaderTemp,
                shooterFollowerPosition,
                shooterFollowerVelocity,
                shooterFollowerTorqueCurrent,
                shooterFollowerVoltage,
                shooterFollowerStatorCurrent,
                shooterFollowerSupplyCurrent,
                shooterFollowerTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // shooterMotorLeader
    inputs.shooterLeaderPosition = shooterLeaderPosition.getValueAsDouble();
    inputs.shooterLeaderVelocity = shooterLeaderVelocity.getValueAsDouble();
    inputs.shooterLeaderTorqueCurrent = shooterLeaderTorqueCurrent.getValueAsDouble();
    inputs.shooterLeaderVoltage = shooterLeaderVoltage.getValueAsDouble();
    inputs.shooterLeaderStatorCurrent = shooterLeaderStatorCurrent.getValueAsDouble();
    inputs.shooterLeaderSupplyCurrent = shooterLeaderSupplyCurrent.getValueAsDouble();
    inputs.shooterLeaderTemp = shooterLeaderTemp.getValueAsDouble();

    // shooterMotorFollower
    inputs.shooterFollowerPosition = shooterFollowerPosition.getValueAsDouble();
    inputs.shooterFollowerVelocity = shooterFollowerVelocity.getValueAsDouble();
    inputs.shooterFollowerTorqueCurrent = shooterFollowerTorqueCurrent.getValueAsDouble();
    inputs.shooterFollowerVoltage = shooterFollowerVoltage.getValueAsDouble();
    inputs.shooterFollowerStatorCurrent = shooterFollowerStatorCurrent.getValueAsDouble();
    inputs.shooterFollowerSupplyCurrent = shooterFollowerSupplyCurrent.getValueAsDouble();
    inputs.shooterFollowerTemp = shooterFollowerTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  @Override
  public void runVelocity(double vel) {
    shooterMotorLeader.setControl(velocityRequest.withVelocity(vel));
  }

  @Override
  public double getShooterSpeed() {
    return shooterMotorLeader.getVelocity().getValueAsDouble() * Shooter.WHEEL_RADIUS_METERS;
  }
}
