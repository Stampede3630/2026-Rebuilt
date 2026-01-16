package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

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

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
  private final TalonFX hoodMotor;
  // private final CANcoder cancoder;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // turret motor
  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<Voltage> turretVoltage;
  private final StatusSignal<Current> turretStatorCurrent;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Temperature> turretTemp;

  // hood motor
  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Current> hoodTorqueCurrent;
  private final StatusSignal<Voltage> hoodVoltage;
  private final StatusSignal<Current> hoodStatorCurrent;
  private final StatusSignal<Current> hoodSupplyCurrent;
  private final StatusSignal<Temperature> hoodTemp;

  // angle offset
  private double angleInitRad = 0.0;
  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public TurretIOTalonFX() {
    // init turret motor
    turretMotor = new TalonFX(30);
    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretTorqueCurrent = turretMotor.getTorqueCurrent();
    turretVoltage = turretMotor.getMotorVoltage();
    turretStatorCurrent = turretMotor.getStatorCurrent();
    turretSupplyCurrent = turretMotor.getSupplyCurrent();
    turretTemp = turretMotor.getDeviceTemp();
    // add turretConfig here

    // init hood motor
    hoodMotor = new TalonFX(31);
    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodTemp = hoodMotor.getDeviceTemp();
    // add hoodConfig here
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretTorqueCurrent,
                turretVoltage,
                turretStatorCurrent,
                turretSupplyCurrent,
                turretTemp,
                hoodPosition,
                hoodVelocity,
                hoodTorqueCurrent,
                hoodVoltage,
                hoodStatorCurrent,
                hoodSupplyCurrent,
                hoodTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // turretMotor
    inputs.turretPosition = turretPosition.getValueAsDouble();
    inputs.turretVelocity = turretVelocity.getValueAsDouble();
    inputs.turretTorqueCurrent = turretTorqueCurrent.getValueAsDouble();
    inputs.turretVoltage = turretVoltage.getValueAsDouble();
    inputs.turretStatorCurrent = turretStatorCurrent.getValueAsDouble();
    inputs.turretSupplyCurrent = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTemp = turretTemp.getValueAsDouble();

    // hoodMotor
    inputs.hoodPosition = hoodPosition.getValueAsDouble();
    inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTemp = hoodTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  /**
   * @param newAngle The angle to set the turret motors adjustment to, in radians
   */
  @Override
  public void setAngleInit(double newAngle) {
    angleInitRad = newAngle;
  }

  @Override
  public void setAngle(Angle angle) {
    // need to subtract angleInitRad here
    turretMotor.setPosition(angle.minus(Radians.of(angleInitRad)));
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodMotor.setPosition(angle);
  }

  @Override
  public double getTurretAngle() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  @Override
  public boolean isInitSet() {
    return initSet;
  }

  @Override
  public void updateInitSet(boolean set) {
    initSet = set;
  }
}
