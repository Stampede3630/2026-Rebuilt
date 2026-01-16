package frc.robot.subsystems.turret;

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
    hoodMotor = new TalonFX(32);
    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodTemp = hoodMotor.getDeviceTemp();
    // add hoodConfig here

    // represents data emperically derived from the optimal speed to use given a certain distance
    // from the hub
    // data = new TreeMap<>();
    // // fake data
    // data.put(2.0, 0.2);
    // data.put(4.0, 0.4);
    // data.put(6.0, 0.7);
    // cancoder = new CANcoder(32);

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

  @Override
  public void setAngle(Angle angle) {
    turretMotor.setPosition(angle);
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodMotor.setPosition(angle);
  }

  @Override
  public double getTurretAngle() {
    return turretMotor.getPosition().getValueAsDouble();
  }
}
