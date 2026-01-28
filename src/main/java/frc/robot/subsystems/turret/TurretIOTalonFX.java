package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

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
  private Angle turretSetpoint = Radians.of(0);

  // hood motor
  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Current> hoodTorqueCurrent;
  private final StatusSignal<Voltage> hoodVoltage;
  private final StatusSignal<Current> hoodStatorCurrent;
  private final StatusSignal<Current> hoodSupplyCurrent;
  private final StatusSignal<Temperature> hoodTemp;
  private Angle hoodSetpoint = Radians.of(0);

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  public TurretIOTalonFX() {
    // init turret motor
    turretMotor = new TalonFX(Constants.TURRET_ID);
    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretTorqueCurrent = turretMotor.getTorqueCurrent();
    turretVoltage = turretMotor.getMotorVoltage();
    turretStatorCurrent = turretMotor.getStatorCurrent();
    turretSupplyCurrent = turretMotor.getSupplyCurrent();
    turretTemp = turretMotor.getDeviceTemp();

    turretConfig
        .withMotorOutput(new MotorOutputConfigs())
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2)); /* set PID */
    turretMotor.getConfigurator().apply(turretConfig);

    // init hood motor
    hoodMotor = new TalonFX(Constants.HOOD_ID);
    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodTemp = hoodMotor.getDeviceTemp();

    hoodConfig.withMotorOutput(new MotorOutputConfigs());
    hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
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
    inputs.turretSetpoint = turretSetpoint.magnitude();

    // hoodMotor
    inputs.hoodPosition = hoodPosition.getValueAsDouble();
    inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTemp = hoodTemp.getValueAsDouble();
    inputs.hoodSetpoint = hoodSetpoint.magnitude();

    // // turretMotor
    // inputs.turret.position = turretPosition.getValueAsDouble();
    // inputs.turret.velocity = turretVelocity.getValueAsDouble();
    // inputs.turret.torqueCurrent = turretTorqueCurrent.getValueAsDouble();
    // inputs.turret.voltage = turretVoltage.getValueAsDouble();
    // inputs.turret.statorCurrent = turretStatorCurrent.getValueAsDouble();
    // inputs.turret.statorCurrent = turretSupplyCurrent.getValueAsDouble();
    // inputs.turret.temp = turretTemp.getValueAsDouble();

    // // hoodMotor
    // inputs.hood.position = hoodPosition.getValueAsDouble();
    // inputs.hood.velocity = hoodVelocity.getValueAsDouble();
    // inputs.hood.torqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    // inputs.hood.voltage = hoodVoltage.getValueAsDouble();
    // inputs.hood.statorCurrent = hoodStatorCurrent.getValueAsDouble();
    // inputs.hood.supplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    // inputs.hood.temp = hoodTemp.getValueAsDouble();
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  /**
   * @param newAngle The angle to set the turret motors adjustment to
   */
  @Override
  public void resetAnglePos(Angle newAngle) {
    turretMotor.setPosition(newAngle);
  }

  @Override
  public void setAngle(Angle angle) {
    // need to subtract angleInitRad here
    // System.out.println("set to " + angle.magnitude());
    turretSetpoint = angle;
    turretMotor.setControl(new PositionTorqueCurrentFOC(angle));
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodSetpoint = angle;
    hoodMotor.setControl(new PositionTorqueCurrentFOC(angle));
  }

  /**
   * @return The current angle of the turret, in radians
   */
  @Override
  public Angle getTurretAngle() {
    return turretMotor.getPosition().getValue();
  }

  @Override
  public boolean isInitSet() {
    return initSet;
  }

  @Override
  public void updateInitSet(boolean set) {
    initSet = set;
  }

  /**
   * @return The current angle of the hood, in rotations NOTE: need to initialize at a specific
   *     angle
   */
  @Override
  public Angle getHoodAngle() {
    return hoodMotor.getPosition().getValue();
  }

  @Override
  public void setTurretMotorControl(VoltageOut volts) {
    turretMotor.setControl(volts);
  }

  @Override
  public void stopTurret() {
    turretMotor.stopMotor();
  }

  @Override
  public void run(double speed) {
    turretMotor.set(speed);
  }
}
