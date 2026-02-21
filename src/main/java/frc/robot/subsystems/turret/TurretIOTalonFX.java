package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
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

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  public TurretIOTalonFX() {
    // init turret motor
    turretMotor = new TalonFX(Constants.TURRET_ID, Constants.SWERVE_BUS);
    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretTorqueCurrent = turretMotor.getTorqueCurrent();
    turretVoltage = turretMotor.getMotorVoltage();
    turretStatorCurrent = turretMotor.getStatorCurrent();
    turretSupplyCurrent = turretMotor.getSupplyCurrent();
    turretTemp = turretMotor.getDeviceTemp();

    turretConfig
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        // .withSlot0(
        //     new Slot0Configs()
        //         .withKS()
        //         .withKV()
        //         .withKA()
        //         .withKP(1.4)
        //         .withKI(0.01)
        //         .withKD(0.2)) /* set PID */
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.TURRET_GEAR_RATIO))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(1.0) // roughly 1.0
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-0.11)); // could be adjusted slightly
    turretMotor.getConfigurator().apply(turretConfig);
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
                turretTemp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // turretMotor
    inputs.position = turretPosition.getValueAsDouble();
    inputs.velocity = turretVelocity.getValueAsDouble();
    inputs.torqueCurrent = turretTorqueCurrent.getValueAsDouble();
    inputs.voltage = turretVoltage.getValueAsDouble();
    inputs.statorCurrent = turretStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = turretSupplyCurrent.getValueAsDouble();
    inputs.temp = turretTemp.getValueAsDouble();
    inputs.setpoint = turretSetpoint.magnitude();
  }

  /**
   * @param newAngle The angle to set the turret motor's adjustment to
   */
  @Override
  public void resetAnglePos(Angle newAngle) {
    turretMotor.setPosition(newAngle);
  }

  @Override
  public void setTurretAngle(Angle angle) {
    // need to subtract angleInitRad here
    // System.out.println("set to " + angle.magnitude());
    turretSetpoint = angle;
    turretMotor.setControl(new PositionTorqueCurrentFOC(angle));
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

  @Override
  public void setTurretMotorControl(VoltageOut volts) {
    turretMotor.setControl(volts);
  }

  @Override
  public void stopTurret() {
    turretMotor.stopMotor();
  }

  @Override
  public void runTurret(double speed) {
    turretMotor.set(speed);
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return turretMotor.getVelocity().getValue();
  }
}
