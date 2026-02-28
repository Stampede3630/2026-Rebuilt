package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
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

  private final PositionVoltage posRequestVoltage = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC posRequestTorqueCurrent =
      new PositionTorqueCurrentFOC(0.0);

  private Angle turretSetpoint = Radians.of(0);

  private final double LEFT_LIMIT = -0.61; // rotations
  private final double RIGHT_LIMIT = 0.5; // rotations

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
        .withSlot0(
            new Slot0Configs() // set PID for PositionVoltage
                .withKS(2.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(100)
                .withKI(0.0)
                .withKD(0.0))
        .withSlot1(
            new Slot1Configs() // set PID for PositionTorqueCurrentFOC
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.TURRET_GEAR_RATIO))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(RIGHT_LIMIT) // roughly 1.0
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(LEFT_LIMIT)); // could be adjusted slightly
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
    Angle leftAngle = angle.minus(Degrees.of(360)); // check near left rotation
    // shouldn't have to worry about rightAngle
    Angle currentAngle = turretMotor.getPosition().getValue();
    // find if leftAngle or angle is closer to currentAngle
    if (currentAngle.minus(leftAngle).abs(Radians) < currentAngle.minus(angle).abs(Radians)) {
      // if leftAngle is closer
      angle = leftAngle;
    }
    turretSetpoint = angle;
    turretMotor.setControl(posRequestVoltage.withPosition(angle).withSlot(0));
  }

  @Override
  public void setTurretAngleTorqueCurrent(Angle angle) {
    turretSetpoint = angle;
    turretMotor.setControl(posRequestTorqueCurrent.withPosition(angle).withSlot(1));
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
