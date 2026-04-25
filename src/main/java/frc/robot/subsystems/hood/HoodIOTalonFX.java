package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // hood motor
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0).withSlot(0);

  private Angle setpoint = Radians.of(0);

  public HoodIOTalonFX() {
    // init hood motor + signals
    hoodMotor = new TalonFX(Constants.HOOD_ID, Constants.SWERVE_BUS);
    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    voltage = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temp = hoodMotor.getDeviceTemp();

    config
        .withMotorOutput( // 72
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(9.0 * 3.0)) // prev 72.0
        .withSoftwareLimitSwitch( // unrealistically large for now
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Rotations.of(0.48)) // 0.25 is sideways
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Rotations.of(0)))
        .withMotionMagic(
            new MotionMagicConfigs().withMotionMagicExpo_kV(1.0).withMotionMagicExpo_kA(2.0))
        .withSlot0( // TODO: tune PID
            new Slot0Configs()
                .withKP(130.0)
                .withKI(0.0)
                .withKD(2.0)
                .withKS(5.5)
                .withKV(0.0)
                .withKA(0.0)
                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign)); /* set PID */
    hoodMotor.getConfigurator().apply(config);

    hoodMotor.setPosition(Rotations.of(0.0));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, torqueCurrent, voltage, statorCurrent, supplyCurrent, temp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // update hoodMotor
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.voltage = voltage.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.temp = temp.getValue();
    inputs.setpoint = setpoint;
  }

  @Override
  public void setHoodPos(Angle pos) {
    hoodMotor.setControl(request.withPosition(pos));
  }

  @Override
  public void setHoodPos(double pos) {
    hoodMotor.setControl(request.withPosition(Rotations.of(pos)));
  }

  @Override
  public void resetHoodAngle(double angle) {
    hoodMotor.setPosition(Rotations.of(angle));
  }

  @Override
  public void setNeutralMode(NeutralModeValue val) {
    // turretConfig.withMotorOutput(new MotorOutputConfigs().withNeutralMode(val));
    hoodMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(val));
  }
}
