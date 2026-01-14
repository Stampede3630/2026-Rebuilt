package frc.robot.subsystems.outtake;

import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeIOTalonFX implements OuttakeIO  {
    private final TalonFX shooterMotorLeader;
    private final TalonFX shooterMotorFollower;
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

    // hood motor
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodStatorCurrent;
    private final StatusSignal<Current> hoodSupplyCurrent;
    private final StatusSignal<Temperature> hoodTemp;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public OuttakeIOTalonFX() {
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

        // init shooterLeader motor
        shooterMotorLeader = new TalonFX(31);
        shooterLeaderPosition = shooterMotorLeader.getPosition();
        shooterLeaderVelocity = shooterMotorLeader.getVelocity();
        shooterLeaderTorqueCurrent = shooterMotorLeader.getTorqueCurrent();
        shooterLeaderVoltage = shooterMotorLeader.getMotorVoltage();
        shooterLeaderStatorCurrent = shooterMotorLeader.getStatorCurrent();
        shooterLeaderSupplyCurrent = shooterMotorLeader.getSupplyCurrent();
        shooterLeaderTemp = shooterMotorLeader.getDeviceTemp();
        // add shooterLeaderConfig here

        // init shooterFollower motor
        shooterMotorFollower = new TalonFX(31);
        shooterFollowerPosition = shooterMotorFollower.getPosition();
        shooterFollowerVelocity = shooterMotorFollower.getVelocity();
        shooterFollowerTorqueCurrent = shooterMotorFollower.getTorqueCurrent();
        shooterFollowerVoltage = shooterMotorFollower.getMotorVoltage();
        shooterFollowerStatorCurrent = shooterMotorFollower.getStatorCurrent();
        shooterFollowerSupplyCurrent = shooterMotorFollower.getSupplyCurrent();
        shooterFollowerTemp = shooterMotorFollower.getDeviceTemp();
        // add shooterFollowerConfig here

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


        // represents data emperically derived from the optimal speed to use given a certain distance from the hub
        // data = new TreeMap<>();
        // // fake data
        // data.put(2.0, 0.2);
        // data.put(4.0, 0.4);
        // data.put(6.0, 0.7);
        // cancoder = new CANcoder(32);
        
    }

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        boolean connected = BaseStatusSignal.refreshAll(
            shooterLeaderPosition,
            shooterLeaderVelocity,
            shooterLeaderTorqueCurrent,
            shooterLeaderVoltage,
            shooterLeaderStatorCurrent,
            shooterLeaderSupplyCurrent,
            shooterLeaderTemp,

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
            hoodTemp
        ).isOK();

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
    public void runVelocity(double vel) {
        shooterMotorLeader.setControl(velocityRequest.withVelocity(vel));
    }

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

    @Override
    public double getShooterSpeed() {
        return shooterMotorLeader.getVelocity().getValueAsDouble() * Outtake.WHEEL_RADIUS_METERS ;
    }
}
