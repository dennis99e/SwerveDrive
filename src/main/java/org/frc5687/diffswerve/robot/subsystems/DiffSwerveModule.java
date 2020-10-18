/* (C)2020 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DifferentialSwerveModule.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.util.Helpers;

public class DiffSwerveModule {
    private TalonFX _rightFalcon; // TODO: correct names when model is finished.
    private TalonFX _leftFalcon; // TODO: correct names when model is finished.
    private DutyCycleEncoder _lampreyEncoder;
    private Translation2d _positionVector;
    private LinearSystem<N3, N2, N2> _swerveModuleModel;
    private KalmanFilter<N3, N2, N2> _swerveObserver;
    private LinearQuadraticRegulator<N3, N2, N2> _swerveController;
    private LinearSystemLoop<N3, N2, N2> _swerveControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.

    public DiffSwerveModule(
            Translation2d positionVector, int leftMotorID, int rightMotorID, int encoderDIO) {
        _lampreyEncoder = new DutyCycleEncoder(encoderDIO);
        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = positionVector;
        _leftFalcon = new TalonFX(leftMotorID);
        _rightFalcon = new TalonFX(rightMotorID);
        _swerveModuleModel =
                createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        INERTIA_STEER,
                        INERTIA_WHEEL,
                        GEAR_RATIO_STEER,
                        GEAR_RATIO_WHEEL);
        _swerveObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N2(),
                        _swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Units.degreesToRadians(MODEL_AZIMUTH_ANGLE_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                MODEL_AZIMUTH_ANG_VELOCITY_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                MODEL_WHEEL_ANG_VELOCITY_NOISE)),
                        Matrix.mat(Nat.N2(), Nat.N1())
                                .fill(
                                        Units.degreesToRadians(SENSOR_AZIMUTH_ANGLE_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                SENSOR_WHEEL_ANG_VELOCITY_NOISE)),
                        0.005);
        _swerveController =
                new LinearQuadraticRegulator<>(
                        _swerveModuleModel,
                        VecBuilder.fill(Q_AZIMUTH, Q_AZIMUTH_ANG_VELOCITY, Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(0.1 / 12.0, 0.1 / 12.0),
                        0.005);
        _swerveControlLoop =
                new LinearSystemLoop<>(
                        _swerveModuleModel, _swerveController, _swerveObserver, 12.0, 0.005);

        _rightFalcon.setInverted(Constants.DriveTrain.RIGHT_INVERTED);
        _leftFalcon.setInverted(Constants.DriveTrain.LEFT_INVERTED);
        _rightFalcon.setSensorPhase(false);
        _leftFalcon.setSensorPhase(false);
        _rightFalcon.setNeutralMode(NeutralMode.Brake);
        _leftFalcon.setNeutralMode(NeutralMode.Brake);
        _rightFalcon.setStatusFramePeriod(2, 5);
        _leftFalcon.setStatusFramePeriod(2, 5);

        _rightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 200);
        _leftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 200);
        _rightFalcon.configForwardSoftLimitEnable(false);
        _leftFalcon.configForwardSoftLimitEnable(false);
        _rightFalcon.config_kP(0, Constants.DriveTrain.VELOCITY_KP, 200);
        _rightFalcon.config_kI(0, Constants.DriveTrain.VELOCITY_KI, 200);
        _rightFalcon.config_kD(0, Constants.DriveTrain.VELOCITY_KD, 200);
        _rightFalcon.config_kF(0, Constants.DriveTrain.VELOCITY_KF, 200);
        _leftFalcon.config_kP(0, Constants.DriveTrain.VELOCITY_KP, 200);
        _leftFalcon.config_kI(0, Constants.DriveTrain.VELOCITY_KI, 200);
        _leftFalcon.config_kD(0, Constants.DriveTrain.VELOCITY_KD, 200);
        _leftFalcon.config_kF(0, Constants.DriveTrain.VELOCITY_KF, 200);
        _rightFalcon.configClosedloopRamp(0);
        _leftFalcon.configClosedloopRamp(0);
        _leftFalcon.configVoltageCompSaturation(12.0, 200);
        _rightFalcon.configVoltageCompSaturation(12.0, 200);
        _leftFalcon.enableVoltageCompensation(true);
        _rightFalcon.enableVoltageCompensation(true);
        _leftFalcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, 200);
        _rightFalcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, 200);
        _lampreyEncoder.setDistancePerRotation(2.0 * Math.PI);
        _swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
    }

    public void setRightFalcon(double speed) {
        _rightFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftFalcon(double speed) {
        _leftFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setRightFalconVoltage(double voltage) {
        Helpers.limit(voltage, -4.5, 4.5);
        _rightFalcon.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public void setLeftFalconVoltage(double voltage) {
        Helpers.limit(voltage, -4.5, 4.5);
        _leftFalcon.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public void setVelocityRPM(double RPM) {
        _rightFalcon.set(
                ControlMode.Velocity,
                (RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS / 600 / GEAR_RATIO_WHEEL));
        _leftFalcon.set(
                ControlMode.Velocity,
                (RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS / 600 / GEAR_RATIO_WHEEL));
    }

    public double getModuleAngle() {
        return -(_lampreyEncoder.getDistance() - 0.178); // * (2.0 * Math.PI);
        //        return 0; // _lampreyEncoder.getDistance()*(2.0*Math.PI); //TODO: Gear Ratio.
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() * Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL
                                + getRightFalconRPM()
                                        * Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL)
                / 2.0;
    }

    public double getRightFalconRPM() {
        return _rightFalcon.getSelectedSensorVelocity()
                / Constants.DriveTrain.TICKS_TO_ROTATIONS
                * 600.0
                * Constants.DriveTrain.GEAR_RATIO;
    }

    public double getLeftFalconRPM() {
        return _leftFalcon.getSelectedSensorVelocity()
                / Constants.DriveTrain.TICKS_TO_ROTATIONS
                * 600.0
                * Constants.DriveTrain.GEAR_RATIO;
    }

    public double getLeftVoltage() {
        return _leftFalcon.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return _rightFalcon.getMotorOutputVoltage();
    }

    public void periodic() {
        _swerveControlLoop.setNextR(_reference);
        _swerveControlLoop.correct(VecBuilder.fill(getModuleAngle(), getWheelAngularVelocity()));
        _swerveControlLoop.predict(0.005);
    }

    public double[] compensatedControllerOutput(double delay) {
        Matrix<N2, N3> K =
                _swerveController
                        .getK()
                        .times(
                                _swerveModuleModel
                                        .getA()
                                        .minus(
                                                _swerveModuleModel
                                                        .getB()
                                                        .times(_swerveController.getK())));
        Matrix<N2, N1> u = K.times(_reference.minus(_swerveControlLoop.getXHat()));
        return u.getData();
    }

    public double getPredictedAzimuthAngularVelocity() {
        return _swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return _swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return _swerveControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return _swerveControlLoop.getNextR(2);
    }

    public void setReference(Matrix<N3, N1> reference) {
        _reference = reference;
    }

    public double getLeftNextVoltage() {
        return _swerveControlLoop.getU(0);
    }

    public double getRightNextVoltage() {
        return _swerveControlLoop.getU(1);
    }

    public static LinearSystem<N3, N2, N2> createDifferentialSwerveModule(
            DCMotor motor, double Js, double Jw, double Gs, double Gw) {
        var Cs = -((Gs * motor.m_KtNMPerAmp) / (motor.m_KvRadPerSecPerVolt * motor.m_rOhms * Js));
        var Cw = -((Gw * motor.m_KtNMPerAmp) / (motor.m_KvRadPerSecPerVolt * motor.m_rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.m_KtNMPerAmp) / (motor.m_rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.m_KtNMPerAmp) / (motor.m_rOhms * Jw));

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, -Vs, Vw, Vw);
        var C = Matrix.mat(Nat.N2(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N2(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }
}
