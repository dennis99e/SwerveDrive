/* (C)2020 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DifferentialSwerveModule.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogEncoder;
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

public class DiffSwerveModule {
    private TalonFX _rightFalcon; // TODO: correct names when model is finished.
    private TalonFX _leftFalcon; // TODO: correct names when model is finished.
    private AnalogEncoder _lampreyEncoder;
    private ModuleID _modID;
    private Translation2d _positionVector;
    private LinearSystem<N3, N2, N2> _swerveModuleModel;
    private KalmanFilter<N3, N2, N2> _swerveObserver;
    private LinearQuadraticRegulator<N3, N2, N2> _swerveController;
    private LinearSystemLoop<N3, N2, N2> _swerveControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.

    public DiffSwerveModule(Translation2d positionVector, int leftMotorID, int rightMotorID) {
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
                        0.020);
        _swerveController =
                new LinearQuadraticRegulator<>(
                        _swerveModuleModel,
                        VecBuilder.fill(Q_AZIMUTH, Q_AZIMUTH_ANG_VELOCITY, Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(12.0, 12.0),
                        0.020);
        _swerveControlLoop =
                new LinearSystemLoop<>(
                        _swerveModuleModel, _swerveController, _swerveObserver, 12.0, 0.020);

        _rightFalcon.setInverted(Constants.DriveTrain.RIGHT_INVERTED);
        _leftFalcon.setInverted(Constants.DriveTrain.LEFT_INVERTED);
        _rightFalcon.setSensorPhase(false);
        _leftFalcon.setSensorPhase(false);

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
        _swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
    }

    public void setRightFalcon(double speed) {
        _rightFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftFalcon(double speed) {
        _leftFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setRightFalconVoltage(double voltage) {
        _rightFalcon.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public void setLeftFalconVoltage(double voltage) {
        _leftFalcon.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public void setVelocityRPM(double RPM) {
        _rightFalcon.set(
                ControlMode.Velocity, (RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS / 600 / 1));
        _leftFalcon.set(
                ControlMode.Velocity, (RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS / 600 / 1));
    }

    public double getModuleAngle() {
        return 0; // _lampreyEncoder.getDistance()*(2.0*Math.PI); //TODO: Gear Ratio.
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
        _swerveControlLoop.predict(0.020);
    }

    public double getPredictedWheelAngularVelocity() {
        return _swerveControlLoop.getXHat(2);
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

    public enum ModuleID {
        FrontRight(0),
        FrontLeft(1),
        BottomRight(2),
        BottomLeft(3);

        private int _value;

        ModuleID(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
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
