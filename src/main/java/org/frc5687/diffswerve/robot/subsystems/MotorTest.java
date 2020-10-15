/* (C)2020 */
package org.frc5687.diffswerve.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.frc5687.diffswerve.robot.Constants;

public class MotorTest {
    private TalonFX _motor;
    private DutyCycleEncoder _encoder;
    private LinearSystem<N1, N1, N1> _motorModel;
    private KalmanFilter<N1, N1, N1> _motorObserver;
    private LinearQuadraticRegulator<N1, N1, N1> _motorController;
    private LinearSystemLoop<N1, N1, N1> _motorLoop;
    private Matrix<N1, N1> _reference;

    public MotorTest(int motorId, int encoderId) {
        _motor = new TalonFX(motorId);
        _encoder = new DutyCycleEncoder(encoderId);
        _motorModel = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.00007, 1.0);
        _motorObserver =
                new KalmanFilter<>(
                        Nat.N1(),
                        Nat.N1(),
                        _motorModel,
                        VecBuilder.fill(1),
                        VecBuilder.fill(0.001),
                        0.020);
        _motorController =
                new LinearQuadraticRegulator<>(
                        _motorModel, VecBuilder.fill(0.5), VecBuilder.fill(12.0), 0.020);
        _motorLoop =
                new LinearSystemLoop<>(_motorModel, _motorController, _motorObserver, 12.0, 0.020);

        _motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 200);
        _motor.configForwardSoftLimitEnable(false);
        _motor.configVoltageCompSaturation(12.0, 200);
        _motor.enableVoltageCompensation(true);
        _reference = VecBuilder.fill(0);
    }

    public double getRPM() {
        return _motor.getSelectedSensorVelocity()
                / Constants.DriveTrain.TICKS_TO_ROTATIONS
                * 600.0
                * Constants.DriveTrain.GEAR_RATIO;
    }

    public void periodic() {
        _motorLoop.setNextR(_reference);
        _motorLoop.correct(VecBuilder.fill(getRPM()));
        _motorLoop.predict(0.020);
        SmartDashboard.putNumberArray("XHat", _motorLoop.getXHat().getData());
    }

    public double getRightNextVoltage() {
        return _motorLoop.getU(0);
    }

    public void setMotorVoltage(double voltage) {
        _motor.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public void setReference(Matrix<N1, N1> ref) {
        _reference = ref;
    }
}
