/* (C)2020 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.RobotMap.CAN.TALONFX.*;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _frontRight;

    public DriveTrain(OutliersContainer container) {
        super(container);
        _frontRight =
                new DiffSwerveModule(
                        Constants.DriveTrain.FRONT_RIGHT_POSITION, FR_LEFT_FALCON, FR_RIGHT_FALCON);
        logMetrics(
                "Left Voltage",
                "Right Voltage",
                "Wheel Angular Velocity",
                "Wheel Predicted Angular Velocity",
                "Wheel Reference Angular Velocity");
        enableMetrics();
    }

    @Override
    public void periodic() {
        _frontRight.periodic();
    }

    @Override
    public void updateDashboard() {
        //        metric("Right RPM",_frontRight.getRightFalconRPM());
        //        metric("Left RPM",_frontRight.getLeftFalconRPM());
        metric("Wanted Left Voltage", _frontRight.getLeftNextVoltage());
        metric("Wanted Right Voltage", _frontRight.getRightNextVoltage());
        metric("Left Voltage", _frontRight.getLeftVoltage());
        metric("Right Voltage", _frontRight.getRightVoltage());
        metric("Wheel Angular Velocity", _frontRight.getWheelAngularVelocity());
        metric("Wheel Predicted Angular Velocity", _frontRight.getPredictedWheelAngularVelocity());
        metric("Wheel Reference Angular Velocity", _frontRight.getReferenceWheelAngularVelocity());
    }

    public void setFrontRightReference(Matrix<N3, N1> reference) {
        _frontRight.setReference(reference);
    }

    public void setFrontRightSpeeds(double speedR, double speedL) {
        _frontRight.setRightFalcon(speedR);
        _frontRight.setLeftFalcon(speedL);
    }

    public double[] getFrontRightWantedVoltages() {
        return new double[] {_frontRight.getLeftNextVoltage(), _frontRight.getRightNextVoltage()};
    }

    public void setFrontRightVoltage(double voltageLeft, double voltageRight) {
        _frontRight.setLeftFalconVoltage(voltageLeft);
        _frontRight.setRightFalconVoltage(voltageRight);
    }

    public void setFrontRightVelocity(double RPM) {
        _frontRight.setVelocityRPM(RPM);
    }
}
