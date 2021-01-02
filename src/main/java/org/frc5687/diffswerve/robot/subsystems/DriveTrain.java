/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DriveTrain.*;
import static org.frc5687.diffswerve.robot.RobotMap.CAN.TALONFX.*;

import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.diffswerve.robot.RobotMap;
import org.frc5687.diffswerve.robot.util.Helpers;
import org.frc5687.diffswerve.robot.util.OutliersContainer;
import org.frc5687.diffswerve.robot.util.Vector2d;

public class DriveTrain extends OutliersSubsystem {
    //    private final DiffSwerveModule _frontRight;
    private final DiffSwerveModule _bottomLeft;

    public DriveTrain(OutliersContainer container) {
        super(container);
        //        _frontRight =
        //                new DiffSwerveModule(
        //                        FRONT_RIGHT_POSITION,
        //                        FR_LEFT_FALCON,
        //                        FR_RIGHT_FALCON,
        //                        RobotMap.Analog.ENCODER_FR);
        _bottomLeft =
                new DiffSwerveModule(
                        BOTTOM_LEFT_POSITION,
                        BL_RIGHT_FALCON,
                        BL_LEFT_FALCON,
                        RobotMap.Analog.ENCODER_BL);
        logMetrics(
                "Left Voltage",
                "Right Voltage",
                "Wanted Left Voltage",
                "Wanted Right Voltage",
                "Wheel Angular Velocity",
                "Wheel Predicted Angular Velocity",
                "Wheel Reference Angular Velocity",
                "Azimuth Predicted Angular Velocity");
        enableMetrics();
        startNotifier(0.005);
    }

    public void update() {
        //        _frontRight.periodic();
        _bottomLeft.periodic();
    }

    @Override
    public void periodic() {}

    @Override
    public void updateDashboard() {
        metric("Right RPM", _bottomLeft.getRightFalconRPM());
        metric("Left RPM", _bottomLeft.getLeftFalconRPM());
        metric("Module Angle", _bottomLeft.getModuleAngle());
        metric("Predicted Angle", _bottomLeft.getPredictedAzimuthAngle());
        metric("Reference Module Angle", _bottomLeft.getReferenceModuleAngle());

        metric("Wanted Left Voltage", _bottomLeft.getLeftNextVoltage());
        metric("Wanted Right Voltage", _bottomLeft.getRightNextVoltage());
        metric("Left Voltage", _bottomLeft.getLeftVoltage());
        metric("Right Voltage", _bottomLeft.getRightVoltage());

        metric("Wheel Angular Velocity", _bottomLeft.getWheelAngularVelocity());
        metric("Wheel Predicted Angular Velocity", _bottomLeft.getPredictedWheelAngularVelocity());
        metric("Wheel Reference Angular Velocity", _bottomLeft.getReferenceWheelAngularVelocity());
    }

    public void setFrontRightModuleVector(Vector2d vec) {
        //        _frontRight.setIdealVector(vec);
        //        _frontRight.setLeftFalconVoltage(getFrontRightWantedVoltages()[0]);
        //        _frontRight.setRightFalconVoltage(getFrontRightWantedVoltages()[1]);
    }

    public void setBottomLeftModuleVector(Vector2d vec) {
        _bottomLeft.setIdealVector(vec);
        _bottomLeft.setLeftFalconVoltage(getBottomLeftWantedVoltages()[0]);
        _bottomLeft.setRightFalconVoltage(getBottomLeftWantedVoltages()[1]);
    }

    public double[] getFrontRightWantedVoltages() {
        //        double lim1 = Helpers.limit(_frontRight.getLeftNextVoltage(), -12, 12);
        //        double lim2 = Helpers.limit(_frontRight.getRightNextVoltage(), -12, 12);
        //        return new double[] {lim1, lim2};
        return new double[] {0, 0};
    }

    public double[] getBottomLeftWantedVoltages() {
        double lim1 = Helpers.limit(_bottomLeft.getLeftNextVoltage(), -12, 12);
        double lim2 = Helpers.limit(_bottomLeft.getRightNextVoltage(), -12, 12);
        return new double[] {lim1, lim2};
    }

    //    public double getFRModuleAngle() {
    //        return _frontRight.getModuleAngle();
    //    }

    public double getBLModuleAngle() {
        return _bottomLeft.getModuleAngle();
    }
}
