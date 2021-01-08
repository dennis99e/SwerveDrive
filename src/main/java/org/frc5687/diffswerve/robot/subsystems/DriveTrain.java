/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DriveTrain.*;
import static org.frc5687.diffswerve.robot.RobotMap.CAN.TALONFX.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import org.frc5687.diffswerve.robot.RobotMap;
import org.frc5687.diffswerve.robot.util.Helpers;
import org.frc5687.diffswerve.robot.util.OutliersContainer;
import org.frc5687.diffswerve.robot.util.Vector2d;
import org.frc5687.lib.T265Camera;

public class DriveTrain extends OutliersSubsystem {
    private final DiffSwerveModule _frontRight;
    private final DiffSwerveModule _backLeft;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDrivePoseEstimator _poseEstimator;

    private boolean hasTarget = false;

    private final AHRS _imu;
    private final T265Camera _slamCamera;

    public DriveTrain(OutliersContainer container, AHRS imu, T265Camera slamCamera) {
        super(container);
        _imu = imu;
        _slamCamera = slamCamera;

        _frontRight =
                new DiffSwerveModule(
                        FRONT_RIGHT_POSITION,
                        FR_LEFT_FALCON,
                        FR_RIGHT_FALCON,
                        RobotMap.Analog.ENCODER_FR);
        _backLeft =
                new DiffSwerveModule(
                        BACK_LEFT_POSITION,
                        BL_RIGHT_FALCON,
                        BL_LEFT_FALCON,
                        RobotMap.Analog.ENCODER_BL);

        _kinematics =
                new SwerveDriveKinematics(
                        FRONT_LEFT_POSITION,
                        FRONT_RIGHT_POSITION,
                        BACK_LEFT_POSITION,
                        BACK_RIGHT_POSITION);
        _poseEstimator =
                new SwerveDrivePoseEstimator(
                        getHeading(),
                        new Pose2d(),
                        _kinematics,
                        STATE_STD_DEVS,
                        LOCAL_MEASUREMENT_STD_DEVS,
                        VISION_MEASUREMENT_STD_DEVS);
        startNotifier(0.005);
    }

    // use for modules as controller is running at 200Hz.
    public void update() {
        _frontRight.periodic();
        _backLeft.periodic();
    }

    @Override
    public void periodic() {}

    public void updateOdometry() {
        _poseEstimator.update(
                getHeading(), null, _frontRight.getState(), _backLeft.getState(), null);
        if (hasTarget) {
            _poseEstimator.setVisionMeasurementStdDevs(
                    VISION_MEASUREMENT_STD_DEVS); // TODO change when have camera and slam
            _poseEstimator.addVisionMeasurement(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    System.currentTimeMillis() - 0.3); // 0.3 is camera latency need to be changed.
        } else {
            _poseEstimator.setVisionMeasurementStdDevs(
                    VISION_MEASUREMENT_STD_DEVS); // TODO change when have camera and slam
            _slamCamera.stop();
            _slamCamera.start(
                    (T265Camera.CameraUpdate update) -> {
                        _poseEstimator.addVisionMeasurement(
                                update.pose, System.currentTimeMillis());
                    });
        }
    }

    @Override
    public void updateDashboard() {
        metric("Position Error", _backLeft.getPositionError());
        metric("Right RPM", _backLeft.getRightFalconRPM());
        metric("Left RPM", _backLeft.getLeftFalconRPM());
        metric("Module Angle", _backLeft.getModuleAngle());
        metric("Predicted Angle", _backLeft.getPredictedAzimuthAngle());
        metric("Reference Module Angle", _backLeft.getReferenceModuleAngle());

        metric("Wanted Left Voltage", _backLeft.getLeftNextVoltage());
        metric("Wanted Right Voltage", _backLeft.getRightNextVoltage());
        metric("Left Voltage", _backLeft.getLeftVoltage());
        metric("Right Voltage", _backLeft.getRightVoltage());

        metric("Wheel Angular Velocity", _backLeft.getWheelAngularVelocity());
        metric("Wheel Predicted Angular Velocity", _backLeft.getPredictedWheelAngularVelocity());
        metric("Wheel Reference Angular Velocity", _backLeft.getReferenceWheelAngularVelocity());
    }

    public void setFrontRightModuleVector(Vector2d vec) {
        _frontRight.setIdealVector(vec);
        _frontRight.setLeftFalconVoltage(getFrontRightWantedVoltages()[0]);
        _frontRight.setRightFalconVoltage(getFrontRightWantedVoltages()[1]);
    }

    public void setBackLeftModuleVector(Vector2d vec) {
        _backLeft.setIdealVector(vec);
        _backLeft.setLeftFalconVoltage(getBackLeftWantedVoltages()[0]);
        _backLeft.setRightFalconVoltage(getBackLeftWantedVoltages()[1]);
    }

    public double[] getFrontRightWantedVoltages() {
        double lim1 = Helpers.limit(_frontRight.getLeftNextVoltage(), -12, 12);
        double lim2 = Helpers.limit(_frontRight.getRightNextVoltage(), -12, 12);
        return new double[] {lim1, lim2};
    }

    public double[] getBackLeftWantedVoltages() {
        double lim1 = Helpers.limit(_backLeft.getLeftNextVoltage(), -12, 12);
        double lim2 = Helpers.limit(_backLeft.getRightNextVoltage(), -12, 12);
        return new double[] {lim1, lim2};
    }

    public double getFRModuleAngle() {
        return _frontRight.getModuleAngle();
    }

    public double getBLModuleAngle() {
        return _backLeft.getModuleAngle();
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }
}
