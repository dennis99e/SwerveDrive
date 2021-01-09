/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DriveTrain.*;
import static org.frc5687.diffswerve.robot.RobotMap.CAN.TALONFX.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.RobotMap;
import org.frc5687.diffswerve.robot.util.GlowWorm;
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
    private final GlowWorm _vision;

    public DriveTrain(OutliersContainer container, AHRS imu, T265Camera slamCamera) {
        super(container);
        _imu = imu;
        _slamCamera = slamCamera;
        _vision = new GlowWorm("glowworm"); //TODO: change name of camera

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
        if (_vision.hasTarget()) {
            _poseEstimator.setVisionMeasurementStdDevs(
                    VISION_MEASUREMENT_STD_DEVS); // TODO change when have camera and slam
            _poseEstimator.addVisionMeasurement(
                    _vision.getTargetPose(),
                    System.currentTimeMillis() - _vision.getLatency()); // 0.3 is camera latency need to be changed.
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
    }

    public void setBackLeftModuleVector(Vector2d vec) {
        _backLeft.setIdealVector(vec);
    }
    public void setFrontRightModuleState(SwerveModuleState state) {
        _frontRight.setModuleState(state);
    }

    public void setBackLeftModuleState(SwerveModuleState state) {
        _backLeft.setModuleState(state);
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

    public void drive(double vx, double vy, double omega, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading())
                        : new ChassisSpeeds(vx, vy, omega));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_MPS);
        setFrontRightModuleState(swerveModuleStates[1]);
        setBackLeftModuleState(swerveModuleStates[2]);
    }

}
