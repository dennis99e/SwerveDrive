/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.subsystems;

import static org.frc5687.diffswerve.robot.Constants.DriveTrain.*;
import static org.frc5687.diffswerve.robot.RobotMap.CAN.TALONFX.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import java.util.concurrent.atomic.AtomicReference;
import org.frc5687.diffswerve.robot.RobotMap;
import org.frc5687.diffswerve.robot.util.GloWorm;
import org.frc5687.diffswerve.robot.util.OutliersContainer;
import org.frc5687.lib.T265Camera;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _frontRight;
    private DiffSwerveModule _frontLeft;
    private DiffSwerveModule _backRight;
    private DiffSwerveModule _backLeft;

    private SwerveDriveKinematics _kinematics;
    private SwerveDrivePoseEstimator _poseEstimator;
    private SwerveDriveOdometry _odomerty;

    private boolean hasTarget = false;

    private AHRS _imu;
    private T265Camera _slamCamera;
    private GloWorm _vision;

    private HolonomicDriveController _controller;

    public DriveTrain(OutliersContainer container, AHRS imu, T265Camera slamCamera) {
        super(container);
        try {
            _imu = imu;
            _slamCamera = slamCamera;
            _vision = new GloWorm("gloworm"); // TODO: change name of camera

            _frontRight =
                    new DiffSwerveModule(
                            FRONT_RIGHT_POSITION,
                            FR_LEFT_FALCON,
                            FR_RIGHT_FALCON,
                            RobotMap.Analog.ENCODER_FR);
            _frontLeft =
                    new DiffSwerveModule(
                            FRONT_LEFT_POSITION,
                            FL_LEFT_FALCON,
                            FL_RIGHT_FALCON,
                            RobotMap.Analog.ENCODER_FL);
            _backRight =
                    new DiffSwerveModule(
                            BACK_RIGHT_POSITION,
                            BR_LEFT_FALCON,
                            BR_RIGHT_FALCON,
                            RobotMap.Analog.ENCODER_BR);
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
            _odomerty =
                    new SwerveDriveOdometry(
                            _kinematics, getHeading(), new Pose2d(0, 0, new Rotation2d(0)));

            _controller =
                    new HolonomicDriveController(
                            new PIDController(kP, kI, kD),
                            new PIDController(kP, kI, kD),
                            new ProfiledPIDController(
                                    kP,
                                    kI,
                                    kD,
                                    new TrapezoidProfile.Constraints(
                                            PROFILE_CONSTRAINT_VEL, PROFILE_CONSTRAINT_ACCEL)));
        } catch (Exception e) {
            error(e.getMessage());
        }
        //        _odomerty.resetPosition(getPose(), getHeading());
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _frontRight.periodic();
        _frontLeft.periodic();
        _backRight.periodic();
        _backLeft.periodic();
    }

    @Override
    public void periodic() {
        _odomerty.update(
                getHeading(),
                _frontLeft.getState(),
                _frontRight.getState(),
                _backLeft.getState(),
                _backRight.getState());
        //        updateOdometry();
        //        metric("estimated Pose", _poseEstimator.getEstimatedPosition().toString());
        //        metric("slam pose", getSlamPose().toString());
        //        SmartDashboard.putString("pose", _odomerty.getPoseMeters().toString());
        //        _poseEstimator.update(
        //                getHeading(),
        //                _frontLeft.getState(),
        //                _frontRight.getState(),
        //                _backLeft.getState(),
        //                _backRight.getState());
        //        _poseEstimator.addVisionMeasurement(getSlamPose(), Timer.getFPGATimestamp());
    }

    public void updateOdometry() {
        _poseEstimator.update(
                getHeading(),
                _frontLeft.getState(),
                _frontRight.getState(),
                _backLeft.getState(),
                _backRight.getState());
        //        if (_vision.hasTarget()) {
        //            //            _poseEstimator.setVisionMeasurementStdDevs(
        //            //                    VISION_MEASUREMENT_STD_DEVS); // TODO change when have
        // camera and
        //            // slam
        //            //            _poseEstimator.addVisionMeasurement(
        //            //                    _vision.getTargetPose(), System.currentTimeMillis() -
        //            // _vision.getLatency());
        //        } else if (_slamCamera != null) {
        //            //            _poseEstimator.setVisionMeasurementStdDevs(
        //            //                    VISION_MEASUREMENT_STD_DEVS); // TODO change when have
        // camera and
        //            //            // slam
        //            _poseEstimator.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp());
        //        }

        //        metric("slam pose", getSlamPose().toString());
    }

    @Override
    public void updateDashboard() {
        //        metric ("Pose Estimation", _poseEstimator.getEstimatedPosition().toString());
        //        metric("Odometry pose", _odomerty.getPoseMeters().toString());
        //        metric("Position Error", _backLeft.getPositionError());
        //        metric("Right RPM", _backLeft.getRightFalconRPM());
        //        metric("Left RPM", _backLeft.getLeftFalconRPM());
        //        metric("Module Angle", _backLeft.getModuleAngle());
        //        metric("Predicted Angle", _backLeft.getPredictedAzimuthAngle());
        //        metric("Reference Module Angle", _backLeft.getReferenceModuleAngle());
        //
        //        metric("Wanted Left Voltage", _backLeft.getLeftNextVoltage());
        //        metric("Wanted Right Voltage", _backLeft.getRightNextVoltage());
        //        metric("Left Voltage", _backLeft.getLeftVoltage());
        //        metric("Right Voltage", _backLeft.getRightVoltage());
        //
        //        metric("Wheel Angular Velocity", _backLeft.getWheelAngularVelocity());
        //        metric("Wheel Predicted Angular Velocity",
        // _backLeft.getPredictedWheelAngularVelocity());
        //        metric("Wheel Reference Angular Velocity",
        // _backLeft.getReferenceWheelAngularVelocity());
        //        metric("FR/angle", _frontRight.getModuleAngle());
        //        metric("FR/vel", _frontRight.getWheelVelocity());
        SmartDashboard.putNumberArray(
                "DriveTrain/FR/state predict", _frontRight.getPredictedState());

        //        metric("FL/angle", _frontLeft.getModuleAngle());
        //        metric("FL/vel", _frontLeft.getWheelVelocity());
        SmartDashboard.putNumberArray(
                "DriveTrain/FL/state predict", _frontLeft.getPredictedState());

        //        metric("BR/angle", _backRight.getModuleAngle());
        //        metric("BR/vel", _backRight.getWheelVelocity());
        SmartDashboard.putNumberArray(
                "DriveTrain/BR/state predict", _backRight.getPredictedState());

        //        metric("BL/angle", _backLeft.getModuleAngle());
        //        metric("BL/vel", _backLeft.getWheelVelocity());
        SmartDashboard.putNumberArray("DriveTrain/BL/state predict", _backLeft.getPredictedState());
    }

    public void setFrontRightModuleState(SwerveModuleState state) {
        _frontRight.setIdealState(state);
    }

    public void setFrontLeftModuleState(SwerveModuleState state) {
        _frontLeft.setIdealState(state);
    }

    public void setBackLeftModuleState(SwerveModuleState state) {
        _backLeft.setIdealState(state);
    }

    public void setBackRightModuleState(SwerveModuleState state) {
        _backRight.setIdealState(state);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        metric("heading", -getYaw());
        return Rotation2d.fromDegrees(-getYaw());
    }

    public void drive(double vx, double vy, double omega, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading())
                                : new ChassisSpeeds(vx, vy, omega));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_MPS);
        SmartDashboard.putNumber("states", swerveModuleStates[0].speedMetersPerSecond);
        if (Math.abs(vx) < DEADBAND && Math.abs(vy) < DEADBAND && Math.abs(omega) < DEADBAND) {
            setFrontRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_frontRight.getModuleAngle())));
            setFrontLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_frontLeft.getModuleAngle())));
            setBackRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backRight.getModuleAngle())));
            setBackLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backLeft.getModuleAngle())));
        } else {
            setFrontRightModuleState(swerveModuleStates[1]);
            setFrontLeftModuleState(swerveModuleStates[0]);
            setBackLeftModuleState(swerveModuleStates[2]);
            setBackRightModuleState(swerveModuleStates[3]);
        }
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, MAX_MPS);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(MAX_MPS, MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(_odomerty.getPoseMeters(), goal, Rotation2d.fromDegrees(0.0));
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, MAX_MPS);

        setFrontLeftModuleState(moduleStates[0]);
        setFrontRightModuleState(moduleStates[1]);
        setBackLeftModuleState(moduleStates[2]);
        setBackRightModuleState(moduleStates[3]);
    }

    public Pose2d getSlamPose() {
        AtomicReference<Pose2d> pose = new AtomicReference<>(new Pose2d());
        if (_slamCamera == null) {
            return new Pose2d();
        }
        _slamCamera.stop();
        _slamCamera.start(
                (T265Camera.CameraUpdate update) -> {
                    metric("slam pose", update.pose.toString());
                    pose.set(update.pose);
                });
        return pose.get();
    }

    public Pose2d getOdometryPose() {
        return _odomerty.getPoseMeters();
    }
}
