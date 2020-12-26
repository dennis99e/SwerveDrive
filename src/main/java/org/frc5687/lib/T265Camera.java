/* (C)2020 */
package org.frc5687.lib;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import java.nio.file.Paths;
import java.util.function.Consumer;
import org.frc5687.diffswerve.robot.util.OutliersProxy;

public class T265Camera extends OutliersProxy {

    private static UnsatisfiedLinkError _linkError;

    private long _nativeCameraObjectPointer = 0;
    private boolean _isStarted = false;
    private Pose2d _robotOffset;
    private Pose2d _origin = new Pose2d();
    private Pose2d _lastReceivedPose = new Pose2d();
    private Consumer<CameraUpdate> _poseConsumer = null;

    static {
        try {
            // TODO: find path
            //            System.loadLibrary("rs");
            //            System.loadLibrary("outliernative");
            //            DriverStation.reportError("loaded Library", false);
            //            System.load("/usr/local/frc/lib/librs.so");
            //            System.load("/home/admin/librs.so");
            System.load(
                    Paths.get(System.getProperty("user.home"), "liboutliernative.so")
                            .toAbsolutePath()
                            .toString());

            Runtime.getRuntime().addShutdownHook(new Thread(() -> T265Camera.cleanup()));
        } catch (UnsatisfiedLinkError e) {
            _linkError = e;
        }
    }

    public T265Camera(Pose2d robotOffset, double odometryCovariance) {
        this(robotOffset, odometryCovariance, "");
    }

    public T265Camera(Pose2d robotOffset, double odometryCovariance, String relocMapPath) {
        if (_linkError != null) {
            error("error is " + _linkError);
            throw _linkError;
        }

        _nativeCameraObjectPointer = newCamera(relocMapPath);
        setOdometryInfo(
                (float) robotOffset.getTranslation().getX(),
                (float) robotOffset.getTranslation().getY(),
                (float) robotOffset.getRotation().getRadians(),
                odometryCovariance);
        _robotOffset = robotOffset;
    }

    public synchronized void start(Consumer<CameraUpdate> poseConsumer) {
        if (_isStarted) {
            throw new RuntimeException("T265 Camera is running already!");
        }
        _poseConsumer = poseConsumer;
        _isStarted = true;
    }

    public synchronized void stop() {
        _isStarted = false;
    }

    /**
     * Exports a binary relocalization map file to the given path. This will stop the camera.
     * Because of a librealsense bug the camera isn't restarted after you call this method. TODO:
     * Fix that.
     *
     * @param path Path (with filename) to export to
     */
    public native void exportRelocalizationMap(String path);

    /**
     * Sends robot velocity as computed from wheel encoders.
     *
     * @param velocityXMetersPerSecond The robot-relative velocity along the X axis in meters/sec.
     * @param velocityYMetersPerSecond The robot-relative velocity along the Y axis in meters/sec.
     */
    public void sendOdometry(double velocityXMetersPerSecond, double velocityYMetersPerSecond) {
        // Only 1 odometry sensor is supported for now (index 0)
        sendOdometryRaw(0, (float) velocityXMetersPerSecond, (float) velocityYMetersPerSecond);
    }

    public synchronized void setPose(Pose2d newPose) {
        _origin = newPose;
    }

    /**
     * This will free the underlying native objects. You probably don't want to use this; on program
     * shutdown the native code will gracefully stop and delete any remaining objects.
     */
    public native void free();

    private native void setOdometryInfo(
            float robotOffsetX,
            float robotOffsetY,
            float robotOffsetRads,
            double measurementCovariance);

    private native void sendOdometryRaw(int sensorIndex, float xVel, float yVel);

    private native long newCamera(String mapPath);

    private static native void cleanup();

    public static enum PoseConfidence {
        Failed,
        Low,
        Medium,
        High,
    }

    public static class CameraUpdate {
        public final Pose2d pose;

        public final ChassisSpeeds velocity;
        public final PoseConfidence confidence;

        public CameraUpdate(Pose2d pose, ChassisSpeeds velocity, PoseConfidence confidence) {
            this.pose = pose;
            this.velocity = velocity;
            this.confidence = confidence;
        }
    }

    private synchronized void consumePoseUpdate(
            float x, float y, float radians, float vx, float vy, float omega, int confOrdinal) {
        // First we apply an offset to go from the camera coordinate system to the
        // robot coordinate system with an origin at the center of the robot. This
        // is not a directional transformation.
        // Then we transform the pose our camera is giving us so that it reports is
        // the robot's pose, not the camera's. This is a directional transformation.
        Transform2d transform =
                new Transform2d(_robotOffset.getTranslation(), _robotOffset.getRotation());
        final Pose2d currentPose =
                new Pose2d(
                                x - _robotOffset.getTranslation().getX(),
                                y - _robotOffset.getTranslation().getY(),
                                new Rotation2d(radians))
                        .transformBy(transform);

        _lastReceivedPose = currentPose;

        if (!_isStarted) return;

        // See
        // https://github.com/IntelRealSense/librealsense/blob/7f2ba0de8769620fd672f7b44101f0758e7e2fb3/include/librealsense2/h/rs_types.h#L115
        // for ordinals
        PoseConfidence confidence;
        switch (confOrdinal) {
            case 0x0:
                confidence = PoseConfidence.Failed;
                break;
            case 0x1:
                confidence = PoseConfidence.Low;
                break;
            case 0x2:
                confidence = PoseConfidence.Medium;
                break;
            case 0x3:
                confidence = PoseConfidence.High;
                break;
            default:
                throw new RuntimeException(
                        "Unknown confidence ordinal \""
                                + confOrdinal
                                + "\" passed from native code");
        }

        Transform2d current =
                new Transform2d(currentPose.getTranslation(), currentPose.getRotation());
        final Pose2d transformedPose = _origin.transformBy(current);
        _poseConsumer.accept(
                new CameraUpdate(transformedPose, new ChassisSpeeds(vx, vy, omega), confidence));
    }

    /** Thrown if something goes wrong in the native code */
    public static class CameraJNIException extends RuntimeException {

        // This must be static _and_ have this constructor if you want it to be
        // thrown from native code
        public CameraJNIException(String message) {
            super(message);
        }
    }

    @Override
    public void updateDashboard() {}
}
