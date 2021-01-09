/* (C)2021 */
package org.frc5687.diffswerve.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.photonvision.PhotonCamera;

public class GlowWorm {

    private final PhotonCamera _camera;
    private final NetworkTable _table;
    private final double[] _pose;

    public GlowWorm(String camName) {
        double[] defaultVal = {0, 0, 0};
        _camera = new PhotonCamera(camName);
        _table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(camName);
        _pose = _table.getEntry("targetPose").getDoubleArray(defaultVal);
    }

    // Latency in milliseconds.
    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    // Should return pose from 3D mode. Not sure, not documented yet.
    public Pose2d getTargetPose() {
        return new Pose2d(new Translation2d(_pose[0], _pose[1]), Rotation2d.fromDegrees(_pose[2]));
    }

    public boolean hasTarget() {
        return _camera.getLatestResult().hasTargets();
    }
}
