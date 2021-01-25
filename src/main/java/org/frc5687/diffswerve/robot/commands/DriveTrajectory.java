/* (C)2021 */
package org.frc5687.diffswerve.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.ArrayList;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final Pose2d _start;
    private final Pose2d _end;
    private final ArrayList<Translation2d> _waypoints;
    private double _startTime;
    private double _time;
    private Trajectory _trajectory;
    private Timer _timer;

    public DriveTrajectory(
            DriveTrain driveTrain, Pose2d start, ArrayList<Translation2d> waypoints, Pose2d end) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _start = start;
        _waypoints = waypoints;
        _end = end;
        _timer = new Timer();
    }

    @Override
    public void initialize() {
        _trajectory =
                TrajectoryGenerator.generateTrajectory(
                        _start, _waypoints, _end, _driveTrain.getConfig());
        _time = _trajectory.getTotalTimeSeconds();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        metric("time", _time);
        metric("timer val", _timer.get());
        Trajectory.State goal = _trajectory.sample(_timer.get());
        metric("goal mps", goal.velocityMetersPerSecond);
        metric("goal", goal.timeSeconds);
        _driveTrain.trajectoryFollower(goal);
    }

    @Override
    public boolean isFinished() {
        return _timer.get() == _time;
        //        return (Math.abs(_timer.get() - _time) < 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
}
