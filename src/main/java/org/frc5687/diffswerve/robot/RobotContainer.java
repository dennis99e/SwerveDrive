/* (C)2020-2021 */
package org.frc5687.diffswerve.robot;

import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.diffswerve.robot.commands.OutliersCommand;
import org.frc5687.diffswerve.robot.subsystems.*;
import org.frc5687.diffswerve.robot.util.*;
import org.frc5687.lib.T265Camera;

public class RobotContainer extends OutliersContainer {

    private DriveTrain _driveTrain;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
    }

    public void init() {
        //        _driveTrain = new DriveTrain(this);
        //        setDefaultCommand(_driveTrain, new DriveSwerveModule(_driveTrain));
        int retryCount = 0;
        T265Camera slamra = null;
        while (++retryCount <= 1 && slamra == null) {
            try {
                slamra = new T265Camera(new Transform2d(), 0.5);
                SmartDashboard.putString("RobotContainer/vslamStatus", "OK");
            } catch (T265Camera.CameraJNIException | UnsatisfiedLinkError e) {
                slamra = null;
                error("RobotContainer: T265 camera is unavailable");
                error(e.getMessage());
                SmartDashboard.putString("RobotContainer/vslamStatus", "BAD!");
            }
        }
        slamra.start(
                (T265Camera.CameraUpdate update) -> {
                    metric("X pos", update.pose.getX());
                });
    }

    public void periodic() {}

    public void disabledPeriodic() {}
    ;

    @Override
    public void disabledInit() {}
    ;

    @Override
    public void teleopInit() {}
    ;

    @Override
    public void autonomousInit() {}

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    @Override
    public void updateDashboard() {
        //        _driveTrain.updateDashboard();
    }
}
