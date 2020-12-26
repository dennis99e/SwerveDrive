/* (C)2020 */
package org.frc5687.diffswerve.robot;

import com.spartronics4915.lib.hardware.sensors.T265Camera;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.diffswerve.robot.commands.OutliersCommand;
import org.frc5687.diffswerve.robot.subsystems.*;
import org.frc5687.diffswerve.robot.util.*;

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
                slamra = new T265Camera(new Pose2d(0, 0, new Rotation2d()), 0.5);
                SmartDashboard.putString("RobotContainer/vslamStatus", "OK");
            } catch (T265Camera.CameraJNIException | UnsatisfiedLinkError e) {
                slamra = null;
                error("RobotContainer: T265 camera is unavailable");
                error(e.getMessage());
                SmartDashboard.putString("RobotContainer/vslamStatus", "BAD!");
            }
        }
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
