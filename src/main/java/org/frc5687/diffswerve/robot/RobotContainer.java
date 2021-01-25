/* (C)2020-2021 */
package org.frc5687.diffswerve.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.diffswerve.robot.commands.*;
import org.frc5687.diffswerve.robot.commands.OutliersCommand;
import org.frc5687.diffswerve.robot.subsystems.*;
import org.frc5687.diffswerve.robot.util.*;
import org.frc5687.lib.T265Camera;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private T265Camera _slamCamera;

    private DriveTrain _driveTrain;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
    }

    public void init() {
        int counter = 0;
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _slamCamera = null;

        while (++counter <= 1 && _slamCamera == null) {
            try {
                _slamCamera =
                        new T265Camera(
                                Constants.DriveTrain.SLAM_TO_ROBOT,
                                Constants.DriveTrain.T265_MEASUREMENT_COVARIANCE);
                metric("Slam Camera Status", "Working");
            } catch (T265Camera.CameraJNIException | UnsatisfiedLinkError e) {
                _slamCamera = null;
                error("T265Camera not found");
                error(e.getMessage());
                metric("Slam Camera Status", "Broken!");
            }
        }

        _driveTrain = new DriveTrain(this, _imu, _slamCamera);
        _oi.initializeButtons(_driveTrain);
        setDefaultCommand(_driveTrain, new DriveSwerveModule(_driveTrain, _oi));
        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

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

        _driveTrain.updateDashboard();
    }
}
