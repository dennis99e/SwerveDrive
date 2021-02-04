/* (C)2020-2021 */
package org.frc5687.diffswerve.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Path;

import org.frc5687.diffswerve.robot.commands.OutliersCommand;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;
import org.frc5687.diffswerve.robot.commands.Drive;
import org.frc5687.diffswerve.robot.subsystems.OutliersSubsystem;
import org.frc5687.diffswerve.robot.util.OutliersContainer;
import org.frc5687.lib.T265Camera;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private T265Camera _slamCamera;

    private Robot _robot;
    private DriveTrain _driveTrain;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
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
        String trajectoryJSON = "output/BarrelRace.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            error("Trajectory successfully opened.");
        } catch (IOException ex) {
            error("Unable to open trajectory: " + trajectoryJSON + ex.getMessage());
        }

        _driveTrain = new DriveTrain(this, _imu, _slamCamera);
        _oi.initializeButtons(_driveTrain, trajectory);
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
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

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
