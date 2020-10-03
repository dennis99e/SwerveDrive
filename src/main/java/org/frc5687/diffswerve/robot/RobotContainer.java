package org.frc5687.diffswerve.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.diffswerve.robot.subsystems.*;
import org.frc5687.diffswerve.robot.util.*;

public class RobotContainer extends OutliersContainer{


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
    }
    public void init() {

    }

    public void periodic() {

    }

    public void disabledPeriodic() {

    };

    @Override
    public void disabledInit() {

    };

    @Override
    public void teleopInit() {

    };

    @Override
    public void autonomousInit() {
    }

    @Override
    public void updateDashboard() {

    }


}
