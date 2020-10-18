/* (C)2020 */
package org.frc5687.diffswerve.robot.commands;

import org.frc5687.diffswerve.robot.subsystems.DriveTrain;

public class DriveSwerveModule extends OutliersCommand {

    private DriveTrain _driveTrain;

    public DriveSwerveModule(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _driveTrain.setFrontRightVoltage(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
