/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.commands;

import org.frc5687.diffswerve.robot.OI;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;
import org.frc5687.diffswerve.robot.util.Vector2d;

public class DriveSwerveModule extends OutliersCommand {

    private DriveTrain _driveTrain;
    private OI _oi;

    public DriveSwerveModule(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        startPeriodic(0.005);
    }

    @Override
    public void execute() {
        super.execute();
        double stickY = _oi.getDriveY();
        double stickX = _oi.getDriveX();
        Vector2d drive = new Vector2d(stickX, stickY);
        _driveTrain.setBottomLeftModuleVector(drive);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
