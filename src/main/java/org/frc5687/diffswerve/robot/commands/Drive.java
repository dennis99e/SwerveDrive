/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.commands;

import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.OI;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private DriveTrain _driveTrain;
    private OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double stickX = _oi.getDriveY() * Constants.DriveTrain.MAX_MPS;
        double stickY = _oi.getDriveX() * Constants.DriveTrain.MAX_MPS;
        double rot = _oi.getRotationX() * Constants.DriveTrain.MAX_ANG_VEL;
        _driveTrain.drive(stickX, stickY, rot, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
