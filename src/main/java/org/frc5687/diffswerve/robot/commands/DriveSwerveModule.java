/* (C)2020 */
package org.frc5687.diffswerve.robot.commands;

import static org.frc5687.diffswerve.robot.util.Helpers.limit;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.OI;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;
import org.frc5687.diffswerve.robot.util.Helpers;

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
    }

    @Override
    public void execute() {
        super.execute();
        double stickY = _oi.getDriveY();
        double stickX = _oi.getDriveX();
        double pow = limit(Math.sqrt(stickX * stickX + stickY * stickY), -1.0, 1.0);
        metric("Power", pow * Constants.DifferentialSwerveModule.MAX_RADS);
        double theta = Helpers.boundHalfAngle(Math.atan2(stickY, stickX), true);
        _driveTrain.setFrontRightModuleState(
                new SwerveModuleState(
                        pow
                                * Constants.DifferentialSwerveModule.MAX_RADS
                                * Constants.DifferentialSwerveModule.WHEEL_RADIUS,
                        new Rotation2d(theta)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
