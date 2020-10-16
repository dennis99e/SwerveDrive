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
        //                _driveTrain.setFrontRightReference(Matrix.mat(Nat.N3(), Nat.N1()).fill(0,
        // 0, 0));
    }

    @Override
    public void execute() {
        super.execute();
        //        _driveTrain.setFrontRightReference(Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 10));
        //        _driveTrain.setFrontRightVoltage(
        //                _driveTrain.getFrontRightWantedVoltages()[0],
        //                _driveTrain.getFrontRightWantedVoltages()[1]);
        //        _driveTrain.setFrontRightVoltage(1,1);
        //        _driveTrain.setFrontRightVelocity(100);
        //        _driveTrain.setFrontRightSpeeds(0.15, 0.15);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
