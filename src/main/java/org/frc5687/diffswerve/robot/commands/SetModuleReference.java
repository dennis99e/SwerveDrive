/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.commands;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;

public class SetModuleReference extends OutliersCommand {

    private DriveTrain _driveTrain;
    private SwerveModuleState _state;

    public SetModuleReference(DriveTrain driveTrain, SwerveModuleState state) {
        _driveTrain = driveTrain;
        _state = state;
    }

    @Override
    public void initialize() {
        super.initialize();
        //        _driveTrain.setFrontRightModuleState(_state);
        startPeriodic(0.05);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
