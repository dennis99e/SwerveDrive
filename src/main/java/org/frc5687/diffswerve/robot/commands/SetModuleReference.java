/* (C)2020 */
package org.frc5687.diffswerve.robot.commands;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;

public class SetModuleReference extends OutliersCommand {

    private DriveTrain _driveTrain;
    private Matrix<N3, N1> _reference;

    public SetModuleReference(DriveTrain driveTrain, Matrix<N3, N1> reference) {
        _driveTrain = driveTrain;
        _reference = reference;
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setFrontRightReference(_reference);
        startPeriodic(0.05);
    }

    @Override
    public void execute() {
        super.execute();
        _driveTrain.setFrontRightVoltage(
                _driveTrain.getFrontRightWantedVoltages()[0],
                _driveTrain.getFrontRightWantedVoltages()[1]);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
