package org.frc5687.diffswerve.robot.subsystems;

import org.frc5687.diffswerve.robot.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _frontRight;

    public DriveTrain(OutliersContainer container){
        super(container);
        _frontRight = new DiffSwerveModule(DiffSwerveModule.ModuleID.FrontRight);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void updateDashboard() {
        metric("Right RF RPM",_frontRight.getRightFalconRPM());
        metric("Left RF RPM",_frontRight.getLeftFalconRPM());


    }

    public void setFrontRightSpeeds(double speedR, double speedL){
        _frontRight.setRightFalcon(speedR);
        _frontRight.setLeftFalcon(speedL);
    }
}
