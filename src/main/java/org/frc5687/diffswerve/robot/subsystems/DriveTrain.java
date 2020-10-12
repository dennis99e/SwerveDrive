package org.frc5687.diffswerve.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.diffswerve.robot.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _frontRight;

    public DriveTrain(OutliersContainer container){
        super(container);
        _frontRight = new DiffSwerveModule(DiffSwerveModule.ModuleID.FrontRight);
        logMetrics("Left RPM","Right RPM");
        enableMetrics();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void updateDashboard() {
        metric("Right RPM",_frontRight.getRightFalconRPM());
        metric("Left RPM",_frontRight.getLeftFalconRPM());
    }

    public void setFrontRightSpeeds(double speedR, double speedL){
        _frontRight.setRightFalcon(speedR);
        _frontRight.setLeftFalcon(speedL);
    }
    public void setFrontRightVelocity(double RPM){
        _frontRight.setVelocityRPM(RPM);
    }

}
