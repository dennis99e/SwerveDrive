package org.frc5687.diffswerve.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.RobotMap;

public class DiffSwerveModule {
    private TalonFX _rightFalcon; //TODO: correct names when model is finished.
    private TalonFX _leftFalcon; //TODO: correct names when model is finished.
    private AnalogEncoder _lampreyEncoder;
    private ModuleID _modID;
    private Translation2d _positionVector;

    public DiffSwerveModule(ModuleID id){
        _modID = id;
        switch (_modID) {
            case FrontRight:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.FR_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.FR_LEFT_FALCON);
                break;
            case FrontLeft:
                _positionVector = new Translation2d(-Constants.DriveTrain.WIDTH/2.0, Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.FL_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.FL_LEFT_FALCON);
                break;
            case BottomRight:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, -Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.BR_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.BR_LEFT_FALCON);
                break;
            case BottomLeft:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, -Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.BL_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.BL_LEFT_FALCON);
                break;
        }
        _rightFalcon.setInverted(Constants.DriveTrain.RIGHT_INVERTED);
        _leftFalcon.setInverted(Constants.DriveTrain.LEFT_INVERTED);
        _rightFalcon.setSensorPhase(false);
        _leftFalcon.setSensorPhase(false);

        _rightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,200);
        _leftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,200);
        _rightFalcon.configForwardSoftLimitEnable(false);
        _leftFalcon.configForwardSoftLimitEnable(false);
        _rightFalcon.config_kP(0,Constants.DriveTrain.VELOCITY_KP,200);
        _rightFalcon.config_kI(0,Constants.DriveTrain.VELOCITY_KI,200);
        _rightFalcon.config_kD(0,Constants.DriveTrain.VELOCITY_KD,200);
        _rightFalcon.config_kF(0,Constants.DriveTrain.VELOCITY_KF,200);
        _leftFalcon.config_kP(0,Constants.DriveTrain.VELOCITY_KP,200);
        _leftFalcon.config_kI(0,Constants.DriveTrain.VELOCITY_KI,200);
        _leftFalcon.config_kD(0,Constants.DriveTrain.VELOCITY_KD,200);
        _leftFalcon.config_kF(0,Constants.DriveTrain.VELOCITY_KF,200);
        _rightFalcon.configClosedloopRamp(0);
        _leftFalcon.configClosedloopRamp(0);
    }



    public void setRightFalcon(double speed){
        _rightFalcon.set(ControlMode.PercentOutput,speed);
    }
    public void setLeftFalcon(double speed){
        _leftFalcon.set(ControlMode.PercentOutput,speed);
    }

    public void setVelocityRPM(double RPM){
        _rightFalcon.set(ControlMode.Velocity,(RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS/ 600/ 1));
        _leftFalcon.set(ControlMode.Velocity,(RPM * Constants.DriveTrain.TICKS_TO_ROTATIONS/ 600/ 1));
    }

    public double getModuleAngle(){
        return _lampreyEncoder.getDistance()*(2.0*Math.PI); //TODO: Gear Ratio.
    }

    public double getRightFalconRPM(){
        return _rightFalcon.getSelectedSensorVelocity() / Constants.DriveTrain.TICKS_TO_ROTATIONS * 600.0 * Constants.DriveTrain.GEAR_RATIO;
    }

    public double getLeftFalconRPM(){
        return _leftFalcon.getSelectedSensorVelocity() / Constants.DriveTrain.TICKS_TO_ROTATIONS * 600.0 * Constants.DriveTrain.GEAR_RATIO;
    }

    public enum ModuleID{
        FrontRight(0),
        FrontLeft(1),
        BottomRight(2),
        BottomLeft(3);

        private int _value;

        ModuleID(int value){
            _value = value;
        }
        public int getValue(){return _value;}

    }
}
