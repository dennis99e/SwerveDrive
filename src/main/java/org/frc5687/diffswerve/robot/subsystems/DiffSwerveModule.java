package org.frc5687.diffswerve.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.ejml.data.DMatrix2x2;
import org.frc5687.diffswerve.robot.Constants;
import org.frc5687.diffswerve.robot.RobotMap;

public class DiffSwerveModule {
    private TalonFX _rightFalcon; //TODO: correct names when model is finished.
    private TalonFX _leftFalcon; //TODO: correct names when model is finished.
    private AnalogEncoder _lampreyEncoder;
    private ModuleID _modID;
    private Translation2d _positionVector;
    private LinearSystem<N3, N2, N2> _swerveModule;
    private KalmanFilter<N3, N2, N2> _swerveObserver;
    private LinearQuadraticRegulator<N3,N2,N2> _swerveController;
    private LinearSystemLoop<N3,N2,N2> _swerveControlLoop;
    private Matrix<N3,N1> _reference; //same thing as a set point.

    public DiffSwerveModule(ModuleID id){
        _modID = id;
        _reference = Matrix.mat(Nat.N3(),Nat.N1()).fill(0,0,0);
        switch (_modID) {
            case FrontRight:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.FR_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.FR_LEFT_FALCON);
                _swerveModule = createDifferentialSwerveModule(DCMotor.getFalcon500(2),Constants.DifferentialSwerveModule.FrontRight.INERTIA_STEER,Constants.DifferentialSwerveModule.FrontRight.INERTIA_WHEEL,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_STEER, Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL
                );
                _swerveObserver = new KalmanFilter<>(Nat.N3(), Nat.N2(), _swerveModule,
                        Matrix.mat(Nat.N3(),Nat.N1()).fill(Units.degreesToRadians(2),Units.rotationsPerMinuteToRadiansPerSecond(50),Units.rotationsPerMinuteToRadiansPerSecond(50)),
                        Matrix.mat(Nat.N2(),Nat.N1()).fill(Units.degreesToRadians(0.001),Units.rotationsPerMinuteToRadiansPerSecond(3)),
                        0.020
                );
                _swerveController = new LinearQuadraticRegulator<>(_swerveModule, VecBuilder.fill(Constants.DifferentialSwerveModule.FrontLeft.Q_AZIMUTH,Constants.DifferentialSwerveModule.FrontLeft.Q_AZIMUTH_ANG_VELOCITY,Constants.DifferentialSwerveModule.FrontLeft.Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(12.0, 12.0),
                        0.020
                );
                _swerveControlLoop = new LinearSystemLoop<>(
                        _swerveModule,
                        _swerveController,
                        _swerveObserver,
                        12.0,
                        0.020
                );
                break;
            case FrontLeft:
                _positionVector = new Translation2d(-Constants.DriveTrain.WIDTH/2.0, Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.FL_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.FL_LEFT_FALCON);
                _swerveModule = createDifferentialSwerveModule(DCMotor.getFalcon500(2),Constants.DifferentialSwerveModule.FrontLeft.INERTIA_STEER,Constants.DifferentialSwerveModule.FrontLeft.INERTIA_WHEEL,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_STEER, Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL
                );
                _swerveObserver = new KalmanFilter<>(Nat.N3(), Nat.N2(), _swerveModule,
                        Matrix.mat(Nat.N3(),Nat.N1()).fill(Units.degreesToRadians(2),Units.rotationsPerMinuteToRadiansPerSecond(50),Units.rotationsPerMinuteToRadiansPerSecond(50)),
                        Matrix.mat(Nat.N2(),Nat.N1()).fill(Units.degreesToRadians(0.001),Units.rotationsPerMinuteToRadiansPerSecond(3)),
                        0.020
                );
                _swerveController = new LinearQuadraticRegulator<>(_swerveModule, VecBuilder.fill(Constants.DifferentialSwerveModule.FrontLeft.Q_AZIMUTH,Constants.DifferentialSwerveModule.FrontLeft.Q_AZIMUTH_ANG_VELOCITY,Constants.DifferentialSwerveModule.FrontLeft.Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(12.0, 12.0),
                        0.020
                );
                _swerveControlLoop = new LinearSystemLoop<>(
                        _swerveModule,
                        _swerveController,
                        _swerveObserver,
                        12.0,
                        0.020
                );

                break;
            case BottomRight:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, -Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.BR_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.BR_LEFT_FALCON);
                _swerveModule = createDifferentialSwerveModule(DCMotor.getFalcon500(2),Constants.DifferentialSwerveModule.BottomRight.INERTIA_STEER,Constants.DifferentialSwerveModule.BottomRight.INERTIA_WHEEL,
                    Constants.DifferentialSwerveModule.GEAR_RATIO_STEER, Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL);
                _swerveObserver = new KalmanFilter<>(Nat.N3(), Nat.N2(), _swerveModule,
                        Matrix.mat(Nat.N3(),Nat.N1()).fill(Units.degreesToRadians(2),Units.rotationsPerMinuteToRadiansPerSecond(50),Units.rotationsPerMinuteToRadiansPerSecond(50)),
                        Matrix.mat(Nat.N2(),Nat.N1()).fill(Units.degreesToRadians(0.001),Units.rotationsPerMinuteToRadiansPerSecond(3)),
                        0.020
                );
                _swerveController = new LinearQuadraticRegulator<>(_swerveModule, VecBuilder.fill(Constants.DifferentialSwerveModule.BottomRight.Q_AZIMUTH,Constants.DifferentialSwerveModule.BottomRight.Q_AZIMUTH_ANG_VELOCITY,Constants.DifferentialSwerveModule.BottomRight.Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(12.0, 12.0),
                        0.020
                );
                _swerveControlLoop = new LinearSystemLoop<>(
                        _swerveModule,
                        _swerveController,
                        _swerveObserver,
                        12.0,
                        0.020
                );
                break;
            case BottomLeft:
                _positionVector = new Translation2d(Constants.DriveTrain.WIDTH/2.0, -Constants.DriveTrain.LENGTH/2.0);
                _rightFalcon = new TalonFX(RobotMap.CAN.TALONFX.BL_RIGHT_FALCON);
                _leftFalcon = new TalonFX(RobotMap.CAN.TALONFX.BL_LEFT_FALCON);
                _swerveModule = createDifferentialSwerveModule(DCMotor.getFalcon500(2),Constants.DifferentialSwerveModule.BottomLeft.INERTIA_STEER,Constants.DifferentialSwerveModule.BottomLeft.INERTIA_WHEEL,
                    Constants.DifferentialSwerveModule.GEAR_RATIO_STEER, Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL);
                _swerveObserver = new KalmanFilter<>(Nat.N3(), Nat.N2(), _swerveModule,
                        Matrix.mat(Nat.N3(),Nat.N1()).fill(Units.degreesToRadians(2),Units.rotationsPerMinuteToRadiansPerSecond(50),Units.rotationsPerMinuteToRadiansPerSecond(50)),
                        Matrix.mat(Nat.N2(),Nat.N1()).fill(Units.degreesToRadians(0.001),Units.rotationsPerMinuteToRadiansPerSecond(3)),
                        0.020
                );
                _swerveController = new LinearQuadraticRegulator<>(_swerveModule, VecBuilder.fill(Constants.DifferentialSwerveModule.BottomLeft.Q_AZIMUTH,Constants.DifferentialSwerveModule.BottomLeft.Q_AZIMUTH_ANG_VELOCITY,Constants.DifferentialSwerveModule.BottomLeft.Q_WHEEL_ANG_VELOCITY),
                        VecBuilder.fill(12.0, 12.0),
                        0.020
                );
                _swerveControlLoop = new LinearSystemLoop<>(
                        _swerveModule,
                        _swerveController,
                        _swerveObserver,
                        12.0,
                        0.020
                );
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
        return 0;//_lampreyEncoder.getDistance()*(2.0*Math.PI); //TODO: Gear Ratio.
    }

    public double getWheelAngularVelocity(){
        return (getLeftFalconRPM() * Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL * getRightFalconRPM() * Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL)/2.0;
    }
    public double getRightFalconRPM(){
        return _rightFalcon.getSelectedSensorVelocity() / Constants.DriveTrain.TICKS_TO_ROTATIONS * 600.0 * Constants.DriveTrain.GEAR_RATIO;
    }

    public double getLeftFalconRPM(){
        return _leftFalcon.getSelectedSensorVelocity() / Constants.DriveTrain.TICKS_TO_ROTATIONS * 600.0 * Constants.DriveTrain.GEAR_RATIO;
    }
    public void periodic(){
        _swerveControlLoop.setNextR(_reference);
        _swerveControlLoop.correct(VecBuilder.fill(getModuleAngle(),getWheelAngularVelocity()));
        _swerveControlLoop.predict(0.020);
    }
    public void setReference(Matrix<N3,N1> reference){
       _reference = reference;
    }
    public double getLeftNextVoltage(){
        return _swerveControlLoop.getU(0);
    }
    public double getRightNextVoltage(){
        return _swerveControlLoop.getU(1);
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

    public static LinearSystem<N3, N2, N2> createDifferentialSwerveModule(DCMotor motor, double Js, double Jw, double Gs, double Gw){
        var Cs = -((Gs * motor.m_KtNMPerAmp)/(motor.m_KvRadPerSecPerVolt * motor.m_rOhms * Js));
        var Cw = -((Gw * motor.m_KtNMPerAmp)/(motor.m_KvRadPerSecPerVolt * motor.m_rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.m_KtNMPerAmp)/(motor.m_rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.m_KtNMPerAmp)/(motor.m_rOhms * Jw));

        var A = Matrix.mat(Nat.N3(), Nat.N3()).fill(
                0.0, 1.0, 0.0,
                0.0, Gs*Cs, 0.0,
                0.0, 0.0, Gw*Cw);
        var B = Matrix.mat(Nat.N3(),Nat.N2()).fill(
                0.0, 0.0,
                Vs, -Vs,
                Vw, Vw);
        var C = Matrix.mat(Nat.N2(),Nat.N3()).fill(
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0);
        var D = Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0.0, 0.0,
                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }
}
