/* (C)2020-2021 */
package org.frc5687.diffswerve.robot;

import static org.frc5687.diffswerve.robot.util.Helpers.applyDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.diffswerve.robot.subsystems.DriveTrain;
import org.frc5687.diffswerve.robot.util.Gamepad;
import org.frc5687.diffswerve.robot.util.OutliersProxy;
import org.frc5687.diffswerve.robot.util.POV;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Button _driverRightStickButton;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    public OI() {
        _driverGamepad = new Gamepad(0);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
    }

    public void initializeButtons(DriveTrain driveTrain) {
        //        _driverAButton.whenPressed(null);
        //        _driverBButton.whenPressed(null);
        //        _driverXButton.whenPressed(null);
    }

    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverGamepad.getPOV()).getDirectionValue();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
