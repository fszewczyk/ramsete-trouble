package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.TurnToAngle;



public class RobotContainer { 

  public Joystick joystick = new Joystick(Constants.PAD_PORT);
  public JoystickButton yButton = new JoystickButton(joystick, Constants.Y_BUTTON);

  public RobotContainer() {
     configureButtonBindings();
  }

  public double getLX() {
    return joystick.getRawAxis(Constants.LX_AXIS);
  }

  public double getLY() {
    return joystick.getRawAxis(Constants.LY_AXIS) * -1;
  }

  public double getRX() {
    return joystick.getRawAxis(Constants.RX_AXIS);
  }

  public double getRY() {
    return joystick.getRawAxis(Constants.RY_AXIS) * -10;
  }

  public double getLTrigger() {
    return joystick.getRawAxis(Constants.L_TRIGGER);
  }

  public double getRTrigger() {
    return joystick.getRawAxis(Constants.R_TRIGGER);
  }

  private void configureButtonBindings() {
    yButton.whenPressed(new TurnToAngle());
  }
/*
  public Command getAutonomousCommand() {
    return m_autoCommand;
 }
 */
}
