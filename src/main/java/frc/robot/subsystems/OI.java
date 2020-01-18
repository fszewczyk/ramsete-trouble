/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OI extends SubsystemBase {
  
  public Joystick pad;
  
  public OI() 
  {
    pad = new Joystick(Constants.PAD_PORT);
    
  }

  public double getLX() {
    return pad.getRawAxis(Constants.LX_AXIS);
  }
  
  public double getLY() {
    return pad.getRawAxis(Constants.LY_AXIS) * -1;
  }
  
  public double getRX() {
    return pad.getRawAxis(Constants.RX_AXIS);
  }

  public double getRY() {
    return pad.getRawAxis(Constants.RY_AXIS) * -1;
  }

  public double getLTrigger() {
    return pad.getRawAxis(Constants.L_TRIGGER);
  }

  public double getRTrigger() {
    return pad.getRawAxis(Constants.R_TRIGGER);
  }
  public boolean getYButton()
  {
    return pad.getRawButton(Constants.Y_BUTTON);
  }

  @Override
  public void periodic() 
  {
  }
}
