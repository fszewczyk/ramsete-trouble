/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.Constants;
import frc.robot.Robot;


public class OISubsystem extends SubsystemBase {
  /**
   * Creates a new OISubsystem.
   */
  
  public Joystick taranis;
  
  public OISubsystem(int port) 
  {
    taranis = new Joystick(port);
    
  }
  
  public double getLY()
  {
    return taranis.getRawAxis(Constants.LY_AXIS) * -1; //forward-back 
  }
  
  public double getRX()
  {
    return taranis.getRawAxis(Constants.RX_AXIS); //right-left
  }

  @Override
  public void periodic() 
  {
    double SA = taranis.getRawAxis(Constants.SA_AXIS); // intake-shoot
    double SC = taranis.getRawAxis(Constants.SC_AXIS); //robot MODE ride-pullup
    double SF = taranis.getRawAxis(Constants.SF_AXIS); //-1->elevator down 0->nothing 1-> up 

    if(SF > 0) //drive MODE
    {
      Robot.driveMode = true;

    }
    else //elevator MODE
    {
      Robot.driveMode = false;
    } 
  }
}
