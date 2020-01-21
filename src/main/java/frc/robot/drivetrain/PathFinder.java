package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class PathFinder extends CommandBase {

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
       Robot.driveTrain.stopDrive();
    }
      
    private void stop(){
        isFinished=true;
    }
    
    @Override
    public boolean isFinished() {
      return isFinished;
    }
}