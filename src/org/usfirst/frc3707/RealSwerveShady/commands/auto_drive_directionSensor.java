// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3707.RealSwerveShady.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3707.RealSwerveShady.Robot;
import org.usfirst.frc3707.RealSwerveShady.RobotMap;
import org.usfirst.frc3707.lib.pwinput.PWInput;

/**
 *
 */
public class auto_drive_directionSensor extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_angle;
    private double m_distance;
    private String m_sensor;
    private boolean m_min;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public auto_drive_directionSensor(double angle, double distance, String sensor, boolean min) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_angle = angle;
        m_distance = distance;
        m_sensor = sensor;
        m_min = min;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSystem);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	Robot.driveSystem.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	Robot.driveSystem.driveSimple(0.5, m_angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	switch(m_sensor) {
    	case "FRONT":
    		return isFrontDistanceComplete();
    	case "RIGHT":
    		return isRightDistanceComplete();
    	case "LEFT":
    		return isLeftDistanceComplete();
    	}
    	
    	//default to returning lidar
    	return isDistanceComplete(RobotMap.lidar);
        
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Robot.driveSystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	Robot.driveSystem.disable();
    }
    
    private boolean isFrontDistanceComplete() {
    	//return isDistanceComplete(RobotMap.forwardRightSonar) && isDistanceComplete(RobotMap.forwardLeftSonar);
    	return true;
    }
    private boolean isLeftDistanceComplete() {
    	//return isDistanceComplete(RobotMap.leftSideForSonar) && isDistanceComplete(RobotMap.leftSideAftSonar);
    	return true;
    }
    private boolean isRightDistanceComplete() {
    	//return isDistanceComplete(RobotMap.rightSideForSonar) && isDistanceComplete(RobotMap.rightSideAftSonar);
    	return true;
    }
    private boolean isBackDistanceComplete() {
    	//return isDistanceComplete(RobotMap.forwardRightSonar) && isDistanceComplete(RobotMap.forwardLeftSonar);
    	return true;
    }
    private boolean isDistanceComplete(PWInput sensor) {
    	if(m_min) {
    		return sensor.getDistance() < m_distance;
    	}
    	else {
    		return sensor.getDistance() > m_distance;
    	}
    }
}
