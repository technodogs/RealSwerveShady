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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import org.usfirst.frc3707.RealSwerveShady.Robot;

/**
 *
 */
public class auto_choose_direction extends ConditionalCommand {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public auto_choose_direction() {
      //super(new auto_drive_directionSensor(16, 20, "FRONT", true), new auto_drive_directionSensor(330, 20, "FRONT", true));
      super(new auto_drive_timer(2.3, 16.0, 0.5), new auto_drive_timer(2.3, 329.0, 0.5));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
      
      //HEY! Robot builder is showing an error on line above? Use this one instead!
      //super(new auto_drive_directionSensor(45, 280), new auto_drive_directionSensor(315, 280));

    }

    @Override
    protected boolean condition(){
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
        
    	//if 'R' returns true and runs first command from super above. Else runs second.
    	return gameData.charAt(0)=='R';
    }
}
