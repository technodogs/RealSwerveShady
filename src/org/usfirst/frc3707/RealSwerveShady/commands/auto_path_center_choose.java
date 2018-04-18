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
public class auto_path_center_choose extends ConditionalCommand {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public auto_path_center_choose() {
      super(new auto_path_center_leftSwitch(), new auto_path_center_rightSwitch());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    }

    @Override
    protected boolean condition(){
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
        
    	//if 'L' returns true and runs first command from super above. Else runs second.
    	return gameData.charAt(0)=='L';
    }
}
