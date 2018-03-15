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

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc3707.RealSwerveShady.subsystems.*;

/**
 *
 */
public class auto_move_toSwitch_right extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public auto_move_toSwitch_right() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

    	addSequential(new dashboard_resetGyro());
    	addParallel(new auto_lift_moveToMiddle(),3);
    	
    	addSequential(new auto_drive_timer(0.5, 90.0, 0.3));
    	addSequential(new auto_drive_timer(1.5, 90.0, 0.75));
    	addSequential(new auto_drive_timer(0.2, 90.0, 0.3));
    	
    	addSequential(new auto_drive_timer(1, 0, 0.4));
    	
        addSequential(new auto_claw_spitTimed(2));
 
    } 
}