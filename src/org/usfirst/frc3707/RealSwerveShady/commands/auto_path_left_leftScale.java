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
public class auto_path_left_leftScale extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public auto_path_left_leftScale() {

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
    	
    	/**** LEFT TO LEFT SCALE ***/
    	addSequential(new dashboard_resetGyro());
    	addSequential(new auto_lift_moveToMiddle(),0.2);
    	addSequential(new auto_lift_moveToLow(),0.2);
    	addSequential(new auto_delay(0.2));
    	addParallel(new auto_lift_moveToMiddle(),1.6);
    	addSequential(new auto_pathfinder("leftToLeftScale"));
    	
    	addSequential(new auto_drive_rotateToAngle(35), 0.3);
    	addParallel(new auto_drive_timer(1.5, 0, 0.2));
    	addSequential(new auto_lift_moveToHigh(),2);
    	addSequential(new auto_claw_spitTimed(0.6));
    	addSequential(new auto_drive_timer(0.5, 0, -0.2), 0.5);
    	
    	addParallel(new auto_lift_moveToLow(), 3);
    	addSequential(new auto_drive_rotateToAngle(140), 1.5);
    	
    	addParallel(new auto_claw_suckTimed(2));
    	addSequential(new auto_drive_timer(2, 0, 0.4));
    	addSequential(new auto_drive_timer(1, 0, -0.4));
    	
    	addParallel(new auto_lift_moveToHigh(),2.5);
    	addSequential(new auto_drive_rotateToAngle(35), 1.5);
    	addSequential(new auto_drive_timer(1.5, 0, 0.3));
    	addSequential(new auto_claw_spitTimed(0.6));
    	
    	addSequential(new auto_drive_timer(0.5, 0, -0.2), 0.5);
    	addParallel(new auto_lift_moveToLow(), 3);
    	
    	/**** LEFT TO LEFT SCALE 67 out of way ***/
//    	addSequential(new dashboard_resetGyro());
//    	addParallel(new auto_lift_moveToMiddle(),1.6);
//    	addSequential(new auto_pathfinder("leftToLeftScale"));
//    	
//
//    	addSequential(new auto_drive_rotateToAngle(35), 0.3);
//    	addParallel(new auto_drive_timer(1.5, 0, 0.2));
//    	addSequential(new auto_lift_moveToHigh(),2);
//    	addSequential(new auto_claw_spitTimed(0.6));
//    	addSequential(new auto_drive_timer(0.5, 0, -0.2), 0.5);
//    	
//    	addParallel(new auto_lift_moveToLow(), 3);
//    	//addSequential(new auto_drive_rotateToAngle(90), 1.5);
//    	addSequential(new auto_drive_timer(1.5, 270, -0.4));
//    	addSequential(new auto_drive_timer(1.2, 0, 0.4));
    	
    	
 
    } 
}
