// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3707.RealSwerveShady.subsystems;

import org.usfirst.frc3707.RealSwerveShady.Robot;
import org.usfirst.frc3707.RealSwerveShady.RobotMap;
import org.usfirst.frc3707.RealSwerveShady.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class clawSubsystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final DigitalInput boxState = RobotMap.clawSubsystemboxState;
    private final WPI_VictorSPX leftClaw = RobotMap.clawSubsystemleftClaw;
    private final WPI_VictorSPX rightClaw = RobotMap.clawSubsystemrightClaw;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new joystick_claw_inAndOut());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public void succ(double succSpeed) {
    	leftClaw.set(succSpeed*-1);
    	rightClaw.set(succSpeed);

    }
    public void spit(double spitSpeed) {
    	spit(spitSpeed, false);
    }
    public void spit(double spitSpeed, boolean slow) {
    	if(slow) {
	    	leftClaw.set(spitSpeed * 0.5);
	    	rightClaw.set((spitSpeed * 0.5)*-1);
    	}
    	else {
    		leftClaw.set(spitSpeed);
	    	rightClaw.set(spitSpeed*-1);
    	}


    }
    public boolean  haveBox () {
    	return !boxState.get();
    }
   
    

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

