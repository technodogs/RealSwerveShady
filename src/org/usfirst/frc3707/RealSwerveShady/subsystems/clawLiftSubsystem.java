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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class clawLiftSubsystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController clawLiftMotor = RobotMap.clawLiftSubsystemclawLiftMotor;
    private final DigitalInput liftLow = RobotMap.clawLiftSubsystemliftLow;
    private final DigitalInput liftMiddle = RobotMap.clawLiftSubsystemliftMiddle;
    private final DigitalInput liftHigh = RobotMap.clawLiftSubsystemliftHigh;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new joystick_lift_upAndDown());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public void upDown(double upDownPower) {
    	clawLiftMotor.set(upDownPower);
    }
    public void stopClawLift() {
    	clawLiftMotor.set(0.12);
    }
    public boolean liftLowPos () {
    	return !liftLow.get();
    }
    public boolean liftMiddlePos () {
    	return !liftMiddle.get();
    }
    public boolean liftHighPos () {
    	return !liftHigh.get();
    }
    public void moveLiftUp() {
    	clawLiftMotor.set(1);
    }
    public void moveLiftUpSlow() {
    	clawLiftMotor.set(0.6);
    }
    public void moveLiftDown () {
    	clawLiftMotor.set(-0.8);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

