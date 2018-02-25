// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3707.RealSwerveShady;

import org.usfirst.frc3707.RealSwerveShady.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Joystick xbox;
    public JoystickButton a;
    public JoystickButton y;
    public JoystickButton x;
    public JoystickButton b;
    public Joystick xbox360;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        xbox360 = new Joystick(1);
        
        b = new JoystickButton(xbox360, 2);
        b.whenPressed(new ohSchnitzel());
        x = new JoystickButton(xbox360, 3);
        x.whenPressed(new moveLiftToMiddle());
        y = new JoystickButton(xbox360, 4);
        y.whenPressed(new moveLiftToHigh());
        a = new JoystickButton(xbox360, 1);
        a.whenPressed(new moveLiftToLow());
        xbox = new Joystick(0);
        


        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Drive", new Drive());
        SmartDashboard.putData("upAndDown", new upAndDown());
        SmartDashboard.putData("inAndOut", new inAndOut());
        SmartDashboard.putData("hoist", new hoist());
        SmartDashboard.putData("hasBox", new hasBox());
        SmartDashboard.putData("moveLiftToMiddle", new moveLiftToMiddle());
        SmartDashboard.putData("moveLiftToLow", new moveLiftToLow());
        SmartDashboard.putData("moveLiftToHigh", new moveLiftToHigh());
        SmartDashboard.putData("ohSchnitzel", new ohSchnitzel());
        SmartDashboard.putData("AutoSpit", new AutoSpit());
        SmartDashboard.putData("AutoLiftAndSpit", new AutoLiftAndSpit());
        SmartDashboard.putData("AutoDriveTime", new AutoDriveTime());
        SmartDashboard.putData("AutoSpitTimed", new AutoSpitTimed());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getxbox() {
        return xbox;
    }

    public Joystick getxbox360() {
        return xbox360;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

