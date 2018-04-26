
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

import org.usfirst.frc3707.RealSwerveShady.RobotMap;
import org.usfirst.frc3707.RealSwerveShady.commands.*;

import org.usfirst.frc3707.lib.swerve.SwerveDrive;
import org.usfirst.frc3707.lib.swerve.SwerveWheel;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveSystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final AnalogPotentiometer frontRightEncoder = RobotMap.driveSystemFrontRightEncoder;
    private final WPI_VictorSPX frontRightSwerve = RobotMap.driveSystemFrontRightSwerve;
    private final PIDController frontRightTwist = RobotMap.driveSystemFrontRightTwist;
    private final AnalogPotentiometer frontLeftEncoder = RobotMap.driveSystemFrontLeftEncoder;
    private final WPI_VictorSPX frontLeftSwerve = RobotMap.driveSystemFrontLeftSwerve;
    private final PIDController frontLeftTwist = RobotMap.driveSystemFrontLeftTwist;
    private final AnalogPotentiometer backRightEncoder = RobotMap.driveSystemBackRightEncoder;
    private final WPI_VictorSPX backRightSwerve = RobotMap.driveSystemBackRightSwerve;
    private final PIDController backRightTwist = RobotMap.driveSystemBackRightTwist;
    private final AnalogPotentiometer backLeftEncoder = RobotMap.driveSystemBackLeftEncoder;
    private final WPI_VictorSPX backLeftSwerve = RobotMap.driveSystemBackLeftSwerve;
    private final PIDController backLeftTwist = RobotMap.driveSystemBackLeftTwist;
    private final SpeedController frontRightDrive = RobotMap.driveSystemFrontRightDrive;
    private final SpeedController frontLeftDrive = RobotMap.driveSystemFrontLeftDrive;
    private final SpeedController backRightDrive = RobotMap.driveSystemBackRightDrive;
    private final SpeedController backLeftDrive = RobotMap.driveSystemBackLeftDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new joystick_drive_swerve());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public final ADXRS450_Gyro gyro = RobotMap.gyro;
    
      //COMP ROBOT
    SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwist, frontLeftDrive, 92); //340
    SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwist, frontRightDrive, 161); //341
    SwerveWheel backLeftWheel = new SwerveWheel(backLeftTwist, backLeftDrive, 41); //210
    SwerveWheel backRightWheel = new SwerveWheel(backRightTwist, backRightDrive, 109); //12
   
    // PRATICE ROBOT
//    SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwist, frontLeftDrive, 219);
//    SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwist, frontRightDrive, 210);
//    SwerveWheel backLeftWheel = new SwerveWheel(backLeftTwist, backLeftDrive, 200);
//    SwerveWheel backRightWheel = new SwerveWheel(backRightTwist, backRightDrive, 326);
    
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, backLeftWheel, backRightWheel, gyro);
    
    public void enable() {
	    	frontLeftTwist.enable();
	    	frontRightWheel.enable();
	    	backLeftWheel.enable();
	    	backRightWheel.enable();
    }
    
    public void disable() {
	    	frontLeftTwist.disable();
	    	frontRightWheel.disable();
	    	backLeftWheel.disable();
	    	backRightWheel.disable();
    }
    public void drive(double directionX, double directionY, double rotation, boolean useGyro, boolean slowSpeed) {
	    	swerve.drive(directionX, directionY, rotation, useGyro, slowSpeed);
	    	SmartDashboard.putData(gyro);
	    	//displayEncoders();
    }
    
    public void publishCanDashboard() {
    		SmartDashboard.putBoolean("frontRight_SwerveID", frontRightSwerve.isAlive());
    		SmartDashboard.putBoolean("frontLeft_SwerveID", frontLeftSwerve.isAlive());
    		SmartDashboard.putBoolean("backRight_SwerveID", backRightSwerve.isAlive());
    		SmartDashboard.putBoolean("backLeft_SwerveID", backLeftSwerve.isAlive());	
    }

//    public void resetEncoders() {
//	    	backLeftDistanceEncoder.reset();
//	    	backRightDistanceEncoder.reset();
//	    	frontLeftDistanceEncoder.reset();
//	    	frontRightDistanceEncoder.reset();
//    }
//    public void displayEncoders() {
//    	SmartDashboard.putNumber("BL", backLeftDistanceEncoder.getDistance());
//    	SmartDashboard.putNumber("BR", backRightDistanceEncoder.getDistance());
//    	SmartDashboard.putNumber("FL", frontLeftDistanceEncoder.getDistance());
//    	SmartDashboard.putNumber("FR", frontRightDistanceEncoder.getDistance());
//    	//System.out.println(frontLeftDistanceEncoder.getDistance());
//    }
    public void driveSimple(double speed, double angle) {
    		swerve.driveSimple(speed, angle);
    }
    
    public void lockStop() {
    		swerve.lockStop();
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

