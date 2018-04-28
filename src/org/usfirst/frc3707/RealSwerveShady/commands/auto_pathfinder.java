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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.File;

import org.usfirst.frc3707.RealSwerveShady.Robot;

/**
 *
 */
public class auto_pathfinder extends Command {
	
	private Notifier notifier;
	private int currentSegment;

	private double max_velocity = 10.5;
	private double max_acceleration = 9;
	
	private int ticksPerRev = 138;
	private double wheelCirc = 1.0625;
	
	private double kv = 1.0/max_velocity;
	
	EncoderFollower frontLeft;
	EncoderFollower frontRight;
	EncoderFollower backLeft;
	EncoderFollower backRight;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public auto_pathfinder(String path) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSystem);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        notifier = new Notifier(this::followPath);
        
        //3.39
    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, 60.0);
    	
    	Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(1, 0, 0)
//                new Waypoint(-12, 0, 0),
//                new Waypoint(-14, -2, Pathfinder.d2r(45)),
//                new Waypoint(-14, -14, Pathfinder.d2r(90))
    			
                //new Waypoint(-2, 2, Pathfinder.d2r(-45)) //B LEFT
                //new Waypoint(-2, -2, Pathfinder.d2r(45)) //B RIGHT
                //new Waypoint(2, -2, Pathfinder.d2r(-45)) //F RIGHT
                //new Waypoint(2, 2, Pathfinder.d2r(45)) //F LEFT

                //new Waypoint(8, 0, 0)

                //new Waypoint(-6, 0, Pathfinder.d2r(0)) //F LEFT
        };
    	
    	if(path == "centerToRightSwitch") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(10.7, -5, 0)
            };
    	}
    	else if(path == "centerToLeftSwitch") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(10.5, 6.5, 0)
            };
    	}
    	else if(path == "leftToLeftScale") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(18, 0, Pathfinder.d2r(-10)),
                    new Waypoint(21, -1, Pathfinder.d2r(-30))
            };
    	}
    	else if(path == "leftToRightScale") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(18, 1.5, Pathfinder.d2r(-20)),
                    new Waypoint(22, -3.5, Pathfinder.d2r(90)),
                    new Waypoint(22, -15, Pathfinder.d2r(90))
            };
    	}
    	else if(path == "leftSwitchToCenterBox") {
    		points = new Waypoint[] {
                    new Waypoint(11, 6, 0),
                    new Waypoint(4.5, 4, Pathfinder.d2r(50))
            };
    	}
    	else if(path == "rightSwitchToCenterBox") {
    		points = new Waypoint[] {
                    new Waypoint(11, -5, 0),
                    new Waypoint(4.5, -3, Pathfinder.d2r(-50))
            };
    	}
    	else if(path == "centerBoxTorightSwitch") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(-180)),
                    new Waypoint(-2, -4, Pathfinder.d2r(90)),
                    new Waypoint(3.5, -6.5, 0)
            };
    	}
    	else if(path == "centerBoxToLeftSwitch") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(-180)),
                    new Waypoint(-2, 4, Pathfinder.d2r(90)),
                    new Waypoint(4.5, 7, 0)
            };
    	}
    	else if(path == "centerBoxToLeftScale") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(90)),
                    new Waypoint(0, 12, Pathfinder.d2r(45)),
                    new Waypoint(3, 13, Pathfinder.d2r(30)),
                    new Waypoint(16, 14, 0)
            };
    	}
    	else if(path == "centerBoxToLeftDefense") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(90)),
                    new Waypoint(0, 7, Pathfinder.d2r(45)),
                    new Waypoint(3, 10, Pathfinder.d2r(30)),
                    new Waypoint(13, 11, 0)
            };
    	}
    	else if(path == "right_centerBoxToLeftScale") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(90)),
                    new Waypoint(0, 12, Pathfinder.d2r(45)),
                    new Waypoint(3, 13, Pathfinder.d2r(30)),
                    new Waypoint(17, 13.5, 0)
            };
    	}
    	else if(path == "right_centerBoxToLeftDefense") {
    		points = new Waypoint[] {
                    new Waypoint(0, 0, Pathfinder.d2r(90)),
                    new Waypoint(0, 13, Pathfinder.d2r(45)),
                    new Waypoint(3, 14, Pathfinder.d2r(30)),
                    new Waypoint(13, 15, 0)
            };
    	}
    	
    	
    	
    	
    	Trajectory trajectory = null;
    	
    	try {
    		String filePath = "/home/lvuser/" + path + ".csv";
    		System.out.println("PATH FILE '" + filePath + "'");
    		File trajFile = new File(filePath);
    		
    		if(trajFile.exists()) {
    			trajectory = Pathfinder.readFromCSV(trajFile);
    		}
    		else {
    			trajectory = Pathfinder.generate(points, config);
    		}
    		
    		//COMMENT THIS OUT TO SET THE FILES
//    		trajectory = Pathfinder.generate(points, config);
//    		if(!trajFile.exists()) {
//    			trajFile.createNewFile();
//    		}
//    		Pathfinder.writeToCSV(trajFile, trajectory);
    	}
    	catch(Exception e) {
    		System.out.println(e);
    	}
        
        // Wheelbase Width = 0.5m, Wheelbase Depth = 0.6m, Swerve Mode = Default
        //SwerveModifier modifier = new SwerveModifier(trajectory).modify(2, 1.8, SwerveModifier.Mode.SWERVE_DEFAULT);
        //TankModifier tank = new TankModifier(trajectory).modify(2);
        
        // Do something with the new Trajectories...
        
//        frontLeft = new DistanceFollower(modifier.getFrontLeftTrajectory());
//        frontRight = new DistanceFollower(modifier.getFrontRightTrajectory());
//        backLeft = new DistanceFollower(modifier.getBackLeftTrajectory());
//        backRight = new DistanceFollower(modifier.getBackRightTrajectory());
        
        frontLeft = new EncoderFollower(trajectory);
        frontRight = new EncoderFollower(trajectory);
        backLeft = new EncoderFollower(trajectory);
        backRight = new EncoderFollower(trajectory);
        
        frontLeft.configureEncoder(0, ticksPerRev, wheelCirc);
        frontRight.configureEncoder(0, ticksPerRev, wheelCirc);
        backLeft.configureEncoder(0, ticksPerRev, wheelCirc);
        backRight.configureEncoder(0, ticksPerRev, wheelCirc);
        
        frontLeft.configurePIDVA(0, 0, 0, kv, 0);
        frontRight.configurePIDVA(0, 0, 0, kv, 0);
        backLeft.configurePIDVA(0, 0, 0, kv, 0);
        backRight.configurePIDVA(0, 0, 0, kv, 0);

    	SmartDashboard.putNumber("segments", trajectory.segments.length);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	currentSegment = 0;
    	Robot.driveSystem.enable();
    	frontLeft.reset();
    	frontRight.reset();
    	backLeft.reset();
    	backRight.reset();
    	//Robot.driveSystem.resetEncoders();
    	notifier.startPeriodic(frontLeft.getSegment().dt);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
//    	SmartDashboard.putNumber("PathTime", Timer.getFPGATimestamp());
//    	SmartDashboard.putNumber("currentSegment", currentSegment);
    	
//    	Trajectory.Segment seg = frontRight.getSegment();
//    	System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//    	        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//    	            seg.acceleration, seg.jerk, seg.heading);
//    	if(!frontRight.isFinished()) {
//    		
//    		Robot.driveSystem.swerve.driveDirect(frontRight,frontLeft,backLeft,backRight);
//    	}
//    	currentSegment++;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return frontRight.isFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	notifier.stop();
    	Robot.driveSystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }
    
    private void followPath() {
    	//SmartDashboard.putNumber("PathTime", Timer.getFPGATimestamp());
    	
    	if(!frontRight.isFinished()) {
    		
//	    	Trajectory.Segment seg = frontRight.getSegment();
//	    	System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//	    	        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//	    	            seg.acceleration, seg.jerk, seg.heading);
    		
    		//SmartDashboard.putNumber("FL_POSITION", frontLeft.getSegment().position);
    		//Robot.driveSystem.displayEncoders();

    	   /* 	Trajectory.Segment seg = frontRight.getSegment();
    	    	double fakeDistance = seg.position;
    	    	double speedx = frontRight.calculate((int)fakeDistance);
    	    	double heading = frontRight.getHeading();
    	    	
    	    	Robot.driveSystem.drive(directionX, directionY, 0, true, false);*/
    		
	    Robot.driveSystem.swerve.driveDirect(frontRight,frontLeft,backLeft,backRight);
	    	
    	}
    	
    }
}
