package org.usfirst.frc3707.lib.swerve;

import org.usfirst.frc3707.RealSwerveShady.Robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;


public class SwerveWheel {
    private PIDController rotation;
    private SpeedController speed;
    private final Encoder encoder;
    private double offset;
    private double kv = 1.0/10.5;
    

	double last_error = 0.0;

    public SwerveWheel(PIDController rotation, SpeedController speed, double offset, Encoder encoder){
        this.rotation  = rotation;
        this.speed = speed;
        this.offset = offset;
        this.encoder = encoder;
    }
    
    public void drive(double newSpeed, double newAngle) {
	    	updateSpeed(newSpeed);
	    	updateRotation(newAngle);
    }
    
    public void drivePath(EncoderFollower follower) {
    	
    	if(!follower.isFinished()) {

	    	Trajectory.Segment seg = follower.getSegment();
	    	double fakeDistance = seg.position;
	    	double speedx = follower.calculate((int) encoder.getDistance());
	    	calculate((int) encoder.getDistance(), follower);
	    	
	    	int encD = (int) encoder.getDistance();
	    	//double dis = ((double) encD / 100) * 1; 
	    	double dis = (double)encD;
	    	double err = fakeDistance - dis;
	    	
	    	double speed = kv * seg.velocity;
	    	double heading = follower.getHeading();
	        
	    	//System.out.printf("%f\t%f\t%f\t%f\t%f\n", 
	    	//		speed, speedx, dis, err, fakeDistance);
	    	updateSpeed(speedx);
	    	
	    	
	    	double gyroAngle = Robot.driveSystem.gyro.getAngle();
			gyroAngle = Math.IEEEremainder(gyroAngle, 360);
			
	    	updateRotation(Pathfinder.r2d(heading) + gyroAngle);
	    	//updateRotation(180);
    	}
    }

    public void updateSpeed(double newSpeed){
        speed.set(newSpeed);
    }

    public void updateRotation(double newAngle){
	    	newAngle = newAngle + offset;
	    	
	    	if(newAngle < 0) {
	    		rotation.setSetpoint(360 - (newAngle * -1));
	    	}
	    	else if (newAngle >360) {
	    		rotation.setSetpoint(newAngle -360);
	    		
	    	}
	    	else {
	    		rotation.setSetpoint(newAngle);
	    	}
    }

    public void disable() {
        rotation.disable();
    }

    public void enable() {
        rotation.enable();
    }

    public double calculate(int encoder_tick, EncoderFollower follower) {
    	if(follower.isFinished()) {
        	return 0;
        }
    	
        // Number of Revolutions * Wheel Circumference
    	double kp = 1.0;
    	double kd = 0.0;
    	double ka = 0.0;
    	double heading = 0;
    	
        double distance_covered = ((double)(encoder_tick) / 138) * 1.0625;
        
        SmartDashboard.putNumber("ETF", distance_covered);
        
        Trajectory.Segment seg = follower.getSegment();
        
        double error = seg.position - distance_covered;
        
        //System.out.println(error);
        System.out.printf("%f\t\t%f\t\t%f\t\t%f\t\n", 
        		error, distance_covered, seg.position, kv * seg.velocity);
        double calculated_value =
                kp * error +                                    // Proportional
                kd * ((error - last_error) / seg.dt) +          // Derivative
                (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        last_error = error;
        heading = seg.heading;

        return calculated_value;
    }
}
