package org.usfirst.frc3707.lib.swerve;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;


public class SwerveWheel {
    private PIDController rotation;
    private SpeedController speed;
    private final Encoder encoder;
    private double offset;
    private double kv = 1.0/11.0;
    

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
	    	
	    	int encD = (int) encoder.getDistance();
	    	//double dis = ((double) encD / 100) * 1; 
	    	double dis = (double)encD;
	    	double err = fakeDistance - dis;
	    	
	    	double speed = kv * seg.velocity;
	    	double heading = follower.getHeading();
	        
	    	System.out.printf("%f\t%f\t%f\t%f\t%f\n", 
	    			speed, speedx, dis, err, fakeDistance);
	    	updateSpeed(speed);
	    	updateRotation(Pathfinder.r2d(heading));
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
        // Number of Revolutions * Wheel Circumference
    	double kp = 1.0;
    	double kd = 0.0;
    	double ka = 0.0;
    	double heading = 0;
    	
        double distance_covered = ((double)(encoder_tick - 0) / 100) * 1;
        Trajectory.Segment seg = follower.getSegment();
        double error = seg.position - distance_covered;
        double calculated_value =
                kp * error +                                    // Proportional
                kd * ((error - last_error) / seg.dt) +          // Derivative
                (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        last_error = error;
        heading = seg.heading;

        return calculated_value;
    }
}
