package org.usfirst.frc3707.lib.swerve;

import org.usfirst.frc3707.RealSwerveShady.RobotMap;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;

public class SwerveDrive implements PIDOutput {

    private SwerveWheel rightFrontWheel;
    private SwerveWheel leftFrontWheel;
    private SwerveWheel leftBackWheel;
    private SwerveWheel rightBackWheel;
    
    private GyroBase gyro;
    
    private double wheelbase = 22.5;
    private double trackwidth = 24.5;
    

    public SwerveDrive(SwerveWheel rightFront, SwerveWheel leftFront, SwerveWheel leftBack, SwerveWheel rightBack, GyroBase gyro){
    	this.rightFrontWheel = rightFront;
    	this.leftFrontWheel = leftFront;
    	this.leftBackWheel = leftBack;
    	this.rightBackWheel = rightBack;
	    
        this.gyro = gyro;
        
    }
    
    public void drive(double directionX, double directionY, double rotation, boolean useGyro, boolean slowSpeed) {
    	
      directionY *= -1; 							//invert Y
      directionX *= -1; 							//invert X
      rotation *= -1;
    	
    	SmartDashboard.putNumber("directionX", directionX);
		SmartDashboard.putNumber("directionY", directionY);
		SmartDashboard.putNumber("rotation", rotation);
		 
		//if BOTH joystick in the center
		if((directionX < 0.2 && directionX > -0.2) && (directionY < 0.2 && directionY > -0.2) && (rotation < 0.2 && rotation > -0.2)) {
			this.rightFrontWheel.updateSpeed(0);
			this.leftFrontWheel.updateSpeed(0);
			this.leftBackWheel.updateSpeed(0);
			this.rightBackWheel.updateSpeed(0);
			return;
		}
		//if ROTATION joystick only near the center (this fixes the rotation drift)
		else if (rotation < 0.2 && rotation > -0.2) {
			rotation = 0;
		}
		
		double L = this.wheelbase; 					//distance between front and back wheels
		double W = this.trackwidth; 				//distance between front wheels
		double r = Math.sqrt ((L * L) + (W * W)); 	//radius of circle (actually it may be the diameter?)
        

		
		if(useGyro) {
			double gyroAngle = gyro.getAngle();
			gyroAngle = Math.IEEEremainder(gyroAngle, 360);
			gyroAngle = reverseAngle(gyroAngle);
			gyroAngle = Math.toRadians(gyroAngle);
			
			final double temp = directionY * Math.cos(gyroAngle) + directionX * Math.sin(gyroAngle);
			directionX = -directionY * Math.sin(gyroAngle) + directionX * Math.cos(gyroAngle);
			directionY = temp;
		}

        double a = directionX - rotation * (L / r); //rear axle
        double b = directionX + rotation * (L / r); //front axle
        double c = directionY - rotation * (W / r); //left track
        double d = directionY + rotation * (W / r); //right track
        
        /*
         *                FRONT
         * 
         *            c          d
         *            | 		 |
         *       b ------------------ b
         *            |          |
         *            |          |
         * LEFT       |          |      RIGHT
         *            |          |
         *            |          |
         *       a ------------------ a
         *            |          |
         *            c          d
         * 
         *                BACK
         */

		//set motor speeds for each wheel
		double backRightSpeed = Math.sqrt ((a * a) + (d * d));
		double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
		double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
		double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

		//set wheel angle for each wheel
		double backRightAngle = (Math.atan2 (a, d) / Math.PI) * 180;
		double backLeftAngle = (Math.atan2 (a, c) / Math.PI) * 180;
		double frontRightAngle = (Math.atan2 (b, d) / Math.PI) * 180;
		double frontLeftAngle = (Math.atan2 (b, c) / Math.PI) * 180;
        
//        if(useGyro) {
//            double gyroAngle = normalizeGyroAngle(gyro.getAngle()); 
//            backRightAngle += gyroAngle;
//            backLeftAngle += gyroAngle;
//            frontRightAngle += gyroAngle;
//            frontLeftAngle += gyroAngle;
//        }
		if(useGyro) {
			frontRightAngle = Math.IEEEremainder(frontRightAngle + 180, 360);
			frontLeftAngle = Math.IEEEremainder(frontLeftAngle + 180, 360);
			backLeftAngle = Math.IEEEremainder(backLeftAngle + 180, 360);
			backRightAngle = Math.IEEEremainder(backRightAngle + 180, 360);
		}
        if(slowSpeed) {
		    	backRightSpeed *= 0.5;
		    	backLeftSpeed *= 0.5;
		    	frontRightSpeed *= 0.5;
		    	frontLeftSpeed *= 0.5;
        }
        
        //update the actual motors
		this.rightFrontWheel.drive(frontRightSpeed, frontRightAngle);
		this.leftFrontWheel.drive(frontLeftSpeed, frontLeftAngle);
		this.leftBackWheel.drive(backLeftSpeed, backLeftAngle);
		this.rightBackWheel.drive(backRightSpeed, backRightAngle);
    }
    public void lockStop() {
    		this.rightFrontWheel.drive(0, 315);
		this.leftFrontWheel.drive(0, 45);
		this.leftBackWheel.drive(0, 315);
		this.rightBackWheel.drive(0, 45);
    }
    public void driveSimple(double speed, double angle) {
    	//reverse it because encoders think 0 is backwards
//    	angle = reverseAngle(angle);
//    	double gyroAngle = normalizeGyroAngle(RobotMap.gyro.getAngle()); 
//    	angle = normalizeGyroAngle(angle + gyroAngle);
    	
    	    this.rightFrontWheel.drive(speed, angle);
		this.leftFrontWheel.drive(speed, angle);
		this.leftBackWheel.drive(speed, angle);
		this.rightBackWheel.drive(speed, angle);
    }
    public void driveDirect(EncoderFollower fr, EncoderFollower fl, EncoderFollower bl, EncoderFollower br) {
    	this.rightFrontWheel.drivePath(fr);
		this.leftFrontWheel.drivePath(fl);
		this.leftBackWheel.drivePath(bl);
		this.rightBackWheel.drivePath(br);
    }
    
//    public void spinSimple(double speed) {
//    	this.rightFrontWheel.drive(speed, reverseAngle(79));
//		this.leftFrontWheel.drive(speed, reverseAngle(346));
//		this.leftBackWheel.drive(speed, reverseAngle(32));
//		this.rightBackWheel.drive(speed, reverseAngle(93));
//    }
    
//    public void spinSimple(double speed) {
//    	//this.rightFrontWheel.drive(speed, 45);
//		//this.leftFrontWheel.drive(speed, 45);
//		//this.leftBackWheel.drive(speed, 45);
//		//this.rightBackWheel.drive(speed, 0);
//    }

	@Override
	public void pidWrite(double output) {
//		System.out.println("X");
//		System.out.println(output);
//		System.out.println("ANGLE");
//		System.out.println(gyro.getAngle());
		drive(0, 0, output, false, false);
	}
	public double reverseAngle(double angle) {
    	double reversed = 180 - angle;
    	if(reversed > 0) {
    		reversed += 360;
    	}
    	return reversed;
    }
    public double normalizeGyroAngle(double angle){
        return (angle - (Math.floor( angle / 360) * 360) );
    }

}
