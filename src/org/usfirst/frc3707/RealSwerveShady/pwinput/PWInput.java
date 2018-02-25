package org.usfirst.frc3707.scout2018R1.pwinput;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PWInput implements PIDSource {

// Though calibration offset was in the lidar code, I'm going to leave it out for now	
//private static final int CALIBRATION_OFFSET = 0;

private Counter counter;
private double msPerUnit = 1;
private double maxPeriod = 1;
private double lastReading = 0;
private double ignoreValue = 0;
private int ignoreCount = 5;
private int maxIgnoreRun = 0;
public int ignoreIndex = 0;
public int runningIndex = 0;

private int printedWarningCount = 5;

public PWInput (DigitalSource source, double ignoreval, int ignorecount, double msperunit, double maxperiod) {
	counter = new Counter(source);
    setMaxPeriod(maxperiod);
    // Configure for measuring rising to falling pulses
    counter.setSemiPeriodMode(true);
    counter.reset();
    setMsPerUnit(msperunit);
    setIgnoreValue(ignoreval);
    setIgnoreCount(ignorecount);
}

public PWInput (DigitalSource source) {
	this(source, 0, 5, 1, 1);
}

public void setMsPerUnit (double newmsperunit) {
	if (newmsperunit > 0) {
		msPerUnit = newmsperunit;
	}
}

public double getmsperunit () {
	return msPerUnit;
}
public double getMaxPeriod() {
	return maxPeriod;
}
public void setMaxPeriod(double newmaxperiod) {
	counter.setMaxPeriod(newmaxperiod);
}
public void setIgnoreValue (double newignoreval) {
	ignoreValue = newignoreval;
}
public void setIgnoreValue (double newignoreval, int newignorecount) {
	ignoreValue = newignoreval;
	setIgnoreCount(newignorecount);
}
public void setIgnoreCount (int newic) {
	if (newic > 0) {
		ignoreCount = newic;
	}
}

public int getMaxIgnoreRun () {
	return maxIgnoreRun;
}
//public double getRawMedian() {
//	
//}
//public double getMedian () {
//	
//}

private double handleIgnore(double val) {
	if (Math.abs(ignoreValue - val) < 0.5) {
		ignoreIndex++;
		runningIndex++;
		if (maxIgnoreRun > 20) maxIgnoreRun = 0;
		maxIgnoreRun = Math.max(maxIgnoreRun, runningIndex);
		if (ignoreIndex == (ignoreCount + 1)) {
			ignoreIndex = 0;
		} else {
			return(lastReading);
		}
	} else {
		runningIndex = 0;
	}
	return(val);
}
public double getRawReading() {
	double origMsPerUnit = msPerUnit;
	double rval = 0;
	msPerUnit = 1;
	rval = getDistance();
	msPerUnit = origMsPerUnit;
	return (rval);
}
	

public double getDistance() {
	double rval;
	if (counter.get() < 1) {
		if (printedWarningCount-- > 0) {
			System.out.println("PWInput: waiting for distance measurement");
		}
		return 0;
	}
	rval = (counter.getPeriod() * 1000000.0 / msPerUnit);
	rval = handleIgnore(rval);
	lastReading = rval;
	return rval;
}

@Override
public void setPIDSourceType(PIDSourceType pidSource) {
	
}

@Override
public PIDSourceType getPIDSourceType() {
	return PIDSourceType.kDisplacement;
}

@Override
public double pidGet() {
	double distance = getDistance();
	System.out.println("DISTANCE");
	System.out.println(distance);
	return distance;
}
}