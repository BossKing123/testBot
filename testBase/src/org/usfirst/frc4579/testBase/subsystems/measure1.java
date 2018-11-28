package org.usfirst.frc4579.testBase.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4579.filters.FirstOrderLPF;
import org.usfirst.frc4579.testBase.commands.*;
import edu.wpi.first.wpilibj.SPI;
import sensors.FlowMotion;
import sensors.MPU6050;

/** This subsystem performs all the primary measurements for the robot.  Sections
 *  can be added as new measurement devices are added to the system.
 */
public class measure1 extends Subsystem {
	public double angle = 0.0;  		// Angle robot is facing (accumulated gyro readings).
	public double angleRate = 0.0;		// Instantaneous gyroZ reading.
	public double angleRateLPF = 0.0;	// Low pass filtered version of angleRate.
	public double angleRateMax = 0.0;   // Max gyroZ reading, always positive.
	public double mpuDeltaT = 0.0;		// delta T measurement from MPU.
	public double motionX = 0.0;		// Accumulated forward motion.
	public double motionY = 0.0;		// Accumulated sideways motion.
	public double speed = 0.0;			// Calculated forward speed.
	public double distance = 0.0;		// Calculated total forward distance.
	public double motionDeltaT = 0.0;	// Flow motion delta t from Flow Sensor.
	
	private FirstOrderLPF filter = new FirstOrderLPF(0.5);
    private final FlowMotion motion = new FlowMotion(SPI.Port.kOnboardCS0, 4);
    private final MPU6050    mpu =    new MPU6050(MPU6050.ACCELFULLSCALE.ACCEL2G, 
			  							          MPU6050.GYROFULLSCALE.DEGSEC250);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Update all the Flow Motion readings.  Put this call in the execute() method of a Command.
    public void updateMotion() {
    	if (motion.goodSensor) {
    		motion.readMotionCounts();
    	}
    	// Compute motion data.
    	motionX += motion.deltaX;
    	motionY += motion.deltaY;
    	speed = motionX / motion.deltaT;
    	distance += speed * motion.deltaT;
    	return;
    }  // End of updateMotion().
    

    // Update all the MPU6050 sensor readings.  Put this call in the execute() method 
    // of a Command.
    public void updateMPU() {
    	if (mpu.goodSensor) {
    		mpu.readAxes();
    	}
    	// Compute gyro turn data.
    	angleRate = mpu.gyroZ;
    	angleRateMax = mpu.gyroZMax;
    	angle = angle + angleRate * mpu.deltaT;
    }  // End of updateMPU().

    public void resetMPUAngle() {
    	angle = 0.0;
    }
    
	public double getAngle() {
		return angle;
	}
	
	public double getAngleRate() {
		return angleRate;
	}
	
	public double getAngleRateMax() {
		return angleRateMax;
	}
	
	public double getAngleFiltered() {
		return angleRateLPF;
	}
	
	public double getMPUDeltaT() {
		return mpuDeltaT;
	}
	
	public double getMotionX() {
		return motionX;
	}
	
	public double getMotionY() {
		return motionY;
	}
	public double getSpeed() {
		return speed;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public double getMotionDeltaT() {
		return motionDeltaT;
	}

    public double getLPFAngle() {
    	angleRateLPF += filter.filter(angleRate) * mpu.deltaT;
    	return angleRateLPF;
    }
    
    

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new Measurement());
    }
}

