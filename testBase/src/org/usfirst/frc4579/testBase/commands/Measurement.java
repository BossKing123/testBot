package org.usfirst.frc4579.testBase.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc4579.testBase.Robot;
/**
 *
 */
public class Measurement extends Command {

    public Measurement() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.measurement);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.measurement.resetFlowMotion();
    	Robot.measurement.resetMPU();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.measurement.getCounts();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
