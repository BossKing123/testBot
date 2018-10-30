// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.testBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveBaseleftMotor;
    public static SpeedController driveBaserightMotor;
    public static DifferentialDrive driveBaserobotDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveBaseleftMotor = new VictorSP(1);
        LiveWindow.addActuator("driveBase", "leftMotor", (VictorSP) driveBaseleftMotor);
        driveBaseleftMotor.setInverted(false);
        driveBaserightMotor = new VictorSP(0);
        LiveWindow.addActuator("driveBase", "rightMotor", (VictorSP) driveBaserightMotor);
        driveBaserightMotor.setInverted(true);
        driveBaserobotDrive = new DifferentialDrive(driveBaseleftMotor, driveBaserightMotor);
        LiveWindow.addActuator("driveBase", "robotDrive", driveBaserobotDrive);
        driveBaserobotDrive.setSafetyEnabled(false);
        driveBaserobotDrive.setExpiration(0.1);
        driveBaserobotDrive.setMaxOutput(1.0);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
