package org.mort11.configuration;

// import org.mort11.commands.autons.pathPlanner.GetPlanned;
// import org.mort11.commands.autons.timed.Taxi;
import org.mort11.library.Swerve.PathPlanner;
import org.mort11.subsystems.Drivetrain;
import static org.mort11.configuration.constants.PIDConstants.DrivetrainConstants.*;
import static org.mort11.configuration.constants.PhysicalConstants.DrivetrainConstants.DRIVETRAIN_RADII_METERS;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {

	private static Drivetrain drivetrain;

	private static SendableChooser<Command> autoChooser;
	// private static SendableChooser<Boolean> isBlue;
	
	/**
	 * Create autonomous commands and chooser
	 */
	public static void configure() {
		drivetrain = Drivetrain.getInstance();

        autoChooser = new SendableChooser<Command>();
		// PathPlanner.configure(drivetrain, drivetrain.swerveDrive, 
		// 	new PIDConstants(PLANNER_TRANSLATIONAL_KP, PLANNER_TRANSLATIONAL_KI, PLANNER_TRANSLATIONAL_KD), 
		// 	new PIDConstants(PLANNER_ROTATIONAL_KP, PLANNER_ROTATIONAL_KI, PLANNER_ROTATIONAL_KD), 
		// 	DRIVETRAIN_RADII_METERS
		// );
		addAutoOptions();
		SmartDashboard.putData(autoChooser);

		// isBlue = new SendableChooser<Boolean>();
		// isBlue.setDefaultOption("Blue", true);
		// isBlue.addOption("Red", false);
		// SmartDashboard.putData("isBlue", isBlue);
	}
	
	public static void addAutoOptions () {
		autoChooser.setDefaultOption("nothing", null);

		// autoChooser.addOption("Forward", new Taxi());
		// autoChooser.addOption("Circle", GetPlanned.getCircle());
	}

	public static Command getAutonomousCommand () {
		return autoChooser.getSelected();
	}

	public static Boolean getIsBlue () {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}
}
