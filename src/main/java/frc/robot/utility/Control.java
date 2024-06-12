package frc.robot.utility;

import static frc.robot.utility.constants.PortConstants.ControllerPorts.*;
import static frc.robot.utility.constants.PhysicalConstants.DrivetrainConstants.*;

import java.sql.DriverPropertyInfo;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

public class Control {
    private Alliance defaultAlliance = Alliance.Blue;
    private static CommandJoystick joystick;
	private static CommandJoystick throttle;
	private static CommandXboxController xboxController;
    private static DoubleSupplier zeroSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return 0.0;
        }
    };

	private static Drivetrain drivetrain;

    public static void init() {
		joystick = new CommandJoystick(JOYSTICK);

		drivetrain = Drivetrain.getInstance();
    }

    public static void configure() {
		drivetrain.setDefaultCommand(
			new Drive(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist)
        );

        joystick.button(0).whileTrue(new InstantCommand(() -> drivetrain.zeroGyroscope(0)));
    }

    //where the axis doesn't take in values near 0, and starts past 0

    /**
     * scaled deadband - removing a value of the region around the zero value and scaling the rest to fit
     * 
     * @param value the total region that has a region near 0 that is a margin of error that needs to be corrected
     * @param deadband the margin of error around a zero point which is considered to be zero. 
     * @return the total region that has had the margin of error 0ed
     */
    public static double deadband(double value, double deadband) {

        if (Math.abs(value) > deadband) {
            //deadband is the region defined as +- deviation equal to 0

            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        }

        else {
            return 0.0;
        }
    }
   

    //raw data of value setting anything less than deadband equal to 0

    /**
     * 
     * @param value
     * @param deadband
     * @return
     */
    public static double unScaledDeadband(double value, double deadband) {

        if (Math.abs(value) > deadband) {
            return (value);
        }
        else {
            return 0.0;
        }
    }

     public static double modifyAxis1(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        throttleValue = (throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
    }

    public static double modifyAxis2(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value * value, value);

        throttleValue = (throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
        // return value * throttleValue;
    }

    public static double modifyAxisTwist(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value, value);

        throttleValue = (throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_ROTATE - MIN_ROTATE) + MIN_ROTATE);
        // return value * throttleValue;
    }

    public static double getThrottle() {
        return throttle.getRawAxis(2);
    }

    /**
     * 
     * @return
     */
    public static double getJoystickX() {
		return modifyAxis1(joystick.getX(), joystick.getRawAxis(2)) * drivetrain.frontLeftModule.getMaxSpeed();
	}

    /**
     * 
     * @return
     */
	public static double getJoystickY() {
		return modifyAxis1(joystick.getY(), joystick.getRawAxis(2)) * drivetrain.frontLeftModule.getMaxSpeed();
	}

    /**
     * 
     * @return
     */
	public static double getJoystickTwist() {
		return modifyAxisTwist(joystick.getRawAxis(3), joystick.getRawAxis(2))
			    * drivetrain.frontLeftModule.getMaxSpeed() / 
                Math.hypot(
                    DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0
                );
    }

    // public static double getLeftJoystickY() {
    //     return xboxController.getLeftY();
    // }

    // public static double getRightJoystickY(){
    //     return xboxController.getRightY();
    // }
 }