package org.mort11.configuration;

import org.mort11.commands.actions.drivetrain.Drive;
import org.mort11.subsystems.Drivetrain;
import static org.mort11.configuration.Inputs.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static org.mort11.configuration.constants.PhysicalConstants.DrivetrainConstants.IMU_TO_ROBOT_FRONT_ANGLE;

public class IO {

	private static Drivetrain drivetrain;

    public static void init() {
		drivetrain = Drivetrain.getInstance();
    }

    public static void configure() {
        init();

		// drivetrain.setDefaultCommand(
		// 	new Drive(IO::getJoystickX, IO::getJoystickY, IO::getJoystickTwist)
        // );
        // drivetrain.setDefaultCommand(
		// 	drivetrain.driveCommand(IO::getJoystickX, IO::getJoystickY, IO::getJoystickTwist)
        // );
        drivetrain.setDefaultCommand(
            new Drive(Inputs::getLeftControllerXSwerve, Inputs::getLeftControllerYSwerve, Inputs::getRightControllerXSwerve)
        );
        // drivetrain.setDefaultCommand(
        //     drivetrain.driveCommand(IO::getLeftControllerXSwerve, IO::getLeftControllerYSwerve, IO::getRightControllerXSwerve)
        // );

        // joystick.button(0).whileTrue(new InstantCommand(() -> drivetrain.swerveDrive.zeroIMU(IMU_TO_ROBOT_FRONT_ANGLE)));
        joystick.button(0).whileTrue(drivetrain.zeroIMUCommand(IMU_TO_ROBOT_FRONT_ANGLE));

        joystick.button(1).whileTrue(new InstantCommand(() -> drivetrain.swerveDrive.resetPosition(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        )));
    }
}
