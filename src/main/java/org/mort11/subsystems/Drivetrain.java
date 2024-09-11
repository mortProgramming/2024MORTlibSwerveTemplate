// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.mort11.configuration.constants.PhysicalConstants.DrivetrainConstants.*;
import static org.mort11.configuration.constants.PortConstants.DrivetrainConstants.*;
import static org.mort11.library.Hardware.Encoder.EncoderTypeEnum.*;
import static org.mort11.library.Hardware.IMU.IMUTypeEnum.*;
import static org.mort11.library.Hardware.Motor.MotorTypeEnum.*;
import static org.mort11.library.Swerve.ModuleTypeEnum.*;

import java.util.function.DoubleSupplier;

import org.mort11.library.Hardware.IMU.IMU;
import org.mort11.library.Swerve.SwerveModule;
import org.mort11.library.Swerve.SwerveDrives.OdometeredSwerveDrive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Drivetrain drivetrain;

  public OdometeredSwerveDrive swerveDrive;

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  public SwerveDriveKinematics kinematics;

  private ChassisSpeeds speeds;

  private IMU imu;

  private Drivetrain() {
    // SmartDashboard.putNumber("swerve thing", 7);

    configureSwerve();
    
    speeds = new ChassisSpeeds(0, 0, 0);
  }

  public void configureSwerve () {
    frontLeftModule = new SwerveModule(
      FALCON, FRONT_LEFT_DRIVE_ID, 
      FALCON, FRONT_LEFT_STEER_ID, 
      CANCODER, FRONT_LEFT_ENCODER_ID, 
      MK4i
    );

    frontRightModule = new SwerveModule(
      FALCON, FRONT_RIGHT_DRIVE_ID, 
      FALCON, FRONT_RIGHT_STEER_ID, 
      CANCODER, FRONT_RIGHT_ENCODER_ID, 
      MK4i
    );

    backLeftModule = new SwerveModule(
      FALCON, BACK_LEFT_DRIVE_ID, 
      FALCON, BACK_LEFT_STEER_ID, 
      CANCODER, BACK_LEFT_ENCODER_ID, 
      MK4i
    );

    backRightModule = new SwerveModule(
      FALCON, BACK_RIGHT_DRIVE_ID, 
      FALCON, BACK_RIGHT_STEER_ID, 
      CANCODER, BACK_RIGHT_ENCODER_ID, 
      MK4i
    );

    frontLeftModule.steerMotor.setDirectionFlip(true);
    frontRightModule.steerMotor.setDirectionFlip(true);
    backLeftModule.steerMotor.setDirectionFlip(true);
    backRightModule.steerMotor.setDirectionFlip(true);

    kinematics = new SwerveDriveKinematics(
            // Front left
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    imu = new IMU(PIGEON2, IMU_ID);

    swerveDrive = new OdometeredSwerveDrive(
      frontLeftModule, frontRightModule, 
      backLeftModule, backRightModule, 
      kinematics, imu
    );

    swerveDrive.setOffsets(FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET);

    swerveDrive.setCanivore(CANIVORE_NAME);
  }

  public void drive(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public Command driveCommand(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickTwist) {
    return new InstantCommand(() -> drive(
        new ChassisSpeeds(joystickX.getAsDouble(), joystickY.getAsDouble(), joystickTwist.getAsDouble())
      ), drivetrain
    );
  }

  public Command zeroIMUCommand(double angle) {
    return new InstantCommand(() -> swerveDrive.zeroIMU(angle), drivetrain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveDrive.setOrientedVelocity(speeds);
    // swerveDrive.setVelocity(speeds);
    // swerveDrive.setOrientedVelocity(new ChassisSpeeds(0, 0, 0));
    swerveDrive.update();

    SmartDashboard.putNumber("XPose", swerveDrive.getPosition().getX());
    SmartDashboard.putNumber("YPose", swerveDrive.getPosition().getY());

    SmartDashboard.putNumber("Yaw", Math.toDegrees(swerveDrive.getRobotRotations().getZ()));
    SmartDashboard.putNumber("Pitch", Math.toDegrees(swerveDrive.getRobotRotations().getY()));
    SmartDashboard.putNumber("Roll", Math.toDegrees(swerveDrive.getRobotRotations().getX()));
  }

  public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}
}
