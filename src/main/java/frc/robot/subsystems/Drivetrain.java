// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.Hardware.IMU.IMU;
import frc.robot.library.Swerve.SwerveModule;
import frc.robot.library.Swerve.SwerveDrives.OdometeredSwerveDrive;
import frc.robot.library.Swerve.SwerveDrives.OrientedSwerveDrive;

import static frc.robot.library.Swerve.ModuleTypeEnum.*;
import static frc.robot.library.Hardware.Motor.MotorTypeEnum.*;
import static frc.robot.library.Hardware.Encoder.EncoderTypeEnum.*;
import static frc.robot.library.Hardware.IMU.IMUTypeEnum.*;

import static frc.robot.utility.constants.PortConstants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.utility.constants.PhysicalConstants.DrivetrainConstants.*;

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

  public Drivetrain() {
    frontLeftModule = new SwerveModule(
      KRAKEN, FRONT_LEFT_DRIVE_ID, 
      KRAKEN, FRONT_LEFT_STEER_ID, 
      CANCODER, FRONT_LEFT_ENCODER_ID, 
      MK4i
    );
    frontLeftModule.setOffset(FRONT_LEFT_OFFSET);

    frontRightModule = new SwerveModule(
      KRAKEN, FRONT_RIGHT_DRIVE_ID, 
      KRAKEN, FRONT_RIGHT_STEER_ID, 
      CANCODER, FRONT_RIGHT_ENCODER_ID, 
      MK4i
    );
    frontRightModule.setOffset(FRONT_RIGHT_OFFSET);

    backLeftModule = new SwerveModule(
      KRAKEN, BACK_LEFT_DRIVE_ID, 
      KRAKEN, BACK_LEFT_STEER_ID, 
      CANCODER, BACK_LEFT_ENCODER_ID, 
      MK4i
    );
    backLeftModule.setOffset(BACK_LEFT_OFFSET);

    backRightModule = new SwerveModule(
      KRAKEN, BACK_RIGHT_DRIVE_ID, 
      KRAKEN, BACK_RIGHT_STEER_ID, 
      CANCODER, BACK_RIGHT_ENCODER_ID, 
      MK4i
    );
    backRightModule.setOffset(BACK_RIGHT_OFFSET);

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

    speeds = new ChassisSpeeds(0, 0, 0);
  }

  public void drive(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public Command driveCommand(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickTwist) {
    return new InstantCommand(() -> drive(
      new ChassisSpeeds(joystickX.getAsDouble(), joystickY.getAsDouble(), joystickTwist.getAsDouble())
    ));
  }

  public Command zeroIMUCommand(double angle) {

    return new InstantCommand(() -> swerveDrive.zeroIMU(angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveDrive.setVelocity(speeds);
    swerveDrive.odometer.update(Rotation2d.fromDegrees(swerveDrive.getFieldRelativeAngle()), swerveDrive.getModulePositions());

    SmartDashboard.putNumber("XPose", swerveDrive.odometer.swervePose.getEstimatedPosition().getX());
    SmartDashboard.putNumber("YPose", swerveDrive.odometer.swervePose.getEstimatedPosition().getY());
  }

  public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
			Shuffleboard.getTab("dt").add(drivetrain);
		}
		return drivetrain;
	}
}
