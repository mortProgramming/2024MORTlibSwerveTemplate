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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.MORTlib.Test.Swerve.Odometer;
import com.MORTlib.Test.Swerve.SwerveDrive;
import com.MORTlib.Test.Swerve.SwerveModule;
import static com.MORTlib.Test.Hardware.MotorTypeEnum.*;
import static com.MORTlib.Test.Hardware.EncoderTypeEnum.*;
import static com.MORTlib.Test.Swerve.ModuleTypeEnum.*;
import static frc.robot.utility.constants.PortConstants.DrivetrainConstants.*;
import static frc.robot.utility.constants.PhysicalConstants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Drivetrain drivetrain;

  public SwerveDrive swerveDrive;

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  public SwerveDriveKinematics kinematics;

  private ChassisSpeeds speeds;

  private AHRS navX;

  private double fieldOrientationOffset;

  public Odometer odometer;

  public Drivetrain() {
    frontLeftModule = new SwerveModule(
    KRAKEN, FRONT_LEFT_DRIVE_ID, 
    KRAKEN, FRONT_LEFT_STEER_ID, 
    CANCODER, FRONT_LEFT_ENCODER_ID, 
    MK4i);
    frontLeftModule.setOffset(FRONT_LEFT_OFFSET);

    frontRightModule = new SwerveModule(
    KRAKEN, FRONT_RIGHT_DRIVE_ID, 
    KRAKEN, FRONT_RIGHT_STEER_ID, 
    CANCODER, FRONT_RIGHT_ENCODER_ID, 
    MK4i);
    frontRightModule.setOffset(FRONT_RIGHT_OFFSET);

    backLeftModule = new SwerveModule(
    KRAKEN, BACK_LEFT_DRIVE_ID, 
    KRAKEN, BACK_LEFT_STEER_ID, 
    CANCODER, BACK_LEFT_ENCODER_ID, 
    MK4i);
    backLeftModule.setOffset(BACK_LEFT_OFFSET);

    backRightModule = new SwerveModule(
    KRAKEN, BACK_RIGHT_DRIVE_ID, 
    KRAKEN, BACK_RIGHT_STEER_ID, 
    CANCODER, BACK_RIGHT_ENCODER_ID, 
    MK4i);
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

    swerveDrive = new SwerveDrive(
      frontLeftModule, frontRightModule, 
      backLeftModule, backRightModule, 
      kinematics
    );

    navX = new AHRS(SPI.Port.kMXP);

    odometer = new Odometer(swerveDrive);

    speeds = new ChassisSpeeds(0, 0, 0);
  }

  public void drive(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public void zeroGyroscope(double angle) {
		fieldOrientationOffset = navX.getAngle()+angle;
	}

  public Rotation2d getGyroscopeRotation() {
		if (navX.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(360.0 - (navX.getFusedHeading()-fieldOrientationOffset));
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - navX.getYaw()-fieldOrientationOffset);
	}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveDrive.setVelocity(speeds);
    odometer.update(getGyroscopeRotation(), swerveDrive);

    SmartDashboard.putNumber("XPose", odometer.swervePose.getEstimatedPosition().getX());
    SmartDashboard.putNumber("YPose", odometer.swervePose.getEstimatedPosition().getY());
  }

  public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
			Shuffleboard.getTab("dt").add(drivetrain);
		}
		return drivetrain;
	}
}
