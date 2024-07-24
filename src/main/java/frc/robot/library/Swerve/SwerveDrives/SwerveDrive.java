package frc.robot.library.Swerve.SwerveDrives;

import frc.robot.library.Hardware.Encoder.EncoderTypeEnum;
import frc.robot.library.Hardware.Motor.MotorTypeEnum;
import frc.robot.library.Swerve.ModuleTypeEnum;
import frc.robot.library.Swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {

    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    public ChassisSpeeds velocity;

    public SwerveDriveKinematics kinematics;

    public double descritizedValue;
    
    public SwerveDrive (
            MotorTypeEnum frontLeftDriveMotorType, int frontLeftDriveMotorID, 
            MotorTypeEnum frontLeftSteerMotorType, int frontLeftSteerMotorID,
            EncoderTypeEnum frontLeftEncoderType, int frontLeftEncoderID,
            ModuleTypeEnum frontLeftModuleType,

            MotorTypeEnum frontRightDriveMotorType, int frontRightDriveMotorID, 
            MotorTypeEnum frontRightSteerMotorType, int frontRightSteerMotorID,
            EncoderTypeEnum frontRightEncoderType, int frontRightEncoderID,
            ModuleTypeEnum frontRightModuleType,

            MotorTypeEnum backLeftDriveMotorType, int backLeftDriveMotorID, 
            MotorTypeEnum backLeftSteerMotorType, int backLeftSteerMotorID,
            EncoderTypeEnum backLeftEncoderType, int backLeftEncoderID,
            ModuleTypeEnum backLeftModuleType,

            MotorTypeEnum backRightDriveMotorType, int backRightDriveMotorID, 
            MotorTypeEnum backRightSteerMotorType, int backRightSteerMotorID,
            EncoderTypeEnum backRightEncoderType, int backRightEncoderID,
            ModuleTypeEnum backRightModuleType,

            double robotLength,
            double robotWidth

        ) {
        this(
            new SwerveModule(
                frontLeftDriveMotorType, frontLeftDriveMotorID,
                frontLeftSteerMotorType, frontLeftSteerMotorID,
                frontLeftEncoderType, frontLeftEncoderID,
                frontLeftModuleType
            ), 
            new SwerveModule(
                frontRightDriveMotorType, frontRightDriveMotorID,
                frontRightSteerMotorType, frontRightSteerMotorID,
                frontRightEncoderType, frontRightEncoderID,
                frontRightModuleType
            ), 
            new SwerveModule(
                backLeftDriveMotorType, backLeftDriveMotorID,
                backLeftSteerMotorType, backLeftSteerMotorID,
                backLeftEncoderType, backLeftEncoderID,
                backLeftModuleType
            ), 
            new SwerveModule (
                backRightDriveMotorType, backRightDriveMotorID,
                backRightSteerMotorType, backRightSteerMotorID,
                backRightEncoderType, backRightEncoderID,
                backRightModuleType
            ), 
            new SwerveDriveKinematics(
				// Front left
				new Translation2d(robotWidth / 2.0, robotLength / 2.0),
				// Front right
				new Translation2d(robotWidth / 2.0, -robotLength / 2.0),
				// Back left
				new Translation2d(-robotWidth / 2.0, robotLength / 2.0),
				// Back right
				new Translation2d(-robotWidth / 2.0, -robotLength / 2.0)
            )
        );
    }
    
    public SwerveDrive (
            SwerveModule frontLeftModule, SwerveModule frontRightModule, 
            SwerveModule backLeftModule, SwerveModule backRightModule, 
            SwerveDriveKinematics kinematics
        ) {
            this.frontLeftModule = frontLeftModule;
            this.frontRightModule = frontRightModule;
            this.backLeftModule = backLeftModule;
            this.backRightModule = backRightModule;

            this.kinematics = kinematics;

        descritizedValue = 0.02;
    }

    public void setVelocity(ChassisSpeeds velocity) {
        this.velocity = velocity;

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(velocity);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, getModule(0).getMaxSpeed());
        setStates(states);
    }

    public void setDescitizedVelocity(ChassisSpeeds velocity) {
        this.velocity = velocity;

        velocity = ChassisSpeeds.discretize(velocity, descritizedValue);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(velocity);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, getModule(0).getMaxSpeed());
        setStates(states);
    }

    public void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            getModule(i).setModuleState(states[i]);
        }
    }

    public void setDriveSpeeds(double[] driveSpeeds) {
        for (int i = 0; i < 4; i++) {
            getModule(i).setDriveSpeedMeters(driveSpeeds[i]);
        }
    }

    public void setSteerPositions(Rotation2d[] rotations) {
        for (int i = 0; i < 4; i++) {
            getModule(i).setPosition(rotations[i]);
        }
    }

    public void setDescritizedValue(double value) {
        descritizedValue = value;
    }

    public void setOffsets(double[] offsets) {
        for (int i = 0; i < 4; i++) {
            getModule(i).setOffset(offsets[i]);
        }
    }

    public void setOffsets(
            double frontLeftOffset, double frontRightOffset, double backLeftOffset, double backRightOffset
        ) {
        frontLeftModule.setOffset(frontLeftOffset);
        frontRightModule.setOffset(frontRightOffset);
        backLeftModule.setOffset(backLeftOffset);
        backRightModule.setOffset(backRightOffset);
    }



    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModule getModule(int num) {
        switch (num) {
            case 1:
                return frontLeftModule;
            case 2:
                return frontRightModule;
            case 3:
                return backLeftModule;
            case 4:
                return backRightModule;
            default:
                return frontLeftModule;

        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(), 
            frontRightModule.getPosition(), 
            backLeftModule.getPosition(), 
            backRightModule.getPosition()
        };
    }
}
