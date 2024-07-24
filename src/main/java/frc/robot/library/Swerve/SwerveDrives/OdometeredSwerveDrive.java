package frc.robot.library.Swerve.SwerveDrives;

import frc.robot.library.Hardware.Encoder.EncoderTypeEnum;
import frc.robot.library.Hardware.IMU.IMU;
import frc.robot.library.Hardware.IMU.IMUTypeEnum;
import frc.robot.library.Hardware.Motor.MotorTypeEnum;
import frc.robot.library.Swerve.ModuleTypeEnum;
import frc.robot.library.Swerve.Odometer;
import frc.robot.library.Swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class OdometeredSwerveDrive extends OrientedSwerveDrive {
    
    public Odometer odometer;
    
    public OdometeredSwerveDrive (
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
            double robotWidth,

            IMUTypeEnum imuType, int imuID

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
            ),
            new IMU(imuType, imuID)
        );
    }
    
    public OdometeredSwerveDrive (
            SwerveModule frontLeftModule, SwerveModule frontRightModule, 
            SwerveModule backLeftModule, SwerveModule backRightModule, 
            SwerveDriveKinematics kinematics, IMU imu
        ) {
        super(frontLeftModule, frontRightModule, 
            backLeftModule, backRightModule, 
            kinematics, imu
        );

        odometer = new Odometer(getKinematics(), getModulePositions());
    }
}
