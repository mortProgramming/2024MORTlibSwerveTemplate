package frc.robot.library.Hardware.IMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface IMUIntf {
    
    public double getAngle();

    public double getRate();

    public void reset();

    public Rotation2d getRotation2d();

    public Rotation3d getRotation3d();

}
