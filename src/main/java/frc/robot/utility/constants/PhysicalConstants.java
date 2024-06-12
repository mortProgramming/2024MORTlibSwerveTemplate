package frc.robot.utility.constants;

import edu.wpi.first.math.util.Units;

public final class PhysicalConstants {
    public final static class DrivetrainConstants {
        // The left-to-right distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.75);
		// The front-to-back distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.75);
    }
}
