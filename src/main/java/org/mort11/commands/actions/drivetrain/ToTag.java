package org.mort11.commands.actions.drivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.Command;

public class ToTag  extends Command {

    private Drivetrain drivetrain;
    private Vision vision;

    public int tagNumber;
    
    public ToTag (int tagNumber) {
        this.tagNumber = tagNumber;

        drivetrain = Drivetrain.getInstance();
        vision = Vision.getInstance();
    }

    public void initialize () {

    }

    public void execute () {
        drivetrain.swerveDrive.moveToPosition(vision.getTagPosition(tagNumber));
    }

    public boolean isFinished () {
        return false;
    }

    public void end(boolean interrupted) {
        drivetrain.drive(
            new ChassisSpeeds(0, 0, 0)
        );
    }
}
