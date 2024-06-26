// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  private Drivetrain drivetrain;

  private DoubleSupplier wantedX;
  private DoubleSupplier wantedY;
  private DoubleSupplier wantedTheta;

  public Drive(DoubleSupplier wantedX, DoubleSupplier wantedY, DoubleSupplier wantedTheta) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain =  Drivetrain.getInstance();

    this.wantedX = wantedX;
    this.wantedX = wantedY;
    this.wantedX = wantedTheta;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        wantedX.getAsDouble(),
				wantedY.getAsDouble(), wantedTheta.getAsDouble(),
				drivetrain.getGyroscopeRotation()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(
      new ChassisSpeeds(
        0, 
        0, 
        0
      )
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
