package org.mort11.commands.autons.pathPlanner;

import org.mort11.commands.autons.pathPlanner.paths.Rest;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class GetPlanned {

    public static Command getCircle () {
        Rest.setCommands();

        return new PathPlannerAuto("Circle");
    }

}
