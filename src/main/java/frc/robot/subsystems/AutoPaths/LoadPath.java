// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AutoPaths;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class LoadPath {
    String path; 
    public static List<PathPlannerTrajectory> pathToFollow;  
    public LoadPath(String path, boolean resetOdo) {
        this.path = path;
        pathToFollow = PathPlanner.loadPathGroup(path, PathPlanner.getConstraintsFromPath(path));
    }
}