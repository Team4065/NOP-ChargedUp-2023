// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double LYJ = -(RobotContainer.XboxC.getRawAxis(1)); // Left Joystick, y-axis
    Double RYJ = -(RobotContainer.XboxC.getRawAxis(5)); // Right Joystick, y-axis

    RobotContainer.m_drivetrain.setRight(RYJ * 0.75);
    RobotContainer.m_drivetrain.setLeft(LYJ * 0.75);
    // System.out.println(RobotContainer.m_drivetrain.distanceMove("right", 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
