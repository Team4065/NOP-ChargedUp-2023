// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.photonvision2;

public class autoAlineYaw extends CommandBase {
  boolean FIN = false;
  PIDController pid = new PIDController(0.01, 0, 0);
  double tarX = photonvision2.tagYaw();
  /** Creates a new autoAlineYaw. */
  public autoAlineYaw() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.d_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tarX = photonvision2.tagYaw();
    RobotContainer.d_drivetrain.setRight(-(pid.calculate(tarX, 0)));
    RobotContainer.d_drivetrain.setleft(pid.calculate(tarX, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return FIN;
  }
}
