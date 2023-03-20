// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class GetOnRamp extends CommandBase {
  /** Creates a new getOnRamp. */
  boolean end;
  double speed;
  float startTime, timePassed;
  boolean isBatterySide;
  public GetOnRamp(boolean isBatterySide) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    this.isBatterySide = isBatterySide;
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("started");
    speed = (isBatterySide == true) ? 0.525 : -0.525;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RAMP");
    double pitch = Math.abs(RobotContainer.m_drivetrain.getPitch());

    if (pitch != Constants.AutoConstants.onRampGyro) {
      RobotContainer.m_drivetrain.tankDrive(speed, speed);
    }

    if (pitch < Constants.AutoConstants.onRampGyro + 1 && pitch > Constants.AutoConstants.onRampGyro - 1) {
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setBreakMode();
    RobotContainer.m_drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
