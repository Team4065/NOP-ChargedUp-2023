// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ChangeSpeed extends CommandBase {
  /** Creates a new ChangeSpeed. */
  private double setPercentOutput;
  private boolean end;
  public ChangeSpeed(double setPercentOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPercentOutput = setPercentOutput;
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    DriveTrain.percentOutput = setPercentOutput;
    if (setPercentOutput == 0.75) {
      RobotContainer.m_drivetrain.changeRate(0.8);
    } else {
      RobotContainer.m_drivetrain.changeRate(0.4);
    }
=======
    DriveTrain.speed = setPercentOutput;
>>>>>>> Stashed changes
    end = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}