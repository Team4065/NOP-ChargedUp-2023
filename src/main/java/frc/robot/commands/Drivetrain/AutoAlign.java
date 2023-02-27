// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoAlign extends CommandBase {
  double tarX = RobotContainer.m_rasppicam.getYaw(0);
  boolean end;

  // PIDController pid = new PIDController(0.024, 0.001, 0.002);
  // double speed;

  /** Creates a new AutoAlign. */
  public AutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tarX = RobotContainer.m_rasppicam.getYaw(0);
    RobotContainer.m_drivetrain.tankDrive(tarX * 0.05, -tarX * 0.05);

    if (tarX < 0.03 && tarX > -0.03) {
      end = true;
    }

    // System.out.println("balancing...");
    // speed = pid.calculate(tarX, Constants.AutoConstants.balancedGyro);

    // if (speed < -0.5) {
    //   RobotContainer.m_drivetrain.tankDrive(0.429 * 1.1, 0.429 * 1.1);
    // } else if (speed > 0.5) {
    //   RobotContainer.m_drivetrain.tankDrive(-0.429 * 1.1, -0.429 * 1.1);
    // } else {
    //   RobotContainer.m_drivetrain.tankDrive(-speed, -speed);
    // }

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
