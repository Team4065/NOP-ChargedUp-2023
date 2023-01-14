// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  Subsystem s_subsystem;
  Double m_speed;
  public TankDrive(Subsystem subsystem, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = s_subsystem;
    speed = m_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double LJY = RobotContainer.getAxisRamped(1);
    Double RJY = RobotContainer.getAxisRamped(5);

    RobotContainer.m_drivetrain.setRight(ControlMode.PercentOutput, m_speed * RJY);
    RobotContainer.m_drivetrain.setLeft(ControlMode.PercentOutput, m_speed * LJY);
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
