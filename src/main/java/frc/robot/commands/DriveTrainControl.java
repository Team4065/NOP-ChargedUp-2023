// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Drivetrain;

public class DriveTrainControl extends CommandBase {
  
  private Drivetrain m_driveTrain;
  private XboxController XBC;

  /** Creates a new DriveTrainControl. */
  public DriveTrainControl(Drivetrain m_Drivetrain, XboxController XBC) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveTrain = m_Drivetrain;
    this.XBC = XBC;
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.drive(XBC.getLeftY(), XBC.getRightX());
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
