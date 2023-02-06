// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DeployIntake extends CommandBase {
  /** Creates a new DeployIntake. */
  Double m_speed;
  boolean m_deploy;
  boolean FIN;
  public DeployIntake(Double Speed, boolean deploy) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
    m_speed = Speed;
    m_deploy = deploy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intake.setIntake(ControlMode.PercentOutput, m_speed);
   // RobotContainer.m_air_system.intakeSolenoid.set(m_deploy);
    if(m_deploy == false) {
      FIN = true;
    }
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
