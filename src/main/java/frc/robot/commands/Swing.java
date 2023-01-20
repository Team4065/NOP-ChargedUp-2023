// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Swing extends CommandBase {
  /** Creates a new Swing. */
  PIDController pid = new PIDController(0.2, 0, 0);
  AnalogInput input = new AnalogInput(Constants.potPort);
  AnalogPotentiometer potVal = new AnalogPotentiometer(input, 180, 0);

  public Swing() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swingarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pot Value", potVal.get());
    RobotContainer.m_swingarm.setMotor(pid.calculate(potVal.get(), 32.5));
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
