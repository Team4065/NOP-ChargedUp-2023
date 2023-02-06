// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  int DriveMode = 0;
  Boolean on = false;
  Boolean CMode = true;
  Boolean done;

  Subsystem s_subsystem;
  double PO;
  public TankDrive(Subsystem subsystem, Double PO) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = s_subsystem;
    this.PO = PO;
    addRequirements(RobotContainer.m_drivetrain);
  }

  public TankDrive(Boolean on){
    done = false;
    this.on = on;
    CMode = true;
    addRequirements(RobotContainer.m_drivetrain);
  }

  public TankDrive(int DriveMode){
    done = false;
    this.DriveMode = DriveMode;
    CMode = false;
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(CMode){
      RobotContainer.m_drivetrain.set(on);
    } if (CMode == false) {
      RobotContainer.m_drivetrain.set(DriveMode);
    } else {
      done = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double RJY = RobotContainer.getDeadZone(4);
    Double LJY = RobotContainer.getDeadZone(1);

    RobotContainer.m_drivetrain.setRight(ControlMode.PercentOutput, RJY);
    RobotContainer.m_drivetrain.setLeft(ControlMode.PercentOutput, LJY);

    System.out.println(RJY + " and " + LJY);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
