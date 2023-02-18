// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  /** Creates a new getOnRamp. */
  // double error, currentAngle, drivePower, kP = 0.08; // kp 0.015
  PIDController pid = new PIDController(0.03, 0, 0.003);
  boolean end;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AUTO BALANCING ENABLED!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("balancing...");
    double angle = RobotContainer.m_drivetrain.getPitch();
    double speed = pid.calculate(angle, Constants.AutoConstants.balancedGyro);
    System.out.println(speed);

    if (speed < -0.5) {
      RobotContainer.m_drivetrain.tankDrive(0.4, 0.4);
    } else if (speed > 0.5) {
      RobotContainer.m_drivetrain.tankDrive(-0.4, -0.4);
    } else {
      RobotContainer.m_drivetrain.tankDrive(-speed, -speed);
    }

    if (angle > Constants.AutoConstants.balancedGyro - Constants.AutoConstants.acceptableAngleRange && angle < Constants.AutoConstants.balancedGyro + 2.5) {
      System.out.println("HOPEFULLY BALANCED!");
      end = true;
    }

    // currentAngle = RobotContainer.m_drivetrain.getPitch();

    // error = Constants.AutoConstants.balancedGyro - currentAngle;
    // drivePower = -Math.min(kP * error, 1);
    
    // if (drivePower < 0) {
    //   drivePower *= Constants.AutoConstants.backwardsBalancingPower;
    // }

    // if (Math.abs(drivePower) > 0.4) {
    //   drivePower = Math.copySign(0.4, drivePower);
    // }

    // RobotContainer.m_drivetrain.tankDrive(drivePower, drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
    // System.out.println("finished");
    // return Math.abs(error) < Constants.AutoConstants.acceptableAngleRange;
  }
}
