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
<<<<<<< Updated upstream
  PIDController pid = new PIDController(0.03, 0, 0.003);
  boolean end;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    end = false;
=======
  PIDController pid = new PIDController(0.024, 0.001, 0.002);
  double speed;
  double angle;
  public static boolean stop;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
>>>>>>> Stashed changes
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AUTO BALANCING ENABLED!");
<<<<<<< Updated upstream
=======
    // DriveTrain.rightM.configOpenloopRamp(0);
    // DriveTrain.leftM.configOpenloopRamp(0);
    // DriveTrain.rightS.configOpenloopRamp(0);
    // DriveTrain.leftS.configOpenloopRamp(0);
    // RobotContainer.m_drivetrain.setBreakMode(); 
    stop = false;
>>>>>>> Stashed changes
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    System.out.println("balancing...");
    double angle = RobotContainer.m_drivetrain.getPitch();
    double speed = pid.calculate(angle, Constants.AutoConstants.balancedGyro);
    System.out.println(speed);
=======
    angle = RobotContainer.m_drivetrain.getPitch();
    speed = pid.calculate(angle, Constants.AutoConstants.balancedGyro);
>>>>>>> Stashed changes

    System.out.println("BALANCE: " + speed);

    if (speed < -0.5) {
<<<<<<< Updated upstream
      RobotContainer.m_drivetrain.tankDrive(0.4, 0.4);
    } else if (speed > 0.5) {
      RobotContainer.m_drivetrain.tankDrive(-0.4, -0.4);
=======
      RobotContainer.m_drivetrain.tankDrive(0.43, 0.43);
    } else if (speed > 0.5) {
      RobotContainer.m_drivetrain.tankDrive(-0.43, -0.43);
>>>>>>> Stashed changes
    } else {
      RobotContainer.m_drivetrain.tankDrive(-speed, -speed);
    }

<<<<<<< Updated upstream
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
=======
    if (angle > (Constants.AutoConstants.balancedGyro - Constants.AutoConstants.acceptableAngleRange) && angle < (Constants.AutoConstants.balancedGyro + Constants.AutoConstants.acceptableAngleRange)) {
      System.out.println("ANGLE REACHED");
      stop = true;
    }
>>>>>>> Stashed changes
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< Updated upstream
=======
    System.out.println("BALANCE STOPPED");
    RobotContainer.m_drivetrain.setBreakMode();
>>>>>>> Stashed changes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<< Updated upstream
    return end;
    // System.out.println("finished");
    // return Math.abs(error) < Constants.AutoConstants.acceptableAngleRange;
=======
    // return Math.abs; 
    return stop;
>>>>>>> Stashed changes
  }
}
