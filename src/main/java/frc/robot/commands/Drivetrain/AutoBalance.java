// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
<<<<<<< Updated upstream
  /** Creates a new AutoBalance. */
  boolean end;
  double pitch, kP = 0.3;
=======
  /** Creates a new getOnRamp. */
  // double error, currentAngle, drivePower, kP = 0.08; // kp 0.015
  PIDController pid = new PIDController(0.024, 0.001, 0.002);
  double speed;
  double angle;
  public static boolean stop;
>>>>>>> Stashed changes
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    stop = false;
  }

  // Called when the command is initially scheduled.
  @Override
<<<<<<< Updated upstream
  public void initialize() {}
=======
  public void initialize() {
    System.out.println("AUTO BALANCING ENABLED!");
    // DriveTrain.rightM.configOpenloopRamp(0);
    // DriveTrain.leftM.configOpenloopRamp(0);
    // DriveTrain.rightS.configOpenloopRamp(0);
    // DriveTrain.leftS.configOpenloopRamp(0);
    // RobotContainer.m_drivetrain.setBreakMode(); 
  }
>>>>>>> Stashed changes

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    pitch = RobotContainer.m_drivetrain.getPitch();
    while (pitch > 2.5 || pitch < -2.5) {
      if (pitch > 0) {
        RobotContainer.m_drivetrain.tankDrive(kP, kP);
      } else if (pitch < 0) {
        RobotContainer.m_drivetrain.tankDrive(-kP, -kP);
      }
    }
    end = true;
=======
    System.out.println("BALANCE");
    angle = RobotContainer.m_drivetrain.getPitch();
    speed = pid.calculate(angle, Constants.AutoConstants.balancedGyro);

    if (speed < -0.5) {
      RobotContainer.m_drivetrain.tankDrive(0.4, 0.42);
    } else if (speed > 0.5) {
      RobotContainer.m_drivetrain.tankDrive(-0.42, -0.429);
    } else {
      RobotContainer.m_drivetrain.tankDrive(-speed, -speed);
    }

    

>>>>>>> Stashed changes
  }
  // Called once the command ends or is interrupted.
  @Override
<<<<<<< Updated upstream
  public void end(boolean interrupted) {}
=======
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setBreakMode();
  }
>>>>>>> Stashed changes

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<< Updated upstream
    return end;
=======
    // return Math.abs; 
    return false;
>>>>>>> Stashed changes
  }
}
