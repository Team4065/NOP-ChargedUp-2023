// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
<<<<<<< Updated upstream

public class SolControl extends CommandBase {
  /** Creates a new SolControl. */
  boolean end, state;
  public SolControl(boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_airsys);
    this.state = state;
=======
import frc.robot.subsystems.Intake.AirSys;

public class SolControl extends CommandBase {
  /** Creates a new SolControl. */
  boolean end;
  public SolControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_airsys);
>>>>>>> Stashed changes
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    RobotContainer.m_airsys.setSols(state);
=======
    if (AirSys.controlState == false) {
      AirSys.controlState = true;
    } else if (AirSys.controlState == true) {
      AirSys.controlState = false;
    }

    RobotContainer.m_airsys.setSols(AirSys.controlState);
>>>>>>> Stashed changes
    end = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
