// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.AirSys;

public class IntakeBB extends CommandBase {
  /** Creates a new BeamBreaks. */
  boolean FIN = false;
  boolean currentBB;
  boolean runINTKE;

  AirSys airsys = RobotContainer.m_airsys;
  frc.robot.subsystems.Intake.MotorSys MotorSys = RobotContainer.m_motorsys;
  

  public IntakeBB(Boolean runINTKE) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_motorsys);
    addRequirements(RobotContainer.m_airsys);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentBB = RobotContainer.m_beamBreaks.getBeamBreak(0);

    if (currentBB == false ){ //if something is in the beambreaks
      MotorSys.setMotor(0);
      airsys.setSols(false);
      

     /*  if(airsys.getSols() == true){
        airsys.setSols(false);
      }

      if(MotorSys.isRunning() == true){
        MotorSys.setMotor(0);
      } */

    } if (currentBB == false && runINTKE == true) { // if nothing is in path AND intake is set to run, run intake sys

      airsys.setSols(true);
      MotorSys.setMotor(Constants.Intake.IntakeSpeed);
       /*  if(airsys.getSols() == false){
          airsys.setSols(true);
        }

        if(MotorSys.isRunning() == false){
          MotorSys.setMotor(Constants.Intake.IntakeSpeed);
        }  */
    } else {
      MotorSys.setMotor(0);
      airsys.setSols(false); 
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
