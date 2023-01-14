// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  SlewRateLimiter ramp = new SlewRateLimiter(0.5);
  public Drivetrain() {
    rightM.setNeutralMode(NeutralMode.Brake);
    leftM.setNeutralMode(NeutralMode.Brake);
    
  }

  private TalonFX rightM = new TalonFX(OperatorConstants.RightMaster); 
  private TalonFX rightS = new TalonFX(OperatorConstants.RightSlave); 
  private TalonFX leftM = new TalonFX(OperatorConstants.LeftMaster); 
  private TalonFX leftS = new TalonFX(OperatorConstants.LeftSlave); 

  public void setRight(ControlMode controlmode, double value){
    rightM.set(controlmode, value);
    rightS.setInverted(true);
    rightS.follow(rightM);
  }

  public void setLeft(ControlMode controlmode, double value){
    leftM.set(controlmode, value);
    leftS.setInverted(true);
    leftS.follow(rightM);
  }

  public void setPos(int POS){
    setRight(ControlMode.Position, POS);
    setLeft(ControlMode.Position, POS);
  }

  public int getFT(){
    return (Integer) null;
  } 
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
