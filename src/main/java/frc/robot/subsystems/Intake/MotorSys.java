// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorSys extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
  /** Creates a new MotorSys. */
  public MotorSys() {}

  public void setMotor(double speed) {
    intakeMotor.set(speed);
  }

  public boolean isRunning(){
    boolean isrun;

    if (intakeMotor.get() != 0){
      isrun = true;
    } else {
      isrun = false;
    }

    return isrun;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
