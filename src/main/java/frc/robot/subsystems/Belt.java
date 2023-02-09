// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Belt extends SubsystemBase {
  /** Creates a new Belt. */
  int mode;
  Boolean on;
  public Belt(int DefaultSpeedMode) {
    on = false;
    mode = DefaultSpeedMode;
    BeltMotor.setInverted(true);
  }


  public void set(boolean on) {
    this.on = on;
  }

  public void set(int Mode) {
    mode = Mode;
  }

  public TalonFX BeltMotor = new TalonFX(Constants.BeltConstants.BeltMotorCAN);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(on) {
      if (mode == 0) {
        BeltMotor.set(ControlMode.PercentOutput, Constants.BeltConstants.BeltSpeed); 

      } else if(mode == 1) {
        BeltMotor.set(ControlMode.PercentOutput, Constants.BeltConstants.NegBeltSped);

      }
    } else {
      BeltMotor.set(ControlMode.PercentOutput, 0);
    }

    SmartDashboard.putBoolean("Belt On", on);
    SmartDashboard.putNumber("Belt Speed", BeltMotor.getMotorOutputPercent());
  }
}
