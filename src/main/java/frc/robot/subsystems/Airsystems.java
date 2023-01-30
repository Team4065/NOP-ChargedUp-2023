// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Airsystems extends SubsystemBase {
  /** Creates a new Airsystems. */
  public Airsystems() {

    compressor = PCM.makeCompressor();
    compressor.enableDigital();
  }

  public static PneumaticsControlModule PCM = new PneumaticsControlModule(1);
  public static Compressor compressor;


  private static Solenoid rightS = PCM.makeSolenoid(1);
  private static Solenoid leftS = PCM.makeSolenoid(2);
  

  public void setLS (boolean value){
    leftS.set(value);

    SmartDashboard.putBoolean("Left Solenoid ", value);
  }

  public void setRS (boolean value){
    rightS.set(value);

    SmartDashboard.putBoolean("Right Solenoid ", value);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
