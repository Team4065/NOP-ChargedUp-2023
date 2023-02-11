// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class photonvision2 extends SubsystemBase {
  /** Creates a new photonvision2. */
  boolean ready = false;
  public photonvision2() {
    SmartDashboard.putBoolean("Ready to shoot", ready);

  }

   public static PhotonCamera p_cam = new PhotonCamera("OV5647");

   public void setReady() {
    ready = true;
   }


   public boolean hasTargets() {
    return (p_cam.getLatestResult().hasTargets());
   }

   public double tagpitch() {
    return (p_cam.getLatestResult().getBestTarget().getPitch());
   }

   public static double tagArea() {
    try {
      return (p_cam.getLatestResult().getBestTarget().getArea());
    } catch (Exception e) {
      return 20;
    }
   }

   public static double tagYaw() {
    try {
      return (p_cam.getLatestResult().getBestTarget().getYaw());
    } catch (Exception e) {
      // TODO: handle exception
      return 0;
    }
   }

   public int targetID() {
    try {
      return (p_cam.getLatestResult().getBestTarget().getFiducialId());
    } catch (Exception e) {
      // TODO: handle exception
      return 0;
    }
   }


   public boolean hastargetstest(){
    var result = p_cam.getLatestResult();
    System.out.println("Result Variable: " + result);

    boolean hastargets = result.hasTargets();
    System.out.println("Hastargets Variable: "+ hastargets);

    return hastargets;
   }



  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Yaw", p_cam.getLatestResult().getBestTarget().getYaw());
  }
    // This method will be called once per scheduler run
  
}
