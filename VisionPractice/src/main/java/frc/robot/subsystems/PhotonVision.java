// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PhotonVision extends SubsystemBase {
//   /** Creates a new PhotonVision. */
//   public PhotonVision() {}

//   // public PhotonCamera cam = new PhotonCamera("photonvision");
//   // public PhotonPipelineResult result = cam.getLatestResult();

//   static public final edu.wpi.first.networktables.NetworkTable photonCam = NetworkTableInstance.getDefault().getTable("photonvision");

//   public static boolean hasTargets() {
//     return (photonCam.getEntry("hasTarget").getBoolean(false));
//   }

//   public static double getArea() {
//     return (photonCam.getEntry("targetArea").getDouble(Double.NaN));
//   }

//   public static double getYaw() {
//     return (photonCam.getEntry("targetYaw").getDouble(Double.NaN));
//   }

//   public static double getpitch() {
//     return (photonCam.getEntry("targetPitch").getDouble(Double.NaN));
//   }


  

//   public static double xaxis() {
//     return (photonCam.getEntry("targetPixelsX").getDouble(Double.NaN));
//   }

//   public static double yaxis() {
//     return (photonCam.getEntry("targetPixelsY").getDouble(Double.NaN));
//   }




//   public static void setLedMode(int mode) {
//     photonCam.getEntry("ledMode").setNumber(mode);
//   }

//   public static void enableVisionProcessing() {
//     photonCam.getEntry("camMode").setNumber(0.0);
//   }

//   public static void dissableVisionProcessing() {
//     photonCam.getEntry("camMode").setNumber(1.0); 
//   }




//   public void setPipeline(int pipeline) {
//     photonCam.getEntry("pipeline").setNumber(pipeline);
//   }

//   public static int getPipeline() {
//     return(int)photonCam.getEntry("pipeline").getDouble(Double.NaN);
//   }
 
  
//   // public PhotonPipelineResult info() {
//   //   // System.out.println("Subsystem returns: " + result.hasTargets());
//   //   return result;
//   // }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
 