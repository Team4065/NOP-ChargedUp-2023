// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain extends SubsystemBase {


  //True = Arcade
  //False = Tank

  CANSparkMax leftMotor = new CANSparkMax(Constants.leftMotor, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(Constants.rightMotor, MotorType.kBrushless);

  RelativeEncoder rightEncoder = rightMotor.getEncoder();
  RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // Values for the distance function
  PIDController moveMotorPID = new PIDController(0.5, 0, 0);
  public double encoderRawVal;
  public double encoderSetpoint;

  DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);
  public static AHRS g_gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry m_odometry;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();

    rightMotor.setInverted(true);
    leftMotor.setInverted(false);
    
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(Constants.AutoConstants.kConversionMeters);
    leftEncoder.setPositionConversionFactor(Constants.AutoConstants.kConversionMeters);
    rightEncoder.setVelocityConversionFactor(Constants.AutoConstants.kConversionMeters / 60);
    leftEncoder.setVelocityConversionFactor(Constants.AutoConstants.kConversionMeters / 60);

    m_odometry = new DifferentialDriveOdometry(g_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    m_odometry.resetPosition(g_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
  }

  public void setBreakMode() {
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getTurnRate() {
    return -g_gyro.getAngle();
  }

  public static double getHeading() {
    return g_gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometery(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(g_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double getAverageEncoderDis() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutputOfDrive(double maxOut) {
    diffDrive.setMaxOutput(maxOut);
  }

  public void zeroHeading() {
    g_gyro.calibrate();
    g_gyro.reset();
  }

  public Gyro getGyro() {
    return g_gyro;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(g_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left encoder values (Meters)", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder values (Meters)", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }

  public void setLeft(double speed) {
    leftMotor.set(speed);
    SmartDashboard.putNumber("Left Drive", speed);

  }

  public void setRight(double speed) {
    rightMotor.set(speed); 
    SmartDashboard.putNumber("Right Drive", speed);

  }

  public void time(int delayPeriod) {
    try {
      Thread.sleep(delayPeriod);
    } catch (InterruptedException e) {
      
    }
  }

  /* 
  public double encoderVal(String motor) {
    double val = 0.0;
    double tempVal = 0.0;
    switch (motor) {
      case "right":
        val = rightEncoder.getPosition();
        break;
      case "left":
        val = leftEncoder.getPosition();
        break;
      case "both":
        val = rightEncoder.getPosition();
        tempVal = leftEncoder.getPosition();
        val = (val + tempVal) / 2;
        break;
      default:
        System.out.println("Motor not picked!");
        break;
    }
    return val;
  }
  */

  /*
  public void moveDistance(String motor, double distanceToMoveInInches) {
    double circum = 4 * Math.PI; // The circumferene is ~12.56
    encoderRawVal = encoderVal(motor);
    double oneRotationOfEncoder = 10;
    
    double pulseToTravel = (distanceToMoveInInches / circum) * oneRotationOfEncoder;
    
    if (calculate == true) {
      encoderSetpoint = encoderRawVal + pulseToTravel;
      calculate = false;
    }

    // System.out.println(encoderRawVal);
    // System.out.println(encoderSetpoint);

    // rightMotor.set(moveMotorPID.calculate(encoderRawVal, encoderSetpoint));
    switch (motor) {
      case "right":
        rightMotor.set(moveMotorPID.calculate(encoderRawVal, encoderSetpoint));
        break;
      case "left":
        leftMotor.set(moveMotorPID.calculate(encoderRawVal, encoderSetpoint));
        break;
      case "both":
        rightMotor.set(moveMotorPID.calculate(encoderRawVal, encoderSetpoint));
        leftMotor.set(moveMotorPID.calculate(encoderRawVal, encoderSetpoint));
        break;
      default:
        System.out.println("Invalid motor selected!");
    }
  }
  */
}