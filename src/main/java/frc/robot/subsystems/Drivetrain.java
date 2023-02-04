// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.AutoPaths.LoadPath;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase {


  //True = Arcade
  //False = Tank

  // CANSparkMax leftMotor = new CANSparkMax(Constants.leftMotor, MotorType.kBrushless);
  // CANSparkMax rightMotor = new CANSparkMax(Constants.rightMotor, MotorType.kBrushless);

  // Create our motor instances
  WPI_TalonFX leftM = new WPI_TalonFX(Constants.DriveConstants.LeftMaster);
  WPI_TalonFX leftS = new WPI_TalonFX(Constants.DriveConstants.LeftSlave);
  WPI_TalonFX rightM = new WPI_TalonFX(Constants.DriveConstants.RightMaster);
  WPI_TalonFX rightS = new WPI_TalonFX(Constants.DriveConstants.RightSlave);

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration(); 

  // Create motor control groups so it's easier to manage
  MotorControllerGroup leftSideDrive = new MotorControllerGroup(leftM, leftS);
  MotorControllerGroup rightSideDrive = new MotorControllerGroup(rightM, rightS);

  // RelativeEncoder rightEncoder = rightMotor.getEncoder();
  // RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // Values for the distance function
  // PIDController moveMotorPID = new PIDController(0.5, 0, 0);
  // public double encoderRawVal;
  // public double encoderSetpoint;

  DifferentialDrive diffDrive;
  public static Gyro g_gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry m_odometry;
  private Field2d m_field = new Field2d();

  
  public static double percentOutput; // This variable controls the percent output
  public static boolean isReversed;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    g_gyro.reset();

    leftS.follow(leftM);
    rightS.follow(rightM);

    rightM.setInverted(false);
    rightS.setInverted(false);
    leftM.setInverted(true);
    leftS.setInverted(true);

    percentOutput = 0.5;
    isReversed = false;

    // leftM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // rightM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    resetEncoders();
    setBreakMode();

    diffDrive = new DifferentialDrive(leftSideDrive, rightSideDrive);

    m_odometry = new DifferentialDriveOdometry(
      g_gyro.getRotation2d(),
      encoderTicksToMeters(leftM.getSelectedSensorPosition()),
      encoderTicksToMeters(rightM.getSelectedSensorPosition())
    );

    Shuffleboard.getTab("Field").add(m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      g_gyro.getRotation2d(),
      encoderTicksToMeters(leftM.getSelectedSensorPosition()),
      encoderTicksToMeters(rightM.getSelectedSensorPosition())
    );

    SmartDashboard.putNumber("Left encoder values (Meters)", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder values (Meters)", getRightEncoderPosition());
    SmartDashboard.putNumber("Right velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Left velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Gyro heading", getHeading());
    SmartDashboard.putNumber("Right encoder", rightM.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left encoder", leftM.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Reversed", isReversed);
    SmartDashboard.putNumber("Percent Speed", percentOutput);
    m_field.setRobotPose(getPose());
    m_field.getObject("traj").setTrajectory(LoadPath.pathToFollow.get(0));
  }

  public Command showTraj(int pathNum) {
    m_field.getObject("traj").setTrajectory(LoadPath.pathToFollow.get(pathNum));
    return new InstantCommand();
  }

  public void setRight(ControlMode controlmode, double value){
    rightM.set(controlmode, -value);
  }

  public void setLeft(ControlMode controlmode, double value){
    leftM.set(controlmode, value);
  }

  // This method can be used to convert encoder ticks to meters 
  public double encoderTicksToMeters(double currentEncoderValue) {
    double motorRotations = (double) currentEncoderValue / Constants.AutoConstants.kEncoderFullRev;
    double wheelRotations = motorRotations / Constants.DriveConstants.kGearRatio;
    double positionMeters = wheelRotations * Units.inchesToMeters(Constants.DriveConstants.kWheelCircumferenceInches);
    return positionMeters;
  }


  public void tankDrive(double leftSpeed, double rightSpeed) {
    setCoastMode();
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void setBreakMode() {
    leftM.setNeutralMode(NeutralMode.Brake);
    rightM.setNeutralMode(NeutralMode.Brake);
    leftS.setNeutralMode(NeutralMode.Brake);
    rightS.setNeutralMode(NeutralMode.Brake);

  }

  public void setCoastMode() {
    leftM.setNeutralMode(NeutralMode.Coast);
    rightM.setNeutralMode(NeutralMode.Coast);
    leftS.setNeutralMode(NeutralMode.Coast);
    rightS.setNeutralMode(NeutralMode.Coast);
  }

  public void resetEncoders() {
    leftM.setSelectedSensorPosition(0);
    leftS.setSelectedSensorPosition(0);
    rightM.setSelectedSensorPosition(0);
    rightS.setSelectedSensorPosition(0);
  }

  public double getRightEncoderPosition() {
    return encoderTicksToMeters(rightM.getSelectedSensorPosition());  
  }

  public double getLeftEncoderPosition() {
    return encoderTicksToMeters(leftM.getSelectedSensorPosition());
  }

  public double getRightEncoderVelocity() {
    // Multiply the raw velocity by 10 since it reports per 100 ms, we want the velocity in m/s
    return encoderTicksToMeters(rightM.getSelectedSensorVelocity()) * 10;
  }

  public double getLeftEncoderVelocity() {
    return encoderTicksToMeters(leftM.getSelectedSensorVelocity()) * 10;
  }

  public double getTurnRate() {
    return -g_gyro.getRate();
  }

  public static double getHeading() {
    return g_gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometery(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
      g_gyro.getRotation2d(),
      encoderTicksToMeters(leftM.getSelectedSensorPosition()),
      encoderTicksToMeters(rightM.getSelectedSensorPosition()),
      pose
    );
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSideDrive.setVoltage(leftVolts);
    rightSideDrive.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double getAverageEncoderDis() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  // public RelativeEncoder getLeftEncoder() {
  //   return rightM.get;
  // }

  // public RelativeEncoder getRightEncoder() {
  //   return rightEncoder;
  // }

  public void setMaxOutputOfDrive(double maxOut) {
    diffDrive.setMaxOutput(maxOut);
  }

  public void zeroHeading() {
    g_gyro.reset();
  }

  public Gyro getGyro() {
    return g_gyro;
  }
}