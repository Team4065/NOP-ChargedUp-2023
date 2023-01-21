// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kWheelDiameter = Units.inchesToMeters(4.0);
  private static final int kEncoderResolution = 2048;
  private static final int kGearRatio = 5;
  
  //PIDs for robot control
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  //Calabreate this!!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  //makes kinematics obj
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  //sets max velocitys
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
  //Convert to wheel speeds
  DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
  //get LR velocity
  double LeftVelocity = wheelSpeeds.leftMetersPerSecond;
  double RightVelocity = wheelSpeeds.rightMetersPerSecond;

  private final DifferentialDriveOdometry m_odometry;

  //makes the DriveTrain Falcon500s
  private TalonFX RM = new TalonFX(Constants.DriveTrain.RIGHT_MOTOR_MASTER);
  private TalonFX RS = new TalonFX(Constants.DriveTrain.RIGHT_MOTOR_SLAVE);
  private TalonFX LM = new TalonFX(Constants.DriveTrain.LEFT_MOTOR_MASTER);
  private TalonFX LS = new TalonFX(Constants.DriveTrain.LEFT_MOTOR_SLAVE);

 //makes the Gyro
  private AHRS NavX = new AHRS(Port.kMXP);

  //gets the Angle of the robot on the Field (360, -360)
  public double getRobotFace () {
    return NavX.getAngleAdjustment();
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(NavX.getRotation2d(), getLeftWheelsRots(), getRightWheelsRots());
    SmartDashboard.putData(NavX);
    
  }
  /** Creates a new DriveTrain. */
  public Drivetrain() {
    //NavX.calibrate();

    RM.setInverted(true);

    RM.setSelectedSensorPosition(0.0);
    RS.setSelectedSensorPosition(0.0);
    LM.setSelectedSensorPosition(0.0);
    LS.setSelectedSensorPosition(0.0);

    m_odometry = new DifferentialDriveOdometry(NavX.getRotation2d(), getLeftWheelsRots(), getRightWheelsRots());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);


    final double leftOutput = m_leftPIDController.calculate(getLeftMotorVOL(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(getRightMotorVOL(), speeds.rightMetersPerSecond);

    setLeftMotorsV(leftOutput + leftFeedforward);
    setRightMotorsV(rightOutput + rightFeedforward);

    SmartDashboard.putNumber("Left output", leftOutput);
    SmartDashboard.putNumber("Right output", rightOutput);
  }

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }


  //sets the speed of Right motors
  public void setRightMotors (double speed) {
    RM.set(ControlMode.PercentOutput, speed);
  }
  //sets the speed of Left motors
  public void setLeftMotors (double speed) {
    LM.set(ControlMode.PercentOutput, speed);
  }


  //sets the Voltage of Right motors
  public void setRightMotorsV (double Voltage) {
    RM.set(ControlMode.Current, Voltage);
  }
  //sets the Voltage of Left motors
  public void setLeftMotorsV (double Voltage) {
    LM.set(ControlMode.Current, Voltage);
  }

  //gets the average pos of the Right motors
  public double getRightMotorAVG () {
    double RME = RM.getSelectedSensorPosition();
    double RSE = RS.getSelectedSensorPosition(); 
    double RAVGPOS = RME + RSE / 2;
    return RAVGPOS;
  }

  //gets the average pos of the Left motors
  public double getLeftMotorAVG () {
    double LME = LM.getSelectedSensorPosition();
    double LSE = LS.getSelectedSensorPosition(); 
    double LAVGPOS = LME + LSE / 2;
    return LAVGPOS;
  }

  //gets the average pos of the Right motors
  public double getRightMotorVOL () {
    double RME = RM.getSelectedSensorVelocity(0);
    double RSE = RS.getSelectedSensorVelocity(0); 
    double RAVGVOL = RME + RSE / 2;
    return RAVGVOL;
  }

  //gets the average pos of the Left motors
  public double getLeftMotorVOL () {
    double LME = LM.getSelectedSensorVelocity(0);
    double LSE = LS.getSelectedSensorVelocity(0); 
    double LAVGVOL = LME + LSE / 2;
    return LAVGVOL;
  }

  //gets the Left motor to wheel rotations
  public double getLeftWheelsRots () {
    return getLeftMotorAVG()/kEncoderResolution * kGearRatio;
  }

  //gets the Right motor to wheel rotations
  public double getRightWheelsRots () {
    return getRightMotorAVG()/kEncoderResolution * kGearRatio;
  }

  //gets the Right distance in meters 
  public double getRightDistanceInMeters () {
    return getRightWheelsRots() * Units.inchesToMeters(kWheelDiameter)* Math.PI;
  }

  //gets the Left distance in meters 
  public double getLeftDistanceInMeters () {
    return getLeftWheelsRots() * Units.inchesToMeters(kWheelDiameter)* Math.PI;
  }

 

}
