// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.signals.ControlModeValue;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  int DriveMode;
  Boolean on;




  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kWheelDiameter = Units.inchesToMeters(4.0);
  private static final int kEncoderResolution = 2048;
  private static final double kGearRatio = 5.4545;
  
  //PIDs for robot control
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    //makes kinematics obj
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  //sets max velocitys
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
  //Convert to wheel speeds
  DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
  //get LR velocity
  double LeftVelocity = wheelSpeeds.leftMetersPerSecond;
  double RightVelocity = wheelSpeeds.rightMetersPerSecond;

  public final Field2d m_Field = new Field2d();

  SlewRateLimiter ramp = new SlewRateLimiter(0.5);

  private TalonFX rightM = new TalonFX(OperatorConstants.RightMaster); 
  private TalonFX rightS = new TalonFX(OperatorConstants.RightSlave); 
  private TalonFX leftM = new TalonFX(OperatorConstants.LeftMaster); 
  private TalonFX leftS = new TalonFX(OperatorConstants.LeftSlave);

/*   private MotorControllerGroup RMG = new MotorControllerGroup(rightM,rightM);
  private MotorControllerGroup LMG = new MotorControllerGroup(leftM,leftS);
*/

  public Drivetrain() {
    rightM.setNeutralMode(NeutralMode.Coast);
    leftM.setNeutralMode(NeutralMode.Coast);
    rightS.setNeutralMode(NeutralMode.Coast);
    leftS.setNeutralMode(NeutralMode.Coast);

    on = true;
    DriveMode = 0;
    
  }

  public void set(boolean on) {
    this.on = on;
  }

  public void set(int DriveMode) {
    this.DriveMode = DriveMode;
  }

  public void setMotors(ControlMode controlmode, double value){
    setRight(controlmode, value);
    setLeft(controlmode, value);
  }

 

  public void setRight(ControlMode controlmode, double value){
    rightM.set(controlmode, -value);
    rightS.follow(rightM);

    SmartDashboard.putNumber("R_Value", value);
    SmartDashboard.putBoolean("R_Inverted", rightM.getInverted());
    SmartDashboard.putString("R_ControlMode", String.valueOf(controlmode));
  }

  public void setLeft(ControlMode controlmode, double value){
    leftM.set(controlmode, value);
    leftS.follow(leftM);

    SmartDashboard.putNumber("L_Value", value);
    SmartDashboard.putBoolean("L_Inverted", leftS.getInverted());
    SmartDashboard.putString("L_ControlMode", String.valueOf(controlmode));
  }

  public void setPos(int POS){
    setRight(ControlMode.Position, POS);
    setLeft(ControlMode.Position, POS);
  }


  public void resetSensors(){
    rightM.setSelectedSensorPosition(0);
    rightS.setSelectedSensorPosition(0);
    leftM.setSelectedSensorPosition(0);
    leftS.setSelectedSensorPosition(0);
  }

/*  public int getFT(){
    return (Integer) null;
  } */ 

  



  @Override
  public void periodic() {
    if(on) {
      switch (DriveMode){
        case 0:
          setMotors(ControlMode.PercentOutput, Constants.DriveTrainConstants.Speed1);

        case 1:
          setMotors(ControlMode.PercentOutput, Constants.DriveTrainConstants.Speed2);

        case 2:
          setMotors(ControlMode.PercentOutput, Constants.DriveTrainConstants.Speed3);
  
      }
    }
  }

 

}



/* // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kWheelDiameter = Units.inchesToMeters(4.0);
  private static final int kEncoderResolution = 2048;
  private static final double kGearRatio = 5.4545;
  
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

  private MotorControllerGroup RMG = new MotorControllerGroup(RM,RS);
  private MotorControllerGroup LMG = new MotorControllerGroup(LM,LS);

  /** Creates a new DriveTrain. 
  public DriveTrain() {
    NavX.calibrate();

    RMG.setInverted(true);

    RM.setRotorPosition(0.0);
    RS.setRotorPosition(0.0);
    LM.setRotorPosition(0.0);
    LS.setRotorPosition(0.0);

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

    LMG.setVoltage(leftOutput + leftFeedforward);
    RMG.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }


  //sets the speed of Right motors
  public void setRightMotors (double speed) {
    RMG.set(speed);
  }
  //sets the speed of Left motors
  public void setLeftMotors (double speed) {
    LMG.set(speed);
  }


  //sets the Voltage of Right motors
  public void setRightMotorsV (double Voltage) {
    RMG.setVoltage(Voltage);
  }
  //sets the Voltage of Left motors
  public void setLeftMotorsV (double Voltage) {
    LMG.setVoltage(Voltage);
  }

  //gets the average pos of the Right motors
  public double getRightMotorAVG () {
    double RME = RM.getPosition().getValue();
    double RSE = RS.getPosition().getValue(); 
    double RAVGPOS = RME + RSE / 2;
    return RAVGPOS;
  }

  //gets the average pos of the Left motors
  public double getLeftMotorAVG () {
    double LME = LM.getPosition().getValue();
    double LSE = LS.getPosition().getValue(); 
    double LAVGPOS = LME + LSE / 2;
    return LAVGPOS;
  }

  //gets the average pos of the Right motors
  public double getRightMotorVOL () {
    double RME = RM.getVelocity().getValue();
    double RSE = RS.getVelocity().getValue(); 
    double RAVGVOL = RME + RSE / 2;
    return RAVGVOL;
  }

  //gets the average pos of the Left motors
  public double getLeftMotorVOL () {
    double LME = LM.getVelocity().getValue();
    double LSE = LS.getVelocity().getValue(); 
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

  //makes the Gyro
  private AHRS NavX = new AHRS(Port.kMXP);

  //gets the Angle of the robot on the Field (360, -360)
  public double getRobotFace () {
    return NavX.getAngleAdjustment();
  }

  /** Updates the field-relative position. 
  public void updateOdometry() {
    m_odometry.update(NavX.getRotation2d(), getLeftWheelsRots(), getRightWheelsRots());
  }

}
 */