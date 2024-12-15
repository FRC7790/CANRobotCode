package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/** Represents a differential drive style drivetrain. */
public class Drivetrain2 extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.55; // meters
  private static final double kWheelRadius = 0.076;//0.0508; // meters
  private static final int kEncoderResolution = 360;

  private final WPI_VictorSPX m_leftLeader = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_rightLeader = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(4);

  public final Encoder m_leftEncoder = new Encoder(0, 1);
  public final Encoder m_rightEncoder = new Encoder(2, 3);

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  private LimeLight m_limelight;
  private AHRS m_ahrs;
  private double startYaw;

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain2(LimeLight limeLight) {

    System.out.println("start drive");
    m_limelight = limeLight;
    m_gyro.reset();

    m_ahrs = new AHRS(Port.kMXP);
    startYaw = m_ahrs.getYaw();
    System.out.println("yaw is :" + startYaw);

     

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);

    updateOdometry();
  }

  public void tankDrive(double left, double right)
  {
    m_leftGroup.setVoltage(left);
    m_rightGroup.setVoltage(right);

  }

  public double getYaw(){
    System.out.println("yaw is :" + m_ahrs.getYaw());
return m_ahrs.getYaw();
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  
    SmartDashboard.putNumber("odom1 x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odom1 y", m_odometry.getPoseMeters().getY());


    SmartDashboard.putNumber("enc 1", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("enc 2", m_rightEncoder.getDistance());
    
        
    SmartDashboard.putNumber("m_gyro", m_gyro.getAngle());
    
  }
}