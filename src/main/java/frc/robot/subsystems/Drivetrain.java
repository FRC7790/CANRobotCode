// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private WPI_VictorSPX left1 = new WPI_VictorSPX(1);
  private WPI_VictorSPX left2 = new WPI_VictorSPX(2);
  private WPI_VictorSPX right1 = new WPI_VictorSPX(3);
  private WPI_VictorSPX right2 = new WPI_VictorSPX(4);

  private MotorControllerGroup leftGroup = new MotorControllerGroup(left1, left2);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(right1, right2);

  private DifferentialDrive drivetrain = new DifferentialDrive(leftGroup, rightGroup);

  private LimeLight m_limelight;

  public Encoder leftEncoder = new Encoder(0,1);
  public Encoder rightEncoder = new Encoder(2,3);

  public Drivetrain(LimeLight limeLight) {
    m_limelight = limeLight;
    left1.configFactoryDefault();
    left2.configFactoryDefault();
    right1.configFactoryDefault();
    right2.configFactoryDefault();

    rightGroup.setInverted(false);
    leftGroup.setInverted(true);

    leftEncoder.setDistancePerPulse(Constants.DistancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.DistancePerPulse);
    
  }

  public void move(double speed, double rotation)
  {
    SmartDashboard.putNumber("Move Speed", speed);
    drivetrain.arcadeDrive(speed, -rotation);
  }
  

  @Override
  public void periodic() {
   //System.out.println("L: " + leftEncoder.getDistance());
   //System.out.println("R: " + rightEncoder.getDistance());
  }

}
