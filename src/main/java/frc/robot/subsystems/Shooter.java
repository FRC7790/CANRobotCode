
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;


public class Shooter extends SubsystemBase {
  //private CANSparkMax shooter1 = new 
  //private WPI_VictorSPX shooter3 = new WPI_VictorSPX(53);
  //private WPI_VictorSPX shooter4 = new WPI_VictorSPX(54);
  //private MotorControllerGroup shooterGroup2 = new MotorControllerGroup(shooter3, shooter4);



private CANSparkMax m_leadMotor;
private CANSparkMax m_followMotor;
private WPI_VictorSPX indexer = new WPI_VictorSPX(52);



  public Shooter() {

    m_leadMotor = new CANSparkMax(15, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(16, MotorType.kBrushless);
     m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();
    indexer.configFactoryDefault();

    m_followMotor.follow(m_leadMotor,true);

  





   
  }

  public void IndexStart() {
    indexer.set(.6);
  }

  public void IndexBack() {
    indexer.set(-.2);
  }

  public void IndexStop() {
    indexer.set(0);
  }
  public void ShooterStart(double speed) {
    //System.out.println("Shooter start");
    m_leadMotor.set(speed);
    
  }

  public void ShooterStop() {
    //System.out.println("Shooter stop");
    m_leadMotor.set(0);
    
  }

  public void ShooterBackwards(){
    m_leadMotor.set(.2);
    //indexer.set(-.2);
  }
  public void ShooterBackwardsStop(){
    m_leadMotor.set(0);
    //indexer.set(0);
  }

  @Override
  public void periodic() {

  }
}
