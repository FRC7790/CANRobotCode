package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;

public class Harvester extends SubsystemBase {
    // hopefully creating harvester subsystem
    private WPI_VictorSPX harvester = new WPI_VictorSPX(5);

    private WPI_VictorSPX index = new WPI_VictorSPX(53);

    private WPI_VictorSPX indexWheel = new WPI_VictorSPX(54);

    //private Servo latch = new Servo(1);

    // deploy harvester motor
    private DoubleSolenoid harvSolenoid = new DoubleSolenoid(33,PneumaticsModuleType.CTREPCM, 0, 1);

    public Harvester() {
        harvester.configFactoryDefault();
        index.configFactoryDefault();
        indexWheel.configFactoryDefault();
        HarvesterDown();
    }

    //public void DropHarvester()
    //{
    //    latch.setAngle(140);
    //}

    public void HarvesterOn() {
        // turn on harvester
        harvester.set(-.65);
        index.set(.4);
        indexWheel.set(.4);
    }

    public void HarvesterOff() {
        // turn off harvester
        harvester.set(0);
        index.set(0);
        indexWheel.set(0);
    }

    public void HarvesterBackwards(){
        harvester.set(.2);
        index.set(-.2);
        indexWheel.set(-.3);
    }

    public void HarvesterBackwardsStop(){
        harvester.set(0);
        index.set(0);
        indexWheel.set(0);
    }

    public void HarvesterUp() {
        harvSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

    public void HarvesterDown(){
       harvSolenoid.set(DoubleSolenoid.Value.kForward);
   }
}
