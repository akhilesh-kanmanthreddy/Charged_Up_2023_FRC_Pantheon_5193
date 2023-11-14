
//Class for Turret controller Neo with SparkMax controller

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {
    public Turret() {};

    private int elevatorID = 2;
    private int turretID = 8;
    private int extenderID = 16;
    private int elbowID = 18;
    private int clamperID = 1;

    private int extenderTouchID = 0;
    
    private TalonFX elevator = new TalonFX(elevatorID);
    private CANSparkMax turret = new CANSparkMax(turretID,MotorType.kBrushless);
    private CANSparkMax extender = new CANSparkMax(extenderID,MotorType.kBrushless);
    private CANSparkMax elbow = new CANSparkMax(elbowID,MotorType.kBrushless);
    private CANSparkMax clamper = new CANSparkMax(clamperID,MotorType.kBrushless);

    private DigitalInput extenderTouch = new DigitalInput(extenderTouchID);
    
    private RelativeEncoder clamperEncoder = clamper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder elbowEncoder = elbow.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder turretEncoder = turret.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder extenderEncoder = extender.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);



    public double getClampPosition(){
      return clamperEncoder.getPosition();
    }

    public boolean getExtenderTouch(){
      return extenderTouch.get();
    }

    public double getElbowPosition(){
      return elbowEncoder.getPosition();
    }

    public double getTurretPosition(){
      return turretEncoder.getPosition();
    }

    public double getExtenderPosition(){
      return extenderEncoder.getPosition();
    }

    public double getElevatorPosition(){
      return elevator.getSelectedSensorPosition();
    }

    // public double getElevatorPosition(){
    //   return elevator.getSelectedSensorPosition();
    // }

    // public void resetElevatorPosition(){
    //   elevator.setSelectedSensorPosition(0);
    // }
    public void resetClampEncoder(){
      clamperEncoder.setPosition(0);
    }

    public void resetElbowEncoder(){
      elbowEncoder.setPosition(0);
    }

    public void resetTurretEncoder(){
      turretEncoder.setPosition(0);
    }

    public void resetExtenderEncoder(){
      extenderEncoder.setPosition(0);
    }

    public void resetElevatorEncoder(){
      elevator.setSelectedSensorPosition(0);
    }
 

    public void setPowerTurret(double power){
        turret.set(power);
    }

    public void setPowerExtender(double power){
        extender.set(power);
    }

    public void setPowerElbow(double power){
        elbow.set(power);
    }

    public void setPowerClamper(double power){
        clamper.set(power);
    }

    public void setPowerElevator(double power){
      elevator.set(ControlMode.PercentOutput, power);
  }

    public void setCoastAndBrake(){
        turret.setIdleMode(IdleMode.kCoast);
        extender.setIdleMode(IdleMode.kBrake);
        elbow.setIdleMode(IdleMode.kBrake);
        clamper.setIdleMode(IdleMode.kBrake);
        elevator.setNeutralMode(NeutralMode.Brake);
    }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
        });
  }



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
