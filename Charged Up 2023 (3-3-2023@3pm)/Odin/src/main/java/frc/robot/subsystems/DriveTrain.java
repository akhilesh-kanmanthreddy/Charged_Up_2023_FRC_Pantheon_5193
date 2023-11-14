package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;


import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;



public class DriveTrain extends SubsystemBase{
    public DriveTrain() {}

    //SparkMaxIDs for each motor
    private int front_leftID = 17;
    private int front_rightID = 5;
    private int back_leftID = 13;
    private int back_rightID = 7;

    //Drive train controllers

    private CANSparkMax fl = new CANSparkMax(front_leftID, MotorType.kBrushless);
    private CANSparkMax bl = new CANSparkMax(back_leftID, MotorType.kBrushless);
    private CANSparkMax fr = new CANSparkMax(front_rightID, MotorType.kBrushless);
    private CANSparkMax br = new CANSparkMax(back_rightID, MotorType.kBrushless);

    private RelativeEncoder flEncoder = fl.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder blEncoder = bl.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder frEncoder = fr.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    private RelativeEncoder brEncoder = br.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);

    //Intake and elevator controllers
   // private  CANSparkMax elevator = new CANSparkMax(elevatorID, MotorType.kBrushless);
    //private CANSparkMax extension = new CANSparkMax(extensionID, MotorType.kBrushless);
   // private PWMTalonFX turret = new PWMTalonFX(turretID);
   // private CANSparkMax clamp = new CANSparkMax(clampID,MotorType.kBrushless)

   public void resetEncoder(){
    flEncoder.setPosition(0);
    blEncoder.setPosition(0);
    frEncoder.setPosition(0);
    brEncoder.setPosition(0);
   }

   public double getflEncoder(){
    return flEncoder.getPosition();
   }
    private AHRS gyro = new AHRS(Port.kMXP);

    //A ratio to decrease power to the wheels for demos and whatnot
    private double detuner = 1;

    //for testing linebreak sensor
    // private int lbid = 0;
    // private DigitalInput linebreak = new DigitalInput(lbid);
    // private ColorSensorV3 proximitySensor = new ColorSensorV3(I2C.Port.kOnboard);
    

    public void setPower(double leftP, double rightP){
        fl.set(-leftP*detuner);
        bl.set(-leftP*detuner);
        br.set(rightP*detuner);
        fr.set(rightP*detuner);
    }

    // public void setelevator(double leftP, double rightP){
    //     elevator.set(rightP*detuner);
    // }


    public void coastMode(){
        fl.setIdleMode(IdleMode.kCoast);
        fr.setIdleMode(IdleMode.kCoast);
        bl.setIdleMode(IdleMode.kCoast);
        br.setIdleMode(IdleMode.kCoast);
    }

    // public boolean getLineBreak(){
    //     return linebreak.get();
    // }

    public double getYRPS(){
        return gyro.getRawGyroY();
    }
    public double getZRPS(){
        return gyro.getRawGyroZ();
    }
    public double getXRPS(){
        return gyro.getRawGyroX();
    }

    public double getPitch(){
        return gyro.getPitch();
    }
    public double getRoll(){
        return gyro.getRoll();
    }
    public double getYaw(){
        return gyro.getYaw();
    }
    public double getAngle(){
        return gyro.getAngle();
    }

    // public double getProximity(){
    //     return proximitySensor.getProximity();
    // }

    // public double getRed(){
    //     return proximitySensor.getRed();
    // }

    // public double getGreen(){
    //     return proximitySensor.getGreen();
    // }

    // public double getBlue(){
    //     return proximitySensor.getBlue();
    // }

    public void resetGyro(){
        gyro.reset();
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
