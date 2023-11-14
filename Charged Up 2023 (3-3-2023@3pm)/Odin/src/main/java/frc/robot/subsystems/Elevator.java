// package frc.robot.subsystems;

// import java.lang.reflect.Array;
// import java.util.List;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.I2C;



// public class Elevator extends SubsystemBase{
//     public Elevator() {}

//     //SparkMaxIDs for each motor
//     //fl = frontleft, br = backright.
//     private int ElevatorID = //elevator motor control ID;


//     private CANSparkMax Elevator = new CANSparkMax(ElevatorID, MotorType.kBrushless);
//     private AHRS gyro = new AHRS(Port.kMXP);

//     //A ratio to decrease power to the wheels for demos and whatnot
//     private double detuner = 1;

//     //for testing linebreak sensor
//     private int lbid = 0;
//     public DigitalInput linebreak = new DigitalInput(lbid);
//     public ColorSensorV3 proximitySensor = new ColorSensorV3(I2C.Port.kOnboard);
    

//     public void setPower(double leftP, double rightP){
//         Elevator.set(-leftP*detuner);
//         Elevator.set(rightP*detuner);
//     }
// }