package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  private final DriveTrain m_driveTrain;
  private XboxController controller1;
  private XboxController controller2;
  private DoubleSupplier leftP;
  private DoubleSupplier rightP;
  private boolean dst = false; // drive straight toggle
  private double averagePower;
  private double targetAngle;
  private double angleOffset;

  public JoystickDrive(DriveTrain subsystem, XboxController gp1, XboxController gp2) {
    m_driveTrain = subsystem;
    controller1 = gp1;
    controller2 = gp2;
    rightP = () -> controller1.getRightY();
    leftP = () -> controller1.getLeftY();
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
    m_driveTrain.coastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    averagePower = (Math.pow(leftP.getAsDouble(), 3) + Math.pow(rightP.getAsDouble(), 3)) / 2;
    if (((Math.abs(leftP.getAsDouble()) > 0.75) && (Math.abs(rightP.getAsDouble()) > 0.75)) /*
                                                                                           * Makes sure sticks are being
                                                                                           * pushed more than 1/2 the
                                                                                           * way
                                                                                           */
        && !dst /* Makes sure drive straight toggle isn't already true */
        && (leftP.getAsDouble() * rightP.getAsDouble() > 0) /*
                                                             * Makes sure both sticks are pointed in the same direction
                                                             */) {
      targetAngle = m_driveTrain.getAngle();
      dst = true;
    } else if ((Math.abs(leftP.getAsDouble()) < .75)
        || (Math.abs(rightP.getAsDouble()) < .75) /* If sticks are let go */) {
      dst = false;
    }

    if ((dst) && (leftP.getAsDouble() > 0)) { /* If drive straight is on and left stick is being pushed forward */
      m_driveTrain.setPower(averagePower - ((targetAngle - m_driveTrain.getAngle()) / 90),
          averagePower - ((targetAngle - m_driveTrain.getAngle()) / 90));

    } else if ((dst) && (leftP.getAsDouble() < 0)) { /* If drive straight is on and left stick is being pulled back */
      m_driveTrain.setPower(averagePower + ((targetAngle - m_driveTrain.getAngle()) / 90),
          (averagePower + ((targetAngle - m_driveTrain.getAngle()) / 90)));
    } else if (((Math.abs(leftP.getAsDouble()) > .2) || (Math.abs((rightP.getAsDouble())) > .2))
        && !dst) { // Cubed control if none of the above conditions apply but the sticks are
                   // pressed above a threshold
      m_driveTrain.setPower(Math.pow((leftP.getAsDouble()), 3), Math.pow((rightP.getAsDouble()), 3));
    } else if (controller1.getYButton()) {
        angleOffset = (m_driveTrain.getYaw());
        if(Math.abs(angleOffset) > 2){
          m_driveTrain.setPower(angleOffset/120, angleOffset/120);
        }//else if (Math.abs(angleOffset) > .3) {
        //   m_driveTrain.setPower(-angleOffset/90, -angleOffset/90);
        // }else if (Math.abs(angleOffset) > .05) {
        //   m_driveTrain.setPower(-angleOffset/60, -angleOffset/60);
        // }
    } else {
      m_driveTrain.setPower(0, 0);
    }

    // if (controller1.getBButton()) {
    //   m_driveTrain.setelevator(leftP:0, rightP:0);
    // }

    // SmartDashboard.putBoolean("LineBreak", m_driveTrain.getLineBreak());

    // SmartDashboard.putNumber("Proximity", m_driveTrain.getProximity());
    // SmartDashboard.putNumber("Red: ", m_driveTrain.getRed());    
    // SmartDashboard.putNumber("Green", m_driveTrain.getGreen());
    // SmartDashboard.putNumber("Blue", m_driveTrain.getBlue());
  

    SmartDashboard.putNumber("Z Rotation: ", m_driveTrain.getZRPS());
    SmartDashboard.putNumber("Y Rotation: ", m_driveTrain.getYRPS());
    SmartDashboard.putNumber("X Rotation: ", m_driveTrain.getXRPS());

    SmartDashboard.putNumber("Pitch: ", m_driveTrain.getPitch());
    SmartDashboard.putNumber("Yaw: ", m_driveTrain.getYaw());
    SmartDashboard.putNumber("Roll: ", m_driveTrain.getRoll());

    // I just used this for trouble shooting purposes
    System.out.println(m_driveTrain.getPitch() + " " + m_driveTrain.getYaw() + " " + m_driveTrain.getRoll());

    SmartDashboard.putNumber("Angle: ", m_driveTrain.getAngle());

    SmartDashboard.putNumber("Target Angle: ", targetAngle);
    SmartDashboard.putBoolean("Drive Straight?", dst);
    SmartDashboard.putNumber("Left Power: ", leftP.getAsDouble());
    SmartDashboard.putNumber("Right Power", rightP.getAsDouble());
    SmartDashboard.putNumber("Average Power", averagePower);
    SmartDashboard.putNumber("FL Encoder: ", m_driveTrain.getflEncoder());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.coastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
