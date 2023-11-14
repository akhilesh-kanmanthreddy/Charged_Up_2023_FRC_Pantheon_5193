package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickTurret extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Turret m_turret;
    private XboxController controller1;
    private XboxController controller2;
    private DoubleSupplier leftY;
    private DoubleSupplier rightY;
    private DoubleSupplier rightX;   


public JoystickTurret(Turret driveTrain, XboxController gp1, XboxController gp2){
    m_turret = driveTrain;
    controller1 = gp1;
    controller2 = gp2;
    leftY = () -> controller2.getLeftY();
    rightX = () -> controller2.getRightX();
    rightY = () -> controller2.getRightY();
    addRequirements(driveTrain);
}

@Override
public void initialize(){
    m_turret.setCoastAndBrake();
    m_turret.resetElbowEncoder();
    m_turret.resetTurretEncoder();
    m_turret.resetExtenderEncoder();
    m_turret.resetElevatorEncoder();
}

@Override
public void execute() {
    if(controller2.getLeftTriggerAxis()>.5){
        m_turret.setPowerClamper(.2);
    } else if(controller2.getRightTriggerAxis()>.5){
        m_turret.setPowerClamper(-.2);
    }else if(controller2.getBButton() && (Math.abs(m_turret.getClampPosition())>0.5)){
        m_turret.setPowerClamper(-(m_turret.getClampPosition()/Math.abs(m_turret.getClampPosition()))*.1);
    } else if(controller2.getAButton() && (Math.abs(m_turret.getClampPosition()-(-6))>0.5)){
        m_turret.setPowerClamper(-((m_turret.getClampPosition()-(-6))/Math.abs(m_turret.getClampPosition()-(-6)))*.1);
    } else if(controller2.getXButton() && (Math.abs(m_turret.getClampPosition()-(-2.880951))>0.5)){
        m_turret.setPowerClamper(-((m_turret.getClampPosition()-(-2.880951))/Math.abs(m_turret.getClampPosition()-(-2.880951)))*.1);
    } else {
        m_turret.setPowerClamper(0);
    }

    if(leftY.getAsDouble() < -0.75 && (m_turret.getExtenderPosition()>-10)){
        m_turret.setPowerExtender(leftY.getAsDouble()*.2);
    } else if(leftY.getAsDouble() > 0.75 && (m_turret.getExtenderPosition()<-2)){
        m_turret.setPowerExtender(leftY.getAsDouble()*.2);
    } else {
        m_turret.setPowerExtender(0);
    }
    if((rightX.getAsDouble() > 0.75) && (m_turret.getTurretPosition()<89)){
        m_turret.setPowerTurret(rightX.getAsDouble()*.2);
    } else if((rightX.getAsDouble() < -0.75) && (m_turret.getTurretPosition()>-64)){
        m_turret.setPowerTurret(rightX.getAsDouble()*.2);
    } else {
        m_turret.setPowerTurret(0);
    }
    if(Math.abs(rightY.getAsDouble()) > 0.75 && m_turret.getElbowPosition()>(-70)){
        m_turret.setPowerElbow(rightY.getAsDouble()*.2);
    } else if(controller2.getYButton() && (Math.abs(m_turret.getElbowPosition())>0.75)){
        m_turret.setPowerElbow(-(m_turret.getElbowPosition()/Math.abs(m_turret.getElbowPosition()))*.1);
    } else {
        m_turret.setPowerElbow(0);
    }

    if(controller2.getRightBumper() && (m_turret.getElevatorPosition()<75766.000000)){
        m_turret.setPowerElevator(0.2);
    } else if(controller2.getLeftBumper() && (m_turret.getElevatorPosition()>-15599.000000)){
        m_turret.setPowerElevator(-0.2);
    } else {
        m_turret.setPowerElevator(0);
    }

    if(!m_turret.getExtenderTouch()){
        m_turret.resetExtenderEncoder();
    }

    SmartDashboard.putNumber("Clamper Encoder: ", m_turret.getClampPosition());
    SmartDashboard.putNumber("Elbow Encoder: ", m_turret.getElbowPosition());
    SmartDashboard.putNumber("Turret Encoder: ", m_turret.getTurretPosition());
    SmartDashboard.putBoolean("Extender Touch: ", m_turret.getExtenderTouch());
    SmartDashboard.putNumber("Extender Encoder: ", m_turret.getExtenderPosition());
    SmartDashboard.putNumber("Elevator Encoder: ", m_turret.getElevatorPosition());
    // SmartDashboard.putNumber("Elevator Encoder: ", m_turret.getElevatorPosition());


}
@Override
public void end(boolean interrupted) {
    m_turret.setCoastAndBrake();
}

@Override
public boolean isFinished() {
    return false;
}

}
