/**OverView
 * This system will controll every that comes after the pivot mostlike wheels and might be a pistion
 */
package frc.robot.subsystems.MechanicalSystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmManipulator extends SubsystemBase {

    private ArmSubsystem armSubsystem;  // Reference to ArmSubsystem

    public ArmManipulator(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;  // Set the reference to ArmSubsystem

        //TODO add motors and PID plus FF for good contorl
    }

    // Method to get wrist position
    public double getWristPosition(){
        return 0.0;  // Placeholder for wrist position
    }

    // Method to get wrist position
    public double getWristVelocity(){
        return 0.0;  // Placeholder for wrist Velocity
    }


            // Method to get wrist position
    public double getWristAccleration(){
        return 0.0;  // Placeholder for wrist Accleration
    }

    // Method to get wrist position
    public void setWristPosition(){
         // ArmSubsystem.DesiredWristPosition; //use ths function for your desired position
        //TODO use PID and FF contoller to make arm move with setVoltage
    }


    @Override
    public void periodic() {
    }
}

//See Wrist for updates