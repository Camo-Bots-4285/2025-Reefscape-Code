package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionPanel extends SubsystemBase{
    

    public static double current9;

    public PowerDistributionPanel(int i) {
        //TODO Auto-generated constructor stub
//9

        try (PowerDistribution examplePD = new PowerDistribution(50, ModuleType.kRev)) {
            
            double current1 = examplePD.getCurrent(1);
            SmartDashboard.putNumber("Current Channel 1", current1);

            double current2 = examplePD.getCurrent(2);
            SmartDashboard.putNumber("Current Channel 2", current2);

            double current3 = examplePD.getCurrent(3);
            SmartDashboard.putNumber("Current Channel 3", current3);

            double current4 = examplePD.getCurrent(4);
            SmartDashboard.putNumber("Current Channel 4", current4);

            double current5 = examplePD.getCurrent(5);
            SmartDashboard.putNumber("Current Channel 5", current5);

            double current6 = examplePD.getCurrent(6);
            SmartDashboard.putNumber("Current Channel 1", current1);

            double current7 = examplePD.getCurrent(7);
            SmartDashboard.putNumber("Current Channel 2", current2);

            double current8 = examplePD.getCurrent(8);
            SmartDashboard.putNumber("Current Channel 3", current3);

            current9 = examplePD.getCurrent(9);
            SmartDashboard.putNumber("Current Channel 4", current4);

            double current10 = examplePD.getCurrent(5);
            SmartDashboard.putNumber("Current Channel 10", current5);
        }
    }

}
