package frc.robot.drivetrain;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PowerControl {
   
    PowerDistribution robotPDH = new PowerDistribution(1, ModuleType.kRev);


    public void checkTemp () {
        // Retrieves the temperature of the PDP, in degrees Celsius.
        double temperatureCelsius = robotPDH.getTemperature();
        SmartDashboard.putNumber("Temperature", temperatureCelsius);
    }


    public void watchForBrownouts () {
        double voltage = robotPDH.getVoltage();
        SmartDashboard.putNumber("Voltage", voltage);
        double brownoutThreshold = 9.5;
       
        if (voltage < brownoutThreshold) {


            DriverStation.reportWarning("Brownout detected! Voltage is low: " + voltage, false);
        }
    }


    public void readCurrentPowerEnergy () {


        //GET THE TOTAL CURRENT OF ALL CHANNELS
        double totalCurrent = robotPDH.getTotalCurrent();
        SmartDashboard.putNumber("Total Current", totalCurrent);


        //GET THE TOTAL POWER OF ALL CHANNELS
        //POWER IS THE BUS VOLTAGE MULTIPLIED BY THE CURRENT WITH THE UNIT WATTS
        double totalPower = robotPDH.getTotalPower();
        SmartDashboard.putNumber("Total Power", totalPower);


        //GET THE TOTAL ENERGY OF ALL CHANNELS
        //ENERGY IS THE TOTAL POWER SUMMONED OVER TIME IN UNITS JOULES
        double totalEnergy = robotPDH.getTotalEnergy();
        SmartDashboard.putNumber("Total Energy", totalEnergy);
    }


}



