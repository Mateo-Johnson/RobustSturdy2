package frc.robot.drivetrain;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PowerControl {
   
    PowerDistribution robotPDH = new PowerDistribution(1, ModuleType.kRev);


    public void checkTemp () {
        //RETRIEVES TEMP OF PDH IN CELSIUS
        double tempCelsius = robotPDH.getTemperature();

        double tempFahrenheitminus32 = tempCelsius*1.8;
        double tempFahrenheit = tempFahrenheitminus32+32;

        SmartDashboard.putNumber("TEMP (°F)", tempFahrenheit);
    }


    public void watchForBrownouts () {
        double voltage = robotPDH.getVoltage();
        SmartDashboard.putNumber("VOLTAGE", voltage);
        double brownoutThreshold = 9.5;
       
        if (voltage < brownoutThreshold) {
            DriverStation.reportWarning("BROWNOUT! VOLTAGE IS LOW: " + voltage, false);
        }
    }


    public void readCurrentPowerEnergy () {


        //GET THE TOTAL CURRENT OF ALL CHANNELS
        double totalCurrent = robotPDH.getTotalCurrent();
        SmartDashboard.putNumber("TOTAL CURRENT", totalCurrent);


        //GET THE TOTAL POWER OF ALL CHANNELS
        //POWER IS THE BUS VOLTAGE MULTIPLIED BY THE CURRENT WITH THE UNIT WATTS
        double totalPower = robotPDH.getTotalPower();
        SmartDashboard.putNumber("TOTAL POWER", totalPower);


        //GET THE TOTAL ENERGY OF ALL CHANNELS
        //ENERGY IS THE TOTAL POWER SUMMONED OVER TIME IN UNITS JOULES
        double totalEnergy = robotPDH.getTotalEnergy();
        SmartDashboard.putNumber("TOTAL ENERGY", totalEnergy);
    }


}



