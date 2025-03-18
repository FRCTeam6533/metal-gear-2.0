// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  PowerDistribution PD = new PowerDistribution(0, ModuleType.kCTRE);
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("PDP");
  /** Creates a new PDP. */
  public PDP() {}
    
    DoublePublisher volt = table.getDoubleTopic("voltage").publish();
    DoublePublisher current = table.getDoubleTopic("Total Current").publish();
    DoublePublisher temperature = table.getDoubleTopic("Temp (*C)").publish();
    DoublePublisher cur1 = table.getDoubleTopic("Cur1").publish();
    DoublePublisher cur2 = table.getDoubleTopic("Cur2").publish();
    DoublePublisher cur3 = table.getDoubleTopic("Cur3").publish();
    DoublePublisher cur4 = table.getDoubleTopic("Cur4").publish();
    DoublePublisher cur5 = table.getDoubleTopic("Cur5").publish();
    DoublePublisher cur6 = table.getDoubleTopic("Cur6").publish();
    DoublePublisher cur7 = table.getDoubleTopic("Cur7").publish();
    DoublePublisher cur8 = table.getDoubleTopic("Cur8").publish();
    DoublePublisher cur9 = table.getDoubleTopic("Cur9").publish();
    DoublePublisher cur10 = table.getDoubleTopic("Cur10").publish();
    DoublePublisher cur11 = table.getDoubleTopic("Cur11").publish();
    DoublePublisher cur12 = table.getDoubleTopic("Cur12").publish();
    DoublePublisher cur13 = table.getDoubleTopic("Cur13").publish();
    DoublePublisher cur14 = table.getDoubleTopic("Cur14").publish();
    DoublePublisher cur15 = table.getDoubleTopic("Cur15").publish();
    DoublePublisher cur16 = table.getDoubleTopic("Cur16").publish();
  

    double voltage = PD.getVoltage();
    double totCurrent = PD.getTotalCurrent();
    double temp = PD.getTemperature();
    double current1 = PD.getCurrent(0);
    double current2 = PD.getCurrent(1);
    double current3 = PD.getCurrent(2);
    double current4 = PD.getCurrent(3);
    double current5 = PD.getCurrent(4);
    double current6 = PD.getCurrent(5);
    double current7 = PD.getCurrent(6);
    double current8 = PD.getCurrent(7);
    double current9 = PD.getCurrent(8);
    double current10 = PD.getCurrent(9);
    double current11 = PD.getCurrent(10);
    double current12 = PD.getCurrent(11);
    double current13 = PD.getCurrent(12);
    double current14 = PD.getCurrent(13);
    double current15 = PD.getCurrent(14);
    double current16 = PD.getCurrent(15);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    volt.set(voltage);
    current.set(totCurrent);
    temperature.set(temp);
    cur1.set(current1);
    cur2.set(current2);
    cur3.set(current3);
    cur4.set(current4);
    cur5.set(current5);
    cur6.set(current6);
    cur7.set(current7);
    cur8.set(current8);
    cur9.set(current9);
    cur10.set(current10);
    cur11.set(current11);
    cur12.set(current12);
    cur13.set(current13);
    cur14.set(current14);
    cur15.set(current15);
    cur16.set(current16);

  }
}
