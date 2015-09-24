package Ex2;

import lejos.nxt.*;
import lejos.nxt.addon.CompassHTSensor;
import lejos.util.Delay;

public class Square {

	double length = 0.5;
	int Nturn = 0;
	double startAngle = 0;
	CompassHTSensor compass = new CompassHTSensor(SensorPort.S1);
    int degreeToTurn=361;
	int v = 360;
	double Kp = 0.4;
	double Ip = 0.01;

	
	
	public void line(double dist) {
		Motor.C.setSpeed(v);
		Motor.B.setSpeed(v);

		Motor.C.resetTachoCount();
		Motor.B.resetTachoCount();

		int tahB = 0;
		int tahC = 0;
		int delta = tahB - tahC;
		double integral = 0;
		double Rev = dist / (3.1415 * 0.057);

		Motor.B.forward();
		Motor.C.forward();

		while (tahC <= Rev*360) {

			tahC = Motor.C.getTachoCount();
			tahB = Motor.B.getTachoCount();
			delta = tahB - tahC;
			if (Math.abs(delta) > 20) {
				delta = 20;
			}

			integral = integral + delta;

			Motor.C.setSpeed((int) (v + Kp * delta + Ip * integral));
			Motor.B.setSpeed((int) (v - Kp * delta - Ip * integral));

			LCD.drawString(Integer.toString(delta), 0, 3);
		}
		Motor.C.setSpeed(v);
		Motor.B.setSpeed(v);

		Motor.B.stop(true);
		Motor.C.stop(true);

		Motor.C.resetTachoCount();
		Motor.B.resetTachoCount();

	}

	public void turn() {
		
		Motor.C.setSpeed(v);
		Motor.B.setSpeed(v);
		
		//Delay.msDelay(1000);
		LCD.clear();
		double sAngle = startAngle + 90 * Nturn;
		double fAngle = sAngle + 90;
		LCD.drawString("Start and fin angle", 0, 1);
		LCD.drawString(Double.toString(sAngle), 0, 2);
		LCD.drawString(Double.toString(fAngle), 0, 3);
		
		double compassMean = 0;

		for (int j = 0; j < 5; j++) {
			compassMean = compassMean + compass.getDegrees();
		}
		compassMean = compassMean / 5;
		LCD.drawString(Double.toString(compassMean), 0, 4);
		//Button.waitForAnyPress();
		if (Math.abs(compassMean-fAngle)<200){
		 adjust(compassMean,sAngle);}

		

		Motor.C.resetTachoCount();
		Motor.B.resetTachoCount();

		int tahB = 0;
		int tahC = 0;
		int delta = tahB - tahC;
		double integral = 0;

		Motor.B.rotate(degreeToTurn+30);
		//Delay.msDelay(1000);

		compassMean = 0;

		for (int j = 0; j < 5; j++) {
			compassMean = compassMean + compass.getDegrees();
		}
		compassMean = compassMean / 5;
		LCD.drawString(Double.toString(compassMean), 0, 4);
		//Button.waitForAnyPress();
		if (Math.abs(compassMean-fAngle)<200){
		 adjust(compassMean,fAngle);}

		Nturn++;
		if(Nturn%4==0)Nturn=0;

	}

	
	
	public void adjust(double compassAngle, double desiredAngle)
	{
		Motor.C.rotate((int)(4*(desiredAngle-compassAngle)));
		
		
	}
	
	public Square() {
		Motor.C.setSpeed(v+100);
		Motor.B.setSpeed(v+100);
		
		System.out.println("Go rectangle");
		Button.waitForAnyPress();
		Delay.msDelay(1000);
		LCD.clear();

	/*	System.out.println("Calibrating the compass");
		Delay.msDelay(1000);
		compass.startCalibration();
		Motor.B.rotate((int) (degreeToTurn * 16));
		compass.stopCalibration();*/
		
		Motor.C.setSpeed(v);
		Motor.B.setSpeed(v);
		
		Motor.B.stop(true);
		
		System.out.println("Calibraton is done. Press any key");
		LCD.clear();
		System.out.println("Set the robot to start position");
		Button.waitForAnyPress();
		startAngle = compass.getDegrees();
		
		LCD.drawString(Double.toString(startAngle), 0, 1);
		Button.waitForAnyPress();
		Delay.msDelay(1000);
		
		LCD.clear();
		
		line(1);
		turn();
     	line(0.5);
		turn();
		line(1);
		turn();
		line(0.5);
		turn();
		line(1);
		turn();
     	line(0.5);
		turn();
		line(1);
		turn();
		line(0.5);
		turn();
		line(1);
		turn();
     	line(0.5);
		turn();
		line(1);
		turn();
		line(0.5);
		turn();

	}

	public static void main(String[] args) {

		new Square();
		// TODO Auto-generated method stub

	}
}