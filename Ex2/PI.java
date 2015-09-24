package Ex2;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;

import ch.aplu.nxt.*;
import lejos.nxt.Button;
import java.lang.*;

public class PI {

	LightSensor ls;
	NxtRobot robot;
	Gear gear;
	int vWhite = -1;
	int vBlack = -1;
	int lightValue;
	double offset;
	double error;
	int Tp = 40;
	double Kp;
	double KI = 0.1;
	double Turn;
	Motor motA;
	Motor motB;
	int SpeedA;
	int SpeedB;
	double integral = 0;
	
	


	public PI() {
		robot = new NxtRobot();

		motA = new Motor(MotorPort.A);
		motB = new Motor(MotorPort.B);
		robot.addPart(motA);
		robot.addPart(motB);

		ls = new LightSensor(SensorPort.S1);
		robot.addPart(ls);
		ls.activate(true);
	}

	public static void main(String[] args) {
		PI LS = new PI();
		LS.calibrate();
		LS.goLine();

	}

	public void goLine() {
		motA.setSpeed(Tp);
		motB.setSpeed(Tp);
		motA.forward();
		motB.forward();
		long time = System.currentTimeMillis();
		long delta = 0;
		long dT = 0;
		long currentTime = System.currentTimeMillis();

		while (true) {
			dT = System.currentTimeMillis() - currentTime;
			currentTime = System.currentTimeMillis();
			lightValue = ls.getValue();
			error = lightValue - offset;
			integral = 2 / 3 * integral + error;
			Turn = Kp * error + KI * integral;
			
	
		/*	if (error > 0){
				delta += System.currentTimeMillis()-time;
				time = System.currentTimeMillis();				
			}
			else {
				delta = 0;
			}
			
			System.out.println (delta);
			
			if (delta > 3000) {
				motA.stop();
				motB.stop();
				break;
			}*/
			SpeedA = (int) (Tp + Turn);
			

			if (SpeedA > 0) {
				System.out.println("Sp A" + SpeedA);
				motA.setSpeed(SpeedA);
				motA.forward();
			} else {
				motA.setSpeed(Math.abs(SpeedA));
				motA.backward();
			}

			SpeedB = (int) (Tp - Turn);
			
			if (SpeedB > 0) {
				System.out.println("Sp B" + SpeedB);
				motB.setSpeed(SpeedB);
				motB.forward();
			} else {
				motB.setSpeed(Math.abs(SpeedB));
				motB.backward();
			}

		}
	}

	public void calibrate() {
		Button.LEFT.waitForPress();
		vWhite = ls.getValue();
		System.out.println(vWhite);
		
		Button.RIGHT.waitForPress();
		vBlack = ls.getValue();
		System.out.println(vBlack);
		
		offset = (vBlack + vWhite) / 2;
		Kp = Tp / (offset - vBlack)/4;
		System.out.println(Kp);
		Button.ENTER.waitForPress();

	}

}
