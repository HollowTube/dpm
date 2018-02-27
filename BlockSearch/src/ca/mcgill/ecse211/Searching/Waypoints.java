package ca.mcgill.ecse211.Searching;

import ca.mcgill.ecse211.Odometry.Odometer;

public class Waypoints implements Runnable{
	public static Odometer odometer;
	private static Navigation nav;
	private static LightLocalizer liLocalizer;
	private static int[] LL = {1,1};
	private static int[] UR = {3,3};
	public static double xyt[] = new double[3];
	public static double currentX; 			//current values represent robot's X,Y,theta
	public static double currentY;
	public static double currentTheta;
	public static double deltaTheta;
	public static final double TILE_SIZE = 30.48;
	public static int[][] waypoints = {{LL[0], LL[1]}, {UR[0], LL[1]}, {UR[0], UR[1]}, {LL[0], UR[1]}, {LL[0], LL[1]}};
	public static double path_angle;		//angle that is calculated from the normal
	public static int wpCtr = 0;    

	public Waypoints(Navigation nav, Odometer odometer){
		this.nav = nav;
		this.odometer = odometer;
	}

	@Override
	public void run(){
		while(wpCtr < waypoints.length){
			if(Thread.interrupted()){
				try{
					Thread.sleep(5000);
				}catch (Exception e){}
			}
			
			xyt = odometer.getXYT();				//get the robots position on the grid
			currentX = xyt[0];						//place data in its respective variables
			currentY = xyt[1];
			currentTheta = xyt[2];
			/*
			 * The following calculations figure out the desired angle that the bot needs to turn.
			 * It uses arctan2 to get the angle in polar coordinates from (0,0) so for every coordinate
			 * it will calculate the angle for the implicit coordinate in relation to (0,0). For example:
			 * if the bot is at (2,2,45) and it needs to turn to (2,0), the logic below will figure out the angle
			 * needed to turn to (0,-2) and then subtract the current angle of the bot.
			 * */
			if(wpCtr > 0){//need to make sure this is not performed for the first coordinate as we assume the bot starts at 0,0
				if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
					path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));  		//angles in positive xy-plane
				}else if(waypoints[wpCtr][1] < 0){							
					path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));	//angles in the negative y-plane
				}else{
					path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE))); 	//angles in negative x-plane and positive y-plane
				}
			}else{
				if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
					path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);  		//angles in positive xy-plane
				}else if(waypoints[wpCtr][1] < 0){							
					path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);	//angles in the negative y-plane
				}else{
					path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]); 	//angles in negative x-plane and positive y-plane
				}
			}

			deltaTheta = path_angle-currentTheta;
			nav.turnTo(deltaTheta);
			nav.travelTo(waypoints[wpCtr][0]*TILE_SIZE, waypoints[wpCtr][1]*TILE_SIZE);
			wpCtr++;
		}
	}
}
