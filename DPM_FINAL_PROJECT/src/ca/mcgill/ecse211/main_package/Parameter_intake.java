package ca.mcgill.ecse211.main_package;


//TODO the wifi inputs should be handled in this class
public class Parameter_intake {
	public int RedTeam; 		//Team starting from red zone
	public int GreenTeam;   	//Team starting from green zone
	public int RedCorner;		//Starting corner for red team
	public int GreenCorner;		//Starting corner for green team
	public int OG;				//color of green opponent flag
	public int OR;				//color of red opponent flag
	public int Red_LL_x;		//x coordinate of lower left hand corner of Red Zone
	public int Red_LL_y;		//y coordinate of lower left hand corner of Red Zone
	public int Red_UR_x;		//x coordinate of upper right hand corner of Red Zone
	public int Red_UR_y;		//y coordinate of upper right hand corner of Red Zone
	public int Green_LL_x;		//x coordinate of lower left hand corner of Green Zone
	public int Green_LL_y;		//y coordinate of lower left hand corner of Green Zone
	public int Green_UR_x;		//x coordinate of upper right hand corner of Green Zone
	public int Green_UR_y;		//y coordinate of upper right hand corner of Green Zone
	public int TN_LL_x;			//x coordinate of lower left corner of the tunnel footprint
	public int TN_LL_y;			//y coordinate of lower left corner of the tunnel footprint
	public int TN_UR_x;			//x coordinate of upper right corner of the tunnel footprint
	public int TN_UR_y;			//y coordinate of upper right corner of the tunnel footprint
	public int BR_LL_x;			//x coordinate of lower left corner of the bridge footprint
	public int BR_LL_y;			//y coordinate of lower left corner of the bridge footprint
	public int BR_UR_x;			//x coordinate of upper right corner of the bridge footprint
	public int BR_UR_y;			//y coordinate of upper right corner of the bridge footprint
	public int SR_LL_x;			//x coordinate of lower left corner of the search region in red player zone
	public int SR_LL_y;			//y coordinate of lower left corner of the search region in red player zone
	public int SR_UR_x;			//x coordinate of upper right corner of the search region in red player zone
	public int SR_UR_y;			//y coordinate of upper right corner of the search region in red player zone
	public int SG_LL_x;			//x coordinate of lower left corner of the search region in green player zone
	public int SG_LL_y;			//y coordinate of lower left corner of the search region in green player zone
	public int SG_UR_x;			//x coordinate of upper right corner of the search region in green player zone
	public int SG_UR_y;			//y coordinate of upper right corner of the search region in green player zone
	
	private static Parameter_intake parameters=null;
	/**
	 * Class constructor
	 * @author Alexandre Coulombe
	 */
	public Parameter_intake(){
		
	}
	/**
	 * Method to receive parameters of the play field from the game controller
	 * @author Alexandre Coulombe
	 * @param RedTeam
	 * @param GreenTeam
	 * @param RedCorner
	 * @param GreenCorner
	 * @param OG
	 * @param OR
	 * @param Red_LL_x
	 * @param Red_LL_y
	 * @param Red_UR_x
	 * @param Red_UR_y
	 * @param Green_LL_x
	 * @param Green_LL_y
	 * @param Green_UR_x
	 * @param Green_UR_y
	 * @param TN_LL_x
	 * @param TN_LL_y
	 * @param TN_UR_x
	 * @param TN_UR_y
	 * @param BR_LL_x
	 * @param BR_LL_y
	 * @param BR_UR_x
	 * @param BR_UR_y
	 * @param SR_LL_x
	 * @param SR_LL_y
	 * @param SR_UR_x
	 * @param SR_UR_y
	 * @param SG_LL_x
	 * @param SG_LL_y
	 * @param SG_UR_x
	 * @param SG_UR_y
	 */
	public void giveParameters(int RedTeam, int GreenTeam, int RedCorner, int GreenCorner, int OG, int OR, int Red_LL_x, int Red_LL_y,
							int Red_UR_x, int Red_UR_y, int Green_LL_x, int Green_LL_y, int Green_UR_x, int Green_UR_y, int TN_LL_x,
							int TN_LL_y, int TN_UR_x, int TN_UR_y, int BR_LL_x, int BR_LL_y, int BR_UR_x, int BR_UR_y, int SR_LL_x, 
							int SR_LL_y, int SR_UR_x, int SR_UR_y, int SG_LL_x, int SG_LL_y, int SG_UR_x, int SG_UR_y){
		this.RedTeam=RedTeam;
		this.GreenTeam=GreenTeam;
		this.RedCorner=RedCorner;
		this.GreenCorner=GreenCorner;
		this.OG=OG;
		this.OR=OR;
		this.Red_LL_x=Red_LL_x;
		this.Red_LL_y=Red_LL_y;
		this.Red_UR_x=Red_UR_x;
		this.Red_UR_y=Red_UR_y;
		this.Green_LL_x=Green_LL_x;
		this.Green_LL_y=Green_LL_y;
		this.Green_UR_x=Green_UR_x;
		this.Green_UR_y=Green_UR_y;
		this.TN_LL_x=TN_LL_x;
		this.TN_LL_y=TN_LL_y;
		this.TN_UR_x=TN_UR_x;
		this.TN_UR_y=TN_UR_y;
		this.BR_LL_x=BR_LL_x;
		this.BR_LL_y=BR_LL_y;
		this.BR_UR_x=BR_UR_x;
		this.BR_UR_y=BR_UR_y;
		this.SR_LL_x=SR_LL_x;
		this.SR_LL_y=SR_LL_y;
		this.SR_UR_x=SR_UR_x;
		this.SG_UR_y=SG_UR_y;
		this.SG_LL_x=SG_LL_x;
		this.SG_LL_y=SG_LL_y;
		this.SG_UR_x=SG_UR_x;
		this.SG_UR_y=SG_UR_y;
	}

	public synchronized static Parameter_intake getParameter(){
		if (parameters != null) { // Return existing object
			return parameters;
		} else { // create object and return it
			parameters = new Parameter_intake();
			return parameters;
		}
	}
	/**
	 * Method to return the color of the target block
	 * @author Alexandre Coulombe
	 * @return color of target block
	 */
	public String Target_color(){
		if(GreenTeam==13){
			switch (OG){
				case 1:
					return "red";
				case 2:
					return "blue";
				case 3:
					return "yellow";
				case 4:
					return "white";
				case 5:
					return "";
			}
		}
		else if(RedTeam==13){
			switch (OR){
				case 1:
					return "red";
				case 2:
					return "blue";
				case 3:
					return "yellow";
				case 4:
					return "white";
				case 5:
					return "";
			}
		}
		return "";
	}
	/**
	 * Method to return the x coordinate of the starting corner of the robot
	 * @author Alexandre Coulombe
	 * @return x coordinate of starting corner
	 */
	 public int start_coord_x(){
		 if(GreenTeam==13){
			switch (GreenCorner){
				case 1:
					return 0;
				case 2:
					return 8;
				case 3:
					return 8;
				case 4:
					return 0;
			}
		 }
		else if(RedTeam==13){
			switch (RedCorner){
				case 1:
					return 0;
				case 2:
					return 8;
				case 3:
					return 8;
				case 4:
					return 0;
			}
		}
		return -1;
	 }
	 /**
	  * Method to return the x coordinate of the starting corner of the robot
	  * @author Alexandre Coulombe
	  * @return x coordinate of starting corner
	  */
	 public int start_coord_y(){
		 if(GreenTeam==13){
			switch (GreenCorner){
				case 1:
					return 0;
				case 2:
					return 0;
				case 3:
					return 8;
				case 4:
					return 8;
			}
		 }
		else if(RedTeam==13){
			switch (RedCorner){
				case 1:
					return 0;
				case 2:
					return 0;
				case 3:
					return 8;
				case 4:
					return 8;
			}
		}
		return -1;
	 }
}
