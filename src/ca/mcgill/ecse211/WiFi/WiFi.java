package ca.mcgill.ecse211.WiFi;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * This class contains the data that will be fetched from the server before
 * the start of the game. It uses the Wifi connection class to fetch data 
 * from the server 
 * 
 * Based off example written by Michael Smith and Tharsan Ponnampalam
 * @author Lara Kollokian
 *
 */
public class WiFi {

	//set at the beginning
	private static final String SERVER_IP = "192.168.2.2";
	private static final int TEAM_NUMBER = 8;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	
	//Team numbers
	private static int redTeam = 0, greenTeam = 0;
	
	//Starting Corners
	private static int redCorner = 0, greenCorner = 0;

	// Red Team area
	private static int Red_LL_x = 0, Red_LL_y = 0, Red_UR_x = 0, Red_UR_y = 0;

	// Green Team area
	private static int Green_LL_x = 0, Green_LL_y = 0, Green_UR_x = 0, Green_UR_y = 0;
	
	//Island coordinates (not needed for beta demo)
	//public static int Island_LL_x = 0, Island_LL_y = 0, Island_UR_x = 0, Island_UR_y = 0;

	// Red Tunnel
	private static int TNR_LL_x = 0, TNR_LL_y = 0, TNR_UR_x = 0, TNR_UR_y = 0;

	// Green Tunnel
	private static int TNG_LL_x = 0, TNG_LL_y = 0, TNG_UR_x = 0, TNG_UR_y = 0;

	
	//OUR COORDS
	public static int corner, localizeX, localizeY, LL_x, LL_y, UR_x, UR_y, TunLL_x, TunLL_y, TunUR_x, TunUR_y;
	
	public static int opp_corner, opp_localizeX, opp_localizeY, opp_LL_x, opp_LL_y, opp_UR_x, opp_UR_y, opp_TunLL_x, opp_TunLL_y, opp_TunUR_x, opp_TunUR_y, opp_Tr_x, opp_Tr_y;
	
	@SuppressWarnings("rawtypes")
	public static void wifi()
	{
		System.out.println("Running..");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA presses the "Start" button
			 * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
			 * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
			 * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
			 * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
			 * but receives corrupted data or a message from the server saying something went wrong. For
			 * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
			 * will receive a message saying an invalid team number was specified and getData() will throw
			 * an exception letting you know.
			 */
			Map data = conn.getData();
//			System.out.println("Map:\n" + data);
			
			//team numbers
			redTeam = ((Long) data.get("RedTeam")).intValue();
			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			
			//corners
			redCorner = ((Long) data.get("RedCorner")).intValue();
			grenCorner = ((Long) data.get("GreenCorner")).intValue();
			
			//red zone
			Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
			Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
			Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
			Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
			
			//green zone
			Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
			Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
			Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
			Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
			
			//island
			//Island_LL_x = ((Long) data.get("Island_LL_x")).intValue();
			//Island_LL_y = ((Long) data.get("Island_LL_y")).intValue();
			//Island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
			//Island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
			
			//red tunnel
			TNR_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
			TNR_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
			TNR_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
			TNR_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
			
			//green tunnel
			TNG_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
			TNG_LL_y = ((Long) data.get("TNG_LL_y")).intValue();
			TNG_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
			TNG_UR_y = ((Long) data.get("TNG_UR_y")).intValue();
			
			
			if(redTeam == TEAM_NUMBER)
			{
				assignTeamColor(0);
			}
			else if(greenTeam == TEAM_NUMBER)
			{
				assignTeamColor(1);
			}
			else
			{
				System.out.println("Team number error");
			}

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		
		System.out.println("Complete!");
	}
	
	/**
	 * sets the parameters to the correct values depending on team color
	 * 
	 * @param color 0 if red team, 1 if green team
	 */
	private static void assignTeamColor(int color)
	{
		if(color == 0) //red
		{
			corner = R;
			LL_x = Red_LL_x;
			LL_y = Red_LL_y;
			UR_x = Red_UR_x;
			UR_y = Red_UR_y;
			TunLL_x = BRR_LL_x;
			TunLL_y = BRR_LL_y;
			TunUR_x = BRR_UR_x;
			TunUR_y = BRR_UR_y;
			Tr_x = TR_x;
			Tr_y = TR_y;
			setStartingCoordinates(R);
			
			opp_corner = G;
			opp_LL_x = Green_LL_x;
			opp_LL_y = Green_LL_y;
			opp_UR_x = Green_UR_x;
			opp_UR_y = Green_UR_y;
			opp_TunLL_x = BRG_LL_x;
			opp_TunLL_y = BRG_LL_y;
			opp_TunUR_x = BRG_UR_x;
			opp_TunUR_y = BRG_UR_y;
			opp_Tr_x = TG_x;
			opp_Tr_y = TG_y;
		}
		else if(color == 1) //green
		{
			corner = G;
			LL_x = Green_LL_x;
			LL_y = Green_LL_y;
			UR_x = Green_UR_x;
			UR_y = Green_UR_y;
			TunLL_x = BRG_LL_x;
			TunLL_y = BRG_LL_y;
			TunUR_x = BRG_UR_x;
			TunUR_y = BRG_UR_y;
			Tr_x = TG_x;
			Tr_y = TG_y;
			setStartingCoordinates(G);
			
			opp_corner = R;
			opp_LL_x = Red_LL_x;
			opp_LL_y = Red_LL_y;
			opp_UR_x = Red_UR_x;
			opp_UR_y = Red_UR_y;
			opp_TunLL_x = BRR_LL_x;
			opp_TunLL_y = BRR_LL_y;
			opp_TunUR_x = BRR_UR_x;
			opp_TunUR_y = BRR_UR_y;
			opp_Tr_x = TR_x;
			opp_Tr_y = TR_y;
		}
		else
		{
			System.out.println("Wrong color input for parameter assignment");
		}
	}
	
	/**
	 * finds the first set of coordinates that the robot will localize to depending on the starting zone and 
	 * the size of the game grid
	 * 
	 * @param zone 0, 1, 2, or 3
	 */
	private static void setStartingCoordinates(int zone)
	{
		if(zone == 0)
		{
			localizeX = 1;
			localizeY = 1;
		}
		else if(zone == 1)
		{
			localizeX = RingChallenge.GAME_GRID_X - 1;
			localizeY = 1;	
		}
		else if(zone == 2)
		{
			localizeX = RingChallenge.GAME_GRID_X - 1;
			localizeY = RingChallenge.GAME_GRID_Y - 1;	
		}
		else if(zone == 3)
		{
			localizeX = 1;
			localizeY = RingChallenge.GAME_GRID_Y - 1;	
		}
		else
		{
			System.out.println("Zone number error");
		}
	}
}
© 2019 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
