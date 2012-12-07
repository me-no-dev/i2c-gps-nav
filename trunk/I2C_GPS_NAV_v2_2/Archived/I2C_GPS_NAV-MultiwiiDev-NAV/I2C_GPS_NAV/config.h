//////////////////////////////////////////////////////////////////////////////
// i2C comm definitions
//
#define I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

// Default PID variables
//////////////////////////////////////////////////////////////////////////////
// POSHOLD control gains
//
#define POSHOLD_P		.11
#define POSHOLD_I		0.0
#define POSHOLD_IMAX		20		        // degrees

#define POSHOLD_RATE_P		1.4			//
#define POSHOLD_RATE_I		0.2			// Wind control
#define POSHOLD_RATE_D		0.010			// try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX	20			// degrees
//////////////////////////////////////////////////////////////////////////////
// Navigation PID gains
//
#define NAV_P			1.4	        	//
#define NAV_I			0.20		        // Wind control
#define NAV_D			0.006		        //
#define NAV_IMAX		20		        // degrees

////////////////////////////////////////////////////////////////////////////////////
// Navigational parameters and limiters initial values
//
#define CROSSTRACK_GAIN            1            // Weighting the cross track error
#define NAV_SPEED_MIN              100          // cm/sec minimum navigational speed when NAV_SLOW_NAV id false
#define NAV_SPEED_MAX              300          // cm/sec maximum navigational speed
#define NAV_BANK_MAX               2500         // 20deg max banking when navigating (just for security and testing)

////////////////////////////////////////////////////////////////////////////////////
// GPS data filtering - moving average filter vector length
//
#define GPS_FILTER_VECTOR_LENGTH 5

