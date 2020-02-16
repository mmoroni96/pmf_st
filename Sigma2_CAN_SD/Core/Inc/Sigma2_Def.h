/*
 *	Sigma2 CAN messages definitions
 *	
 *
 */

#define	MS			0x131U
#define	DS			0x141U
#define	CS			0x151U
#define	DC			0x161U
#define	BCL			0x1F0U
#define	SN			0x610U
#define	RSN			0x620U
#define	BROADCAST	0x010U

//Digital inputs position
#define FD	0x0U
#define RD	0x1U
#define FS	0x2U
#define EN	0x3U
#define SL1	0x4U
#define SL2	0x5U
#define HB	0x6U
#define NU	0x7U

typedef struct{
	uint16_t		MotorSpeed;
	int16_t			MotorCurrent;
	uint8_t			MotorVoltage;
	uint8_t			BatteryVoltage;
	int16_t			BatteryCurrent;
}MS_typedef;

typedef struct{
	int16_t			ActualTorque;
	int16_t			ActualSpeed;
	uint8_t			DriveStatusIndicator;
	uint8_t			SpeedLimitIndicator;
	uint8_t			TorqueLimitIndicator;
	uint8_t			MotorLimitIndicator;
	uint8_t			FaultCode;
	uint8_t			Code;					//To rearrange
}DS_typedef;

typedef struct{
	uint8_t			ControllerTemperature;
	uint8_t			MotorTemperature;
	uint8_t			BDI;
	uint16_t		FaultSubCode;
}CS_typedef;

typedef struct{
	uint16_t		Trottle;
	uint16_t		BrakePedal;
	int16_t			AD3;
	uint8_t			DigitalInputs;
	uint8_t			ToggleSecurityBit;
}DC_typedef;

typedef struct{
	uint16_t		DriveBatteryLimit;
	uint16_t		RegenBatteryLimit;
	uint16_t		BDIvaiCAN;
	uint8_t			NotUsed;
	uint8_t			SecurityBit;
}BCL_typedef;
