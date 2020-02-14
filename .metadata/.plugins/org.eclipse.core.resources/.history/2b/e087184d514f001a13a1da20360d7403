/*
 *	Sigma2 CAN messages definitions
 *	
 *
 */

#define	MS			0x130U
#define	DS			0x140U
#define	CS			0x150U
#define	DC			0x160U
#define	BCL			0x1F0U
#define	SN			0x610U
#define	RSN			0x620U
#define	Broadcast	0x010U

typedef struct{
	uint16_t		MotorSpeed;
	int16_t			MotorCurrent;
	unsigned char	MotorVoltage;
	unsigned char	BatteryVoltage;
	int16_t			BatteryCurrent;
}MS_typedef;

typedef struct{
	int16_t			ActualTorque;
	int16_t			ActualSpeed;
	unsigned char	DriveStatusIndicator;
	unsigned char	SpeedLimitIndicator;
	unsigned char	TorqueLimitIndicator;
	unsigned char	MotorLimitIndicator;
	unsigned char	FaultCode;
	unsigned char	Code;					//To rearrange
}DS_typedef;

typedef struct{
	unsigned char	ControllerTemperature;
	unsigned char	MotorTemperature;
	unsigned char	BDI;
	uint16_t		FaultSubCode;
}CS_typedef;
