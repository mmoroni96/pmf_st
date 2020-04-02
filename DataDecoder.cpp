//ISACC DECRIPTATOR

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
typedef struct{
	uint8_t		Id;
	uint32_t	Time;
	int16_t		AccX;
	int16_t		AccY;
	int16_t		AccZ;
	int16_t		GyrX;
	int16_t		GyrY;
	uint16_t	Temp;
	uint8_t		Hum;
}buffer_typedef;


int main() {
	FILE * pf ;
	FILE * ne ;
	if((pf=fopen("DATA_LOG.BIN", "rb"))==NULL) {
		printf("Errore nell'apertura del file");
		exit(1);
	}
	if((ne=fopen("datalog.csv", "w"))==NULL) {
		printf("Errore nella creazione del file");
		exit(1);
	}
	uint32_t buffer[4];
	buffer_typedef data;
	uint64_t i=0;
	fprintf(ne,"Id,Count,AccX,AccY,AccZ,GyrX,GyrY,Temp,Hum\n");
	while (!feof(pf)) {
		i++;
		uint64_t n = fread(&buffer[0], sizeof(uint32_t),4, pf);
		
		data.Id   = (buffer[0]&0x3F)>>1;
		data.Time = (buffer[0]>>16 & 0xFF) | (buffer[0] & 0xFF00) | (buffer[0] & 0x01)<<16;
		data.AccX = (buffer[0] & 0xFF000000)>>16 | (buffer[1] & 0xFF);
		data.AccY = (buffer[1] & 0xFF00) | (buffer[1] & 0xFF0000)>>16;
		data.AccZ = (buffer[1] & 0xFF000000)>>16 | (buffer[2] & 0xFF);
		data.GyrX = (buffer[2] & 0xFF00) | (buffer[2] & 0xFF0000)>>16;
		data.GyrY = (buffer[2] & 0xFF000000)>>16 | (buffer[3] & 0xFF);
		data.Temp = (buffer[3]>>16) & 0xFF | ((buffer[3]>>8 &0xFF)<<8);
		data.Hum  = (buffer[3]>>24) & 0xFF;
		
		fprintf(ne,"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",data.Id,data.Time,data.AccX,data.AccY,data.AccZ,data.GyrX,data.GyrY,data.Temp,data.Hum);
		
		//fprintf(ne,"%d,%d,%d,%d\n",buffer[0],buffer[1],buffer[2],buffer[3]);	
	}
	           fclose(pf);
	fclose(ne);
}



