#include "Controllo.h"
#define rk 		0.1238 		// raggio rotolamento, ruota tacchettata m
#define J 		0.0235 		// inerzia lato kart riferita ad albero kart kgm^2
#define c 		0.013 		// smorzamento lato kart Nm/rad *s
#define dt 		0.01 		// sampling time
#define tau 	0.5 		// rapporto di trasmissione riduttore
#define dummy  1			// serve a far funzionare il controllo in trazione e franta e far tornare le convenzioni if -1 = frenata, if 1= acc

void Controllo(struct ControllerStruct *ControlStatus, struct ABSControlStruct *ABS_control, struct ControllerStruct *ControlRef ){


	// da Status e ABS_control prendo tutte le info sullo stato attuale del sistema e del controllo


    int    phase         = ABS_control 		-> phase;
    int	   k			 = ABS_control      -> k;
    int	   ks			 = ABS_control      -> ks;
    double controlTorque = ABS_control 		-> controlTorque; //torque alla ruota. Finchè non la riniz. è la torque al passo prec
    double speed         = ControlStatus 	-> Speed_filt;    //m/s
    double acc		     = ControlStatus 	-> Acc_filt;      //m/s^2
    double wheelTorque 	 = (ControlRef		-> Torque)/(dummy *tau);//torque attuale alla ruota (segno + se in traction control !!!)
    double DFDT 		 = ABS_control   	-> DFDT;
    double FxD 		 	 = ABS_control   	-> FxD;
    double Fmax			 = ABS_control      -> Fmax;
    double Tmax			 = ABS_control		-> Tmax;
    double errFx		 = ABS_control		-> errFx;

    //==================================================================

    k++;


	double w1 = ((speed)/rk);


    double w1dot = ((acc)/rk);

    double Troll = (double) (2.65e-7 * (w1* w1* w1) -7e-5 * (w1 *w1) +0.0012235*w1 + 0.5658);
    double Tres  = 0.655;

    if(w1==0){
    	Troll = 0;
    	Tres  = 0;
    };

    //==================================================================

    ABS_control ->Fx0 = ABS_control ->Fx;

    double Fx;

	Fx = (wheelTorque  +dummy*( -J * w1dot - c * w1 - Tres - Troll) ) /(rk); //DA USARE PER ABS


    FxD  = ( Fx - (ABS_control ->Fx0) ) / dt;
    DFDT = ( Fx - (ABS_control ->Fx0) ) /(wheelTorque - controlTorque);

    if (Fmax > 0) {
    errFx= (Fmax - Fx)/Fmax;

    }else{
    	errFx = 0;
    }

    //===================================================================

    if( (phase==1) & (ABS_control ->DFDT_filt < 4) & ((k - ks) >20) ){  // limite = 0 per segnale sporco
    	phase = 2;
    	ks = k;
    }else if((phase==2) & (ABS_control ->FxD_filt < 50) & ((k-ks) > 3)){
    	phase = 3;
    	Fmax  = Fx;
    	Tmax  = controlTorque;
    	ks    = k;
    }else if((phase==3) & ( errFx > 0.15) & ((k-ks)> 5) ){
    	phase = 4;
    	ks    = k;
    }else if((phase==4) & ( errFx < 0.02) & ((k-ks)> 10) ){
        phase = 2;
        ks    = k;
    };

    //==================================================================
    double exp(double);


    switch (phase){
    case 1:
    	controlTorque = wheelTorque;
    	break;
    case 2:
    	controlTorque = controlTorque + 0.05;
    	break;
    case 3:
    	controlTorque = controlTorque + (controlTorque - 0.7*Tmax)*(exp(-7*dt)-1);
    	break;
    case 4:
    	controlTorque = controlTorque + (controlTorque - 1.1*Tmax)*(exp(-8*dt)-1);

    	break;
    default:
    	break;
    }

    if(controlTorque<0){
       controlTorque=0;
    }

    //==================================================================

    //faccio uscire tutte le variabili da scrivere in ABS_control, per avere memoria degli step prec

    ABS_control->phase         = phase;
    ABS_control->controlTorque = controlTorque;
    ABS_control->Fx 		   = Fx;
    ABS_control->errFx 		   = errFx;
    ABS_control->FxD		   = FxD;
    ABS_control->DFDT	       = DFDT;
    ControlRef ->Torque 	   = controlTorque*(dummy *tau);  //va al motore (segno + se in traction control !!!)
    ABS_control->k  		   = k;
    ABS_control->ks  		   = ks;
    ABS_control->Fmax		   = Fmax;
    ABS_control->Tmax		   = Tmax;
}
