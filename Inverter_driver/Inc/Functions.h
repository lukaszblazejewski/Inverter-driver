#ifndef Functions_h
#define Functions_h

void SkalowanieF(int TIMER[], int N, int WARTOSC_EMPIRYCZNA);
void CzasRampy(int FZAD, int *FZAD_OLD, float TIMEUP, float TIMEDOWN, int *T1, float ZMIANAF);
void RampaLiniowaF(int FZAD, int FPOM, float *F, float ZMIANAF);
void RampaLiniowaMA(float BOOST, float RAMPALINIOWA[]);
void RampaGladkaMA(float BOOST, float RAMPAGLADKA[]);
void TworzenieSIN(int USIN[], int N);
void ZmianaKierunku(int *KIERUNEK, int KIERUNEKZAD, float *FZAD, float F);
void PWM(int USIN[], int *i, int N, int KIERUNEK);
void SterowanieUdoF(int KSZTALT, float F, int USIN[], int USINPWM[], int N, float boost, int FPOM, float RAMPL[], float RAMPG[]);
void ZmianaMA(int KSZTALT, float *FZAD, float MA, float BOOST);
float Pomiary_TEMP_DCV(unsigned short int Pomiary[], float *DCV, int *L);
float Pomiar_DCI(float TAB_DCI[], unsigned short int Pomiary[]);
float Rampa_Gladka(float ksztalt, float boost, float f, float f_old, int USIN[], int USINPWM[], double Rampa[], double Rampa_U_do_F[], int n);
void Tworzenie_Rampy_U_do_F(double Rampa[], double Rampa_U_do_F[], float boost, float *boost_old, int n);
void Tworzenie_Rampy(double Rampa[], float *boost_old, int n);
float RampaGladkaF(int *x, int *m, float ksztalt_ros, float ksztalt_op, float f, float fzad, int *fzad_old,
				float czas_ros, float czas_op, double Rampa[], int n);

#endif
