#include "Functions.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>

void SkalowanieF(int TIMER[], int N, int WARTOSC_EMPIRYCZNA)
{

	int i = 0;
	float x = 0;

	for(i = 1; i < N + 1; i++)
	{
		x = 1.0 / i;
		x = x * 500;
		x = x * WARTOSC_EMPIRYCZNA;
		TIMER[i] = (int)x;
	}
}

void CzasRampy(int FZAD, int *FZAD_OLD, float TIMEUP, float TIMEDOWN, int *T1, float ZMIANAF)
{

	int y = 0;
	float x = 0;

	y = FZAD - *FZAD_OLD;
	y = fabs(y);
	x = 1.0 / y;
	if(FZAD >= *FZAD_OLD)
		x = x * TIMEUP * 1000 * ZMIANAF * 10;
	if(FZAD <= *FZAD_OLD)
		x = x * TIMEDOWN * 1000 * ZMIANAF * 10;

	*T1 = (int)x;
	*FZAD_OLD = FZAD;
}

void RampaLiniowaF(int FZAD, int FPOM, float *F, float ZMIANAF)
{

	float x = 0;
	static float y = 0;

	if(fabs(FZAD / 10.0 - *F) <= ZMIANAF)
		y = fabs(FZAD / 10.0 - *F);
	else
		y = ZMIANAF;

	if(FZAD > FPOM)	*F = *F + y;
	if(FZAD < FPOM)	*F = *F - y;

	x = *F * 10;
	x = round(x);
	*F = x / 10;
}

void RampaGladkaMA(float BOOST, float RAMPAGLADKA[])
{

	float add = 1, div = -1, k = 50.0 * 85.0, x = 4.437 * BOOST * BOOST * BOOST - 3.728 * BOOST * BOOST + 1.424 * BOOST + 0.099;
	int i = 0;

	for(i = 0; i < 501; i++)
	{
		if(i < BOOST * 500.0)
			RAMPAGLADKA[i] = BOOST;
		else if(i >= 500)
			RAMPAGLADKA[i] = 1.0;
		else
		{
			add = RAMPAGLADKA[i - 1] + 1;
			div = RAMPAGLADKA[i - 1] - 1;
			RAMPAGLADKA[i] = 0.5 * (add + div * cos(2 * M_PI * (i - (x * 500.0))/(k)));
		}
	}
}

void RampaLiniowaMA(float BOOST, float RAMPALINIOWA[])
{

	float x = 0.0004 * BOOST * 500.0 + BOOST;
	int i = 0;

	for(i = 0; i < 501 ; i++)
	{
		if(i < BOOST * 500.0)
			RAMPALINIOWA[i] = 0.0004 * i + BOOST;
		else
		{
			RAMPALINIOWA[i] = x + (1 - x) / (500.0 - (BOOST * 500.0)) * (i - BOOST * 500.0);
		}
	}
}

void TworzenieSIN(int USIN[], int N)
{

	int i = 0;
	float x = 0, nn = N;

		for(i = 0;i<N;i++)
		{
			x = sinf(2 * M_PI * i/nn);
			x = 100 * x;
			x = round(x);
			USIN[i] = (int)x;
		}
}

void ZmianaKierunku(int *KIERUNEK, int KIERUNEKZAD, float *FZAD, float F)
{

	static int x = 0;
	int i = 0;

	if(*KIERUNEK != KIERUNEKZAD)
	{
		if(*FZAD != 0) x = *FZAD;
		*FZAD = 0;
		if(F == 0)
		{
			for(i = 0; i < 12000000; i++);
			*KIERUNEK = KIERUNEKZAD;
			*FZAD = x;
		}
	}
}

void PWM(int USIN[], int *i, int N, int KIERUNEK)
{

	int n2 = *i + N/3, n3 = *i + 2 * N/3;

	n2 = n2%120;
	n3 = n3%120;

	if(KIERUNEK == 0)
	{
		TIM1->CCR1 = USIN[*i];
		TIM1->CCR2 = USIN[n2];
		TIM1->CCR3 = USIN[n3];
	*i = (*i + 1) % 120;
	}

	if(KIERUNEK == 1)
	{
		TIM1->CCR1 = USIN[*i];
		TIM1->CCR2 = USIN[n3];
		TIM1->CCR3 = USIN[n2];
	*i = (*i + 1) % 120;
	}
}

void SterowanieUdoF(int KSZTALT, float F, int USIN[], int USINPWM[], int N, float BOOST, int FPOM, float RAMPL[], float RAMPG[])
{
	int i = 0;
	float x = 0;

	if(KSZTALT==0)
	{
		if(F<BOOST * 50)
		{
			for(i = 0;i<N;i++)
			{
				x = BOOST * USIN[i] + 100;
				USINPWM[i] = (int)x;
			}
		}
		if(F >= BOOST * 50)
		{
			for(i = 0; i < N; i++)
			{
				x = RAMPL[FPOM] * USIN[i] + 100;
				USINPWM[i] = (int)x;
			}
		}
	}

	if(KSZTALT == 1)
	{
		if(F<BOOST * 50)
		{
			for(i = 0; i < N; i++)
			{
				x = BOOST * USIN[i] + 100;
				USINPWM[i] = (int)x;
			}
		}
		if(F >= BOOST * 50)
		{
			for(i = 0;i<N;i++)
			{
				x = RAMPG[FPOM] * USIN[i] + 100;
				USINPWM[i] = (int)x;
			}
		}
	}
}

void ZmianaMA(int KSZTALT, float *PARAMETRF, float MA, float BOOST){

	if(KSZTALT == 0)
	{
		if(MA > BOOST)	*PARAMETRF = MA * 50;
		else *PARAMETRF = BOOST * 50;
	}

	if(KSZTALT == 1)
	{
		if(MA > BOOST)	*PARAMETRF = MA * 50;
		else *PARAMETRF = BOOST * 50;
	}
}

float Pomiary_TEMP_DCV(unsigned short int Pomiary[],		// Tablica z pomiarami zaczerpnietymi bezposrednio z ADC
		float *DCV,
		int *L)												// Zmienna pomocnicza powodujaca rzadsze realizowanie funkcji ze wzgledu
{															// na niewielka predkosc zmian temperatury i DCV
	float Temp = 0;									// Zmienna przechowujaca przeliczona wartosc temperatury
	if((*L) >= 50)									// Ograniczenie czestotliwosci wykonywania przeliczen
	{															// Przeliczenia z wartosci otzymanych z ADC na rzeczywiste
		Temp = (3344 - Pomiary[1]) * 0.0407000407;				// (3344 - Pomiary[1])/24.57
		(*DCV) = Pomiary[2] * 0.1651683611196;				// Pomiary[2]/2771.0*450.0
		L = 0;
	}
	(*L)++;
	return Temp;										// Zwrocenie wartosci temperatury na zewnatrz funkcji
}

float Pomiar_DCI(float TAB_DCI[],		// Tablica zawierajaca probki pomiarow DCI
		unsigned short int Pomiary[])	// Tablica z pomiarami zaczerpnietymi z ADC
{
	float DCI_Chwilowe = 0, Usrednione_DCI = 0, suma = 0;			// Zmienne pomocnicze w liczeniu wartosci sredniej
	int i = 0;

	DCI_Chwilowe = (Pomiary[0] - 1674) * 0.0058616647;		//  ((Pomiary[0] - 1705)/1705.0 * 10.0)
	for(i = 149; i > 0; i--)
	{
		TAB_DCI[i] = TAB_DCI[i - 1];						// Przesuniecie tablicy elementow DCI
		suma = suma + TAB_DCI[i];							// Sumowanie calej tablicy
	}
	TAB_DCI[0] = (DCI_Chwilowe);
	suma = suma + (DCI_Chwilowe);
	Usrednione_DCI = suma * 6.666666667;		// Wyliczenie sredniej;  suma/150 * 1000 mA

	return Usrednione_DCI;								// Zwrocenie wartosci sredniej calej tablicy probek na zewnatrz funkcji
}

void Tworzenie_Rampy(double Rampa[],		//tablica zawierajaca wartosci wzgledne rampy
					float *boost_old,
					int n)					//liczba elementow tablicy - liczba probek, obecnie ustawiona na 240
{
	double p = 0;
	int i = 0;
	for(i = 0; i < n; i++)
	{
		p = 10000 * i / (n - 1);				//zwiekszenie dokladnosci tak jak w przypadku tablicy sinusow
		p = p / 10000;
		Rampa[i] = (-1.355 * p * p * p + 1.975 * p * p + 0.391 * p + 0.001);	//przypisanie odpowiedniej wartosci kazdemu z elementow
		if(Rampa[i] < 0)
			Rampa[i] = 0;
		if(Rampa[i] > 1)
			Rampa[i] = 1;
	}
	*boost_old = 99;

}
void Tworzenie_Rampy_U_do_F(double Rampa[], double Rampa_U_do_F[], float boost, float *boost_old, int n)
{
	int i = 0;
	if(*boost_old != boost)
	{
		for(i = 0; i < n; i++)
		{
			Rampa_U_do_F[i] = Rampa[i] + (1.0 - 1000.0 * i / (1000.0 * n)) * boost * 0.01;
			if(Rampa_U_do_F[i] < 0)
				Rampa_U_do_F[i] = 0;
			if(Rampa_U_do_F[i] > 1)
				Rampa_U_do_F[i] = 1;
		}
		*boost_old = boost;
	}
}

float Rampa_Gladka(float ksztalt,		//zmienna okreslajaca ksztalt U/f; 0 - liniowa, 1 - gladka
		 float boost,		//boost napieciowy
		 float f, 			//obecna czestotliwosc
		 float f_old,		//poprzednia czestotliwosc
		 int USIN[],		//tablica sinusow w zakresie -499,5; 499,5
		 int USINPWM[],		//tablica z wprowadzonym offsetem
		 double Rampa[],	//tablica z wartosciami wzglednymi rampy gladkiej
		 double Rampa_U_do_F[],
		 int n)
{
	int i = 0, p = 0;
	if(f_old != f)
	{
		if (ksztalt == 1)		//rampa g³adka
		{
			p = round(f * 0.02 * (n - 1));	//jako element tablicy trzeba podac wartosc typu int, wiec nalezy zaokraglic wartosc
			for(i = 0; i < n; i++)						//rownanie jest dopasowane tak, aby rampa od wartosci wzglednych na osi Y od 0 do 1
				USINPWM[i] = round(Rampa_U_do_F[p] * USIN[i] + 100);	//przeskalowana byla do wartosci od boost do fmax
		}
		f_old = f;
	}//zapewnienie wyznaczenia tablicy tylko podczas zmiany czestoliwosci
	return f_old;
}

float RampaGladkaF(int *x, 	//zewnetrzna zmienna pomocnicza sluzaca do sterowania zmiana czasu narastania i opadania rampy
		  int *m, 	//zewnetrzna zmienna pomocnicza wskazujaca kolejne elementy rampy
		  float ksztalt_ros, 			//zmienna okreslajaca ksztalt rampy narastajacej; 1 - ramp gladka, 0 - liniowa
		  float ksztalt_op,			//zmienna okreslajaca ksztalt rampy opadajacej; 1 - ramp gladka, 0 - liniowa
		  float f, 						//aktualna czestotliwosc
		  float fzad, 					//czestotliwosc obecnie zadana
		  int *fzad_old, 	//poprzednio zadana czestotliwosc
		  float czas_ros, 				//czas narastania rampy
		  float czas_op,				//czas opadania rampy
		  double Rampa[], 				//tablica z elementami rampy
		  int n)
{

	if(((*fzad_old) < fzad))
	{
		if((*x) >= (1000 * czas_ros / n) && ksztalt_ros == 1)  //narastanie z ramp¹ g³adk¹, w ciagu zadanego czasu ponizszy kod wykona sie n razy
		{
			(*x) = 0;
			if(fabs(fzad - f) > 0.01)
			{
				f = f + (fzad - (*fzad_old)) * (Rampa[(*m)] - Rampa[(*m) - 1]); //zmiana czestotliwosci o roznice w fzad i fzad_old pomnozona przez
				(*m)++;													//roznice kolejnych elementow rampy
			}
		}
		if((*m) >= (n - 1))		//powrot do wartosci 1 po wskazaniu wszystkich elementow rampy
			(*m) = 1;
		(*x)++;
	}
	if(((*fzad_old) > fzad))
	{
		if((*x) >= (1000 * czas_op / n) && ksztalt_op == 1)		//narastanie z ramp¹ g³adk¹, w ciagu zadanego czasu ponizszy kod wykona sie n razy
		{
			(*x) = 0;
			if(fabs(fzad - f) > 0.05)
			{
				f = f + (fzad - (*fzad_old)) * (Rampa[(*m)] - Rampa[(*m) - 1]); //zmiana czestotliwosci o roznice w fzad i fzad_old pomnozona przez
				(*m)++;													//roznice kolejnych elementow rampy
			}
		}
		if((*m) >= (n - 1))		//powrot do wartosci 1 po wskazaniu wszystkich elementow rampy
			(*m) = 1;
		(*x)++;
	}
	if(fabs(f - fzad) <= 0.1)		//zapewnienie dopasowania f do fzad oraz zmiana fzad_old na fzad dopiero po rzeczywistej zmianie f
	{							//dzieki temu nie mozna zmienic fzad w trakcie zmiany f
		f = fzad;
		*(fzad_old) = fzad;
	}
	if(f > 50)			//zabezpieczenie przed przekroczeniem czestotliwosci wiekszej niz 50
	{
		f = 50;
		fzad = 50;
		(*fzad_old) = 50;
	}
	return f;
}
