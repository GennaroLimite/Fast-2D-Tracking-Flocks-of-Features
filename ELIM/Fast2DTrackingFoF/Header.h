/***************************************************************************
Fast 2D Hand Tracking with Flock of Features and Multi-Cue Integration
di Mathias Kölsch e Matthew Turk

A cura di :     De Crescenzo Alfonso 0124000937
				Gennaro Limite		 0124000891

Filename : Header.h
*****************************************************************************/

#include <iostream>
#include <stdio.h>
#include <ctype.h>
#include <windows.h>
#include "opencv2\opencv.hpp"
#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#define NUMBER_OF_FEATURES 50
#define MIN_DIST 3
#define WHITE_THRESHOLD 128
#define WIDTH 640
#define HEIGHT 480
using namespace cv;
using namespace std;
Point2f CalcMedianFeatures(vector<Point2f> value);
Point2f CalcMeanFeatures(vector<Point2f> value);
bool CalculateDistanceFeatures(vector<Point2f> feature, int index);
void ChangeBckgrnd(Mat backproj);
void SaveScreenshot(int i);
void ClearTracking();
void ConsoleInstructions();


/*
input:

bnd_box - rectanglar area containing hand
mindst - minimum pixel distance between features
n - number of features to track
winsize - size of feature search windows

initialization:
learn color histogram
find n*k good-features-to-track with mindist
rank them based on color and fixed hand mask
pick the n highest-ranked features

tracking:
update KLT feature locations with image pyramids
compute median feature
for each feature
if  less than mindist from any other feature
or outside bnd_box , centered at median or
low match correlation
then relocate feature onto good color spot that meets the flocking conditions

output:
the average feature location


The Flocks of Features tracking algorithm. k is an empirical value,
chosen so that enough features end up on good colors; we use k = 3.
The fixed hand is a known spatial distribution for pixels belonging
to some part of the hand in the initialization posture.
*/
Mat image;
Mat histimg;
bool backprojMode = false; //booleana utilizzata per cambiare tipologia di visualizzazione camera
bool selectObject = false; //booleana che tiene conto se l'oggetto è stato selezionato
int trackObject = 0; //variabile per gestire il comportamento del tracking dell'oggetto
					 
Point origin;
Rect selection;

/*
Smin e Vmin servono per escludere il rumore che interferische il tracking.
Questi parametri definiscono le soglie per ignorare pixel che sono troppo vicini alla neutralità.
Vmin imposta la soglia per "quasi nero," e Smin per "quasi grigio."
Questi due livelli di soglia dovranno essere aggiustati per la configurazione per
ottenere i risultati desiderati per ogni circostanza.
Vmax, permette di fissare una soglia per pixel che sono troppo luminose.
NB : Smin ha l'effetto collaterale di eliminare anche i pixel che sono vicini al bianco,
quindi non dovrebbe essere necessario modificare Vmax per ottenere buoni risultati.
*/

int vmin, vmax, smin;
vector<Point2f> points[2];
TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
Size subPixWinSize(10, 10);
Size winSize(61, 61);

bool needToInit = false;//booleane utilizzate per controllare se è richiesta la necessità di inizializzare le features

// L'utente seleziona riquadro attorno all'oggetto da monitorare.
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
		break;
	}
}
