/***************************************************************************
Fast 2D Hand Tracking with Flock of Features and Multi-Cue Integration
di Mathias Kölsch e Matthew Turk

A cura di :     De Crescenzo Alfonso 0124000937
				Gennaro Limite		 0124000891

Filename : Source.cpp
*****************************************************************************/


#include "Header.h"

int main(int argc, const char** argv)
{
	Point2f mean, median; //punto media delle feature e mediano 
	Rect trackWindow; //finestra presente l'oggetto trackato
	int hsize = 16; // hist size
	float hranges[] = { 0,180 }; //hist range
	const float* phranges = hranges; 

	//Apri Cam /////////////////
	bool playVideo = true;
	VideoCapture cap;
	cout << "//////////////////////////////////////////////////////////////////////" << endl;
	cout << "//  Select :" << endl;
	cout << "//  [1] Open your Cam" << endl;
	cout << "//  [2] Example in the laboratory" << endl;
	cout << "//////////////////////////////////////////////////////////////////////" << endl<<endl<<endl<<endl;
	cout << "//////////////////////////////////////////////////////////////////////" << endl;
	cout << "//  Instructions " << endl;
	cout << "//  * Select the object to track" << endl;
	cout << "//  * Adjust the colors' parameters to have a good backprojection" << endl;
	cout << "//  * Press [p] to pause video and press again to resume" << endl;
	cout << "//  * Press [b] to watch backprojection and press again to restore" << endl;
	cout << "//  * Press [r] to initialize features" << endl;
	cout << "//  * Press [c] to clear" << endl;
	cout << "//  * Press [s] to save a screenshot" << endl;
	cout << "//////////////////////////////////////////////////////////////////////" << endl << endl << endl;
	char settingCam;
	cin >> settingCam;
	switch (settingCam)
	{
	case '1':cap.open(0);			  vmin = 10, vmax = 256, smin = 30;  break;
	case '2': cap.open("Filmato.mp4"); vmin = 61, vmax = 256, smin = 45; break;
	default: cap.open(0);			  vmin = 10, vmax = 256, smin = 30;  break;
	}

	ConsoleInstructions();

	if (!cap.isOpened())
		return EXIT_FAILURE;
	//////////////////////////////////
	namedWindow("Histogram", 0);
	namedWindow("Demo", 0);
	cvResizeWindow("Demo", WIDTH, HEIGHT);
	//////////////////////////////////
	setMouseCallback("Demo", onMouse, 0); //gestisce l'evento mouse
	// creazione dei slider HSV
	createTrackbar("Vmin", "Demo", &vmin, 256, 0);
	createTrackbar("Vmax", "Demo", &vmax, 256, 0);
	createTrackbar("Smin", "Demo", &smin, 256, 0);
	//////////////////////////////////

	Mat frame, hsv, hue, mask, hist;
	histimg = Mat::zeros(200, 320, CV_8UC3);

	// Back Projection è un modo di registrare quanto bene i pixel di una data immagine 
	// misura la distribuzione dei pixel in un modello istogramma.
	Mat backproj;
	Mat  prevBackproj;


	while (1)
	{
		try
		{
			// Inizializzazione della cam ///////////
			if (playVideo)
				cap >> frame;
			if (frame.empty())
				break;
			//copia i valori di frame e mettilo nella matrice image
			frame.copyTo(image);
			//otteniamo dalla matrice image una matrice convertita HSV
			cvtColor(image, hsv, COLOR_BGR2HSV);
			////////////////////////////////////////

			if (trackObject)
			{
				int _vmin = vmin, _vmax = vmax;
				//effettuiamo un'operazione di thresholding sull'immagine HSV , l'ultimo parametro risulterà il suo output
				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				int ch[] = { 0, 0 };
				//ricavo solo della componente hue
				hue.create(hsv.size(), hsv.depth());
				
				//Copia specifico channels e lo metto quello specifico channel in un parametro di output 
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if (trackObject < 0)
				{
					//Roi che serve per selezionare il riquadro di interesse
					Mat roi(hue, selection),
						maskroi(mask, selection); //maskroi serve successivamente per selezionare il riquadro di interesse della maschera
					//Calcolo istogramma
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);//normalizzo l'istogramma
					trackWindow = selection; //assume la finestra le dimensioni e specifiche del riquadro di selezione
					// Non resettare, a meno che non viene selezionata una nuova regione di interesse
					trackObject = 1;
					//cancella i valori della matrice dove vi conterrà l'istogramma RGB
					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					// buffer colori
					Mat buf(1, hsize, CV_8UC3);
					//QUESTA NON HO CAPITO BENE: SAREBBE QUELO CHE CI COLORA LE COLONNE DI COLORI NELL'HISTOGRAM? 
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					//conversione in RGB da HSV
					cvtColor(buf, buf, COLOR_HSV2BGR);
					//rappresentazione Istogramma degli eventuali colori presenti nella finestra//////////////////////
					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows), Point((i + 1)*binW, histimg.rows - val), Scalar(buf.at<Vec3b>(i)), -1, 8);
					}
					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				}

				//calcolo la back projection dell'istogramma
				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				//effettuiamo una AND bit a bit della mask e l'assegnamo a backproj, in modo tale da ricavarci l'immagine in scala di grigi
				backproj &= mask;
				///////////////////////////////////////////
				ChangeBckgrnd(backproj);// premendo 'b' si cambia tipologia di rappresentazione sfondo
				// premendo il tasto 'r' da tastiera
				if (needToInit)
				{
					// Inizializzazione features
					//La seguente può essere usata per inizializzare un point-based tracker di un oggetto.
					//NB : Utilizza Harris Corner Detection
					goodFeaturesToTrack(backproj, points[1], NUMBER_OF_FEATURES, 0.01, MIN_DIST, Mat(), 3, 0, 0.04);
					//Affina le locazioni dei corner
					cornerSubPix(backproj, points[1], subPixWinSize, Size(-1, -1), termcrit);

				}
				else if (!points[0].empty())
				{
					vector<uchar> status;
					vector<float> err;
					if (prevBackproj.empty())
						prevBackproj.copyTo(backproj);

					//Calcola un flusso ottico per una serie di features sparse con il metodo iterativo Lucas-Kanade pyramid-based
					//(x) Update KLT feature locations with image pyramids
					calcOpticalFlowPyrLK(prevBackproj, backproj, points[0], points[1], status, err, winSize, 10, termcrit, 0, 0.001);

					//La variabile tiene conto solo dei punti che hanno rispettato la condizione che presenta ogni feature [(x) for each feature]
					vector<Point2f> good_features_to_track;
					//(x) for each feature
					for (int i = 0; i < points[1].size(); i++)
					{
						// Otteniamo il livello di bianco dal punto dell' immagine. [128,0,0,0] se il primo 
						// parametro della variabile è superiore di una certa soglia di bianco
						Scalar colour = backproj.at<uchar>(Point2f(points[1][i].x, points[1][i].y));
						//(x) the "for each features" conditions 
						if (points[1][i].inside(trackWindow) && CalculateDistanceFeatures(points[1], i) &&
							status[i] == true && colour.val[0] > WHITE_THRESHOLD &&
							points[1][i].x < WIDTH && points[1][i].y < HEIGHT && points[1][i].x > 0 && points[1][i].y > 0)
						{
							//Disegna feature
							circle(image, points[1][i], 3, Scalar(0, 0, 255), -1, 8);
							good_features_to_track.push_back(points[1][i]);
						}
						else
						{
							//(x) relocate features onto good color spot that meets the flocking behavior
							while (1)
							{
								//otteniamo randomicamente delle nuove coordinate per i punti (x,y) della feature
								points[1][i].x = trackWindow.x + (rand() % trackWindow.width - 1);
								points[1][i].y = trackWindow.y + (rand() % trackWindow.height - 1);
								//Otteniamo il livello di bianco dal punto dell' immagine
								Scalar colour = backproj.at<uchar>(Point(points[1][i].x, points[1][i].y));
								// [128,0,0,0] se il primo parametro della variabile è superiore di una certa soglia di bianco
								if (colour.val[0] > WHITE_THRESHOLD)
								{	//disegna la features rilocata
									circle(image, (points[1][i]), 3, Scalar(0, 0, 255), -1, 8);
									break;
								}
							}
						}
					}
					//Calcolo della media in base ai features collocati correttamente 
					mean = CalcMeanFeatures(good_features_to_track);
					//Disegna il punto della media
					circle(image, mean, 8, Scalar(0, 255, 0), -1, 8);
					//(x) Compute median feature
					median = CalcMedianFeatures(good_features_to_track);
					//disegno punto mediano
					//circle(image, median, 8, Scalar(0, 171, 255), -1, 8);
				}

				needToInit = false;//non serve inizializzare in automatico i features

				if (trackObject)
				{
					//controllo se il mediano è stato calcolato
					if (median.x != 0 && median.y != 0)
					{
						//se la x e la y del mediano sono diversi da 0, vuol dire che il mediano l'abbiamo calcolato
						//allora dobbiamo far muovere la trackWindow in base alla media
						//quindi calcoliamo l'angolo in alto a sinistra per calcolarci la nuova trackwindow
						Point2f topLeft;
						//mi prendo il vertice superiore sinistro semplicemnte sottraeno alla x e alla y della media
						//rispettivamente la metà della larghezza e la metà dell'altezza.
						topLeft.x = median.x - (trackWindow.width / 2);
						topLeft.y = median.y - (trackWindow.height / 2);
						//calcolo la nuova trackwindow passandogli le coordinate x,y del topleft corner
						//e gli stessi valori di larghezza e altezza
						Rect newTrack = Rect(topLeft.x, topLeft.y, trackWindow.width, trackWindow.height);
						//assegno alla trackWindow la nuova trak appena calcolata
						trackWindow = newTrack;
						//disegno la nuova trackwindow
						rectangle(image, trackWindow, Scalar(255, 0, 0), 3, LINE_AA);
					}
					else
					{
						//qui entro solo se il mediano non è stato calcolato
						rectangle(image, trackWindow, Scalar(255, 0, 0), 3, LINE_AA);
					}
				}
				else
					//qui entro solo se il trakObject non c'è (ovvero false)
					rectangle(image, trackWindow, Scalar(255, 0, 0), 3, LINE_AA);
			}

			// nuova ROI condition , viene chiamata ogni qual volta viene fatta una selezione riquadro tramite Mouse Event
			// Rende user-friendly la selezione.
			if (selectObject && selection.width > 0 && selection.height > 0)
			{
				Mat roi(image, selection);
				bitwise_not(roi, roi);
			}
			//mostra le finestre /////////
			imshow("Demo", image);
			imshow("Histogram", histimg);
			///////////////////////////////
			//aggiornamento punti NB : points[0] : previous point diventeranno i punti attuali, 
			//points[1] : passeranno da i prossimi punti da calcolare ai precedenti punti calcolati
			swap(points[1], points[0]);
			//aggiornamento back projection
			swap(prevBackproj, backproj);

			// Opzioni //////////////////////////////
			int index = 0; //contatore dei file .jpg salvati
			switch (waitKey(10))
			{
				//uscita dal programma
			case 27: return 0;
				//passa backprojection premendo 'b' (se già è backprojection, ritorna default)
			case 'b':
				cout << endl << "You pressed [b]" << endl;
				backprojMode = !backprojMode;
				break;
			case 'r':
				cout << endl << "You pressed [r]" << endl;
				needToInit = true;
				break;
				//pulizia tracker
			case 'c':
				cout << endl << "You pressed [c]" << endl;
				needToInit = false;
				points[0].clear();
				points[1].clear();
				trackObject = 0;
				histimg = Scalar::all(0);
				break;
				// gestione pausa/riprendi
			case 'p':
				cout << endl << "You pressed [p]" << endl;
				playVideo = !playVideo;
				break;
				//salvataggio frame corrente in un file .jpg
			case 's':
				SaveScreenshot(index);
				break;
			case 'm':
				ConsoleInstructions();
				break;
			default: break;
			}
			////////////////////////////////////////
		}
		catch(Exception e)
		{
			//imposto la x e la y del mediano in modo tale che 
			//la trackWindow si resetti e non gestisca la sua posizione 
			//in base al mediano ma in base alla selezione.
			median.x = 0;
			median.y = 0;
			ClearTracking();				
			system("CLS");
			cout << "Out of main window or no good features' location";
			Sleep(3000);
			ConsoleInstructions();
					
		}
	}
	return 0;
}

/*
Change in Default to Backprojection
*/
void ChangeBckgrnd(Mat backproj)
{
	if (backprojMode)
		cvtColor(backproj, image, COLOR_GRAY2BGR);
}

/*
Save the current frame into a file .jpg
*/
void SaveScreenshot(int i)
{
	bool saved = false;
	do
	{
		if (imread("img" + to_string(i) + ".jpg", 1).empty())
		{
			imwrite("img" + to_string(i) + ".jpg", image);
			cout << "Save img" + to_string(i) << endl;
			saved = true;
		}
		else
			i++;
	} while (saved == false);
}

/*
Compute the median features
*/
Point2f CalcMedianFeatures(vector<Point2f> value)
{
	float medianX, medianY;
	Point2f median_result;
	int size = value.size();
	if (size == 0)
	{
		return Point2f(0, 0);
	}
	//salviamo le x e le y di tutti i punti in 2 array temporanei
	vector<float> valuesX(size);
	vector<float> valuesY(size);
	for (int i = 0; i < size; i++)
	{
		valuesX[i] = value[i].x;
		valuesY[i] = value[i].y;
	}
	//ordiniamo l'array temporaneo delle x e delle y in modo crescente
	sort(valuesX.begin(), valuesX.end());
	sort(valuesY.begin(), valuesY.end());

	//ricaviamo la x e y mediana all'interno degli array
	if (size % 2 == 0)
	{
		medianX = (valuesX[size / 2 - 1] + valuesX[size / 2]) / 2;
		medianY = (valuesY[size / 2 - 1] + valuesY[size / 2]) / 2;
	}
	else
	{
		medianX = valuesX[size / 2];
		medianY = valuesY[size / 2];
	}
	//assegnazione delle x e y ottenute
	median_result.x = medianX;
	median_result.y = medianY;
	return median_result;
}

/*
Calculate the average feature location
*/
Point2f CalcMeanFeatures(vector<Point2f> value)
{
	Point2f sum = 0;
	if (value.size() == 0)
	{
		cout <<endl<<"Retry. (Adjust the colors' parameters) "<<endl;
		ClearTracking();
	}
	for (int i = 0; i < value.size(); i++)
	{
		sum += value[i];
	}
	int size = value.size();
	return sum / size;
}

bool CalculateDistanceFeatures(vector<Point2f> feature, int index)
{
	for (index; index < feature.size() - 1; index++)
	{
		//calcolo della distanza fra due punti
		float distance = sqrt(((feature[index + 1].x - feature[index].x) * (feature[index + 1].x - feature[index].x)) + ((feature[index + 1].y - feature[index].y) * (feature[index + 1].y - feature[index].y)));
		if (distance <= MIN_DIST)
		{
			//non rispetta la distanza e quindi dovrà avvenire la rilocazione
			return false;
		}
	}
	return true; //il punto rispetta la distanza minima da ogni altri punto
}

/*
	Clear Tracking function
*/
void ClearTracking()
{
	needToInit = false;
	points[0].clear();
	points[1].clear();
	trackObject = 0;
	histimg = Scalar::all(0);
}

void ConsoleInstructions()
{
	system("CLS");
	cout << "//////////////////////////////////////////////////////////////////////" << endl;
	cout << "//  Instructions " << endl;
	cout << "//  * Select the object to track" << endl;
	cout << "//  * Adjust the colors' parameters" << endl;
	cout << "//  * Press [p] to pause video and press again to resume" << endl;
	cout << "//  * Press [b] to watch backprojection and press again to restore" << endl;
	cout << "//  * Press [r] to initialize features" << endl;
	cout << "//  * Press [c] to clear" << endl;
	cout << "//  * Press [s] to save a screenshot" << endl;
	cout << "//  * Press [m] to clear the console window " << endl;
	cout << "//  * Press [Esc] to exit to the camera's application" << endl;
	cout << "//////////////////////////////////////////////////////////////////////" << endl << endl << endl;
}