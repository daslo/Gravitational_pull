#include <stdio.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#define MAX 10
using namespace std;
const float FPS = 60;
//rozmiar ekranu
const int SCREEN_W = 400;
const int SCREEN_H = 300;
const int lMax = 10; //maksymalna liczba ciał
const float PI = 3.1415;

const float* getBodyParams() { //pobranie parametrów ciała z klawiatury
	float p[6];
	cout << "x:";
	cin >> p[0];
	cout << "y:";
	cin >> p[1];
	cout << "vx:";
	cin >> p[2];
	cout << "vy:";
	cin >> p[3];
	cout << "m:";
	cin >> p[4];
	cout << "r:";
	cin >> p[5];

	return (const float*) p;
}

class BODY {
public:
	float x; //współrzędne
	float y;
	float vx; //składowe prędkości
	float vy;
	float m; //masa
	float r; //promień
public:
	BODY() {//konstruktor domyślny
		x = 0;
		y = 0;
		vx = 0;
		vy = 0;
		m = 1;
		r = 1;
	}
	BODY(float x, float y, float vx, float vy, float m, float r) {//konstruktor
		set (x, y, vx, vy, m, r);
	}
	void set( float x, float y, float vx, float vy, float m, float r) {
		
		this->x = x;
		this->y = y;
		this->vx = vx;
		this->vy = vy;
		this->m = m;
		this->r = r;
	}
	string info() {
		string s = to_string(x) + " " + to_string(y) + " " + to_string(vx) + " " + to_string(vy) + " " + to_string(m) + " " + to_string(r);
		return s;
	}
	void Print() { //wypisz info o ciele
		cout << "(" << x << "," << y << ") (" << vx << "," << vy << "), m=" << m << " r=" << r << endl;
	}
	void move(float dt) {
		x += vx*dt;
		y += vy*dt;
	}
	void accelerate(float ax, float ay, float dt) {
		vx += ax*dt;
		vy += ay*dt;
	}
};

class World {
public:
	float G = 1; //stała grawitacyjna
	float dt = 0.01; //odcinek czasu
	float cor = 1; //współczynnik restytucji
	bool bounds = true; //ramka wokół ekranu
	int lB; //liczba ciał
	BODY tB[MAX]; //tablica ciał

	World() {
		lB = 0;
		for (int i = 0; i < MAX; i++) {
			tB[i] = BODY();
		}
	}
	void accelerate() {
		for (int i = 0; i < lB; i++) {
			for (int j = 0; j < lB; j++) {
				if (i == j);
				else {
					float ay = G*tB[j].m*sin_a(i, j) / dist_2(i, j); //Fy=GMm/r^2 * sin(a)
					float ax = G*tB[j].m*cos_a(i, j) / dist_2(i, j); //Fx=GMm/r^2 * cos(a)
					tB[i].accelerate(ax, ay, dt);
				}
			}
		}
	}
	void mkBody() { //dodanie ciała
		const float* p = getBodyParams();
		tB[lB] = BODY(p[0], p[1], p[2], p[3], p[4], p[5]); //stworzenie obiektu
		cout << "new body";
		tB[lB].Print();
		lB++; //zwiększ licznik ciał
	}
	void mkBody(float p0, float p1, float p2, float p3, float p4, float p5) {
		tB[lB] = BODY(p0, p1, p2, p3, p4, p5); //stworzenie obiektu
		cout << "new body";
		tB[lB].Print();
		lB++; //zwiększ licznik ciał
	}
	void mkBody(float p[]) {
		tB[lB] = BODY(p[0], p[1], p[2], p[3], p[4], p[5]); //stworzenie obiektu
		cout << "new body";
		tB[lB].Print();
		lB++; //zwiększ licznik ciał
	}
	void modifyBody(int id) {
		const float *p = getBodyParams();
		tB[id].set(p[0], p[1], p[2], p[3], p[4], p[5]);
	}
	void delBody(int id) { //usunięcie ciała
		for (int i = id; i < lB; i++) {
			cout << "DELETED " << id << endl;
			tB[i] = tB[i + 1];
			lB--;
		}
	}
	void collide(int i,int  j) {
		cout << "COLLISION: " << i << "," << j << endl;
		//obliczanie parametrów zderzenia z wykorzystaniem wektorów
		float vix, viy, vjx, vjy, vix2, viy2, vjx2, vjy2, mi, mj;
		vix = tB[i].vx;
		viy = tB[i].vy;
		vjx = tB[j].vx;
		vjy = tB[j].vy;
		mi = tB[i].m;
		mj = tB[j].m;
		
		float nix, niy, tix, tiy;
		nix = (tB[j].x - tB[i].x) / dist(i, j); //wektor jednostkowy łączący środki ciał (normalny)
		niy = (tB[j].y - tB[i].y) / dist(i, j);

		tix = -niy; //wektor styczny
		tiy = nix;

		float vi_t, vj_t, vi_n, vj_n, vi_n2, vj_n2;

		vi_t = tix*vix + tiy*viy;//obliczenie składowych stycznych
		vj_t = tix*vjx + tiy*vjy;

		vi_n = nix*vix + niy*viy;//obliczenie składowych normalnych
		vj_n = nix*vjx + niy*vjy;
		if (cor == 1) {
			vi_n2 = (vi_n*(mi - mj) + 2 * mj*vj_n) / (mi + mj); //nowe prędkości normalne
			vj_n2 = (vj_n*(mj - mi) + 2 * mi*vi_n) / (mi + mj);

			vix2 = vi_n2*nix + tix*vi_t;//nowe współrzędne prędkości
			viy2 = vi_n2*niy + tiy*vi_t;

			vjx2 = vj_n2*nix + tix*vj_t;
			vjy2 = vj_n2*niy + tiy*vj_t;
			tB[i].vx = vix2; //przypisanie nowych współrzędnych prędkości
			tB[i].vy = viy2;
			tB[j].vx = vjx2;
			tB[j].vy = vjy2;

		}
		if (cor == 0) {
			float r = sqrt(tB[j].r*tB[j].r + tB[i].r*tB[i].r); //promień nowego ciała
			float m = mi + mj;//masa nowego ciała
			float v_t = (vi_t*mi + vj_t*mj) / m; //prędkości styczna
			float v_n = (vi_n*mi + vj_n*mj) / m; //prędkość normalna
			float r_x = (tB[i].x + tB[j].x) / 2; //współrzędne położenia nowego ciała
			float r_y = (tB[i].y + tB[j].y) / 2;
			float v_x = v_t*tix + v_n*nix; //współrzędne prędkości nowego ciała
			float v_y = v_t*tiy + v_n*niy;
			delBody(j > i ? j : i);//usunięcie ciał
			delBody(j > i ? i : j);
			mkBody(r_x, r_y, v_x, v_y, m, r);//dodanie nowego ciała
		}
		

	}
	void move() {
		for (int i = 0; i < lB; i++) {
			tB[i].move(dt);
		}
	}
	bool distanceCheck(int i, int j) {//porównanie odległości między środkami z sumą promieni
		if (dist(i, j) > (tB[i].r + tB[j].r)) {
			return 1;
		}
		else {
			return 0;
		}
	}
	float dist_2(int i, int j) { //odległość między środkami ^2
		float x1 = tB[i].x;
		float x2 = tB[j].x;
		float y1 = tB[i].y;
		float y2 = tB[j].y;
		return (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
	}
	float dist(int i, int j) { //odległość między środkami
		float x1 = tB[i].x;
		float x2 = tB[j].x;
		float y1 = tB[i].y;
		float y2 = tB[j].y;
		return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	}
	float sin_a(int i, int j) { //sinu kąta skierowanego
		return (tB[j].y - tB[i].y) / dist(i, j);
	}
	float cos_a(int i, int j) { //cosinus kąta skierowanego
		return (tB[j].x - tB[i].x) / dist(i, j);
	
	}
	void collideWall(int i) { //zderzenie ze ścianą vn2=-vn1
		if (tB[i].x - tB[i].r <= 0) tB[i].vx = -tB[i].vx;
		if (tB[i].x + tB[i].r >= SCREEN_W) tB[i].vx = -tB[i].vx;
		if (tB[i].y - tB[i].r <= 0) tB[i].vy = -tB[i].vy;
		if (tB[i].y + tB[i].r >= SCREEN_H) tB[i].vy = -tB[i].vy;
	}
	bool detectCollisions() {//wykrywanie kolizji
		for (int i = 0; i < lB; i++) {
			for (int j = 0; j < i; j++) {//sprawdź każdą parę dokładnie 1 raz
				if (!distanceCheck(i, j)) {
					collide(i, j);
				}
			}
			if (bounds) { //zderzenie ze ścianą, jeśli ramka włączona
				collideWall(i);
			}
		}
		return 1;
	}

	string info() {
		string s;
		s = to_string(G) + " " + to_string(dt) + " " + to_string(lB) + "\n";
		for (int i = 0; i < lB; i++) {
			s += tB[i].info() + "\n";
		}
		return s;
		
	}
	//ustawianie dt i G:
	void setG() {
		float s;
		cout << "G= ";
		cin >> s;
		G = s;
	}
	void setdt() {
		float s;
		cout << "dt= ";
		cin >> s;
		dt = s;
	}
	void destroy() {
		for (int i = 0; i < lB; i++) delBody(i);
		lB = 0;
	}
	void start() {
		tB[0] = BODY( 200, 150, 0, 0, 20000, 20);
		tB[1] = BODY( 250, 150, 0, 20, 0.01, 5);
		tB[2] = BODY( 200, 250, -14.14, 0, 0.05, 10);
		dt = 0.01;
		lB = 3;
		G = 1;
	}
	void Print() {
		cout << "G=" << G << " dt=" << dt << " lB=" << lB << endl;
		for (int i = 0; i < lB; i++) {
			tB[i].Print();
		}
	}
};

int main(int argc, char **argv) {
	//Stworzenie obiektów z biblioteki allegro
	ALLEGRO_DISPLAY *display = NULL;
	ALLEGRO_EVENT_QUEUE *event_queue = NULL;
	ALLEGRO_TIMER *timer = NULL;


	string s= "";
	bool bExit = false; //czy wyjść?
	bool bPause = false; //czy symulacja jest zapauzowana?
	bool redraw = true; //czy odświeżać obraz?
	bool bType = false; //tryb wprowadzania znaków
	int focus = 0; //wyświetlanie właściwości n-tego ciała; -1=żaden

	World W1; //stworzenie świata
	W1.start();
	al_init_primitives_addon(); //do rysowania prostych figur
	{
		if (!al_init()) {
			fprintf(stderr, "failed to initialize allegro!\n");
			return -1;
		}
		if (!al_install_keyboard()) {
			fprintf(stderr, "failed to initialize the keyboard!\n");
			return -1;
		}
		if (!al_install_mouse()) {
			fprintf(stderr, "failed to initialize the mouse!\n");
			return -1;
		}
		timer = al_create_timer(1.0 / FPS);
		if (!timer) {
			fprintf(stderr, "failed to create timer!\n");
			return -1;
		}
		display = al_create_display(SCREEN_W, SCREEN_H);
		if (!display) {
			fprintf(stderr, "failed to create display!\n");
			al_destroy_timer(timer);
			return -1;
		}
	}

	al_set_target_bitmap(al_get_backbuffer(display));

	event_queue = al_create_event_queue();

	if (!event_queue) {
		fprintf(stderr, "failed to create event_queue!\n");
		al_destroy_display(display);
		al_destroy_timer(timer);
		return -1;
	}

	al_register_event_source(event_queue, al_get_display_event_source(display));
	al_register_event_source(event_queue, al_get_timer_event_source(timer));
	al_register_event_source(event_queue, al_get_keyboard_event_source());
	al_register_event_source(event_queue, al_get_mouse_event_source());
	al_clear_to_color(al_map_rgb(0, 0, 0)); //clear screen
	al_flip_display();
	al_start_timer(timer);

	while (!bExit)
	{
		ALLEGRO_EVENT ev;
		al_wait_for_event(event_queue, &ev);

		if (ev.type == ALLEGRO_EVENT_TIMER) {
			if (!bPause) {
				W1.detectCollisions();
				W1.move();
				W1.accelerate();
				
				redraw = true;
			}
		}
		else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
			break;
		}
		else if (ev.type == ALLEGRO_EVENT_MOUSE_BUTTON_DOWN) {
			switch (ev.mouse.button) {
			case 1:
				cout << "(" << ev.mouse.x << ", " << ev.mouse.y << ")" << endl;
				break;
			}
		}
		else if (ev.type == ALLEGRO_EVENT_KEY_CHAR) {
			switch (ev.keyboard.keycode) {
			case ALLEGRO_KEY_UP:
				if (W1.lB && focus > -1 && focus <= W1.lB) W1.tB[focus].vy -= 10;
				break;
			case ALLEGRO_KEY_DOWN:
				if (W1.lB && focus > -1 && focus <= W1.lB) W1.tB[focus].vy += 10;
				break;
			case ALLEGRO_KEY_LEFT:
				if (W1.lB && focus > -1 && focus <= W1.lB) W1.tB[focus].vx -= 10;
				break;
			case ALLEGRO_KEY_RIGHT:
				if (W1.lB && focus > -1 && focus <= W1.lB) W1.tB[focus].vx += 10;
				break;
			}
		}
		else if (ev.type == ALLEGRO_EVENT_KEY_DOWN) {
			switch (ev.keyboard.keycode) {
			case ALLEGRO_KEY_N://New
				al_stop_timer(timer);
				W1.mkBody();
				al_start_timer(timer);
				break;
			case ALLEGRO_KEY_F://nr sfocusowanego ciała
				cout << "FOCUS: " << focus << endl;
				break;
			case ALLEGRO_KEY_T: //ustaw dt
				al_stop_timer(timer);
				W1.setdt();
				al_start_timer(timer);
				break;
			case ALLEGRO_KEY_Z: //typ zderzenia
				if (W1.cor == 1) W1.cor = 0;
				else if (W1.cor == 0) W1.cor = 1;
				cout << "COR=" << W1.cor << endl;
				break;
			case ALLEGRO_KEY_G: //ustaw G
				al_stop_timer(timer);
				W1.setG();
				al_start_timer(timer);
				break;
			case ALLEGRO_KEY_D: //usuń sfocusowane ciało
				al_stop_timer(timer);
				if (focus > -1 && focus < W1.lB) {
					W1.delBody(focus);
					cout << "DELETED " << focus << endl;
				}
				focus = -1;
				al_start_timer(timer);
				redraw = true;
				break;
			case ALLEGRO_KEY_M://Modify
				al_stop_timer(timer);
				W1.modifyBody(focus);
				al_start_timer(timer);
				redraw = true;
				break;

			case ALLEGRO_KEY_1: if (W1.lB >= 2) focus = 1; break;//zmiana sfocusowanego ciała
			case ALLEGRO_KEY_2: if (W1.lB >= 3) focus = 2; break;
			case ALLEGRO_KEY_3: if (W1.lB >= 4) focus = 3; break;
			case ALLEGRO_KEY_4: if (W1.lB >= 5) focus = 4; break;
			case ALLEGRO_KEY_5: if (W1.lB >= 6) focus = 5; break;
			case ALLEGRO_KEY_6: if (W1.lB >= 7) focus = 6; break;
			case ALLEGRO_KEY_7: if (W1.lB >= 8) focus = 7; break;
			case ALLEGRO_KEY_8: if (W1.lB >= 9) focus = 8; break;
			case ALLEGRO_KEY_9: if (W1.lB >= 10) focus = 9; break;
			case ALLEGRO_KEY_0: if (W1.lB >= 1) focus = 0; break;

			case ALLEGRO_KEY_B:
				W1.bounds = !W1.bounds;//włącz/wyłącz ramkę
				cout << "RAMKA: " <<( W1.bounds ? "ON" : "OFF" )<< endl;
				break;
			case ALLEGRO_KEY_C://clear
				al_stop_timer(timer);
				cout << "CLEAR" << endl;
				W1.destroy();
				al_start_timer(timer);
				break;
			case ALLEGRO_KEY_R://reset
				al_stop_timer(timer);
				W1.destroy();
				W1.start();
				cout << "RESET" << endl;
				al_start_timer(timer);
				break;
			case ALLEGRO_KEY_P://pause
				cout << "PAUSE" << endl;
				bPause = !bPause;
				break;
			case ALLEGRO_KEY_Q://quit
				bExit = 1;
				break;
			case ALLEGRO_KEY_I://Info
				W1.Print();
				break;
			}
			redraw = true;
		}

		if (redraw && al_is_event_queue_empty(event_queue)) {
			redraw = false;
			al_clear_to_color(al_map_rgb(0, 0, 0));
			for (int i = 0; i < W1.lB; i++) {
				if(focus==i) al_draw_filled_circle(W1.tB[i].x, W1.tB[i].y, W1.tB[i].r+3, al_map_rgb(0, 255, 0));//obwódka
				al_draw_filled_circle(W1.tB[i].x, W1.tB[i].y, W1.tB[i].r, al_map_rgb(255, 255, 255));//ciało
			}
			al_flip_display();
		}
	}

	al_destroy_timer(timer);
	al_destroy_display(display);
	al_destroy_event_queue(event_queue);
	return 0;
}