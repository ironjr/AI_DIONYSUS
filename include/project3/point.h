struct point {
	double x;
	double y;
	double th;

	double distanceWith(point &p) {
		return sqrt(pow(this->x - p.x, 2) + pow(this->y - p.y, 2));
	}
};

struct GridMapPoint {
	int i;
	int j;

	GridMapPoint() {
		i = 0;
		j = 0;
	}

	GridMapPoint(point p, double res, double map_origin_x, double map_origin_y) {
		i = floor((map_origin_x + 0.5) + p.x / res);
		j = floor((map_origin_y + 0.5) + p.y / res);
	}
};