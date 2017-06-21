

ring ThomasAlgorithm::thomasAlgorithm(std::vector<float> x, std::vector<float> v){
	float sumx = 0;
	float sumx2 = 0;
	float sumx3 = 0;
	float sumy = 0;
	float sumy2 = 0;
	float sumy3 = 0;
	float sumxy = 0;
	float sumx2y = 0;
	float sumxy2 = 0;

	for(int i = 0; i < x.size(); i++){
		sumx = sumx + x[i];
		sumx2 = sumx2 + pow(x[i], 2);
		sumx3 = sumx3 + pow(x[i], 3);
		sumy = sumy + v[i];
		sumy2 = sumy2 + pow(v[i], 2);
		sumy3 = sumy3 + pow(v[i], 3);
		sumxy = sumxy + (x[i] * v[i];
		sumx2y = sumx2y + (pow(x[i], 2) * v[i];
		sumxy2 = sumxy2 + (x[i] * pow(v[i], 2));
	}

	float a1, b1, c1, a2, b2, c2, x1, y, R, temp;
	a1 = 2(pow(sumx, 2) - (x.size() * sumx2));
	b1 = 2((sumx * sumy) - (x.size * sumxy);
	a2 = b1;
	b2 = 2(pow(sumy, 2) - (x.size() * sumy2));
	c1 = (sumx2 * sumx) - (x.size() * sumx3) + (sumy * sumy2) - (x.size() * sumxy2);
	c2 = (sumx2 * sumy) - (x.size() * sumy3) + (sumy * sumy2) - (x.size() * sumx2y);

	x1 = ((c1 * b2) - (c2 * b1))/((a1 * b2) - (a2 * b1));
	y = ((a1 * c2) - (a2 * c1))/((a1 * b2) - (a2 * b1));

	temp = (1/x.size()) * (sumx2 - (2 * sumx * x1) + (x.size() * pow(x1, 2)) + sumy2 - (2 * sumy * y) + (x.size() * pow(y, 2)));

	R = pow(temp, 0.5);

	ring estring;
	estring.centerX = x1;
	estring.centerY = y;
	estring.radius = R;

	return estring;
}
