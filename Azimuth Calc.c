////-----------------------------------------------------------------------------------------------
float convert_decimal_degrees (float num)
{
	float num1 = fabs(num);
	int deg = num1;
	float mind = (((num1-deg)*100)/60);
	float decdeg = deg+mind;
	if (num <1)
	{
		decdeg=-decdeg;
	}
	decdeg= ((decdeg*PI)/PI_Degrees);
	return decdeg;
}
////-----------------------------------------------------------------------------------------------
double azi (float lat1, float lon1, float lat2, float lon2)
{
	float declat1 = convert_decimal_degrees (lat1);
	float declon1 = convert_decimal_degrees (lon1);
	float declat2 = convert_decimal_degrees (lat2);
	float declon2 = convert_decimal_degrees (lon2);

	double londif = declon2-declon1;
	float theta = atan2((sin(londif)*cos(declat2)),(cos(declat1)*sin(declat2)-(sin(declat1)*cos(declat2)*cos(londif))));
	theta = RAD2DEG*theta;
	return theta;
}
