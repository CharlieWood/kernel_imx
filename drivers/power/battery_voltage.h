struct battery_voltage {
	int voltage;
	int percent;
};
struct battery_voltage discharg_percent [] = {
	{817, 100},
	{795, 90},
	{770, 80},
	{745, 55},
	{725, 15},
	{715, 5},
	{705, 0}
};

struct battery_voltage charg_percent [] = {
	{860, 100},
	{820, 50},
	{810, 20},
	{790, 10},
	{770, 5},
	{750, 0}
};
